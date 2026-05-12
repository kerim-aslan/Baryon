/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/rhi/pipeline.hh"
#include "mochi/module/device.hh"
#include "mochi/module/swapchain.hh"
#include "mochi/core.hh"
#include "mochi/rhi/shader.hh"
#include "mochi/types.hh"
#include <vulkan/vulkan_raii.hpp>



namespace mochi::rhi
{
  constexpr inline fun align_size(u64 size, u64 alignment) { if (alignment == 0) return size; else return ((size + alignment - 1) / alignment) * alignment; }


  info<pipeline>::info(std::vector<pushSlot> push, std::vector<vertexSlot> vertex, std::vector<descriptorSlot> descriptor)
    : m_push(push)
    , m_vertex(vertex)
    , m_descriptor(descriptor)
  {
    /// Push Constants
    u64 off{};
    for (auto typ: push) {
      off = align_size(off, typ.type().align());

      vk_PushConstant.push_back(vk::PushConstantRange(
        typ.shaderStage(),
        off,
        typ.type().size() * typ.type().count()
      ));

      // DÜZELTME: Sadece size değil, count kadar atlamalıyız!
      off += typ.type().size() * typ.type().count();
    }


    /// Vertex Buffers
    u32 binding_idx{}, location_idx{};
    for (auto ibuf: vertex) {
      u32 v_offset{};

      for (auto &typ: ibuf.ibuf()->items()) {
        v_offset = align_size(v_offset, typ.align());
        
        for (u32 c{}; c < typ.count(); c++) {
          vk_via.push_back(vk::VertexInputAttributeDescription(
            location_idx++,
            binding_idx,
            typ.format(),
            v_offset
          ));
          v_offset += typ.size();
        }
      }

      vk_vib.push_back(vk::VertexInputBindingDescription(
        binding_idx++,
        ibuf.ibuf()->stride(),
        ibuf.inputRate()
      )); 
    }
  
    
    // Descriptors
    u32 desc_binding_idx{};
    for (auto d: descriptor)
      vk_DescriptorBindings.push_back(vk::DescriptorSetLayoutBinding(
        desc_binding_idx++,
        d.kind(),
        1,
        d.shaderStage(),
        nil
      ));
    
  }
  



  pipeline::pipeline(core &core, rhi::info<pipeline> *info, std::vector<shaderSlot> shaders)
    : m_info(info), vk_layout(nil), vk_pipeline(nil)
  {
    bool is_compute = (shaders.size() == 1 && shaders[0].shaderStage() == vk::ShaderStageFlagBits::eCompute);

    // Descriptor & Layout
    vk::DescriptorSetLayoutCreateInfo set_info({}, info->vkDescriptorBindings());
    vk_desc_layout = vk::raii::DescriptorSetLayout(core.sub<module::device>().vdevice(), set_info);

    
    vk::DescriptorPoolSize pool_size(vk::DescriptorType::eUniformBuffer, 1000);
    vk::DescriptorPoolCreateInfo pool_info(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet, 1000, pool_size);
    vk_desc_pool = vk::raii::DescriptorPool(core.sub<module::device>().vdevice(), pool_info);

    vk::PipelineLayoutCreateInfo layout_info({}, *vk_desc_layout, info->vkPushConstant());
    vk_layout = vk::raii::PipelineLayout(core.sub<module::device>().vdevice(), layout_info);


    // Pipeline
    if (is_compute) 
    {
      vk::PipelineShaderStageCreateInfo compute_stage(
        {}, shaders[0].shaderStage(), shaders[0].shader().module(), shaders[0].shader().entry().c_str()
      );

      vk::ComputePipelineCreateInfo compute_info({}, compute_stage, *vk_layout);
      
      // Compute pipeline
      vk_pipeline = vk::raii::Pipeline(core.sub<module::device>().vdevice(), nil, compute_info);
    } 
    else 
    {
      std::vector<vk::PipelineShaderStageCreateInfo> shader_stages;
      shader_stages.reserve(shaders.size());
      for (auto &sh: shaders) {
        shader_stages.push_back(vk::PipelineShaderStageCreateInfo(
          {}, sh.shaderStage(), sh.shader().module(), sh.shader().entry().c_str()
        ));
      }

      std::array<vk::DynamicState, 2> dynamic_states = { vk::DynamicState::eViewport, vk::DynamicState::eScissor };
      vk::PipelineDynamicStateCreateInfo dynamic_info({}, dynamic_states);

      vk::PipelineInputAssemblyStateCreateInfo input_assembly({}, vk::PrimitiveTopology::eTriangleList, VK_FALSE);
      vk::PipelineViewportStateCreateInfo      viewport_state({}, 1, nil, 1, nil);
      vk::PipelineMultisampleStateCreateInfo   multisampling({}, vk::SampleCountFlagBits::e1, VK_FALSE);

      vk::PipelineRasterizationStateCreateInfo rasterizer(
        {}, VK_FALSE, VK_FALSE, vk::PolygonMode::eFill, 
        vk::CullModeFlagBits::eBack, vk::FrontFace::eCounterClockwise, // Düzeltilmiş hali
        VK_FALSE, 0.0f, 0.0f, 0.0f, 1.0f
      );

      vk::PipelineColorBlendAttachmentState blend_attach(
        VK_FALSE, vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA
      );
      vk::PipelineColorBlendStateCreateInfo color_blending({}, VK_FALSE, vk::LogicOp::eCopy, 1, &blend_attach);

      vk::PipelineDepthStencilStateCreateInfo depth_stencil(
        {}, VK_TRUE, VK_TRUE, vk::CompareOp::eLess,
        VK_FALSE, VK_FALSE, {}, {}, 0.0f, 1.0f
      );

      vk::Format depth_format = core.sub<module::swapchain>().depth_format();
      vk::PipelineRenderingCreateInfo rendering_info(
        0, 1, &core.sub<module::swapchain>().format(),
        depth_format, vk::Format::eUndefined
      );

      auto VertexInput = info->vkVertexInput();
      vk::GraphicsPipelineCreateInfo pipeline_info(
        {}, shader_stages, &VertexInput, &input_assembly, nil,
        &viewport_state, &rasterizer, &multisampling, &depth_stencil,
        &color_blending, &dynamic_info, *vk_layout
      );
      pipeline_info.pNext = &rendering_info;

      // Graphics Pipeline
      vk_pipeline = vk::raii::Pipeline(core.sub<module::device>().vdevice(), nil, pipeline_info);
    }
  }
  


  fun pipeline::make(core &core, rhi::info<pipeline> *info, std::vector<shaderSlot> shaders) -> pipeline*
  {
    auto obj = new pipeline(core, info, std::move(shaders));

    core.sub<module::memory>().push<pipeline>(obj);
    return obj;
  }

}
