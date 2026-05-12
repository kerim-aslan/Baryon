/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include "mochi/types.hh"
#include "mochi/rhi/shader.hh"
#include "vulkan/vulkan.hpp"
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.hpp>



namespace mochi::rhi
{

  /** @brief Represents a slot for push constants in a pipeline. */
  struct pushSlot
  {
    public:
      /**
       * @brief Construct a push slot.
       * @param type The Vulkan type format.
       * @param shaderStage The shader stages that can access this push constant.
       */
      inline pushSlot(vt type, vk::ShaderStageFlags shaderStage)
        : m_type(type)
        , m_shaderStage(shaderStage)
      {}


    private:
      vt m_type;
      vk::ShaderStageFlags m_shaderStage;

    public:
      /** @brief Get the Vulkan type format. */
      inline fun type() { return m_type; }
      /** @brief Get the shader stages. */
      inline fun shaderStage() { return m_shaderStage; }

  };


  /** @brief Represents a slot for vertex input in a pipeline. */
  struct vertexSlot
  {
    public:
      /**
       * @brief Construct a vertex slot.
       * @param ibuf Pointer to the buffer info defining the layout.
       * @param inputRate The vertex input rate (per-vertex or per-instance).
       */
      inline vertexSlot(info<buffer> *ibuf, vk::VertexInputRate inputRate)
        : m_ibuf(ibuf)
        , m_inputRate(inputRate)
      {}


    private:
      info<buffer> *m_ibuf;
      vk::VertexInputRate m_inputRate;

    public:
      /** @brief Get the associated buffer info. */
      inline fun ibuf() { return m_ibuf; }
      /** @brief Get the vertex input rate. */
      inline fun inputRate() { return m_inputRate; }

  };


  /** @brief Represents a slot for a descriptor set in a pipeline. */
  struct descriptorSlot
  {
    public:
      /**
       * @brief Construct a descriptor slot.
       * @param ibuf Pointer to the buffer info.
       * @param kind The descriptor type (e.g., Uniform Buffer).
       * @param shaderStage The shader stages that can access this descriptor.
       */
      inline descriptorSlot(info<buffer> *ibuf, vk::DescriptorType kind, vk::ShaderStageFlags shaderStage)
        : m_ibuf(ibuf)
        , m_kind(kind)
        , m_shaderStage(shaderStage)
      {}

      
    private:
      info<buffer> *m_ibuf;
      vk::DescriptorType m_kind;
      vk::ShaderStageFlags m_shaderStage;

    public:
      /** @brief Get the associated buffer info. */
      inline fun ibuf() { return m_ibuf; }
      /** @brief Get the descriptor type. */
      inline fun kind() { return m_kind; }
      /** @brief Get the shader stages. */
      inline fun shaderStage() { return m_shaderStage; }
  };


  /** @brief Represents a shader stage and its module in a pipeline. */
  struct shaderSlot
  {
    public:
      /**
       * @brief Construct a shader slot.
       * @param shaderStage The specific shader stage bit.
       * @param shader The shader module.
       */
      inline shaderSlot(vk::ShaderStageFlagBits shaderStage, shader shader)
        : m_shaderStage(shaderStage)
        , m_shader(std::move(shader))
      {}


    private:
      vk::ShaderStageFlagBits m_shaderStage;
      shader m_shader;

    public:
      /** @brief Access the shader module. */
      inline fun& shader() { return m_shader; }
      /** @brief Get the shader stage. */
      inline fun  shaderStage() { return m_shaderStage; }

  };





  /** @brief Specialization of the info template for Vulkan pipelines. */
  template<>
  struct info<pipeline>
  {
    public:
      /**
       * @brief Construct pipeline info from slots.
       * @param push List of push constant slots.
       * @param vertex List of vertex input slots.
       * @param descriptor List of descriptor slots.
       */
      explicit info(std::vector<pushSlot> push, std::vector<vertexSlot> vertex, std::vector<descriptorSlot> descriptor);

    private:
      std::vector<pushSlot> m_push;
      std::vector<vertexSlot> m_vertex;
      std::vector<descriptorSlot> m_descriptor;

      std::vector<vk::VertexInputBindingDescription> vk_vib;
      std::vector<vk::VertexInputAttributeDescription> vk_via;

      std::vector<vk::PushConstantRange> vk_PushConstant;
      std::vector<vk::DescriptorSetLayoutBinding> vk_DescriptorBindings;
      

    public:
      /** @brief Get the push constant slots. */
      inline fun push() { return m_push; }
      /** @brief Get the vertex slots. */
      inline fun vertex() { return m_vertex; }
      /** @brief Get the descriptor slots. */
      inline fun descriptor() { return m_descriptor; }

      /** @brief Get the compiled Vulkan push constant ranges. */
      inline fun& vkPushConstant() { return vk_PushConstant; }
      /** @brief Get the compiled Vulkan vertex input state. */
      inline fun  vkVertexInput() { return vk::PipelineVertexInputStateCreateInfo({}, vk_vib, vk_via); }
      /** @brief Get the compiled Vulkan descriptor set layout bindings. */
      inline fun& vkDescriptorBindings() { return vk_DescriptorBindings; }
  };




  /** @brief Represents a Vulkan graphics pipeline. */
  struct pipeline
  {
    private:
      /**
       * @brief Construct a new pipeline.
       * @param core The mochi core instance.
       * @param info Pointer to the pipeline info structure.
       * @param shaders List of shader slots for this pipeline.
       */
      explicit pipeline(core &core, info<pipeline> *info, std::vector<shaderSlot> shaders);

    public:
      /**
       * @brief Factory method to create a new pipeline.
       * @param core The mochi core instance.
       * @param info Pointer to the pipeline info structure.
       * @param shaders List of shader slots for this pipeline.
       * @return Pointer to the newly created pipeline.
       */
      static fun make(core &core, info<pipeline> *info, std::vector<shaderSlot> shaders) -> pipeline*;
    

    private:
      info<pipeline> *m_info{};
      vk::raii::PipelineLayout vk_layout;
      vk::raii::Pipeline       vk_pipeline;
      vk::raii::DescriptorSetLayout vk_desc_layout{nil};
      vk::raii::DescriptorPool      vk_desc_pool{nil};

    public:
      /** @brief Get the pipeline info structure. */
      inline fun info() { return m_info; }
      /** @brief Access the underlying Vulkan RAII pipeline. */
      inline fun& get() { return vk_pipeline; }
      /** @brief Access the Vulkan RAII pipeline layout. */
      inline fun& layout() { return vk_layout; }
      /** @brief Access the Vulkan RAII descriptor set layout. */
      inline fun& desc_layout() { return vk_desc_layout; }
      /** @brief Access the Vulkan RAII descriptor pool. */
      inline fun& desc_pool() { return vk_desc_pool; }
  };
  
}
