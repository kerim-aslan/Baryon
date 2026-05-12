/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "Basis.hh"
#include "mochi/world/camera.hh"
#include "mochi/world/light.hh"
#include "mochi/geometry.hh"
#include "mochi/module/renderer.hh"
#include "mochi/module/device.hh"
#include "mochi/module/memory.hh"
#include "mochi/rhi/buffer.hh"
#include "mochi/types.hh"
#include <cstring>
#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  memory::memory(device &device, renderer &renderer)
    : m_device(device)
    , m_renderer(renderer)
  {
    auto props = device.phys_dev().getProperties();
    vk::PhysicalDeviceMemoryProperties mem_props = device.phys_dev().getMemoryProperties();

    m_pushConstant = props.limits.maxPushConstantsSize;

    m_uniformRange = props.limits.maxUniformBufferRange;
    m_uniformAlign = props.limits.minUniformBufferOffsetAlignment;

    m_storageRange = props.limits.maxStorageBufferRange;
    m_storageAlign = props.limits.minStorageBufferOffsetAlignment;

    m_allocCount = props.limits.maxMemoryAllocationCount;
    m_mapAlign   = props.limits.minMemoryMapAlignment;

    m_vramSize = {};
    for (auto &X: mem_props.memoryHeaps)
      if (X.flags & vk::MemoryHeapFlagBits::eDeviceLocal)
        m_vramSize += X.size;


    u64 visible_vram{};
    auto target_flags = vk::MemoryPropertyFlagBits::eDeviceLocal | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent;

    for (auto &X: mem_props.memoryTypes)
      if ((X.propertyFlags & target_flags) == target_flags) {
        u32 heap_index = X.heapIndex;
        u64 heap_size = mem_props.memoryHeaps[heap_index].size;
        if (heap_size > visible_vram)
          visible_vram = heap_size;
      }



    if (props.deviceType == vk::PhysicalDeviceType::eIntegratedGpu)
      m_sharedMemory = true;
    else
      m_sharedMemory = (m_vramSize > 0 && visible_vram >= (m_vramSize * 0.9));



    prepare_light(0);
  }



  fun memory::prepare_camera(u32 count) -> void
  {
    if (!m_camera_ubo && count)
      m_camera_ubo = load_UniformBuffer(&camera_i, count, [](void*){});

    
    if (m_camera_ubo && (m_camera_ubo->size() / camera_i.stride()) != count) {

      auto bak = m_camera_ubo;
      m_camera_ubo = load_UniformBuffer(&camera_i, count, [](void*){});

      memcpy(m_camera_ubo->mapped(), bak->mapped(), std::min(bak->size(), m_camera_ubo->size()));
    }
  }


  fun memory::find_camera(camera *cam) -> u64
  {
    auto cams = list<camera>();

    auto it = std::find(cams.begin(), cams.end(), cam);
    if (it == cams.end())
      throw std::runtime_error("Kamera mochi::memory ye kayıtlı değil.");

      
    return (it - cams.begin());
  }

  fun memory::sync_camera(camera *cam) -> void
  {
    auto &mem = ((camera_t*)m_camera_ubo->mapped())[find_camera(cam)];

    mem.view = cam->view();
    mem.proj = cam->proj();
  }



  fun memory::prepare_light(u32 count) -> void
  {
    if (!m_light_ubo)
      m_light_ubo = load_UniformBuffer(&light_i, 33, [&count](void*){});


    *((u32*)m_light_ubo->mapped()) = count;

    
    //if (m_light_ubo && (m_light_ubo->size() / light_i.stride()) != count) {
    //
    //  auto bak = m_light_ubo;
    //  m_light_ubo = load_UniformBuffer(&light_i, count+1, [](void*){});
    //
    //  memcpy(m_light_ubo->mapped(), bak->mapped(), std::min(bak->size(), m_light_ubo->size()));
    //  //*((u32*)m_light_ubo->mapped()) = count;
    //}
  }


  fun memory::find_light(light *cam) -> u64
  {
    auto cams = list<light>();

    auto it = std::find(cams.begin(), cams.end(), cam);
    if (it == cams.end())
      throw std::runtime_error("Kamera mochi::memory ye kayıtlı değil.");

      
    return (it - cams.begin())+1;
  }

  fun memory::sync_light(light *cam) -> void
  {
    auto &mem = ((light_t*)m_light_ubo->mapped())[find_light(cam)];

    mem.position = {cam->getWorldPos(), 0};
    mem.color = {cam->getColor(), cam->getIntensity()};
  }




  fun memory::find_memory_type(u32 type_filter, vk::MemoryPropertyFlags properties) -> u32
  {
    auto mem_props = m_device.phys_dev().getMemoryProperties();

    for (u32 i{}; i < mem_props.memoryTypeCount; i++)
      if ((type_filter & (1 << i)) && (mem_props.memoryTypes[i].propertyFlags & properties) == properties) {
        return i;
      }
    
    throw std::runtime_error("Uygun GPU bellek türü bulunamadı!");
  }



  fun memory::load_UMA_UniformBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count, 
      vk::BufferUsageFlagBits::eUniformBuffer,
      vk::MemoryPropertyFlagBits::eDeviceLocal | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    );

    if (data) data(ret->mapped());

    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }

  fun memory::load_DISC_UniformBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count, 
      vk::BufferUsageFlagBits::eUniformBuffer,
      vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    );

    if (data) data(ret->mapped());

    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }
  


  fun memory::load_UMA_StorageBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count,
      vk::BufferUsageFlagBits::eStorageBuffer,
      vk::MemoryPropertyFlagBits::eDeviceLocal | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    );

    if (data) data(ret->mapped());

    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }

  fun memory::load_DISC_StorageBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count,
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst, 
      vk::MemoryPropertyFlagBits::eDeviceLocal
    );


    if (data) {
      // Host Buffer
      auto host_buffer = rhi::buffer(
        m_device, *this, info, count,
        vk::BufferUsageFlagBits::eTransferSrc, 
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
      );
      data(host_buffer.mapped());


      // Transfer
      vk::CommandBufferAllocateInfo alloc_info(*m_renderer.cmd_pool(), vk::CommandBufferLevel::ePrimary, 1);
      vk::raii::CommandBuffer temp_cmd = std::move(vk::raii::CommandBuffers(m_device.vdevice(), alloc_info).front());
      vk::BufferCopy copy_region(0, 0, ret->size());

      temp_cmd.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
      temp_cmd.copyBuffer(*host_buffer.get(), ret->get(), copy_region);
      temp_cmd.end();

      vk::SubmitInfo submit_info({}, {}, *temp_cmd, {});
      m_device.transfer_q().best().submit(submit_info, nil);
      m_device.transfer_q().best().waitIdle();
    }
    
    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }



  fun memory::load_UMA_VertexBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count,
      vk::BufferUsageFlagBits::eVertexBuffer,
      vk::MemoryPropertyFlagBits::eDeviceLocal | vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    );

    if (data) data(ret->mapped());

    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }

  fun memory::load_DISC_VertexBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*
  {
    auto ret = new rhi::buffer(
      m_device, *this, info, count,
      vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eTransferDst, 
      vk::MemoryPropertyFlagBits::eDeviceLocal
    );


    if (data) {
      // Host Buffer
      auto host_buffer = rhi::buffer(
        m_device, *this, info, count,
        vk::BufferUsageFlagBits::eTransferSrc, 
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
      );
      data(host_buffer.mapped());


      // Transfer
      vk::CommandBufferAllocateInfo alloc_info(*m_renderer.cmd_pool(), vk::CommandBufferLevel::ePrimary, 1);
      vk::raii::CommandBuffer temp_cmd = std::move(vk::raii::CommandBuffers(m_device.vdevice(), alloc_info).front());
      vk::BufferCopy copy_region(0, 0, ret->size());
      vk::MemoryBarrier memory_barrier(
        vk::AccessFlagBits::eTransferWrite,
        vk::AccessFlagBits::eVertexAttributeRead
      );

      temp_cmd.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
      temp_cmd.copyBuffer(*host_buffer.get(), ret->get(), copy_region);

      temp_cmd.pipelineBarrier(
        vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eVertexInput,
        {},
        {memory_barrier},
        {},
        {}
      );
      
      temp_cmd.end();

      vk::SubmitInfo submit_info({}, {}, *temp_cmd, {});
      m_device.transfer_q().best().submit(submit_info, nil);
      m_device.transfer_q().best().waitIdle();
    }
    
    std::get<std::vector<rhi::buffer*>>(m_owned).push_back(ret);
    return ret;
  }

}
