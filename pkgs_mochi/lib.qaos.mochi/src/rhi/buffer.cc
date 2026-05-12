/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/rhi/buffer.hh"
#include "mochi/module/device.hh"
#include "mochi/module/memory.hh"
#include "vulkan/vulkan.hpp"



namespace mochi::rhi
{

  buffer::buffer(module::device &device, module::memory &memory, rhi::info<buffer> *info, u64 count, vk::BufferUsageFlags usage, vk::MemoryPropertyFlags properties)
    : m_info(info)
    , m_buffer(nil)
    , m_memory(nil)
    , m_size(info->stride() * count)
  {
    // Create Buffer
    vk::BufferCreateInfo buffer_info(
      {},
      m_size,
      usage,
      vk::SharingMode::eExclusive
    );
    m_buffer = vk::raii::Buffer(device.vdevice(), buffer_info);

    vk::MemoryRequirements mem_reqs = m_buffer.getMemoryRequirements();


    // Find Suitable Memory
    vk::MemoryAllocateInfo alloc_info(
      mem_reqs.size,
      memory.find_memory_type(mem_reqs.memoryTypeBits, properties)
    );
    m_memory = vk::raii::DeviceMemory(device.vdevice(), alloc_info);


    // Bind Buffer
    m_buffer.bindMemory(*m_memory, 0);

    // Maping
    m_mapped = m_memory.mapMemory(0, m_size, {});
  }


  buffer::~buffer()
  {
    m_memory.unmapMemory();
  }

}
