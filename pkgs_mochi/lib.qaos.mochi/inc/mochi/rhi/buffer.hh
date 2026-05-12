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
#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>



namespace mochi::rhi
{

  /** @brief Specialization of the info template for Vulkan buffers. */
  template<>
  struct info<buffer>
  {
    public:
      /**
       * @brief Initialize buffer info with a stride and item types.
       * @param stride The size of a single element/vertex in bytes.
       * @param items A list of Vulkan types describing the buffer's layout.
       */
      explicit inline info<buffer>(u64 stride, std::vector<mochi::vt> items)
        : m_stride(stride)
        , m_items(items)
      {}


    private:
      std::vector<mochi::vt> m_items;
      u64 m_stride;

    public:
      /** @brief Get the list of Vulkan types describing the layout. */
      inline fun items() { return m_items; }
      /** @brief Get the stride (size in bytes) of a single element. */
      inline fun stride() { return m_stride; }
  };




  /** @brief Represents a Vulkan buffer with associated memory. */
  struct buffer
  {
    friend struct module::memory;

    private:
      /**
       * @brief Construct a new buffer. Typically called by the memory allocator.
       * @param device The logical device.
       * @param memory The memory allocator.
       * @param info Pointer to the buffer info structure.
       * @param count Number of elements in the buffer.
       * @param usage Buffer usage flags.
       * @param properties Memory property flags.
       */
      explicit buffer(module::device &device, module::memory &memory, info<buffer> *info, u64 count, vk::BufferUsageFlags usage, vk::MemoryPropertyFlags properties);

    public:
      /** @brief Destructor. Automatically cleans up resources. */
      ~buffer();


    private:
      info<buffer> *m_info{};
      vk::raii::Buffer m_buffer;
      vk::raii::DeviceMemory m_memory;
      vk::DeviceSize m_size;
      void* m_mapped{};

    public:
      /** @brief Get the buffer info structure. */
      inline fun  info() { return m_info; }
      /** @brief Access the underlying Vulkan RAII buffer. */
      inline fun& get() { return m_buffer; }
      /** @brief Access the underlying Vulkan RAII device memory. */
      inline fun& memory() const { return m_memory; }
      /** @brief Get the total size of the buffer in bytes. */
      inline fun  size() const { return m_size; }
      /** @brief Get the mapped host memory pointer, if any. */
      inline fun  mapped() const { return m_mapped; }
  };

}
