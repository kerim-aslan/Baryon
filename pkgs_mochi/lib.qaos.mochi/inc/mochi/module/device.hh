/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi::module
{

  /** @brief Represents a group of Vulkan queues for a specific capability. */
  struct queue_group
  {
    friend struct device;
    
    private:
      std::vector<vk::raii::Queue> m_primary;   // Dedicated (specialized) queues
      std::vector<vk::raii::Queue> m_secondary; // Shared queues

    public:
      /** @brief Check if any queue is available in this group. */
      inline fun available() const { return !m_primary.empty() || !m_secondary.empty(); }
    
      /** @brief Get the best available single queue (prefers dedicated). */
      inline fun& best() const {
        return !m_primary.empty() ? m_primary.front() : m_secondary.front();
      }

      /** @brief Get the best available list of queues (prefers dedicated). */
      inline fun& bests() const {
        return !m_primary.empty() ? m_primary : m_secondary;
      }

  };


  /** @brief Represents a Vulkan logical device and its associated resources. */
  struct device
  {
    public:
      /**
       * @brief Initialize a logical device from a physical device.
       * @param phys_dev The Vulkan physical device to create the logical device from.
       */
      explicit device(vk::raii::PhysicalDevice phys_dev);


    private:
      vk::raii::PhysicalDevice vk_phys_dev;
      vk::raii::Device vk_device;
      queue_group  m_graphics_q, m_compute_q, m_transfer_q;

      struct { u32 graphics, compute, transfer; } vk_indices;


    public:
      /** @brief Access the Vulkan RAII physical device. */
      inline fun& phys_dev() { return vk_phys_dev; }
      /** @brief Access the Vulkan RAII logical device. */
      inline fun& vdevice() { return vk_device; }
      /** @brief Access the graphics queue group. */
      inline fun& graphics_q() { return m_graphics_q; }
      /** @brief Access the compute queue group. */
      inline fun& compute_q() { return m_compute_q; }
      /** @brief Access the transfer queue group. */
      inline fun& transfer_q() { return m_transfer_q; }
  };

}
