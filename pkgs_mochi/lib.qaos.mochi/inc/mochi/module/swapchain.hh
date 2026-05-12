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
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi::module
{

  /** @brief Manages the Vulkan swapchain and its associated resources. */
  struct swapchain
  {
    public:
      /**
       * @brief Initialize a new swapchain.
       * @param device The logical device instance.
       * @param window The window instance to attach the swapchain to.
       */
      explicit swapchain(module::device &device, module::window &window);


    private:
      module::device &m_device;
      module::window &m_window;
      
      vk::raii::SwapchainKHR m_swapchain{nil};
      std::vector<vk::Image> m_images;
      std::vector<vk::raii::ImageView> m_image_views;
      vk::Format m_format;
      vk::Extent2D m_extent;
      u32 m_image_count;
      vk::raii::Image m_depth_image{nil};
      vk::raii::DeviceMemory m_depth_memory{nil};
      vk::raii::ImageView m_depth_view{nil};
      vk::Format m_depth_format{vk::Format::eD32Sfloat};

    public:
      /** @brief Rebuild Swapchain based on current window sizes. */
      fun recreate() -> void;

      /** @brief Access the underlying Vulkan RAII swapchain. */
      inline fun& get() { return m_swapchain; }
      /** @brief Get the list of swapchain images. */
      inline fun& images() { return m_images; }
      /** @brief Get the list of swapchain image views. */
      inline fun& image_views() { return m_image_views; }
      /** @brief Get the image format used by the swapchain. */
      inline fun& format() { return m_format; }
      /** @brief Get the 2D extent (dimensions) of the swapchain images. */
      inline fun  extent() const { return m_extent; }
      /** @brief Get the number of images in the swapchain. */
      inline fun  image_count() { return m_image_count; }
      /** @brief Access the depth buffer image. */
      inline fun& depth_image() { return m_depth_image; }
      /** @brief Access the depth buffer image view. */
      inline fun& depth_view() { return m_depth_view; }
      /** @brief Get the format of the depth buffer. */
      inline fun  depth_format() const { return m_depth_format; }

  };

}
