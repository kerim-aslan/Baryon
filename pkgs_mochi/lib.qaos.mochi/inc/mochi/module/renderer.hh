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
#include "mochi/module/device.hh"
#include "mochi/module/swapchain.hh"
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  /** @brief Manages the rendering loop, command buffers, and frame synchronization. */
  struct renderer
  {
    public:
      /** @brief Maximum number of frames that can be processed concurrently. */
      static constexpr u32 MAX_FRAMES_IN_FLIGHT = 2;

    public:
      /**
       * @brief Initialize a new renderer.
       * @param device The logical device.
       * @param swapchain The swapchain to render to.
       */
      explicit renderer(module::device &device, module::window &window, module::swapchain &swapchain);


    private:
      module::device &m_device;
      module::window &m_window;
      module::swapchain &m_swapchain;

      vk::raii::CommandPool                m_cmd_pool;
      std::vector<vk::raii::CommandBuffer> m_cmd_buffers;

      // Synchronization Objects (per frame)
      std::vector<vk::raii::Semaphore> m_image_available_sems;
      std::vector<vk::raii::Semaphore> m_render_finished_sems;
      std::vector<vk::raii::Fence>     m_in_flight_fences;

      u32 m_current_frame{}; // Current frame in flight (e.g., 0 or 1)
      u32 m_image_index{};   // Index of the image acquired from the swapchain

    public:
      /** @brief Access the Vulkan RAII command pool. */
      inline fun& cmd_pool() { return m_cmd_pool; }
      /** @brief Get the index of the currently acquired swapchain image. */
      inline fun image_index() const { return m_image_index; }
      /** @brief Get the current frame in flight index. */
      inline fun current_frame() const { return m_current_frame; }

    public:
      /**
       * @brief Begin a rendering pass on the swapchain image.
       * @param cmd The command buffer to record into.
       * @param clear_color The color used to clear the screen (RGBA).
       */
      fun begin_swapchain_rendering(vk::raii::CommandBuffer &cmd, const std::array<float, 4> &clear_color) -> void;
      
      /**
       * @brief End the rendering pass on the swapchain image.
       * @param cmd The command buffer being recorded.
       */
      fun end_swapchain_rendering(vk::raii::CommandBuffer &cmd) -> void;


      /**
       * @brief Start a new frame, acquiring an image and setting up synchronization.
       * @return Reference to the active command buffer for the frame.
       */
      fun begin_frame() -> vk::raii::CommandBuffer&;
      
      /**
       * @brief Submit the command buffer and present the frame.
       * @param cmd The active command buffer to submit.
       */
      fun end_frame(vk::raii::CommandBuffer &cmd) -> void;
  };
  
}
