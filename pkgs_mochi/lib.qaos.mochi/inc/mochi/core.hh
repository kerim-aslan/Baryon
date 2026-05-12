/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include "mochi/module/bridge.hh"
#include "mochi/module/window.hh"
#include "mochi/module/device.hh"
#include "mochi/module/memory.hh"
#include "mochi/module/swapchain.hh"
#include "mochi/module/renderer.hh"
#include <cassert>
#include <functional>
#include <vulkan/vulkan_raii.hpp>



namespace mochi
{

  /** @brief The central class that initializes and manages all mochi subsystems. */
  struct core
  {
    public:
      /**
       * @brief Construct the core mochi engine instance.
       * @param GpuPicker A callback to select the preferred Vulkan physical device.
       * @param Idle A callback executed every frame, typically used for game logic updates.
       */
      explicit core(
        std::function<vk::raii::PhysicalDevice (vk::raii::PhysicalDevices)> GpuPicker,
        std::function<void (f32 dt)> Idle
      );

      /** @brief Destructor to safely terminate the engine and wait for device idle. */
      ~core();



    // Properties
    private:
      camera *m_camera{};
      node *m_scene{};

    public:
      /** @brief Access the active camera pointer. */
      inline fun& camera() { return m_camera; }
      /** @brief Access the root scene node pointer. */
      inline fun& scene() { return m_scene; }
      


    // Sub Module
    private:
      module::bridge    m_bridge;
      module::window    m_window;
      module::device    m_device;
      module::memory    m_memory;
      module::swapchain m_swapchain;
      module::renderer  m_renderer;

      std::function<void (f32 dt)> m_idle;


    public:
      /**
       * @brief Get a reference to a specific sub-module.
       * @tparam T The requested sub-module type.
       * @return Reference to the sub-module.
       */
      template <typename T>
        requires std::is_same_v<T, module::window>
      inline fun& sub() { return m_window; }

      template <typename T>
        requires std::is_same_v<T, module::device>
      inline fun& sub() { return m_device; }

      template <typename T>
        requires std::is_same_v<T, module::memory>
      inline fun& sub() { return m_memory; }

      template <typename T>
        requires std::is_same_v<T, module::swapchain>
      inline fun& sub() { return m_swapchain; }


    // Functions
    public:
      /** @brief Start the main application loop. */
      fun run() -> void;

    private:
      /** @brief Internal method to process and draw a single frame. */
      fun draw() -> void;
  };

}
