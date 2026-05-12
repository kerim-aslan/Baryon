/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "mochi/types.hh"
#include "Basis.hh"
#include <string_view>
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>



namespace mochi::module
{

  /** @brief Represents a system window and its Vulkan surface. */
  struct window
  {
    public:
      /**
       * @brief Initialize a new window.
       * @param bridge The Vulkan bridge instance.
       * @param title The title of the window.
       * @param width The initial width of the window.
       * @param height The initial height of the window.
       */
      explicit window(module::bridge &bridge, std::string_view title, int width, int height);


    private:
      GLFWwindow *m_window{};
      vk::raii::SurfaceKHR vk_surface;

      i32 m_width{}, m_height{};
      bool m_resized{};


    public:
      /** @brief Access the underlying GLFW window pointer. */
      inline fun  glfw() { return m_window; }
      /** @brief Access the Vulkan RAII surface. */
      inline fun& surface() { return vk_surface; }
      /** @brief Get the current width of the window. */
      inline fun  width()  { return m_width; }
      /** @brief Get the current height of the window. */
      inline fun  height() { return m_height; }

      
    public:
      /**
       * @brief Get the required Vulkan extensions for GLFW.
       * @return A vector of extension names.
       */
      static inline fun extensions() -> std::vector<const char*> {
        u32 glfwExtensionCount{};
        const char **glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        return std::vector<const char*>(glfwExtensions, glfwExtensions + glfwExtensionCount);
      }


    private:
      /**
       * @brief Callback for window resize events.
       * @param win The GLFW window.
       * @param width New width.
       * @param height New height.
       */
      static void framebuffer_resize_callback(GLFWwindow* win, int width, int height);


    public:
      /**
       * @brief Process window events.
       * @return True if the window should stay open, false if it should close.
       */
      fun proc_events() -> bool;

      /** @brief Check if window was resized. */
      inline fun& resized() { return m_resized; }

  };

}
