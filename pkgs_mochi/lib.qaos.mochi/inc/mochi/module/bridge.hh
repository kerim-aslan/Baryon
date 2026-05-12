/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include <string_view>
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi::module
{

  /** @brief Manages the Vulkan context and instance. */
  struct bridge
  {
    public:
      /**
       * @brief Initialize the Vulkan bridge.
       * @param appName The name of the application.
       * @param appVer The version of the application (Major, Minor, Patch, Tweak).
       */
      explicit bridge(std::string_view appName, std::array<u32, 4> appVer);


    private:
      vk::raii::Context vk_ctx;
      vk::raii::Instance vk_inst;

    public:
      /** @brief Access the Vulkan RAII context. */
      inline fun& ctx() { return vk_ctx; }
      /** @brief Access the Vulkan RAII instance. */
      inline fun& inst() { return vk_inst; }


    public:
      /**
       * @brief Get a list of available physical devices.
       * @return vk::raii::PhysicalDevices collection of available GPUs.
       */
      inline fun physicalDevices() -> vk::raii::PhysicalDevices {
        return vk::raii::PhysicalDevices(inst());
      }

  };

}
