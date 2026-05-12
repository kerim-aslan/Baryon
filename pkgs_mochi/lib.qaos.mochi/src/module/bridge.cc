/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/module/bridge.hh"
#include "mochi/module/window.hh"
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  bridge::bridge(std::string_view appName, std::array<u32, 4> appVer)
    : vk_ctx(), vk_inst(nil)
  {
    // AppInfo
    vk::ApplicationInfo appInfo {
      appName.data(),
      VK_MAKE_API_VERSION(appVer[0], appVer[1], appVer[2], appVer[3]),
      "Re-Engine",
      VK_MAKE_API_VERSION(0, 1, 0, 0),
      vk::ApiVersion14
    };

    // Layers & Extensions
    auto layers = std::vector<const char*>{  /*"VK_LAYER_KHRONOS_validation"*/  };
    auto extensions = window::extensions();

    // Create Instance
    vk::InstanceCreateInfo createInfo({}, &appInfo, layers, extensions);
    vk_inst = vk::raii::Instance(ctx(), createInfo);
  }

}
