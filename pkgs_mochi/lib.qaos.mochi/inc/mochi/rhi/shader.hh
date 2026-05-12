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
#include <string_view>
#include <vulkan/vulkan_raii.hpp>



namespace mochi::rhi
{

  /** @brief Represents a Vulkan shader module and its entry point. */
  struct shader
  {
    public:
      /** 
       * @brief Construct a new shader from a file path.
       * @param core The mochi core instance.
       * @param fpath The file path to the shader bytecode (e.g., SPIR-V).
       * @param entry The entry point name of the shader.
       */
      explicit shader(mochi::core &core, std::string_view fpath, std::string_view entry);
      

    private:
      vk::raii::ShaderModule vk_module;
      std::string m_entry;

    public:
      /** @brief Access the underlying Vulkan shader module. */
      inline fun& module() { return vk_module; }
      /** @brief Access the shader entry point string. */
      inline fun& entry() { return m_entry; }
  };

}
