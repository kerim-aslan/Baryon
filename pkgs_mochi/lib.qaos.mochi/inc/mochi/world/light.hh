/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include "mochi/world/node.hh"
#include "mochi/geometry.hh"
#include "mochi/types.hh"
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi
{

  /** @brief Represents the raw light data passed to shaders. */
  struct light_t
  {
    /** @brief X, Y, Z position of the light. */
    vec4<f32> position;
    /** @brief R, G, B color. W component represents Intensity. */
    vec4<f32> color;
  };
  
  /** @brief Buffer info describing the light_t memory layout. */
  extern rhi::info<rhi::buffer> light_i;


  
  
  /** @brief Represents a point light node in the scene. */
  struct light: node
  {
    public:
      /**
       * @brief Construct a new light.
       * @param core The mochi core instance.
       * @param parent The parent node in the hierarchy.
       * @param model Initial model transform matrix.
       * @param color The RGB color of the light.
       * @param intensity The brightness/intensity of the light.
       */
      explicit light(core &core, node *parent, mat4<f32> model, vec3<f32> color, f32 intensity);
      
      /** @brief Destructor. */
      ~light();
      

    private:
      module::memory &m_memory;

      vec3<f32> m_color;
      f32 m_intensity;

    public:
      /** @brief Get the color of the light. */
      inline fun getColor() { return m_color; }
      /** @brief Get the intensity of the light. */
      inline fun getIntensity() { return m_intensity; }

  };

}
