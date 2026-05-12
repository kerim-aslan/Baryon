/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include "mochi/geometry.hh"
#include "mochi/world/node.hh"
#include "mochi/types.hh"
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi
{

  /** @brief Represents the raw camera data passed to shaders (View and Projection matrices). */
  struct camera_t {
    mat4<f32> view;
    mat4<f32> proj;
  };
  
  /** @brief Buffer info describing the camera_t memory layout. */
  extern rhi::info<rhi::buffer> camera_i;
  

  

  /** @brief Represents a camera node in the scene, extending the base transform node. */
  struct camera: node
  {
    public:
      /**
       * @brief Construct a new camera.
       * @param core The mochi core instance.
       * @param parent The parent node in the hierarchy.
       * @param model Initial model transform matrix.
       * @param fov Field of view in degrees.
       * @param near Near clipping plane distance.
       * @param far Far clipping plane distance.
       */
      explicit camera(core &core, node *parent, mat4<f32> model, f32 fov, f32 near, f32 far);
      
      /** @brief Destructor. */
      ~camera();


    private:
      module::memory &m_memory;
      module::window &m_window;

      f32 m_fov, m_near, m_far;
      mat4<f32> m_view, m_proj;

    public:
      /** @brief Access the field of view (FOV). */
      inline fun& fov() { return m_fov; }
      /** @brief Access the near clipping plane distance. */
      inline fun& near() { return m_near; }
      /** @brief Access the far clipping plane distance. */
      inline fun& far() { return m_far; }
      
      /** @brief Get the computed view matrix. */
      inline fun view() { return m_view; }
      /** @brief Get the computed projection matrix. */
      inline fun proj() { return m_proj; }
      

    public:
      /** @brief Recalculate the view and projection matrices based on current transform and window size. */
      fun recalc() -> void;

      /**
       * @brief Override the base model transform setter to trigger matrix recalculation.
       * @param val The new model matrix.
       */
      inline fun setModel(mat4<f32> val) -> void override {
        node::setModel(val);
        recalc();
      }

  };

}
