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
#include "mochi/types.hh"
#include <algorithm>
#include <stdexcept>
#include <vulkan/vulkan_raii.hpp>
#include <vulkan/vulkan.h>



namespace mochi
{
  
  /** @brief Represents a node in the scene graph with hierarchical transform capabilities. */
  struct node
  {
    public:
      /**
       * @brief Construct a new node.
       * @param parent Pointer to the parent node, or nil if this is a root node.
       * @param model Initial local transformation matrix.
       */
      explicit node(node *parent, mat4<f32> model);
      
      /** @brief Virtual destructor for proper inheritance cleanup. */
      virtual inline ~node() {}
      

    private:
      mat4<f32> m_model;
      std::vector<node*> m_childs;
      node *m_parent{};


    public:
      /** @brief Get the local model transformation matrix. */
      inline fun getModel() { return m_model; }
      
      /** @brief Set the local model transformation matrix. */
      virtual inline fun setModel(mat4<f32> val) -> void { m_model = val; }

      /**
       * @brief Get the absolute world position of this node.
       * @return The 3D world position vector.
       */
      inline fun getWorldPos() -> vec3<f32> {
        auto mat = getModel();
        return vec3<f32>(mat.SwVec[0][3], mat.SwVec[1][3], mat.SwVec[2][3]);
      }

      /** @brief Access the list of child nodes. */
      inline fun& childs() { return m_childs; }

      /** @brief Get the parent node. */
      inline fun getParent() { return m_parent; }
      
      /**
       * @brief Set or change the parent node.
       * Automatically updates the parent-child relationship.
       * @param val Pointer to the new parent node.
       * @throws std::runtime_error If hierarchy corruption is detected.
       */
      inline fun setParent(node *val) {
        if (m_parent) {
          auto it = std::find(m_parent->childs().begin(), m_parent->childs().end(), this);

          if (it == m_parent->childs().end())
            throw std::runtime_error("Hierarchy corruption.");

          m_parent->childs().erase(it);
          m_parent = nil;
        }

        if (val && !m_parent) {
          m_parent = val;
          m_parent->m_childs.push_back(this);
        }
      }

  };

}
