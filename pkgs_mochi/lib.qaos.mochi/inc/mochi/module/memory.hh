/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/rhi/buffer.hh"
#include "mochi/rhi/pipeline.hh"
#include "mochi/world/visual.hh"
#include "mochi/module/device.hh"
#include "mochi/module/renderer.hh"
#include "mochi/types.hh"
#include <algorithm>
#include <cassert>
#include <functional>
#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  /** @brief Manages Vulkan memory allocation, device limits, and entity ownership. */
  struct memory
  {
    public:
      /**
       * @brief Initialize the memory allocator.
       * @param device The logical device.
       * @param renderer The renderer.
       */
      explicit memory(module::device &device, module::renderer &renderer);


    private:
      module::device &m_device;
      module::renderer &m_renderer;

      
    #pragma region Limits
    private:
      bool m_sharedMemory{};

      u32 m_pushConstant;
      u64 m_uniformRange;
      u32 m_uniformAlign;
      u64 m_storageRange;
      u32 m_storageAlign;
      u64 m_allocCount;
      u64 m_mapAlign;
      u64 m_vramSize;


    public:
      /** @brief Check if the device uses Unified Memory Architecture (UMA). */
      inline fun sharedMemory() { return m_sharedMemory; }

      /** @brief Get the maximum push constant size. */
      inline fun pushConstant() { return m_pushConstant; }
      /** @brief Get the uniform buffer range limit. */
      inline fun uniformRange() { return m_uniformRange; }
      /** @brief Get the uniform buffer alignment requirement. */
      inline fun uniformAlign() { return m_uniformAlign; }
      /** @brief Get the storage buffer range limit. */
      inline fun storageRange() { return m_storageRange; }
      /** @brief Get the storage buffer alignment requirement. */
      inline fun storageAlign() { return m_storageAlign; }
    #pragma endregion


    
    #pragma region OwnerShip
    private:
      std::tuple<
        std::vector<node*>,
        std::vector<camera*>,
        std::vector<light*>,
        std::vector<visual*>,

        std::vector<asset::mesh*>,
      
        std::vector<rhi::buffer*>,
        std::vector<rhi::pipeline*>
        
      > m_owned;

    public:
      /**
       * @brief Register an entity to be managed by this memory module.
       * @tparam T The entity type.
       * @param obj Pointer to the entity.
       */
      template <typename T>
      inline fun push(T *obj) {
        auto &vec = std::get<std::vector<T*>>(m_owned);

        assert(std::find(vec.begin(), vec.end(), obj) == vec.end() && "already owned");

        vec.push_back(obj);
      }

      /**
       * @brief Get the list of registered entities of a specific type.
       * @tparam T The entity type.
       * @return Reference to the vector of entity pointers.
       */
      template <typename T>
      inline fun& list() { return std::get<std::vector<T*>>(m_owned); }



    private: // Camera
      u64 m_camera_count{};
      rhi::buffer *m_camera_ubo{};
      fun prepare_camera(u32 count) -> void;
      
    public:
      /** @brief Find the index of a registered camera. */
      fun find_camera(camera *cam) -> u64;
      /** @brief Synchronize the camera data to its Uniform Buffer Object. */
      fun sync_camera(camera *cam) -> void;
      /** @brief Get the global camera Uniform Buffer Object. */
      inline fun camera_ubo() { return m_camera_ubo; }

      /** @brief Register a camera entity and update the UBO. */
      template <typename T>
        requires std::is_same_v<camera, T>
      inline fun push(T *obj) {
        auto &vec = std::get<std::vector<T*>>(m_owned);
        assert(std::find(vec.begin(), vec.end(), obj) == vec.end() && "already owned");
        vec.push_back(obj);

        prepare_camera(++m_camera_count);
        sync_camera(obj);
      }

      /** @brief Unregister a camera entity. */
      template <typename T>
        requires std::is_same_v<camera, T>
      inline fun pop(T *obj) {
        auto &vec = std::get<std::vector<T*>>(m_owned);
        auto it = std::find(vec.begin(), vec.end(), obj);
        assert(it != vec.end() && "not registered");
        vec.erase(it);

        prepare_camera(--m_camera_count);
      }


    private: // Light
      u64 m_light_count{};
      rhi::buffer *m_light_ubo{};
      fun prepare_light(u32 count) -> void;
      
    public:
      /** @brief Find the index of a registered light. */
      fun find_light(light *cam) -> u64;
      /** @brief Synchronize the light data to its Uniform Buffer Object. */
      fun sync_light(light *cam) -> void;
      /** @brief Get the global light Uniform Buffer Object. */
      inline fun light_ubo() { return m_light_ubo; }

      /** @brief Register a light entity and update the UBO. */
      template <typename T>
        requires std::is_same_v<light, T>
      inline fun push(T *obj) {
        auto &vec = std::get<std::vector<T*>>(m_owned);
        assert(std::find(vec.begin(), vec.end(), obj) == vec.end() && "already owned");
        vec.push_back(obj);

        prepare_light(++m_light_count);
        sync_light(obj);
      }

      /** @brief Unregister a light entity. */
      template <typename T>
        requires std::is_same_v<light, T>
      inline fun pop(T *obj) {
        auto &vec = std::get<std::vector<T*>>(m_owned);
        auto it = std::find(vec.begin(), vec.end(), obj);
        assert(it != vec.end() && "not registered");
        vec.erase(it);

        prepare_light(--m_light_count);
      }
    #pragma endregion



    #pragma region Allocation
    public:
      /**
       * @brief Find a suitable memory type index.
       * @param type_filter Bitmask of suitable memory types.
       * @param properties Required memory properties.
       * @return The index of the found memory type.
       */
      fun find_memory_type(u32 type_filter, vk::MemoryPropertyFlags properties) -> u32;

    private:
      fun load_UMA_UniformBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;
      fun load_UMA_StorageBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;
      fun load_UMA_VertexBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;

      fun load_DISC_UniformBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;
      fun load_DISC_StorageBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;
      fun load_DISC_VertexBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer*;

    public:
      /** @brief Allocate and optionally initialize a uniform buffer (handles UMA vs Discrete). */
      inline fun load_UniformBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer* { return m_sharedMemory ? load_UMA_UniformBuffer(info, count, data) : load_DISC_UniformBuffer(info, count, data); }
      /** @brief Allocate and optionally initialize a storage buffer (handles UMA vs Discrete). */
      inline fun load_StorageBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer* { return m_sharedMemory ? load_UMA_StorageBuffer(info, count, data) : load_DISC_StorageBuffer(info, count, data); }
      /** @brief Allocate and optionally initialize a vertex buffer (handles UMA vs Discrete). */
      inline fun load_VertexBuffer(rhi::info<rhi::buffer> *info, u64 count, std::function<void (void*)> data) -> rhi::buffer* { return m_sharedMemory ? load_UMA_VertexBuffer(info, count, data) : load_DISC_VertexBuffer(info, count, data); }
    #pragma endregion

  };

}
