/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once


#include "Basis.hh"
#include "vulkan/vulkan.hpp"
#include <string_view>
#include <type_traits>

#define ef else if



namespace mochi
{

  /** @brief Core instance. */
  struct core;
  

  namespace module
  {
    /** @brief Bridge for graphics operations. */
    struct bridge;
    /** @brief Window instance. */
    struct window;
    /** @brief Logical device instance. */
    struct device;
    /** @brief Swapchain instance. */
    struct swapchain;
    /** @brief Renderer instance. */
    struct renderer;
    /** @brief Memory allocator. */
    struct memory;
  }



  namespace rhi
  {
    /** @brief Graphics pipeline instance. */
    struct pipeline;
    /** @brief Data buffer instance. */
    struct buffer;
    /** @brief Shader module instance. */
    struct shader;

    /** @brief Information structure for a type. */
    template <typename T>
    struct info;
  }


  /** @brief Scene node. */
  struct node;
  /** @brief Camera node. */
  struct camera;
  /** @brief Light node. */
  struct light;



  namespace asset
  {
    /** @brief 3D Mesh object. */
    struct mesh;
  }




  /// Geometry
  
  /** @brief 2D Vector structure. */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec2;

  /** @brief 3D Vector structure. */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec3;

  /** @brief 4D Vector structure. */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec4;

  /** @brief Quaternion structure. */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct quaternion;

  /** @brief 4x4 Matrix structure. */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct mat4;




  /// Vulkan Types
  
  /** 
   * @brief Helper structure for mapping C++ types to Vulkan formats. 
   */
  struct vt
  {
    private:
      /** 
       * @brief Construct a new vt object.
       * @param size Size in bytes.
       * @param align Alignment in bytes.
       * @param format Vulkan format.
       * @param count Vulkan sub count (default is 1).
       */
      vt(u64 size, u64 align, vk::Format format, u8 count = 1)
        : m_size(size)
        , m_align(align)
        , m_format(format)
        , m_count(count)
      {}


    public:
      /**
       * @brief Create a vt instance mapped to the specified type T.
       * @tparam T The C++ type to map.
       * @return vt The mapped Vulkan type info.
       */
      template <typename T>
      static inline fun make() -> vt {
        // primitive
        if constexpr (std::is_same_v<T, u8>)  return {1, 1, vk::Format::eR8Uint};
        ef constexpr (std::is_same_v<T, u16>) return {2, 2, vk::Format::eR16Uint};
        ef constexpr (std::is_same_v<T, u32>) return {4, 4, vk::Format::eR32Uint};
        ef constexpr (std::is_same_v<T, u64>) return {8, 8, vk::Format::eR64Uint};

        ef constexpr (std::is_same_v<T, i8>)  return {1, 1, vk::Format::eR8Sint};
        ef constexpr (std::is_same_v<T, i16>) return {2, 2, vk::Format::eR16Sint};
        ef constexpr (std::is_same_v<T, i32>) return {4, 4, vk::Format::eR32Sint};
        ef constexpr (std::is_same_v<T, i64>) return {8, 8, vk::Format::eR64Sint};

        ef constexpr (std::is_same_v<T, nu8>)  return {1, 1, vk::Format::eR8Unorm};
        ef constexpr (std::is_same_v<T, nu16>) return {2, 2, vk::Format::eR16Unorm};

        ef constexpr (std::is_same_v<T, ni8>)  return {1, 1, vk::Format::eR8Snorm};
        ef constexpr (std::is_same_v<T, ni16>) return {2, 2, vk::Format::eR16Snorm};

        ef constexpr (std::is_same_v<T, f16>) return {2, 2, vk::Format::eR16Sfloat};
        ef constexpr (std::is_same_v<T, f32>) return {4, 4, vk::Format::eR32Sfloat};
        ef constexpr (std::is_same_v<T, f64>) return {8, 8, vk::Format::eR64Sfloat};


        // vec2
        ef constexpr (std::is_same_v<T, vec2<u8>>)  return {2,   2,  vk::Format::eR8G8Uint};
        ef constexpr (std::is_same_v<T, vec2<u16>>) return {4,   4,  vk::Format::eR16G16Uint};
        ef constexpr (std::is_same_v<T, vec2<u32>>) return {8,   8,  vk::Format::eR32G32Uint};
        ef constexpr (std::is_same_v<T, vec2<u64>>) return {16,  16, vk::Format::eR64G64Uint};

        ef constexpr (std::is_same_v<T, vec2<i8>>)  return {2,   2,  vk::Format::eR8G8Sint};
        ef constexpr (std::is_same_v<T, vec2<i16>>) return {4,   4,  vk::Format::eR16G16Sint};
        ef constexpr (std::is_same_v<T, vec2<i32>>) return {8,   8,  vk::Format::eR32G32Sint};
        ef constexpr (std::is_same_v<T, vec2<i64>>) return {16,  16, vk::Format::eR64G64Sint};

        ef constexpr (std::is_same_v<T, vec2<nu8>>)  return {2,   2,  vk::Format::eR8G8Unorm};
        ef constexpr (std::is_same_v<T, vec2<nu16>>) return {4,   4,  vk::Format::eR16G16Unorm};

        ef constexpr (std::is_same_v<T, vec2<ni8>>)  return {2,   2,  vk::Format::eR8G8Snorm};
        ef constexpr (std::is_same_v<T, vec2<ni16>>) return {4,   4,  vk::Format::eR16G16Snorm};

        ef constexpr (std::is_same_v<T, vec2<f16>>) return {4,   4,  vk::Format::eR16G16Sfloat};
        ef constexpr (std::is_same_v<T, vec2<f32>>) return {8,   8,  vk::Format::eR32G32Sfloat};
        ef constexpr (std::is_same_v<T, vec2<f64>>) return {16,  16, vk::Format::eR64G64Sfloat};


        // vec3
        ef constexpr (std::is_same_v<T, vec3<u8>>)  return {3,   3,  vk::Format::eR8G8B8Uint};
        ef constexpr (std::is_same_v<T, vec3<u16>>) return {6,   6,  vk::Format::eR16G16B16Uint};
        ef constexpr (std::is_same_v<T, vec3<u32>>) return {12,  12, vk::Format::eR32G32B32Uint};
        ef constexpr (std::is_same_v<T, vec3<u64>>) return {24,  24, vk::Format::eR64G64B64Uint};

        ef constexpr (std::is_same_v<T, vec3<i8>>)  return {3,   3,  vk::Format::eR8G8B8Sint};
        ef constexpr (std::is_same_v<T, vec3<i16>>) return {6,   6,  vk::Format::eR16G16B16Sint};
        ef constexpr (std::is_same_v<T, vec3<i32>>) return {12,  12, vk::Format::eR32G32B32Sint};
        ef constexpr (std::is_same_v<T, vec3<i64>>) return {24,  24, vk::Format::eR64G64B64Sint};
        
        ef constexpr (std::is_same_v<T, vec3<nu8>>)  return {3,   3,  vk::Format::eR8G8B8Unorm};
        ef constexpr (std::is_same_v<T, vec3<nu16>>) return {6,   6,  vk::Format::eR16G16B16Unorm};

        ef constexpr (std::is_same_v<T, vec3<ni8>>)  return {3,   3,  vk::Format::eR8G8B8Snorm};
        ef constexpr (std::is_same_v<T, vec3<ni16>>) return {6,   6,  vk::Format::eR16G16B16Snorm};

        ef constexpr (std::is_same_v<T, vec3<f16>>) return {6,   6,  vk::Format::eR16G16B16Sfloat};
        ef constexpr (std::is_same_v<T, vec3<f32>>) return {12,  12, vk::Format::eR32G32B32Sfloat};
        ef constexpr (std::is_same_v<T, vec3<f64>>) return {24,  24, vk::Format::eR64G64B64Sfloat};


        // vec4
        ef constexpr (std::is_same_v<T, vec4<u8>>)  return {4,   4,  vk::Format::eR8G8B8A8Uint};
        ef constexpr (std::is_same_v<T, vec4<u16>>) return {8,   8,  vk::Format::eR16G16B16A16Uint};
        ef constexpr (std::is_same_v<T, vec4<u32>>) return {16,  16, vk::Format::eR32G32B32A32Uint};
        ef constexpr (std::is_same_v<T, vec4<u64>>) return {32,  32, vk::Format::eR64G64B64A64Uint};

        ef constexpr (std::is_same_v<T, vec4<i8>>)  return {4,   4,  vk::Format::eR8G8B8A8Sint};
        ef constexpr (std::is_same_v<T, vec4<i16>>) return {8,   8,  vk::Format::eR16G16B16A16Sint};
        ef constexpr (std::is_same_v<T, vec4<i32>>) return {16,  16, vk::Format::eR32G32B32A32Sint};
        ef constexpr (std::is_same_v<T, vec4<i64>>) return {32,  32, vk::Format::eR64G64B64A64Sint};

        ef constexpr (std::is_same_v<T, vec4<nu8>>)  return {4,   4,  vk::Format::eR8G8B8A8Unorm};
        ef constexpr (std::is_same_v<T, vec4<nu16>>) return {8,   8,  vk::Format::eR16G16B16A16Unorm};

        ef constexpr (std::is_same_v<T, vec4<ni8>>)  return {4,   4,  vk::Format::eR8G8B8A8Snorm};
        ef constexpr (std::is_same_v<T, vec4<ni16>>) return {8,   8,  vk::Format::eR16G16B16A16Snorm};

        ef constexpr (std::is_same_v<T, vec4<f16>>) return {8,   8,  vk::Format::eR16G16B16A16Sfloat};
        ef constexpr (std::is_same_v<T, vec4<f32>>) return {16,  16, vk::Format::eR32G32B32A32Sfloat};
        ef constexpr (std::is_same_v<T, vec4<f64>>) return {32,  32, vk::Format::eR64G64B64A64Sfloat};


        // mat4
        ef constexpr (std::is_same_v<T, mat4<f32>>) return {16, 16, vk::Format::eR32G32B32A32Sfloat, 4};
        ef constexpr (std::is_same_v<T, mat4<f64>>) return {32, 32, vk::Format::eR64G64B64A64Sfloat, 4};
        else
          static_assert(false, "unsupported type");
      }


      /**
       * @brief Create a list of vt instances for the given types.
       * @tparam Ts The C++ types to map.
       * @return std::vector<vt> The list of mapped Vulkan type infos.
       */
      template <typename... Ts>
      static inline fun make_list() -> std::vector<vt> {
        return { vt::make<Ts>()... };
      }


    private:
      u64 m_size, m_align;
      vk::Format m_format;
      u8 m_count;

    public:
      /** @brief Get the size in bytes. */
      inline fun size() { return m_size; }
      /** @brief Get the alignment in bytes. */
      inline fun align() { return m_align; }
      /** @brief Get the Vulkan format. */
      inline fun format() { return m_format; }
      /** @brief Get the Vulkan sub count. */
      inline fun count() { return m_count; }
  };




  /// Mapped Memory
  
  /** 
   * @brief Helper structure for memory-mapped files. 
   */
  struct mappedFile
  {
    public:
      /** 
       * @brief Construct a new mappedFile object.
       * @param fpath The path to the file to map.
       */
      explicit mappedFile(std::string fpath);

      /** @brief Destroy the mappedFile object and unmap memory. */
      ~mappedFile();

    public:
      /**
       * @brief Get a string view of the mapped file data.
       * @return std::string_view View of the file data.
       */
      inline fun view() const -> std::string_view {
        return {static_cast<const char*>(data), size};
      }

    private:
      #if defined(__unix__) || defined(__APPLE__)
      int fd{-1};
      void *data{(void*)(-1)};
      #elif defined(_WIN32)
      void* hFile;
      void* hMapping;
      void *data{};
      #endif
      
      u0 size{};

    public:
      inline operator ::data () { return {data, size}; }

  };

}
