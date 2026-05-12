/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <pthread.h>
#include <stdexcept>
#include <type_traits>
#include <vector>



/// Modern C++
#define fun auto




/// Int

/** @brief 8-bit signed integer. */
using i8   = int8_t;
/** @brief 16-bit signed integer. */
using i16  = int16_t;
/** @brief 32-bit signed integer. */
using i32  = int32_t;
/** @brief 64-bit signed integer. */
using i64  = int64_t;
/** @brief 128-bit signed integer. */
using i128 = signed __int128;

/** @brief Architecture-dependent signed integer. */
#if INTPTR_MAX == INT64_MAX
  using i0 = i64;
#else
  using i0 = i32;
#endif




/// UInt

/** @brief 8-bit unsigned integer. */
using u8   = uint8_t;
/** @brief 16-bit unsigned integer. */
using u16  = uint16_t;
/** @brief 32-bit unsigned integer. */
using u32  = uint32_t;
/** @brief 64-bit unsigned integer. */
using u64  = uint64_t;
/** @brief 128-bit unsigned integer. */
using u128 = unsigned __int128;

/** @brief Architecture-dependent unsigned integer. */
#if INTPTR_MAX == INT64_MAX
  using u0 = u64;
#else
  using u0 = u32;
#endif




/// Float

/** @brief Brain floating-point 16-bit. */
using bf16 = __bf16;
/** @brief 16-bit floating-point. */
using f16  = __fp16;
/** @brief 32-bit floating-point. */
using f32  = float;
/** @brief 64-bit floating-point. */
using f64  = double;
/** @brief 128-bit floating-point. */
using f128 = __float128;

/** @brief Architecture-dependent floating-point. */
#if INTPTR_MAX == INT64_MAX
  using f0 = f64;
#else
  using f0 = f32;
#endif




/// Norm Types

/** 
 * @brief Normalized integer type structure.
 * @tparam T The underlying integral type.
 */
template <typename T>
  requires std::is_integral_v<T>
struct __norm
{
  public:
    /** @brief Default constructor. */
    constexpr inline __norm() {}

    /** 
     * @brief Construct from a floating-point value.
     * @param _ The floating-point value to normalize.
     */
    constexpr inline __norm(f32 _): m_value(f_to_float(_)) {}


  private:
    T m_value;

    /**
     * @brief Convert floating-point value to normalized integer.
     * @param value The floating-point value.
     * @return T The converted integer value.
     */
    constexpr static inline fun f_to_float(f32 value) -> T {
      constexpr f32 max_val = std::numeric_limits<T>::max();

      if constexpr (std::is_unsigned_v<T>) {
        auto clamped = std::clamp<f32>(value, 0.0, +1.0);

        return T(clamped * max_val + 0.5);
      }
      else {
        auto clamped = std::clamp<f32>(value, -1.0, +1.0);

        return T(clamped * max_val + (clamped >= 0 ? 0.5 : -0.5));
      }
    }


  public:
    /** @brief Access the underlying integer value. */
    constexpr inline fun& value() { return m_value; }

    /** @brief Implicit conversion back to floating-point. */
    constexpr inline operator f32() const {
      constexpr f32 max_val = std::numeric_limits<T>::max();
      
      if constexpr (std::is_unsigned_v<T>)
        return f32(m_value) / max_val;
      else
        return std::max(-1.0f, f32(m_value) / max_val);
    }

};


/** @brief Trait to check if a type is a normalized integer. */
template <typename T>
struct is_norm_type: std::false_type {};

template <typename T>
struct is_norm_type<__norm<T>>: std::true_type {};

/** @brief Helper variable template for is_norm_type. */
template <typename _Tp>
inline constexpr bool is_norm_v = is_norm_type<_Tp>::value;


/** @brief Normalized 8-bit unsigned integer. */
using nu8  = __norm<u8>;
/** @brief Normalized 16-bit unsigned integer. */
using nu16 = __norm<u16>;
/** @brief Normalized 32-bit unsigned integer. */
using nu32 = __norm<u32>;

/** @brief Normalized 8-bit signed integer. */
using ni8  = __norm<i8>;
/** @brief Normalized 16-bit signed integer. */
using ni16 = __norm<i16>;
/** @brief Normalized 32-bit signed integer. */
using ni32 = __norm<i32>;




/// Vector

/**
 * @brief A generic SIMD vector structure.
 * @tparam T The element type.
 * @tparam S The number of elements in the vector.
 */
template <typename T, u0 S>
  requires std::is_arithmetic_v<T>
struct alignas(sizeof(T) *S) vec
{
private:
  using vector_t = T __attribute__((vector_size(sizeof(T)*S)));
  vector_t _elems;


public:
  /** @brief Default constructor. Initializes all elements to zero. */
  vec(): _elems{0} {}
  
  /** @brief Construct from SIMD vector type. */
  vec(vector_t V) : _elems(V) {}

  /** @brief Broadcast a single value to all elements. */
  vec(T val)
  {
    std::vector<T> Data(S);

    for (auto &X: Data) X = val;

    __builtin_memcpy(&_elems, Data.data(), sizeof(vector_t));
  }
  
  /** @brief Construct from std::array. */
  vec(std::array<T,S> V)
  {
    __builtin_memcpy(&_elems, V.data(), sizeof(vector_t));
  }

  /** @brief Construct from initializer list. */
  vec(std::initializer_list<T> V)
  {
    if (V.size() > S)
      throw std::out_of_range("List size out of bounds");

    
    std::copy(V.begin(), V.end(), (T*)&_elems);
  }

  /** @brief Construct from raw pointer. */
  vec(T *V)
  {
    __builtin_memcpy(&_elems, V, sizeof(vector_t));
  }


public:
  /** 
   * @brief Check if all elements are equal to another vector.
   * @param It The vector to compare with.
   * @return true if all elements are equal, false otherwise.
   */
  [[nodiscard]] inline bool is_equal(const vec<T,S> &It) const
  {
    auto Mask = (_elems == It._elems);
    for (u0 i = 0; i < S; ++i)
      if (!Mask[i])
        return false;
    
    return true;
  }
  
  /** @brief Convert the vector to std::array. */
  [[nodiscard]] inline std::array<T,S> to_array() const
  {
    std::array<T,S> Arr;
    
    __builtin_memcpy(Arr.data(), &_elems, sizeof(vector_t));
    return Arr;
  }

  /** @brief Convert the vector to std::vector. */
  [[nodiscard]] inline std::vector<T> to_vector() const
  {
    std::vector<T> Vec(S);
    
    __builtin_memcpy(Vec.data(), &_elems, sizeof(vector_t));
    return Vec;
  }

  /** @brief Get the number of elements in the vector. */
  inline u0 size() const { return S; }
  
  /** @brief Set an element at a specific index. */
  inline void set(u0 index, T val)
  {
    if (index >= S) throw std::out_of_range("Index out of bounds");
      _elems[index] = val;
  }

  /** @brief Get an element at a specific index. */
  [[nodiscard]] inline T get(size_t index)
  {
    if (index >= S) throw std::out_of_range("Index out of bounds");
    return _elems[index];
  }


  /** @brief Return a vector with the component-wise minimum. */
  inline vec min(const vec &It) const { return (_elems < It._elems) ? _elems : It._elems; }
  
  /** @brief Return a vector with the component-wise maximum. */
  inline vec max(const vec &It) const { return (_elems > It._elems) ? _elems : It._elems; }


public:  
  inline T operator[](u0 __n) const { return _elems[__n]; }
  
  inline vec operator== (const vec &It) const { return (_elems == It._elems); }

  inline vec operator+ (const vec &It) const { return (_elems + It._elems); }
  inline vec operator- (const vec &It) const { return (_elems - It._elems); }
  inline vec operator* (const vec &It) const { return (_elems * It._elems); }
  inline vec operator/ (const vec &It) const { return (_elems / It._elems); }
  inline vec operator% (const vec &It) const { return (_elems % It._elems); }

  inline vec& operator+=  (const vec &It) { _elems += It._elems; return *this; }
  inline vec& operator-=  (const vec &It) { _elems -= It._elems; return *this; }
  inline vec& operator*=  (const vec &It) { _elems *= It._elems; return *this; }
  inline vec& operator/=  (const vec &It) { _elems /= It._elems; return *this; }
  inline vec& operator%=  (const vec &It) { _elems %= It._elems; return *this; }

};




/// Data

/** @brief A generic raw data representation. */
struct data
{
  public:
    /** @brief Default constructor. */
    inline data() {}

    /** 
     * @brief Construct from pointer and size.
     * @param ptr Pointer to the data.
     * @param size Size of the data in bytes.
     */
    inline data(void* ptr, u0 size)
      : m_ptr(ptr)
      , m_size(size)
    {}


  private:
    void* m_ptr{};
    u0    m_size{};

  public:
    /** @brief Access the data pointer. */
    inline fun& ptr() { return m_ptr; }
    /** @brief Access the data size. */
    inline fun& size() { return m_size; }
};




/// Pointer

/** @brief A shorthand null pointer constant. */
inline constexpr decltype(nullptr) nil = nullptr;


/** @brief Unique pointer alias. */
template <typename T>
using uptr = std::unique_ptr<T>;

/** @brief Helper function to create a unique pointer. */
template <typename T>
fun make_uptr(T* obj) -> uptr<T> { return uptr<T>(obj); }


/** @brief Shared pointer alias. */
template <typename T>
#ifdef __GLIBCXX__
using sptr = std::__shared_ptr<T, std::_Lock_policy::_S_single>;
#else
using sptr = std::shared_ptr<T>;
#endif

/** @brief Helper function to create a shared pointer. */
template <typename T>
fun make_sptr(T* obj) -> sptr<T> { return sptr<T>(obj); }


/** @brief Weak pointer alias. */
template <typename T>
#ifdef __GLIBCXX__
using wptr = std::__weak_ptr<T, std::_Lock_policy::_S_single>;
#else
using wptr = std::weak_ptr<T>;
#endif
