/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#pragma once

#include "Basis.hh"
#include <cmath>
#include <type_traits>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_shared.hpp>

#include <immintrin.h>
#include <xmmintrin.h> // SSE
#include <emmintrin.h> // SSE2



namespace mochi
{

  /** 
   * @brief A generic 2-dimensional vector.
   * @tparam T The element type.
   */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec2
  {
    public:
      T X{}, Y{};

    public:
      /** @brief Default constructor. */
      inline constexpr vec2() {}
      /** @brief Construct with a single broadcasted value. */
      inline constexpr vec2(T V): X(V), Y(V) {}
      /** @brief Construct with specific X and Y values. */
      inline constexpr vec2(T nX, T nY): X(nX), Y(nY) {}

      /** @brief Construct from another vector of different type. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      inline constexpr vec2(vec2<J> V): X(V.X), Y(V.Y) {}

    public:
      /** @brief Calculate the squared length of the vector. */
      [[nodiscard]] inline constexpr fun dot() const noexcept -> T { return (X*X) + (Y*Y); }

      /** @brief Calculate the dot product with another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dot(const vec2<J> &it) const noexcept -> T { return (X*it.X) + (Y*it.Y); }


      /** @brief Calculate the squared distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist_sq(const vec2<J> &it) const noexcept -> T {
        T dx = X -it.X;
        T dy = Y -it.Y;
        return (dx*dx) + (dy*dy);
      }


      /** @brief Calculate the length of the vector. */
      [[nodiscard]] inline constexpr fun dist() const noexcept -> const T { return std::sqrt(this->dot()); }
      
      /** @brief Calculate the distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist(const vec2<J> &it) const noexcept -> T { return std::sqrt(this->dist_sq(it)); }


      /** @brief Calculate the 2D cross product with another vector. */
      [[nodiscard]] inline constexpr fun cross(const vec2<T>& it) const -> T { return (X*it.Y) - (Y*it.X); }


      /** @brief Return a normalized copy of this vector. */
      [[nodiscard]] inline constexpr fun normalize() const noexcept -> vec2 {
        T len = dist();
        return vec2(X/len, Y/len);
      }
      

    public:
      inline constexpr fun operator==(const vec2 &it) const noexcept -> const bool { return (X == it.X && Y == it.Y); }
      inline constexpr fun operator!=(const vec2 &it) const noexcept -> const bool { return !(*this == it); }

      inline constexpr fun operator+(const vec2 &it) const -> const vec2 { return vec2(X +it.X, Y +it.Y); }
      inline constexpr fun operator-(const vec2 &it) const -> const vec2 { return vec2(X -it.X, Y -it.Y); }
      inline constexpr fun operator*(const vec2 &it) const -> const vec2 { return vec2(X *it.X, Y *it.Y); }
      inline constexpr fun operator/(const vec2 &it) const -> const vec2 { return vec2(X /it.X, Y /it.Y); }

      inline constexpr fun operator+=(const vec2 &It) noexcept -> vec2& { X += It.X; Y += It.Y; return *this; }
      inline constexpr fun operator-=(const vec2 &It) noexcept -> vec2& { X -= It.X; Y -= It.Y; return *this; }
      inline constexpr fun operator*=(const vec2 &It) noexcept -> vec2& { X *= It.X; Y *= It.Y; return *this; }
      inline constexpr fun operator/=(const vec2 &It) noexcept -> vec2& { X /= It.X; Y /= It.Y; return *this; }


      inline constexpr fun operator+(const T it) const -> const vec2 { return vec2(X +it, Y +it); }
      inline constexpr fun operator-(const T it) const -> const vec2 { return vec2(X -it, Y -it); }
      inline constexpr fun operator*(const T it) const -> const vec2 { return vec2(X *it, Y *it); }
      inline constexpr fun operator/(const T it) const -> const vec2 { return vec2(X /it, Y /it); }

      inline constexpr fun operator+=(const T It) noexcept -> vec2& { X += It; Y += It; return *this; }
      inline constexpr fun operator-=(const T It) noexcept -> vec2& { X -= It; Y -= It; return *this; }
      inline constexpr fun operator*=(const T It) noexcept -> vec2& { X *= It; Y *= It; return *this; }
      inline constexpr fun operator/=(const T It) noexcept -> vec2& { X /= It; Y /= It; return *this; }
  };



  /** 
   * @brief A generic 3-dimensional vector.
   * @tparam T The element type.
   */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec3
  {
    public:
      T X{}, Y{}, Z{};

    public:
      /** @brief Default constructor. */
      inline vec3() {}
      /** @brief Construct with a single broadcasted value. */
      inline vec3(T V): X(V), Y(V), Z(V) {}
      /** @brief Construct with specific X, Y, and Z values. */
      inline vec3(T nX, T nY, T nZ): X(nX), Y(nY), Z(nZ) {}

      /** @brief Construct from another vector of different type. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      inline constexpr vec3(vec3<J> V): X(V.X), Y(V.Y), Z(V.Z) {}

    public:
      /** @brief Calculate the squared length of the vector. */
      [[nodiscard]] inline constexpr fun dot() const noexcept -> T { return (X*X) + (Y*Y) + (Z*Z); }

      /** @brief Calculate the dot product with another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dot(const vec3<J> &it) const noexcept -> T { return (X*it.X) + (Y*it.Y) + (Z*it.Z); }

      /** @brief Calculate the squared distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist_sq(const vec3<J> &it) const noexcept -> T {
        T dx = X -it.X;
        T dy = Y -it.Y;
        T dz = Z -it.Z;
        return (dx*dx) + (dy*dy) + (dz*dz);
      }

      /** @brief Calculate the length of the vector. */
      [[nodiscard]] inline constexpr fun dist() const noexcept -> T { return std::sqrt(this->dot()); }

      /** @brief Calculate the distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist(const vec3<J> &it) const noexcept -> T { return std::sqrt(this->dist_sq(it)); }
      

      /** @brief Calculate the cross product with another vector. */
      [[nodiscard]] inline constexpr fun cross(const vec3<T>& it) const -> vec3<T> {
        return vec3<T>(
          (Y*it.Z) - (Z*it.Y),
          (Z*it.X) - (X*it.Z),
          (X*it.Y) - (Y*it.X)
        );
      }


      /** @brief Return a normalized copy of this vector. */
      [[nodiscard]] inline constexpr fun normalize() const noexcept -> vec3 {
        T len = dist();
        return vec3(X/len, Y/len, Z/len);
      }
      

    public:
      inline constexpr fun operator==(const vec3 &it) const noexcept -> const bool { return (X == it.X && Y == it.Y && Z == it.Z); }
      inline constexpr fun operator!=(const vec3 &it) const noexcept -> const bool { return !(*this == it); }

      inline constexpr fun operator+(const vec3 &it) const -> const vec3 { return vec3(X +it.X, Y +it.Y, Z +it.Z); }
      inline constexpr fun operator-(const vec3 &it) const -> const vec3 { return vec3(X -it.X, Y -it.Y, Z -it.Z); }
      inline constexpr fun operator*(const vec3 &it) const -> const vec3 { return vec3(X *it.X, Y *it.Y, Z *it.Z); }
      inline constexpr fun operator/(const vec3 &it) const -> const vec3 { return vec3(X /it.X, Y /it.Y, Z /it.Z); }

      inline constexpr fun operator+=(const vec3 &It) noexcept -> vec3& { X += It.X; Y += It.Y; Z += It.Z; return *this; }
      inline constexpr fun operator-=(const vec3 &It) noexcept -> vec3& { X -= It.X; Y -= It.Y; Z -= It.Z; return *this; }
      inline constexpr fun operator*=(const vec3 &It) noexcept -> vec3& { X *= It.X; Y *= It.Y; Z *= It.Z; return *this; }
      inline constexpr fun operator/=(const vec3 &It) noexcept -> vec3& { X /= It.X; Y /= It.Y; Z /= It.Z; return *this; }


      inline constexpr fun operator+(const T it) const -> const vec3 { return vec3(X +it, Y +it, Z +it); }
      inline constexpr fun operator-(const T it) const -> const vec3 { return vec3(X -it, Y -it, Z -it); }
      inline constexpr fun operator*(const T it) const -> const vec3 { return vec3(X *it, Y *it, Z *it); }
      inline constexpr fun operator/(const T it) const -> const vec3 { return vec3(X /it, Y /it, Z /it); }

      inline constexpr fun operator+=(const T It) noexcept -> vec3& { X += It; Y += It; Z += It; return *this; }
      inline constexpr fun operator-=(const T It) noexcept -> vec3& { X -= It; Y -= It; Z -= It; return *this; }
      inline constexpr fun operator*=(const T It) noexcept -> vec3& { X *= It; Y *= It; Z *= It; return *this; }
      inline constexpr fun operator/=(const T It) noexcept -> vec3& { X /= It; Y /= It; Z /= It; return *this; }
  };



  /** 
   * @brief A generic 4-dimensional vector.
   * @tparam T The element type.
   */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct vec4
  {
    public:
      T X{}, Y{}, Z{}, W{};

    public:
      /** @brief Default constructor. */
      inline vec4() {}
      /** @brief Construct with a single broadcasted value. */
      inline vec4(T V): X(V), Y(V), Z(V), W(V) {}
      /** @brief Construct with specific X, Y, Z, and W values. */
      inline vec4(T nX, T nY, T nZ, T nW): X(nX), Y(nY), Z(nZ), W(nW) {}

      /** @brief Construct from another vector of different type. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      inline constexpr vec4(vec4<J> V): X(V.X), Y(V.Y), Z(V.Z), W(V.W) {}

      /** @brief Construct from a 3-dimensional vector and a scalar W. */
      inline constexpr vec4(vec3<T> V, T nW): X(V.X), Y(V.Y), Z(V.Z), W(nW) {}

    public:
      /** @brief Calculate the squared length of the vector. */
      [[nodiscard]] inline constexpr fun dot() const noexcept -> T { return (X*X) + (Y*Y) + (Z*Z) +(W*W); }

      /** @brief Calculate the dot product with another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dot(const vec4<J> &it) const noexcept -> T { return (X*it.X) + (Y*it.Y) + (Z*it.Z) + (W+it.W); }

      /** @brief Calculate the squared distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist_sq(const vec4<J> &it) const noexcept -> T {
        T dx = X -it.X;
        T dy = Y -it.Y;
        T dz = Z -it.Z;
        T dw = W -it.W;
        return (dx*dx) + (dy*dy) + (dz*dz) + (dw*dw);
      }


      /** @brief Calculate the length of the vector. */
      [[nodiscard]] inline constexpr fun dist() const noexcept -> T { return std::sqrt(this->dot()); }

      /** @brief Calculate the distance to another vector. */
      template <typename J>
        requires std::is_arithmetic_v<J>
      [[nodiscard]] inline constexpr fun dist(const vec4<J> &it) const noexcept -> T { return std::sqrt(this->dist_sq(it)); }
      

      /** @brief Return a normalized copy of this vector. */
      [[nodiscard]] inline constexpr fun normalize() const noexcept -> vec4 {
        T len = dist();
        return vec4(X/len, Y/len, Z/len, W/len);
      }


    public:
      inline constexpr fun operator==(const vec4 &it) const noexcept -> const bool { return (X == it.X && Y == it.Y && Z == it.Z && W == it.W); }
      inline constexpr fun operator!=(const vec4 &it) const noexcept -> const bool { return !(*this == it); }

      inline constexpr fun operator+(const vec4 &it) const -> const vec4 { return vec4(X +it.X, Y +it.Y, Z +it.Z, W +it.W); }
      inline constexpr fun operator-(const vec4 &it) const -> const vec4 { return vec4(X -it.X, Y -it.Y, Z -it.Z, W -it.W); }
      inline constexpr fun operator*(const vec4 &it) const -> const vec4 { return vec4(X *it.X, Y *it.Y, Z *it.Z, W *it.W); }
      inline constexpr fun operator/(const vec4 &it) const -> const vec4 { return vec4(X /it.X, Y /it.Y, Z /it.Z, W /it.W); }

      inline constexpr fun operator+=(const vec4 &It) noexcept -> vec4& { X += It.X; Y += It.Y; Z += It.Z; W += It.W; return *this; }
      inline constexpr fun operator-=(const vec4 &It) noexcept -> vec4& { X -= It.X; Y -= It.Y; Z -= It.Z; W -= It.W; return *this; }
      inline constexpr fun operator*=(const vec4 &It) noexcept -> vec4& { X *= It.X; Y *= It.Y; Z *= It.Z; W *= It.W; return *this; }
      inline constexpr fun operator/=(const vec4 &It) noexcept -> vec4& { X /= It.X; Y /= It.Y; Z /= It.Z; W /= It.W; return *this; }


      inline constexpr fun operator+(const T it) const -> const vec4 { return vec4(X +it, Y +it, Z +it, W +it); }
      inline constexpr fun operator-(const T it) const -> const vec4 { return vec4(X -it, Y -it, Z -it, W -it); }
      inline constexpr fun operator*(const T it) const -> const vec4 { return vec4(X *it, Y *it, Z *it, W *it); }
      inline constexpr fun operator/(const T it) const -> const vec4 { return vec4(X /it, Y /it, Z /it, W /it); }

      inline constexpr fun operator+=(const T It) noexcept -> vec4& { X += It; Y += It; Z += It; W += It; return *this; }
      inline constexpr fun operator-=(const T It) noexcept -> vec4& { X -= It; Y -= It; Z -= It; W -= It; return *this; }
      inline constexpr fun operator*=(const T It) noexcept -> vec4& { X *= It; Y *= It; Z *= It; W *= It; return *this; }
      inline constexpr fun operator/=(const T It) noexcept -> vec4& { X /= It; Y /= It; Z /= It; W /= It; return *this; }
  };



  /** 
   * @brief A generic quaternion for representing rotations.
   * @tparam T The element type.
   */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct quaternion
  {
    public:
      T W{1}, X{}, Y{}, Z{};

    public:
      /** @brief Default constructor (Identity quaternion). */
      explicit inline quaternion() {}
      /** @brief Construct with specific W, X, Y, and Z values. */
      inline quaternion(T nW, T nX, T nY, T nZ): W(nW), X(nX), Y(nY), Z(nZ) {}

      /**
       * @brief Create a quaternion from an axis and an angle.
       * @param angle The rotation angle in radians.
       * @param axis The rotation axis.
       * @return A quaternion representing the rotation.
       */
      [[nodiscard]] inline static fun fromAxisAngle(T angle, const vec3<T> &axis) -> quaternion {
        axis = axis.normalize();
        T halfAngle = angle *0.5f;
        T s = std::sin(halfAngle);

        return quaternion(
          std::cos(halfAngle),
          axis.X *s,
          axis.Y *s,
          axis.Z *s
        );
      }

      /**
       * @brief Create a quaternion that rotates vector u to vector v.
       * @param u The source vector.
       * @param v The destination vector.
       * @return A quaternion representing the rotation from u to v.
       */
      [[nodiscard]] inline static fun fromTwoVector(vec3<T> u, vec3<T> v) -> quaternion  {
        u = u.normalize();
        v = v.normalize();
        
        T cosTheta = u.dot(v);
        vec3<T> rotationAxis;

        if (cosTheta < -1.0 + 1e-6) {
          rotationAxis = vec3<T>{0, 1, 0}.cross(u);
          if (rotationAxis.dist() < 0.01)
            rotationAxis = vec3<T>{1, 0, 0}.cross(u);
          return fromAxisAngle(M_PI, rotationAxis.normalize());
        }

        rotationAxis = u.cross(v);
        T s = std::sqrt((1.0 + cosTheta) * 2.0);
        T invS = 1.0 / s;

        return quaternion(
          s * 0.5,
          rotationAxis.X * invS,
          rotationAxis.Y * invS,
          rotationAxis.Z * invS
        ).normalize();
      }

      /**
       * @brief Create a look-at rotation quaternion.
       * @param pos The eye position.
       * @param target The target position to look at.
       * @param up The up vector.
       * @return A quaternion representing the look-at rotation.
       */
      [[nodiscard]] inline static fun lookAt(const vec3<T> &pos, const vec3<T> &target, const vec3<T> &up) -> quaternion  {
        vec3<T> forward = (target - pos).normalize();
        
        vec3<T> right = forward.cross(up).normalize();
        vec3<T> orthoUp = right.cross(forward).normalize();
        
        quaternion q1 = fromTwoVector(vec3<T>(0, 0, -1), forward);
        
        vec3<T> currentUp = q1.rotate(vec3<T>(0, 1, 0));
        
        quaternion q2 = fromTwoVector(currentUp, orthoUp);
        
        return q2 * q1;
      }

      /**
       * @brief Spherical linear interpolation between two quaternions.
       * @param q1 The starting quaternion.
       * @param q2 The ending quaternion.
       * @param t The interpolation parameter [0, 1].
       * @return The interpolated quaternion.
       */
      [[nodiscard]] inline static fun slerp(quaternion q1, quaternion q2, T t) -> quaternion  {
        q1 = q1.normalize();
        q2 = q2.normalize();

        T dot = (q1.W * q2.W) + (q1.X * q2.X) + (q1.Y * q2.Y) + (q1.Z * q2.Z);

        if (dot < 0.0f) {
          q2 = quaternion(-q2.W, -q2.X, -q2.Y, -q2.Z);
          dot = -dot;
        }

        if (dot > 0.9995)
          return quaternion(
            q1.W + t * (q2.W - q1.W),
            q1.X + t * (q2.X - q1.X),
            q1.Y + t * (q2.Y - q1.Y),
            q1.Z + t * (q2.Z - q1.Z)
          ).normalize();

        T theta_0 = std::acos(dot);
        T theta = theta_0 * t;
        T sin_theta_0 = std::sin(theta_0);
        T sin_theta = std::sin(theta);

        T s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        T s1 = sin_theta / sin_theta_0;

        return quaternion(
          (s0 * q1.W) + (s1 * q2.W),
          (s0 * q1.X) + (s1 * q2.X),
          (s0 * q1.Y) + (s1 * q2.Y),
          (s0 * q1.Z) + (s1 * q2.Z)
        );
      }


    public:
      inline fun operator*(const quaternion &it) const -> quaternion {
        return quaternion(
          W *it.W - X *it.X - Y *it.Y - Z *it.Z,  // w
          W *it.X + X *it.W + Y *it.Z - Z *it.Y,  // x
          W *it.Y - X *it.Z + Y *it.W + Z *it.X,  // y
          W *it.Z + X *it.Y - Y *it.X + Z *it.W   // z
        );
      }

      
    public:
      /**
       * @brief Rotate a 3D vector by this quaternion.
       * @param v The vector to rotate.
       * @return The rotated vector.
       */
      [[nodiscard]] inline fun rotate(const vec3<T> &v) const -> vec3<T> {
        vec3<T> u{X,Y,Z};

        vec3<T> t = u.cross(v) *2; 
        
        return v + (t *W) +u.cross(t);
      }

      /** @brief Get the conjugate of this quaternion. */
      [[nodiscard]] inline fun conjugate() const -> quaternion {
        return quaternion(W, -X, -Y, -Z);
      }

      /** @brief Return a normalized copy of this quaternion. */
      [[nodiscard]] inline fun normalize() const -> quaternion {
        T len = std::sqrt(W*W + X*X + Y*Y + Z*Z);
        if (len > 0) {
          T invLen = 1.0 / len;

          return quaternion(W*invLen, X*invLen, Y*invLen, Z*invLen);
        }
        return quaternion();
      }
    
  };



  /** 
   * @brief Traits class for SIMD operations.
   * @tparam T The element type.
   */
  template <typename T>
  struct SimdTraits;

  template <>
  struct SimdTraits<f64> {
    using reg_t = __m256d;
    static inline reg_t load(const f64 *p) { return _mm256_load_pd(p); }
    static inline void store(f64 *p, reg_t r) { _mm256_store_pd(p, r); }
    static inline reg_t set1(f64 v) { return _mm256_set1_pd(v); }
    static inline reg_t mul(reg_t a, reg_t b) { return _mm256_mul_pd(a, b); }
    static inline reg_t sub(reg_t a, reg_t b) { return _mm256_sub_pd(a, b); }
    
    static inline reg_t identity_row(int i) {
      if (i == 0) return _mm256_setr_pd(1.0, 0.0, 0.0, 0.0);
      if (i == 1) return _mm256_setr_pd(0.0, 1.0, 0.0, 0.0);
      if (i == 2) return _mm256_setr_pd(0.0, 0.0, 1.0, 0.0);
      return _mm256_setr_pd(0.0, 0.0, 0.0, 1.0);
    }
  };

  
  template <>
  struct SimdTraits<f32> {
    using reg_t = __m128;

    static inline reg_t load(const f32* p) { return _mm_load_ps(p); }
    static inline void store(f32* p, reg_t r) { _mm_store_ps(p, r); }
    static inline reg_t set1(f32 v) { return _mm_set1_ps(v); }
    static inline reg_t mul(reg_t a, reg_t b) { return _mm_mul_ps(a, b); }
    static inline reg_t sub(reg_t a, reg_t b) { return _mm_sub_ps(a, b); }

    static inline reg_t identity_row(int i) {
      if (i == 0) return _mm_setr_ps(1.0f, 0.0f, 0.0f, 0.0f);
      if (i == 1) return _mm_setr_ps(0.0f, 1.0f, 0.0f, 0.0f);
      if (i == 2) return _mm_setr_ps(0.0f, 0.0f, 1.0f, 0.0f);
      return _mm_setr_ps(0.0f, 0.0f, 0.0f, 1.0f);
    }
  };



  /** 
   * @brief A generic 4x4 matrix, primarily using SIMD for operations.
   * @tparam T The element type.
   */
  template <typename T>
    requires (std::is_arithmetic_v<T> || is_norm_v<T> || std::is_same_v<T, f16>)
  struct mat4
  {
    public:
      union {
        T SwVec[4][4];
        vec4<T> SwVec4[4];
        vec<T,4> HwVec[4];
      };

      using ST = SimdTraits<T>;
      using reg_t = typename ST::reg_t;


    public:
      /** @brief Default constructor (Identity matrix). */
      inline mat4(): SwVec4(
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}
      ) {}

      /** @brief Construct from 4 row vectors. */
      inline mat4(vec4<T> R1, vec4<T> R2, vec4<T> R3, vec4<T> R4): SwVec4(R1,R2,R3,R4) {}

    
    public:
      inline fun operator*(const mat4 &it) const -> mat4 {
        mat4 result;
        for (int i{}; i < 4; i++)
        {
          vec<T, 4> x(SwVec[i][0]);
          vec<T, 4> y(SwVec[i][1]);
          vec<T, 4> z(SwVec[i][2]);
          vec<T, 4> w(SwVec[i][3]);

          auto r = x * it.HwVec[0];
          r = (y * it.HwVec[1]) + r;
          r = (z * it.HwVec[2]) + r;
          r = (w * it.HwVec[3]) + r;

          result.HwVec[i] = r;
        }
        return result;
      }

      inline fun operator*(const vec4<T> &v) const -> vec4<T> {
        return vec4<T>(
          SwVec[0][0] * v.X + SwVec[0][1] * v.Y + SwVec[0][2] * v.Z + SwVec[0][3] * v.W,
          SwVec[1][0] * v.X + SwVec[1][1] * v.Y + SwVec[1][2] * v.Z + SwVec[1][3] * v.W,
          SwVec[2][0] * v.X + SwVec[2][1] * v.Y + SwVec[2][2] * v.Z + SwVec[2][3] * v.W,
          SwVec[3][0] * v.X + SwVec[3][1] * v.Y + SwVec[3][2] * v.Z + SwVec[3][3] * v.W
        );
      }


    public:
      inline fun setZero() -> void {
        vec<T, 4> zero{};
        HwVec[0] = HwVec[1] = HwVec[2] = HwVec[3] = zero;
      }

      /**
       * @brief Calculate the inverse of the matrix.
       * @return The inverted matrix.
       */
      inline fun inverse() const -> mat4 {
        mat4 dest;
        reg_t rows[4], arows[4];

        for (int i = 0; i < 4; ++i) {
          rows[i] = ST::load(SwVec[i]);
          arows[i] = ST::identity_row(i);
        }

        T tmp alignas(32) [8];
        const T EPS = static_cast<T>(1e-12);

        for (int k = 0; k < 4; k++) {
          int pivot = k;
          T maxv = 0;
          for (int i = k; i < 4; i++) {
            ST::store(tmp, rows[i]);
            T val = std::abs(tmp[k]);
            if (val > maxv) { maxv = val; pivot = i; }
          }

          if (maxv < EPS) return mat4<T>(); // Singular

          if (pivot != k) {
            std::swap(rows[k], rows[pivot]);
            std::swap(arows[k], arows[pivot]);
          }

          ST::store(tmp, rows[k]);
          T pv = tmp[k];
          reg_t v_inv_pv = ST::set1(static_cast<T>(1.0) / pv);

          rows[k] = ST::mul(rows[k], v_inv_pv);
          arows[k] = ST::mul(arows[k], v_inv_pv);

          for (int i = 0; i < 4; ++i) {
            if (i == k) continue;
            ST::store(tmp, rows[i]);
            T factor = tmp[k];
            if (std::abs(factor) < EPS) continue;

            reg_t vfac = ST::set1(factor);
            rows[i] = ST::sub(rows[i], ST::mul(vfac, rows[k]));
            arows[i] = ST::sub(arows[i], ST::mul(vfac, arows[k]));
          }
        }

        for (int i = 0; i < 4; ++i) ST::store(dest.SwVec[i], arows[i]);
        return dest;
      }

      /**
       * @brief Create a rotation matrix from Euler angles and multiply it with this matrix.
       * @param deg The Euler angles (pitch, yaw, roll) in radians.
       * @return The resulting rotated matrix.
       */
      inline fun rotation(vec3<T> deg) -> mat4 {
        T cx = cosf(deg.X); T sx = sinf(deg.X);
        T cy = cosf(deg.Y); T sy = sinf(deg.Y);
        T cz = cosf(deg.Z); T sz = sinf(deg.Z);

        mat4 res = mat4();

        // Birleşik Rotasyon Matrisi (Z * Y * X)
        res.SwVec[0][0] = cy * cz;
        res.SwVec[0][1] = sx * sy * cz - cx * sz;
        res.SwVec[0][2] = cx * sy * cz + sx * sz;

        res.SwVec[1][0] = cy * sz;
        res.SwVec[1][1] = sx * sy * sz + cx * cz;
        res.SwVec[1][2] = cx * sy * sz - sx * cz;

        res.SwVec[2][0] = -sy;
        res.SwVec[2][1] = sx * cy;
        res.SwVec[2][2] = cx * cy;

        return (*this)*res;
      }


    public:
      /**
       * @brief Create a perspective projection matrix.
       * @param fovDeg Field of view in degrees.
       * @param aspect Aspect ratio (width / height).
       * @param nearZ Near clipping plane.
       * @param farZ Far clipping plane.
       */
      static inline fun perspective(T fovDeg, T aspect, T nearZ, T farZ) -> mat4 {
        mat4 res;
        res.setZero();
        
        T tanHalfFov = std::tan(fovDeg * 0.5 * M_PI / 180.0);
        
        res.SwVec[0][0] = 1.0 / (aspect * tanHalfFov);
        res.SwVec[1][1] = -(1.0 / tanHalfFov);
        res.SwVec[2][2] = -farZ / (farZ - nearZ);
        res.SwVec[2][3] = -(farZ * nearZ) / (farZ - nearZ);
        res.SwVec[3][2] = -1.0;
        res.SwVec[3][3] = 0.0;
        
        return res;
      }

      /**
       * @brief Create a view matrix using a camera position, target, and up vector.
       * @param eye The camera position.
       * @param center The target point to look at.
       * @param up The up direction.
       * @return The view matrix.
       */
      static inline fun lookAt(vec3<T> eye, vec3<T> center, vec3<T> up) -> mat4 {
        // F (Forward): Center - Eye
        auto f = (center-eye).normalize();

        // S (Side): Forward x Up (Cross Product)
        auto s = vec3<T>(
          f.Y*up.Z - f.Z*up.Y,
          f.Z*up.X - f.X*up.Z,
          f.X*up.Y - f.Y*up.X
        ).normalize();

        // U (Re-calculated Up): Side x Forward
        auto u = vec3<T>(
          s.Y*f.Z - s.Z*f.Y,
          s.Z*f.X - s.X*f.Z,
          s.X*f.Y - s.Y*f.X
        );


        mat4 res;
        res.HwVec[0] = {+s.X, +s.Y, +s.Z, 0.0};
        res.HwVec[1] = {+u.X, +u.Y, +u.Z, 0.0};
        res.HwVec[2] = {-f.X, -f.Y, -f.Z, 0.0};
        res.HwVec[3] = {+0.0, +0.0, +0.0, 1.0};

        // Translation
        mat4 translation;
        translation.SwVec[0][3] = -eye.X;
        translation.SwVec[1][3] = -eye.Y;
        translation.SwVec[2][3] = -eye.Z;
        
        res.SwVec[0][3] = -(s.X*eye.X + s.Y*eye.Y + s.Z*eye.Z);
        res.SwVec[1][3] = -(u.X*eye.X + u.Y*eye.Y + u.Z*eye.Z);
        res.SwVec[2][3] =  (f.X*eye.X + f.Y*eye.Y + f.Z*eye.Z);

        return res;
      }

      /**
       * @brief Create a model transformation matrix.
       * @param position The translation vector.
       * @param rotate The rotation quaternion.
       * @param scale The scale vector.
       * @return The model matrix.
       */
      static inline fun model(vec3<T> position, quaternion<T> rotate, vec3<T> scale) -> mat4 {
        auto q = rotate.normalize();

        T xx = q.X * q.X;
        T yy = q.Y * q.Y;
        T zz = q.Z * q.Z;
        T xy = q.X * q.Y;
        T xz = q.X * q.Z;
        T yz = q.Y * q.Z;
        T wx = q.W * q.X;
        T wy = q.W * q.Y;
        T wz = q.W * q.Z;

        // MATRİSİN SATIRLARI (Sütun Öncelikli Matematik İçin)
        return mat4(
          // Satır 0
          {
            (1.0f - 2.0f * (yy + zz)) * scale.X, // R00
            (2.0f * (xy - wz)) * scale.Y,        // R01
            (2.0f * (xz + wy)) * scale.Z,        // R02
            position.X                           // Sütun 3 (Öteleme X)
          },

          // Satır 1
          {
            (2.0f * (xy + wz)) * scale.X,        // R10
            (1.0f - 2.0f * (xx + zz)) * scale.Y, // R11
            (2.0f * (yz - wx)) * scale.Z,        // R12
            position.Y                           // Sütun 3 (Öteleme Y)
          },

          // Satır 2
          {
            (2.0f * (xz - wy)) * scale.X,        // R20
            (2.0f * (yz + wx)) * scale.Y,        // R21
            (1.0f - 2.0f * (xx + yy)) * scale.Z, // R22
            position.Z                           // Sütun 3 (Öteleme Z)
          },

          // Satır 3 (Perspektif Yuvaları)
          {
            0.0f,
            0.0f,
            0.0f,
            1.0f
          }
        );
      }


  };

}
