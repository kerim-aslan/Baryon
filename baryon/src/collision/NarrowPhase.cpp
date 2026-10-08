/*
 * Baryon - A custom physics engine
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file NarrowPhase.cpp
 * @brief Dar Faz (Narrow Phase) çarpışma algılama algoritmaları.
 * @details GJK algoritması ile cisimlerin kesişimini kontrol eder, kesişim
 * varsa EPA algoritması ile çarpışmanın şiddetini (derinlik) ve yönünü (normal)
 * hesaplar. Ayrıca ışın fırlatma (raycast) ve destek noktası (support point)
 * fonksiyonlarını içerir.
 */

#include "Baryon/collision/NarrowPhase.hpp"
#include "Baryon/Core/DebugManager.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace Baryon::collision {

namespace {

/**
 * @brief GJK ve EPA hesaplamaları için Minkowski farkı üzerindeki nokta
 * verileri.
 */
struct GjkWitness {
  Vector3 w;    ///< Minkowski farkı uzayındaki nokta (w = supA - supB)
  Vector3 supA; ///< A objesi üzerindeki en uç nokta
  Vector3 supB; ///< B objesi üzerindeki en uç nokta
};

/**
 * @brief İki şeklin Minkowski farkı üzerindeki destek noktasını hesaplar.
 * @details Destek noktası, bir şeklin belirli bir yöndeki en uç koordinatıdır.
 */
static void computeSupport(const CollisionShape &shapeA, const Pose &transformA,
                           const CollisionShape &shapeB, const Pose &transformB,
                           const Vector3 &dir, GjkWitness &out) {
  out.supA = NarrowPhase::getSupportPoint(shapeA, transformA, dir);
  out.supB = NarrowPhase::getSupportPoint(shapeB, transformB, dir * -1.0f);
  out.w = out.supA - out.supB;
}

/**
 * @brief Merkezin (0,0,0) bir doğru parçasına en yakın noktasını bulur.
 */
static Vector3 closestPointOnSegmentToOrigin(const Vector3 &a, const Vector3 &b,
                                             float &u, float &v) {
  Vector3 ab = b - a;
  float denom = ab.lengthSquare();
  if (denom < 1e-20f) {
    u = 1.0f;
    v = 0.0f;
    return a;
  }
  float t = -a.dot(ab) / denom;
  t = std::clamp(t, 0.0f, 1.0f);
  u = 1.0f - t;
  v = t;
  return a + ab * t;
}

/**
 * @brief Merkezin (0,0,0) bir üçgen yüzeyine en yakın noktasını bulur.
 */
static Vector3 closestPointOnTriangleToOrigin(const Vector3 &a,
                                              const Vector3 &b,
                                              const Vector3 &c, float &u,
                                              float &v, float &w) {
  const Vector3 ab = b - a;
  const Vector3 ac = c - a;
  const Vector3 ap = -a;

  const float d1 = ab.dot(ap);
  const float d2 = ac.dot(ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    u = 1.0f;
    v = w = 0.0f;
    return a;
  }

  const Vector3 bp = -b;
  const float d3 = ab.dot(bp);
  const float d4 = ac.dot(bp);
  if (d3 >= 0.0f && d4 <= d3) {
    u = 0.0f;
    v = 1.0f;
    w = 0.0f;
    return b;
  }

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float denom = d1 - d3;
    const float t = denom > 1e-20f ? d1 / denom : 0.0f;
    u = 1.0f - t;
    v = t;
    w = 0.0f;
    return a + ab * t;
  }

  const Vector3 cp = -c;
  const float d5 = ab.dot(cp);
  const float d6 = ac.dot(cp);
  if (d6 >= 0.0f && d5 <= d6) {
    u = 0.0f;
    v = 0.0f;
    w = 1.0f;
    return c;
  }

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float denom = d2 - d6;
    const float t = denom > 1e-20f ? d2 / denom : 0.0f;
    u = 1.0f - t;
    v = 0.0f;
    w = t;
    return a + ac * t;
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float denom = (d4 - d3) + (d5 - d6);
    const float t = denom > 1e-20f ? (d4 - d3) / denom : 0.0f;
    u = 0.0f;
    v = 1.0f - t;
    w = t;
    return b + (c - b) * t;
  }

  const float denom = va + vb + vc;
  if (std::abs(denom) < 1e-20f) {
    u = v = w = 1.0f / 3.0f;
    return (a + b + c) * (1.0f / 3.0f);
  }

  const float rv = vb / denom;
  const float rw = vc / denom;
  u = 1.0f - rv - rw;
  v = rv;
  w = rw;
  return a + ab * rv + ac * rw;
}

/**
 * @brief Orijinin bir dörtyüzlü (tetrahedron) içinde olup olmadığını kontrol
 * eder.
 */
static bool originInsideTetrahedron(const Vector3 &a, const Vector3 &b,
                                    const Vector3 &c, const Vector3 &d) {
  Vector3 n0 = (b - a).cross(c - a);
  if (n0.dot(d - a) > 0.0f)
    n0 = -n0;
  if (n0.dot(-a) > 0.0f)
    return false;

  Vector3 n1 = (c - a).cross(d - a);
  if (n1.dot(b - a) > 0.0f)
    n1 = -n1;
  if (n1.dot(-a) > 0.0f)
    return false;

  Vector3 n2 = (d - a).cross(b - a);
  if (n2.dot(c - a) > 0.0f)
    n2 = -n2;
  if (n2.dot(-a) > 0.0f)
    return false;

  Vector3 n3 = (d - b).cross(c - b);
  if (n3.dot(a - b) > 0.0f)
    n3 = -n3;
  if (n3.dot(-b) > 0.0f)
    return false;

  return true;
}

/**
 * @brief Orijinin bir dörtyüzlüye en yakın noktasını ve bu noktanın oranlarını
 * bulur.
 */
static Vector3 closestPointOnTetrahedronToOrigin(const Vector3 &a,
                                                 const Vector3 &b,
                                                 const Vector3 &c,
                                                 const Vector3 &d, float &u,
                                                 float &v, float &w, float &x) {
  if (originInsideTetrahedron(a, b, c, d)) {
    u = v = w = x = 0.25f;
    return Vector3(0, 0, 0);
  }
  float bestDistSq = 1e30f;
  Vector3 best(0, 0, 0);
  u = 1.0f;
  v = w = x = 0.0f;

  {
    float bu, bv, bw;
    Vector3 p = closestPointOnTriangleToOrigin(a, b, c, bu, bv, bw);
    float dsq = p.lengthSquare();
    if (dsq < bestDistSq) {
      bestDistSq = dsq;
      best = p;
      u = bu;
      v = bv;
      w = bw;
      x = 0.0f;
    }
  }
  {
    float bu, bv, bw;
    Vector3 p = closestPointOnTriangleToOrigin(a, b, d, bu, bv, bw);
    float dsq = p.lengthSquare();
    if (dsq < bestDistSq) {
      bestDistSq = dsq;
      best = p;
      u = bu;
      v = bv;
      w = 0.0f;
      x = bw;
    }
  }
  {
    float bu, bv, bw;
    Vector3 p = closestPointOnTriangleToOrigin(a, c, d, bu, bv, bw);
    float dsq = p.lengthSquare();
    if (dsq < bestDistSq) {
      bestDistSq = dsq;
      best = p;
      u = bu;
      v = 0.0f;
      w = bv;
      x = bw;
    }
  }
  {
    float bu, bv, bw;
    Vector3 p = closestPointOnTriangleToOrigin(b, c, d, bu, bv, bw);
    float dsq = p.lengthSquare();
    if (dsq < bestDistSq) {
      bestDistSq = dsq;
      best = p;
      u = 0.0f;
      v = bu;
      w = bv;
      x = bw;
    }
  }

  float eu, ev;
  Vector3 pe = closestPointOnSegmentToOrigin(a, b, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = eu;
    v = ev;
    w = x = 0.0f;
    bestDistSq = pe.lengthSquare();
  }
  pe = closestPointOnSegmentToOrigin(a, c, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = eu;
    v = 0.0f;
    w = ev;
    x = 0.0f;
    bestDistSq = pe.lengthSquare();
  }
  pe = closestPointOnSegmentToOrigin(a, d, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = eu;
    v = w = 0.0f;
    x = ev;
    bestDistSq = pe.lengthSquare();
  }
  pe = closestPointOnSegmentToOrigin(b, c, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = 0.0f;
    v = eu;
    w = ev;
    x = 0.0f;
    bestDistSq = pe.lengthSquare();
  }
  pe = closestPointOnSegmentToOrigin(b, d, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = 0.0f;
    v = eu;
    w = 0.0f;
    x = ev;
    bestDistSq = pe.lengthSquare();
  }
  pe = closestPointOnSegmentToOrigin(c, d, eu, ev);
  if (pe.lengthSquare() < bestDistSq) {
    best = pe;
    u = 0.0f;
    v = eu;
    w = 0.0f;
    x = ev;
  }

  return best;
}

/**
 * @brief Minkowski uzayındaki koordinatları kullanarak orijinal şekiller (A ve
 * B) üzerindeki gerçek temas noktalarını hesaplar.
 */
static void gjkWitnessFromBarycentric(const GjkWitness *s, float bu, float bv,
                                      float bw, float bx, Vector3 &outA,
                                      Vector3 &outB) {
  outA = s[0].supA * bu + s[1].supA * bv + s[2].supA * bw + s[3].supA * bx;
  outB = s[0].supB * bu + s[1].supB * bv + s[2].supB * bw + s[3].supB * bx;
}

/**
 * @brief İki şekil arasındaki en kısa mesafeyi bulan GJK varyantı.
 * @details Sürekli çarpışma algılama (CCD) sistemi için mesafe sorgularında
 * kullanılır.
 */
static float gjkDistanceImpl(const CollisionShape &shapeA,
                             const Pose &transformA,
                             const CollisionShape &shapeB,
                             const Pose &transformB, Vector3 &outPointA,
                             Vector3 &outPointB) {
  GjkWitness simplex[4];
  int n = 0;

  Vector3 dir = transformA.position - transformB.position;
  if (dir.lengthSquare() < 1e-12f)
    dir = Vector3(1.0f, 0.0f, 0.0f);
  else
    dir.normalize();

  computeSupport(shapeA, transformA, shapeB, transformB, dir, simplex[0]);
  n = 1;

  constexpr int kMaxIter = 64;
  constexpr float kConvergence = 1e-5f;
  constexpr float kEpsilon = 1e-8f;
  constexpr float kEpsR = 1e-6f;

  for (int iter = 0; iter < kMaxIter; ++iter) {
    Vector3 closest;
    float bu = 1.0f, bv = 0.0f, bw = 0.0f, bx = 0.0f;

    if (n == 1) {
      closest = simplex[0].w;
      bu = 1.0f;
    } else if (n == 2) {
      float uSeg, vSeg;
      closest =
          closestPointOnSegmentToOrigin(simplex[0].w, simplex[1].w, uSeg, vSeg);
      bu = uSeg;
      bv = vSeg;
    } else if (n == 3) {
      closest = closestPointOnTriangleToOrigin(simplex[0].w, simplex[1].w,
                                               simplex[2].w, bu, bv, bw);
    } else {
      if (originInsideTetrahedron(simplex[0].w, simplex[1].w, simplex[2].w,
                                  simplex[3].w)) {
        outPointA = simplex[0].supA;
        outPointB = simplex[0].supB;
        return 0.0f;
      }
      closest = closestPointOnTetrahedronToOrigin(simplex[0].w, simplex[1].w,
                                                  simplex[2].w, simplex[3].w,
                                                  bu, bv, bw, bx);
    }

    const float distSq = closest.lengthSquare();
    if (distSq < kEpsilon * kEpsilon) {
      gjkWitnessFromBarycentric(simplex, bu, bv, bw, bx, outPointA, outPointB);
      return 0.0f;
    }

    dir = (-closest).getNormalized();
    GjkWitness wnew;
    computeSupport(shapeA, transformA, shapeB, transformB, dir, wnew);

    const float delta = wnew.w.dot(dir) - closest.dot(dir);
    if (delta < kConvergence) {
      gjkWitnessFromBarycentric(simplex, bu, bv, bw, bx, outPointA, outPointB);
      return std::sqrt(distSq);
    }

    for (int i = 0; i < n; ++i) {
      if ((wnew.w - simplex[i].w).lengthSquare() < 1e-10f) {
        gjkWitnessFromBarycentric(simplex, bu, bv, bw, bx, outPointA,
                                  outPointB);
        return std::sqrt(distSq);
      }
    }

    simplex[n++] = wnew;

    if (n == 4) {
      if (originInsideTetrahedron(simplex[0].w, simplex[1].w, simplex[2].w,
                                  simplex[3].w)) {
        outPointA = simplex[0].supA;
        outPointB = simplex[0].supB;
        return 0.0f;
      }
      closest = closestPointOnTetrahedronToOrigin(simplex[0].w, simplex[1].w,
                                                  simplex[2].w, simplex[3].w,
                                                  bu, bv, bw, bx);
    } else if (n == 3) {
      closest = closestPointOnTriangleToOrigin(simplex[0].w, simplex[1].w,
                                               simplex[2].w, bu, bv, bw);
      bx = 0.0f;
    } else if (n == 2) {
      float uSeg, vSeg;
      closest =
          closestPointOnSegmentToOrigin(simplex[0].w, simplex[1].w, uSeg, vSeg);
      bu = uSeg;
      bv = vSeg;
      bw = bx = 0.0f;
    }

    GjkWitness newS[4];
    int k = 0;
    if (bu > kEpsR)
      newS[k++] = simplex[0];
    if (n >= 2 && bv > kEpsR)
      newS[k++] = simplex[1];
    if (n >= 3 && bw > kEpsR)
      newS[k++] = simplex[2];
    if (n >= 4 && bx > kEpsR)
      newS[k++] = simplex[3];
    if (k == 0) {
      newS[0] = wnew;
      k = 1;
    }
    for (int i = 0; i < k; ++i)
      simplex[i] = newS[i];
    n = k;
  }

  outPointA = transformA.position;
  outPointB = transformB.position;
  return 0.0f;
}

} // namespace

/**
 * @brief EPA (Expanding Polytope Algorithm) için nokta ve yüzey tanımları.
 */
struct EPAVertex {
  Vector3 minkowskiPoint; ///< Minkowski uzayındaki nokta koordinatı
  Vector3
      supportA; ///< Temas noktasını geri hesaplamak için A şeklindeki karşılığı
};

struct EPAFace {
  size_t v1, v2, v3; ///< Yüzeyi oluşturan tepe noktalarının indeksleri
  Vector3 normal;    ///< Yüzeyin dışarı bakan dik vektörü
  float distance;    ///< Orijine (merkeze) olan en kısa mesafe
};

float NarrowPhase::GJK_distance(const CollisionShape &shapeA,
                                const Pose &transformA,
                                const CollisionShape &shapeB,
                                const Pose &transformB, Vector3 &outPointA,
                                Vector3 &outPointB) {
  return gjkDistanceImpl(shapeA, transformA, shapeB, transformB, outPointA,
                         outPointB);
}

/**
 * @brief Geometrik şekillerin destek noktasını (en uç nokta) hesaplar.
 * @details GJK ve EPA algoritmalarının temelini oluşturur. Şekil tipine göre
 *          (Küre, Kutu, Kapsül, Konveks, Üçgen) özel matematiksel çözümler
 * sunar.
 */
Vector3 NarrowPhase::getSupportPoint(const CollisionShape &shape,
                                     const Pose &transform,
                                     const Vector3 &direction) {
  return std::visit(
      [&transform, &direction](auto &&arg) -> Vector3 {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, SphereShape>) {
          Vector3 dirNorm = direction;
          if (dirNorm.lengthSquare() > 0.0001f)
            dirNorm.normalize();
          return transform.position + (dirNorm * arg.radius);
        } else if constexpr (std::is_same_v<T, BoxShape>) {
          Quaternion invRot = transform.orientation.getConjugate();
          Vector3 localDir = invRot * direction;
          Vector3 localResult;
          localResult.x =
              (localDir.x > 0.0f) ? arg.halfExtents.x : -arg.halfExtents.x;
          localResult.y =
              (localDir.y > 0.0f) ? arg.halfExtents.y : -arg.halfExtents.y;
          localResult.z =
              (localDir.z > 0.0f) ? arg.halfExtents.z : -arg.halfExtents.z;
          return transform.position + (transform.orientation * localResult);
        } else if constexpr (std::is_same_v<T, CapsuleShape>) {
          Quaternion invRot = transform.orientation.getConjugate();
          Vector3 localDir = invRot * direction;
          float halfH = arg.height * 0.5f;
          Vector3 top(0, halfH, 0);
          Vector3 bottom(0, -halfH, 0);
          Vector3 point =
              (localDir.dot(top) > localDir.dot(bottom)) ? top : bottom;
          Vector3 localResult = point + (localDir.getNormalized() * arg.radius);
          return transform.position + (transform.orientation * localResult);
        } else if constexpr (std::is_same_v<T, ConvexHullShape>) {
          if (arg.vertices.empty())
            return transform.position;
          Quaternion invRot = transform.orientation.getConjugate();
          Vector3 localDir = invRot * direction;
          float maxDot = -1e10f;
          Vector3 bestVertex = arg.vertices[0];
          for (const auto &v : arg.vertices) {
            float d = v.dot(localDir);
            if (d > maxDot) {
              maxDot = d;
              bestVertex = v;
            }
          }
          return transform.position + (transform.orientation * bestVertex);
        } else if constexpr (std::is_same_v<T, TriangleShape>) {
          Quaternion invRot = transform.orientation.getConjugate();
          Vector3 localDir = invRot * direction;
          float d0 = arg.v0.dot(localDir), d1 = arg.v1.dot(localDir),
                d2 = arg.v2.dot(localDir);
          Vector3 bestLocal = arg.v0;
          float maxD = d0;
          if (d1 > maxD) {
            maxD = d1;
            bestLocal = arg.v1;
          }
          if (d2 > maxD) {
            maxD = d2;
            bestLocal = arg.v2;
          }
          return transform.position + (transform.orientation * bestLocal);
        }
        return transform.position;
      },
      shape.getVariant());
}

/**
 * @brief Işın (Ray) ile geometrik şekiller arasında kesişim testi yapar.
 * @details Küre için analitik, Kutu için OBB (Oriented Bounding Box) testi
 * uygular.
 */
bool NarrowPhase::raycast(const CollisionShape &shape, const Pose &transform,
                          const Ray &ray, RaycastHit &hit) {
  return std::visit(
      [&](const auto &s) -> bool {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, SphereShape>) {
          Vector3 m = ray.origin - transform.position;
          float b = m.dot(ray.direction);
          float c = m.dot(m) - s.radius * s.radius;
          if (c > 0.0f && b > 0.0f) return false;
          float discr = b * b - c;
          if (discr < 0.0f) return false;
          float t = -b - std::sqrt(discr);
          if (t < 0.0f) t = 0.0f;
          if (t > ray.maxDistance) return false;
          hit.hasHit = true;
          hit.distance = t;
          hit.position = ray.origin + ray.direction * t;
          hit.normal = (hit.position - transform.position).getNormalized();
          return true;
        } else if constexpr (std::is_same_v<T, BoxShape>) {
          Vector3 p = transform.getInverse() * ray.origin;
          Vector3 d = transform.orientation.getInverse() * ray.direction;
          float tmin = 0.0f;
          float tmax = ray.maxDistance;
          Vector3 normal(0, 0, 0);

          for (int i = 0; i < 3; ++i) {
            float invD = 1.0f / (std::abs(d[i]) > 1e-6f ? d[i] : (d[i] < 0.0f ? -1e-6f : 1e-6f));
            float t0 = (-s.halfExtents[i] - p[i]) * invD;
            float t1 = (s.halfExtents[i] - p[i]) * invD;
            Vector3 signNormal(0, 0, 0);
            signNormal[i] = -1.0f;
            if (invD < 0.0f) {
              std::swap(t0, t1);
              signNormal[i] = 1.0f;
            }
            if (t0 > tmin) {
              tmin = t0;
              normal = signNormal;
            }
            tmax = std::min(tmax, t1);
            if (tmax < tmin) return false;
          }
          if (tmin > ray.maxDistance) return false;
          hit.hasHit = true;
          hit.distance = tmin;
          hit.position = ray.origin + ray.direction * tmin;
          hit.normal = (transform.orientation * normal).getNormalized();
          return true;
        } else if constexpr (std::is_same_v<T, CapsuleShape>) {
          Vector3 p = transform.getInverse() * ray.origin;
          Vector3 d = transform.orientation.getInverse() * ray.direction;
          float halfH = s.height * 0.5f;
          float radius = s.radius;

          float closestT = ray.maxDistance + 1.0f;
          Vector3 localHitNorm(0, 1, 0);
          bool hitAny = false;

          float a = d.x * d.x + d.z * d.z;
          float b = p.x * d.x + p.z * d.z;
          float c = p.x * p.x + p.z * p.z - radius * radius;
          if (a > 1e-7f) {
            float discr = b * b - a * c;
            if (discr >= 0.0f) {
              float sqrtD = std::sqrt(discr);
              float t0 = (-b - sqrtD) / a;
              float t1 = (-b + sqrtD) / a;
              for (float tCandidate : {t0, t1}) {
                if (tCandidate > 0.0f && tCandidate <= ray.maxDistance) {
                  float yAtHit = p.y + d.y * tCandidate;
                  if (yAtHit >= -halfH && yAtHit <= halfH) {
                    if (tCandidate < closestT) {
                      closestT = tCandidate;
                      localHitNorm = Vector3(p.x + d.x * tCandidate, 0.0f, p.z + d.z * tCandidate).getNormalized();
                      hitAny = true;
                    }
                  }
                }
              }
            }
          }

          for (float capY : {halfH, -halfH}) {
            Vector3 center(0.0f, capY, 0.0f);
            Vector3 m = p - center;
            float bSphere = m.dot(d);
            float cSphere = m.dot(m) - radius * radius;
            float discrSphere = bSphere * bSphere - cSphere;
            if (discrSphere >= 0.0f) {
              float tSphere = -bSphere - std::sqrt(discrSphere);
              if (tSphere > 0.0f && tSphere <= ray.maxDistance) {
                Vector3 hitP = p + d * tSphere;
                if ((capY > 0.0f && hitP.y >= halfH) || (capY < 0.0f && hitP.y <= -halfH)) {
                  if (tSphere < closestT) {
                    closestT = tSphere;
                    localHitNorm = (hitP - center).getNormalized();
                    hitAny = true;
                  }
                }
              }
            }
          }

          if (hitAny && closestT <= ray.maxDistance) {
            hit.hasHit = true;
            hit.distance = closestT;
            hit.position = ray.origin + ray.direction * closestT;
            hit.normal = (transform.orientation * localHitNorm).getNormalized();
            return true;
          }
          return false;
        } else if constexpr (std::is_same_v<T, TriangleShape>) {
          Vector3 v0 = transform.position + (transform.orientation * s.v0);
          Vector3 v1 = transform.position + (transform.orientation * s.v1);
          Vector3 v2 = transform.position + (transform.orientation * s.v2);
          Vector3 edge1 = v1 - v0;
          Vector3 edge2 = v2 - v0;
          Vector3 h = ray.direction.cross(edge2);
          float a = edge1.dot(h);
          if (std::abs(a) < 1e-7f) return false;
          float f = 1.0f / a;
          Vector3 sVec = ray.origin - v0;
          float u = f * sVec.dot(h);
          if (u < 0.0f || u > 1.0f) return false;
          Vector3 q = sVec.cross(edge1);
          float v = f * ray.direction.dot(q);
          if (v < 0.0f || u + v > 1.0f) return false;
          float t = f * edge2.dot(q);
          if (t > 1e-4f && t <= ray.maxDistance) {
            hit.hasHit = true;
            hit.distance = t;
            hit.position = ray.origin + ray.direction * t;
            hit.normal = edge1.cross(edge2).getNormalized();
            if (hit.normal.dot(ray.direction) > 0.0f) hit.normal = -hit.normal;
            return true;
          }
          return false;
        } else if constexpr (std::is_same_v<T, StaticMeshShape>) {
          if (!s.mesh) return false;
          if (!s.mesh->getBounds().testCollision(ray.computeAABB())) return false;

          bool hitAny = false;
          float closestT = ray.maxDistance;
          Vector3 hitNorm(0, 1, 0);

          s.mesh->queryTriangles(ray.computeAABB(), [&](const TriangleShape &tri) {
            Vector3 v0 = transform.position + (transform.orientation * tri.v0);
            Vector3 v1 = transform.position + (transform.orientation * tri.v1);
            Vector3 v2 = transform.position + (transform.orientation * tri.v2);
            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 h = ray.direction.cross(edge2);
            float a = edge1.dot(h);
            if (std::abs(a) < 1e-7f) return;
            float f = 1.0f / a;
            Vector3 sVec = ray.origin - v0;
            float u = f * sVec.dot(h);
            if (u < 0.0f || u > 1.0f) return;
            Vector3 q = sVec.cross(edge1);
            float v = f * ray.direction.dot(q);
            if (v < 0.0f || u + v > 1.0f) return;
            float t = f * edge2.dot(q);
            if (t > 1e-4f && t < closestT) {
              closestT = t;
              hitNorm = edge1.cross(edge2).getNormalized();
              if (hitNorm.dot(ray.direction) > 0.0f) hitNorm = -hitNorm;
              hitAny = true;
            }
          });

          if (hitAny) {
            hit.hasHit = true;
            hit.distance = closestT;
            hit.position = ray.origin + ray.direction * closestT;
            hit.normal = hitNorm;
            return true;
          }
          return false;
        }
        return false;
      },
      shape.getVariant());
}


bool NarrowPhase::testSphereSphere(const SphereShape &sphereA, const Pose &transformA,
                                   const SphereShape &sphereB, const Pose &transformB,
                                   CollisionInfo &outInfo) {
  Vector3 delta = transformA.position - transformB.position;
  float distSq = delta.lengthSquare();
  float totalRadius = sphereA.radius + sphereB.radius;
  if (distSq >= totalRadius * totalRadius) {
    return false;
  }
  float dist = std::sqrt(distSq);
  outInfo.hasCollision = true;
  if (dist > 1e-6f) {
    outInfo.normal = delta / dist; // B'den A'ya doğru bakar
    outInfo.penetration = totalRadius - dist;
    outInfo.contactPoint = transformB.position + outInfo.normal * (sphereB.radius - outInfo.penetration * 0.5f);
  } else {
    outInfo.normal = Vector3(0.0f, 1.0f, 0.0f);
    outInfo.penetration = totalRadius;
    outInfo.contactPoint = transformB.position;
  }
  return true;
}

bool NarrowPhase::testSphereBox(const SphereShape &sphereA, const Pose &transformA,
                               const BoxShape &boxB, const Pose &transformB,
                               CollisionInfo &outInfo) {
  Quaternion invRotB = transformB.orientation.getConjugate();
  Vector3 relPos = transformA.position - transformB.position;
  Vector3 localSpherePos = invRotB * relPos;

  Vector3 closestPointLocal(
      std::clamp(localSpherePos.x, -boxB.halfExtents.x, boxB.halfExtents.x),
      std::clamp(localSpherePos.y, -boxB.halfExtents.y, boxB.halfExtents.y),
      std::clamp(localSpherePos.z, -boxB.halfExtents.z, boxB.halfExtents.z));

  Vector3 localDelta = localSpherePos - closestPointLocal;
  float distSq = localDelta.lengthSquare();

  if (distSq > sphereA.radius * sphereA.radius) {
    return false;
  }

  outInfo.hasCollision = true;
  if (distSq > 1e-8f) {
    float dist = std::sqrt(distSq);
    Vector3 localNormal = localDelta / dist;
    outInfo.normal = transformB.orientation * localNormal; // B'den A'ya doğru bakar
    outInfo.penetration = sphereA.radius - dist;
    outInfo.contactPoint = transformB.position + (transformB.orientation * closestPointLocal);
  } else {
    // Küre merkezi kutunun içinde kalmışsa en yakın yüze doğru it
    Vector3 d(boxB.halfExtents.x - std::abs(localSpherePos.x),
              boxB.halfExtents.y - std::abs(localSpherePos.y),
              boxB.halfExtents.z - std::abs(localSpherePos.z));
    Vector3 localNormal(0, 1, 0);
    float minD = d.y;
    if (d.x < minD) {
      minD = d.x;
      localNormal = Vector3((localSpherePos.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f);
    }
    if (d.y < minD) {
      minD = d.y;
      localNormal = Vector3(0.0f, (localSpherePos.y >= 0.0f) ? 1.0f : -1.0f, 0.0f);
    }
    if (d.z < minD) {
      minD = d.z;
      localNormal = Vector3(0.0f, 0.0f, (localSpherePos.z >= 0.0f) ? 1.0f : -1.0f);
    }
    outInfo.normal = transformB.orientation * localNormal;
    outInfo.penetration = sphereA.radius + minD;
    outInfo.contactPoint = transformA.position - (outInfo.normal * sphereA.radius);
  }
  return true;
}

namespace {

struct ClipVertex {
    Vector3 point;
    float depth{0.0f};
};

static uint32_t clipPolygonWithPlane(const ClipVertex inVerts[], uint32_t inCount,
                                     const Vector3& planeOrigin, const Vector3& planeNormal,
                                     ClipVertex outVerts[]) {
    uint32_t outCount = 0;
    if (inCount == 0) return 0;

    for (uint32_t i = 0; i < inCount; ++i) {
        const ClipVertex& v1 = inVerts[i];
        const ClipVertex& v2 = inVerts[(i + 1) % inCount];

        float d1 = (v1.point - planeOrigin).dot(planeNormal);
        float d2 = (v2.point - planeOrigin).dot(planeNormal);

        if (d1 <= 0.0f) {
            outVerts[outCount++] = v1;
        }
        if ((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) {
            float t = d1 / (d1 - d2);
            ClipVertex vInter;
            vInter.point = v1.point + (v2.point - v1.point) * t;
            outVerts[outCount++] = vInter;
        }
    }
    return outCount;
}

} // namespace

bool NarrowPhase::testBoxBox(const BoxShape& boxA, const Pose& transformA,
                             const BoxShape& boxB, const Pose& transformB,
                             ContactManifold& outManifold) {
    Vector3 axesA[3] = {
        transformA.orientation * Vector3(1, 0, 0),
        transformA.orientation * Vector3(0, 1, 0),
        transformA.orientation * Vector3(0, 0, 1)
    };
    Vector3 axesB[3] = {
        transformB.orientation * Vector3(1, 0, 0),
        transformB.orientation * Vector3(0, 1, 0),
        transformB.orientation * Vector3(0, 0, 1)
    };
    Vector3 extA = boxA.halfExtents;
    Vector3 extB = boxB.halfExtents;
    Vector3 D = transformA.position - transformB.position; // Points B to A

    float minPenetration = 1e30f;
    Vector3 bestAxis(0, 0, 0);
    int bestAxisIndex = -1;

    auto testAxis = [&](const Vector3& rawAxis, int axisId, float bias = 1.0f) -> bool {
        float lenSq = rawAxis.lengthSquare();
        if (lenSq < 1e-6f) return true;
        Vector3 L = rawAxis * (1.0f / std::sqrt(lenSq));
        if (L.dot(D) < 0.0f) L = -L;

        float rA = extA.x * std::abs(axesA[0].dot(L)) +
                   extA.y * std::abs(axesA[1].dot(L)) +
                   extA.z * std::abs(axesA[2].dot(L));
        float rB = extB.x * std::abs(axesB[0].dot(L)) +
                   extB.y * std::abs(axesB[1].dot(L)) +
                   extB.z * std::abs(axesB[2].dot(L));
        float s = std::abs(D.dot(L));
        float pen = (rA + rB) - s;
        if (pen <= 0.0f) return false;

        if (pen * bias < minPenetration) {
            minPenetration = pen * bias;
            bestAxis = L;
            bestAxisIndex = axisId;
        }
        return true;
    };

    // 1. Face testleri: A kutusunun yüzeyleri
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(axesA[i], i)) return false;
    }
    // 2. Face testleri: B kutusunun yüzeyleri
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(axesB[i], 3 + i)) return false;
    }
    // 3. Kenar-Kenar testleri: A x B
    int edgeIdx = 6;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3 edgeAxis = axesA[i].cross(axesB[j]);
            if (!testAxis(edgeAxis, edgeIdx++, 1.02f)) return false;
        }
    }

    outManifold.normal = bestAxis;
    outManifold.penetration = minPenetration;

    // Yüzey teması: Sutherland-Hodgman yüzey kırpma
    if (bestAxisIndex < 6) {
        bool refIsA = (bestAxisIndex < 3);
        const Pose& refPose = refIsA ? transformA : transformB;
        const Pose& incPose = refIsA ? transformB : transformA;
        const Vector3& refExt = refIsA ? extA : extB;
        const Vector3& incExt = refIsA ? extB : extA;
        const Vector3* refAxes = refIsA ? axesA : axesB;
        const Vector3* incAxes = refIsA ? axesB : axesA;

        Vector3 refFaceNormal = refIsA ? -bestAxis : bestAxis;
        int refFaceAxisIdx = refIsA ? bestAxisIndex : (bestAxisIndex - 3);
        Vector3 refAxis = refAxes[refFaceAxisIdx];
        if (refAxis.dot(refFaceNormal) < 0.0f) refAxis = -refAxis;
        refFaceNormal = refAxis;

        int incFaceAxisIdx = 0;
        float minDot = 1e30f;
        for (int i = 0; i < 3; ++i) {
            float d1 = incAxes[i].dot(refFaceNormal);
            float d2 = (-incAxes[i]).dot(refFaceNormal);
            if (d1 < minDot) { minDot = d1; incFaceAxisIdx = i; }
            if (d2 < minDot) { minDot = d2; incFaceAxisIdx = i; }
        }
        Vector3 incFaceNormal = incAxes[incFaceAxisIdx];
        if (incFaceNormal.dot(refFaceNormal) > 0.0f) incFaceNormal = -incFaceNormal;

        Vector3 incU, incV;
        if (incFaceAxisIdx == 0) {
            incU = incAxes[1] * incExt.y;
            incV = incAxes[2] * incExt.z;
        } else if (incFaceAxisIdx == 1) {
            incU = incAxes[0] * incExt.x;
            incV = incAxes[2] * incExt.z;
        } else {
            incU = incAxes[0] * incExt.x;
            incV = incAxes[1] * incExt.y;
        }
        float incDist = (incFaceAxisIdx == 0 ? incExt.x : (incFaceAxisIdx == 1 ? incExt.y : incExt.z));
        Vector3 incFaceCenter = incPose.position + incFaceNormal * incDist;

        ClipVertex poly1[8];
        ClipVertex poly2[8];
        poly1[0].point = incFaceCenter + incU + incV;
        poly1[1].point = incFaceCenter - incU + incV;
        poly1[2].point = incFaceCenter - incU - incV;
        poly1[3].point = incFaceCenter + incU - incV;
        uint32_t polyCount = 4;

        Vector3 refU, refV;
        float refHalfU, refHalfV;
        if (refFaceAxisIdx == 0) {
            refU = refAxes[1]; refHalfU = refExt.y;
            refV = refAxes[2]; refHalfV = refExt.z;
        } else if (refFaceAxisIdx == 1) {
            refU = refAxes[0]; refHalfU = refExt.x;
            refV = refAxes[2]; refHalfV = refExt.z;
        } else {
            refU = refAxes[0]; refHalfU = refExt.x;
            refV = refAxes[1]; refHalfV = refExt.y;
        }

        Vector3 sidePlanes[4][2] = {
            { refPose.position + refU * refHalfU, refU },
            { refPose.position - refU * refHalfU, -refU },
            { refPose.position + refV * refHalfV, refV },
            { refPose.position - refV * refHalfV, -refV }
        };

        for (int i = 0; i < 4; ++i) {
            polyCount = clipPolygonWithPlane(poly1, polyCount, sidePlanes[i][0], sidePlanes[i][1], poly2);
            for (uint32_t k = 0; k < polyCount; ++k) poly1[k] = poly2[k];
        }

        float refDist = (refFaceAxisIdx == 0 ? refExt.x : (refFaceAxisIdx == 1 ? refExt.y : refExt.z));
        Vector3 refFaceCenter = refPose.position + refFaceNormal * refDist;
        for (uint32_t i = 0; i < polyCount && outManifold.contactCount < 4; ++i) {
            float depth = (refFaceCenter - poly1[i].point).dot(refFaceNormal);
            if (depth >= -0.005f) {
                ContactPoint cp;
                cp.worldPosition = poly1[i].point;
                cp.penetration = std::max(0.0f, depth);
                cp.localPointA = transformA.orientation.getConjugate() * (cp.worldPosition - transformA.position);
                cp.localPointB = transformB.orientation.getConjugate() * (cp.worldPosition - transformB.position);
                cp.normalImpulse = 0.0f;
                cp.tangentImpulse1 = 0.0f;
                cp.tangentImpulse2 = 0.0f;
                cp.splitImpulse = 0.0f;
                outManifold.addContact(cp);
            }
        }
    }

    if (outManifold.contactCount == 0) {
        ContactPoint cp;
        Vector3 pA = NarrowPhase::getSupportPoint(CollisionShape(boxA), transformA, -bestAxis);
        Vector3 pB = NarrowPhase::getSupportPoint(CollisionShape(boxB), transformB, bestAxis);
        cp.worldPosition = (pA + pB) * 0.5f;
        cp.penetration = minPenetration;
        cp.localPointA = transformA.orientation.getConjugate() * (cp.worldPosition - transformA.position);
        cp.localPointB = transformB.orientation.getConjugate() * (cp.worldPosition - transformB.position);
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse1 = 0.0f;
        cp.tangentImpulse2 = 0.0f;
        cp.splitImpulse = 0.0f;
        outManifold.addContact(cp);
    }

    outManifold.computeTangents();
    return true;
}

/**
 * @brief Standart GJK çarpışma testi.
 * @details Sadece çarpışma olup olmadığını hızlıca kontrol eder. EPA'nın
 * çalışması için gerekli olan başlangıç dörtyüzlüsünü (Simpleks) oluşturur.
 */
bool NarrowPhase::GJK(const CollisionShape &shapeA, const Pose &transformA,
                      const CollisionShape &shapeB, const Pose &transformB,
                      Simplex &outSimplex) {
  Vector3 direction = transformB.position - transformA.position;
  if (direction.lengthSquare() < 0.0001f)
    direction = Vector3(1.0f, 0.0f, 0.0f);

  outSimplex = Simplex();
  Vector3 support = getSupportPoint(shapeA, transformA, direction) -
                    getSupportPoint(shapeB, transformB, direction * -1.0f);

  outSimplex.push_front(support);
  direction = support * -1.0f;

  for (int i = 0; i < 64; ++i) {
    support = getSupportPoint(shapeA, transformA, direction) -
              getSupportPoint(shapeB, transformB, direction * -1.0f);
    if (support.dot(direction) < 0.0f)
      return false;
    outSimplex.push_front(support);
    if (handleSimplex(outSimplex, direction))
      return true;
  }
  return false;
}

/**
 * @brief GJK simpleks yapısını günceller ve bir sonraki arama yönünü belirler.
 */
bool NarrowPhase::handleSimplex(Simplex &simplex, Vector3 &direction) {
  if (simplex.size() == 2) {
    Vector3 a = simplex[0], b = simplex[1];
    Vector3 ab = b - a, ao = a * -1.0f;
    Vector3 cross = ab.cross(ao);
    if (cross.lengthSquare() < 1e-8f) {
      Vector3 perp = (std::abs(ab.x) < 0.85f * ab.length()) ? Vector3(1.0f, 0.0f, 0.0f) : Vector3(0.0f, 1.0f, 0.0f);
      direction = ab.cross(perp);
      if (direction.lengthSquare() < 1e-8f) {
        direction = Vector3(0.0f, 0.0f, 1.0f);
      }
    } else {
      direction = cross.cross(ab);
    }
  } else if (simplex.size() == 3) {
    Vector3 a = simplex[0], b = simplex[1], c = simplex[2];
    Vector3 ab = b - a, ac = c - a, ao = a * -1.0f;
    Vector3 abc = ab.cross(ac);
    if (abc.cross(ac).dot(ao) > 0.0f) {
      if (ac.dot(ao) > 0.0f) {
        simplex.assign({a, c});
        direction = ac.cross(ao).cross(ac);
      } else {
        if (ab.dot(ao) > 0.0f) {
          simplex.assign({a, b});
          direction = ab.cross(ao).cross(ab);
        } else {
          simplex.assign({a});
          direction = ao;
        }
      }
    } else {
      if (ab.cross(abc).dot(ao) > 0.0f) {
        if (ab.dot(ao) > 0.0f) {
          simplex.assign({a, b});
          direction = ab.cross(ao).cross(ab);
        } else {
          simplex.assign({a});
          direction = ao;
        }
      } else {
        if (abc.dot(ao) > 0.0f) {
          direction = abc;
        } else {
          simplex.assign({a, c, b});
          direction = abc * -1.0f;
        }
      }
    }
  } else {
    Vector3 a = simplex[0], b = simplex[1], c = simplex[2], d = simplex[3];
    Vector3 ab = b - a, ac = c - a, ad = d - a, ao = a * -1.0f;
    Vector3 abc = ab.cross(ac), acd = ac.cross(ad), adb = ad.cross(ab);
    if (abc.dot(ao) > 0.0f) {
      simplex.assign({a, b, c});
      return handleSimplex(simplex, direction);
    }
    if (acd.dot(ao) > 0.0f) {
      simplex.assign({a, c, d});
      return handleSimplex(simplex, direction);
    }
    if (adb.dot(ao) > 0.0f) {
      simplex.assign({a, d, b});
      return handleSimplex(simplex, direction);
    }
    return true;
  }
  return false;
}

/**
 * @brief EPA (Expanding Polytope Algorithm) ile çarpışma detaylarını hesaplar.
 * @details Cisimlerin ne kadar iç içe girdiğini ve hangi yöne itilmeleri
 * gerektiğini bulur. Performans için bellek tahsisatı (allocation) yerine
 * `thread_local` tamponlar kullanır.
 */
CollisionInfo NarrowPhase::EPA(const Simplex &simplex,
                               const CollisionShape &shapeA,
                               const Pose &transformA,
                               const CollisionShape &shapeB,
                               const Pose &transformB) {
  thread_local std::vector<EPAVertex> vertices;
  thread_local std::vector<EPAFace> faces;
  thread_local std::vector<std::pair<size_t, size_t>> edges;

  vertices.clear();
  faces.clear();
  if (vertices.capacity() < 96)
    vertices.reserve(96);
  if (faces.capacity() < 192)
    faces.reserve(192);
  if (edges.capacity() < 96)
    edges.reserve(96);

  for (int i = 0; i < simplex.size(); ++i) {
    Vector3 p = simplex[i];
    vertices.push_back({p, getSupportPoint(shapeA, transformA, p)});
  }

  auto getFace = [&](size_t i1, size_t i2, size_t i3) {
    Vector3 v1 = vertices[i1].minkowskiPoint, v2 = vertices[i2].minkowskiPoint,
            v3 = vertices[i3].minkowskiPoint;
    Vector3 n = (v2 - v1).cross(v3 - v1).getNormalized();
    float d = n.dot(v1);
    if (d < 0) {
      n = -n;
      d = -d;
      std::swap(i1, i2);
    }
    return EPAFace{i1, i2, i3, n, d};
  };

  faces.push_back(getFace(0, 1, 2));
  faces.push_back(getFace(0, 2, 3));
  faces.push_back(getFace(0, 3, 1));
  faces.push_back(getFace(1, 3, 2));

  EPAFace closestFace;
  for (int iter = 0; iter < 32; ++iter) {
    float minDistance = 1e10f;
    size_t closestFaceIndex = 0;
    for (size_t i = 0; i < faces.size(); ++i) {
      if (faces[i].distance < minDistance) {
        minDistance = faces[i].distance;
        closestFaceIndex = i;
      }
    }
    closestFace = faces[closestFaceIndex];
    Vector3 searchDir = closestFace.normal;
    Vector3 pA = getSupportPoint(shapeA, transformA, searchDir);
    Vector3 p = pA - getSupportPoint(shapeB, transformB, searchDir * -1.0f);

    if (p.dot(searchDir) - closestFace.distance < 0.001f)
      break;

    vertices.push_back({p, pA});
    size_t newVIdx = vertices.size() - 1;
    edges.clear();
    auto addEdge = [&](size_t a, size_t b) {
      for (auto it = edges.begin(); it != edges.end(); ++it) {
        if (it->first == b && it->second == a) {
          edges.erase(it);
          return;
        }
      }
      edges.push_back({a, b});
    };

    for (size_t i = 0; i < faces.size();) {
      if (faces[i].normal.dot(p - vertices[faces[i].v1].minkowskiPoint) > 0) {
        addEdge(faces[i].v1, faces[i].v2);
        addEdge(faces[i].v2, faces[i].v3);
        addEdge(faces[i].v3, faces[i].v1);
        faces.erase(faces.begin() + i);
      } else
        i++;
    }
    for (auto &edge : edges)
      faces.push_back(getFace(edge.first, edge.second, newVIdx));
  }

  CollisionInfo info;
  info.hasCollision = true;
  info.normal = closestFace.normal;
  info.penetration = closestFace.distance;
  BARYON_LOG_NARROW("EPA Collision Found | Pen: ", info.penetration, " | N: (",
                    info.normal.x, ", ", info.normal.y, ", ", info.normal.z,
                    ")");

  Vector3 v1 = vertices[closestFace.v1].minkowskiPoint,
          v2 = vertices[closestFace.v2].minkowskiPoint,
          v3 = vertices[closestFace.v3].minkowskiPoint;
  Vector3 faceNormal = (v2 - v1).cross(v3 - v1);
  float areaSq = faceNormal.lengthSquare();
  Vector3 pOnFace = closestFace.normal * closestFace.distance;

  float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f;
  if (areaSq > 1e-12f) {
    w1 = faceNormal.dot((v2 - pOnFace).cross(v3 - pOnFace)) / areaSq;
    w2 = faceNormal.dot((v3 - pOnFace).cross(v1 - pOnFace)) / areaSq;
    w3 = 1.0f - w1 - w2;
  } else {
    w1 = 0.3333f;
    w2 = 0.3333f;
    w3 = 0.3334f;
  }

  info.contactPoint = (vertices[closestFace.v1].supportA * w1) +
                      (vertices[closestFace.v2].supportA * w2) +
                      (vertices[closestFace.v3].supportA * w3);
  return info;
}

} // namespace Baryon::collision
