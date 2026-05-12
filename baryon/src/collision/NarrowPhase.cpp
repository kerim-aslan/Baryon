/**
 * @file NarrowPhase.cpp
 * @brief Dar Faz (Narrow Phase) çarpışma algılama algoritmaları.
 * @details GJK algoritması ile cisimlerin kesişimini kontrol eder, kesişim varsa 
 *          EPA algoritması ile çarpışmanın şiddetini (derinlik) ve yönünü (normal) hesaplar.
 *          Ayrıca ışın fırlatma (raycast) ve destek noktası (support point) fonksiyonlarını içerir.
 */

#include "Baryon/collision/NarrowPhase.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace Baryon::collision {

namespace {

/**
 * @brief GJK ve EPA hesaplamaları için Minkowski farkı üzerindeki nokta verileri.
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
static void computeSupport(const CollisionShape& shapeA, const Pose& transformA,
                           const CollisionShape& shapeB, const Pose& transformB,
                           const Vector3& dir, GjkWitness& out) {
    out.supA = NarrowPhase::getSupportPoint(shapeA, transformA, dir);
    out.supB = NarrowPhase::getSupportPoint(shapeB, transformB, dir * -1.0f);
    out.w = out.supA - out.supB;
}

/**
 * @brief Merkezin (0,0,0) bir doğru parçasına en yakın noktasını bulur.
 */
static Vector3 closestPointOnSegmentToOrigin(const Vector3& a, const Vector3& b,
                                             float& u, float& v) {
    Vector3 ab = b - a;
    float denom = ab.lengthSquare();
    if (denom < 1e-20f) {
        u = 1.0f; v = 0.0f;
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
static Vector3 closestPointOnTriangleToOrigin(const Vector3& a, const Vector3& b, const Vector3& c,
                                              float& u, float& v, float& w) {
    const Vector3 ab = b - a;
    const Vector3 ac = c - a;
    const Vector3 ap = -a;

    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        u = 1.0f; v = w = 0.0f;
        return a;
    }

    const Vector3 bp = -b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.0f && d4 <= d3) {
        u = 0.0f; v = 1.0f; w = 0.0f;
        return b;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float denom = d1 - d3;
        const float t = denom > 1e-20f ? d1 / denom : 0.0f;
        u = 1.0f - t; v = t; w = 0.0f;
        return a + ab * t;
    }

    const Vector3 cp = -c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.0f && d5 <= d6) {
        u = 0.0f; v = 0.0f; w = 1.0f;
        return c;
    }

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float denom = d2 - d6;
        const float t = denom > 1e-20f ? d2 / denom : 0.0f;
        u = 1.0f - t; v = 0.0f; w = t;
        return a + ac * t;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float denom = (d4 - d3) + (d5 - d6);
        const float t = denom > 1e-20f ? (d4 - d3) / denom : 0.0f;
        u = 0.0f; v = 1.0f - t; w = t;
        return b + (c - b) * t;
    }

    const float denom = va + vb + vc;
    if (std::abs(denom) < 1e-20f) {
        u = v = w = 1.0f / 3.0f;
        return (a + b + c) * (1.0f / 3.0f);
    }

    const float rv = vb / denom;
    const float rw = vc / denom;
    u = 1.0f - rv - rw; v = rv; w = rw;
    return a + ab * rv + ac * rw;
}

/**
 * @brief Orijinin bir dörtyüzlü (tetrahedron) içinde olup olmadığını kontrol eder.
 */
static bool originInsideTetrahedron(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d) {
    Vector3 n0 = (b - a).cross(c - a);
    if (n0.dot(d - a) > 0.0f) n0 = -n0;
    if (n0.dot(-a) > 0.0f) return false;

    Vector3 n1 = (c - a).cross(d - a);
    if (n1.dot(b - a) > 0.0f) n1 = -n1;
    if (n1.dot(-a) > 0.0f) return false;

    Vector3 n2 = (d - a).cross(b - a);
    if (n2.dot(c - a) > 0.0f) n2 = -n2;
    if (n2.dot(-a) > 0.0f) return false;

    Vector3 n3 = (d - b).cross(c - b);
    if (n3.dot(a - b) > 0.0f) n3 = -n3;
    if (n3.dot(-b) > 0.0f) return false;

    return true;
}

/**
 * @brief Orijinin bir dörtyüzlüye en yakın noktasını ve bu noktanın oranlarını bulur.
 */
static Vector3 closestPointOnTetrahedronToOrigin(const Vector3& a, const Vector3& b,
                                                 const Vector3& c, const Vector3& d,
                                                 float& u, float& v, float& w, float& x) {
    if (originInsideTetrahedron(a, b, c, d)) {
        u = v = w = x = 0.25f;
        return Vector3(0, 0, 0);
    }
    float bestDistSq = 1e30f;
    Vector3 best(0, 0, 0);
    u = 1.0f; v = w = x = 0.0f;

    {
        float bu, bv, bw;
        Vector3 p = closestPointOnTriangleToOrigin(a, b, c, bu, bv, bw);
        float dsq = p.lengthSquare();
        if (dsq < bestDistSq) {
            bestDistSq = dsq; best = p;
            u = bu; v = bv; w = bw; x = 0.0f;
        }
    }
    {
        float bu, bv, bw;
        Vector3 p = closestPointOnTriangleToOrigin(a, b, d, bu, bv, bw);
        float dsq = p.lengthSquare();
        if (dsq < bestDistSq) {
            bestDistSq = dsq; best = p;
            u = bu; v = bv; w = 0.0f; x = bw;
        }
    }
    {
        float bu, bv, bw;
        Vector3 p = closestPointOnTriangleToOrigin(a, c, d, bu, bv, bw);
        float dsq = p.lengthSquare();
        if (dsq < bestDistSq) {
            bestDistSq = dsq; best = p;
            u = bu; v = 0.0f; w = bv; x = bw;
        }
    }
    {
        float bu, bv, bw;
        Vector3 p = closestPointOnTriangleToOrigin(b, c, d, bu, bv, bw);
        float dsq = p.lengthSquare();
        if (dsq < bestDistSq) {
            bestDistSq = dsq; best = p;
            u = 0.0f; v = bu; w = bv; x = bw;
        }
    }

    float eu, ev;
    Vector3 pe = closestPointOnSegmentToOrigin(a, b, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = eu; v = ev; w = x = 0.0f; bestDistSq = pe.lengthSquare();
    }
    pe = closestPointOnSegmentToOrigin(a, c, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = eu; v = 0.0f; w = ev; x = 0.0f; bestDistSq = pe.lengthSquare();
    }
    pe = closestPointOnSegmentToOrigin(a, d, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = eu; v = w = 0.0f; x = ev; bestDistSq = pe.lengthSquare();
    }
    pe = closestPointOnSegmentToOrigin(b, c, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = 0.0f; v = eu; w = ev; x = 0.0f; bestDistSq = pe.lengthSquare();
    }
    pe = closestPointOnSegmentToOrigin(b, d, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = 0.0f; v = eu; w = 0.0f; x = ev; bestDistSq = pe.lengthSquare();
    }
    pe = closestPointOnSegmentToOrigin(c, d, eu, ev);
    if (pe.lengthSquare() < bestDistSq) {
        best = pe; u = 0.0f; v = eu; w = 0.0f; x = ev;
    }

    return best;
}

/**
 * @brief Minkowski uzayındaki koordinatları kullanarak orijinal şekiller (A ve B) 
 *        üzerindeki gerçek temas noktalarını hesaplar.
 */
static void gjkWitnessFromBarycentric(const GjkWitness* s, float bu, float bv, float bw, float bx,
                                    Vector3& outA, Vector3& outB) {
    outA = s[0].supA * bu + s[1].supA * bv + s[2].supA * bw + s[3].supA * bx;
    outB = s[0].supB * bu + s[1].supB * bv + s[2].supB * bw + s[3].supB * bx;
}

/**
 * @brief İki şekil arasındaki en kısa mesafeyi bulan GJK varyantı.
 * @details Sürekli çarpışma algılama (CCD) sistemi için mesafe sorgularında kullanılır.
 */
static float gjkDistanceImpl(const CollisionShape& shapeA, const Pose& transformA,
                             const CollisionShape& shapeB, const Pose& transformB,
                             Vector3& outPointA, Vector3& outPointB) {
    GjkWitness simplex[4];
    int n = 0;

    Vector3 dir = transformA.position - transformB.position;
    if (dir.lengthSquare() < 1e-12f) dir = Vector3(1.0f, 0.0f, 0.0f);
    else dir.normalize();

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
            closest = simplex[0].w; bu = 1.0f;
        } else if (n == 2) {
            float uSeg, vSeg;
            closest = closestPointOnSegmentToOrigin(simplex[0].w, simplex[1].w, uSeg, vSeg);
            bu = uSeg; bv = vSeg;
        } else if (n == 3) {
            closest = closestPointOnTriangleToOrigin(simplex[0].w, simplex[1].w, simplex[2].w, bu, bv, bw);
        } else {
            if (originInsideTetrahedron(simplex[0].w, simplex[1].w, simplex[2].w, simplex[3].w)) {
                outPointA = simplex[0].supA; outPointB = simplex[0].supB;
                return 0.0f;
            }
            closest = closestPointOnTetrahedronToOrigin(simplex[0].w, simplex[1].w, simplex[2].w, simplex[3].w, bu, bv, bw, bx);
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
                gjkWitnessFromBarycentric(simplex, bu, bv, bw, bx, outPointA, outPointB);
                return std::sqrt(distSq);
            }
        }

        simplex[n++] = wnew;

        if (n == 4) {
            if (originInsideTetrahedron(simplex[0].w, simplex[1].w, simplex[2].w, simplex[3].w)) {
                outPointA = simplex[0].supA; outPointB = simplex[0].supB;
                return 0.0f;
            }
            closest = closestPointOnTetrahedronToOrigin(simplex[0].w, simplex[1].w, simplex[2].w, simplex[3].w, bu, bv, bw, bx);
        } else if (n == 3) {
            closest = closestPointOnTriangleToOrigin(simplex[0].w, simplex[1].w, simplex[2].w, bu, bv, bw);
            bx = 0.0f;
        } else if (n == 2) {
            float uSeg, vSeg;
            closest = closestPointOnSegmentToOrigin(simplex[0].w, simplex[1].w, uSeg, vSeg);
            bu = uSeg; bv = vSeg; bw = bx = 0.0f;
        }

        GjkWitness newS[4];
        int k = 0;
        if (bu > kEpsR) newS[k++] = simplex[0];
        if (n >= 2 && bv > kEpsR) newS[k++] = simplex[1];
        if (n >= 3 && bw > kEpsR) newS[k++] = simplex[2];
        if (n >= 4 && bx > kEpsR) newS[k++] = simplex[3];
        if (k == 0) { newS[0] = wnew; k = 1; }
        for (int i = 0; i < k; ++i) simplex[i] = newS[i];
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
    Vector3 supportA;       ///< Temas noktasını geri hesaplamak için A şeklindeki karşılığı
};

struct EPAFace {
    size_t v1, v2, v3; ///< Yüzeyi oluşturan tepe noktalarının indeksleri
    Vector3 normal;    ///< Yüzeyin dışarı bakan dik vektörü
    float distance;    ///< Orijine (merkeze) olan en kısa mesafe
};

float NarrowPhase::GJK_distance(const CollisionShape& shapeA, const Pose& transformA,
                                 const CollisionShape& shapeB, const Pose& transformB,
                                 Vector3& outPointA, Vector3& outPointB) {
    return gjkDistanceImpl(shapeA, transformA, shapeB, transformB, outPointA, outPointB);
}

/**
 * @brief Geometrik şekillerin destek noktasını (en uç nokta) hesaplar.
 * @details GJK ve EPA algoritmalarının temelini oluşturur. Şekil tipine göre 
 *          (Küre, Kutu, Kapsül, Konveks, Üçgen) özel matematiksel çözümler sunar.
 */
Vector3 NarrowPhase::getSupportPoint(const CollisionShape& shape, const Pose& transform, const Vector3& direction) {
    return std::visit([&transform, &direction](auto&& arg) -> Vector3 {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, SphereShape>) {
            Vector3 dirNorm = direction;
            if (dirNorm.lengthSquare() > 0.0001f) dirNorm.normalize();
            return transform.position + (dirNorm * arg.radius);
        }
        else if constexpr (std::is_same_v<T, BoxShape>) {
            Quaternion invRot = transform.orientation.getConjugate();
            Vector3 localDir = invRot * direction;
            Vector3 localResult;
            localResult.x = (localDir.x > 0.0f) ? arg.halfExtents.x : -arg.halfExtents.x;
            localResult.y = (localDir.y > 0.0f) ? arg.halfExtents.y : -arg.halfExtents.y;
            localResult.z = (localDir.z > 0.0f) ? arg.halfExtents.z : -arg.halfExtents.z;
            return transform.position + (transform.orientation * localResult);
        }
        else if constexpr (std::is_same_v<T, CapsuleShape>) {
            Quaternion invRot = transform.orientation.getConjugate();
            Vector3 localDir = invRot * direction;
            float halfH = arg.height * 0.5f;
            Vector3 top(0, halfH, 0);
            Vector3 bottom(0, -halfH, 0);
            Vector3 point = (localDir.dot(top) > localDir.dot(bottom)) ? top : bottom;
            Vector3 localResult = point + (localDir.getNormalized() * arg.radius);
            return transform.position + (transform.orientation * localResult);
        }
        else if constexpr (std::is_same_v<T, ConvexHullShape>) {
            if (arg.vertices.empty()) return transform.position;
            Quaternion invRot = transform.orientation.getConjugate();
            Vector3 localDir = invRot * direction;
            float maxDot = -1e10f;
            Vector3 bestVertex = arg.vertices[0];
            for (const auto& v : arg.vertices) {
                float d = v.dot(localDir);
                if (d > maxDot) { maxDot = d; bestVertex = v; }
            }
            return transform.position + (transform.orientation * bestVertex);
        }
        else if constexpr (std::is_same_v<T, TriangleShape>) {
            Quaternion invRot = transform.orientation.getConjugate();
            Vector3 localDir = invRot * direction;
            float d0 = arg.v0.dot(localDir), d1 = arg.v1.dot(localDir), d2 = arg.v2.dot(localDir);
            Vector3 bestLocal = arg.v0;
            float maxD = d0;
            if (d1 > maxD) { maxD = d1; bestLocal = arg.v1; }
            if (d2 > maxD) { maxD = d2; bestLocal = arg.v2; }
            return transform.position + (transform.orientation * bestLocal);
        }
        return transform.position;
    }, shape.getVariant());
}

/**
 * @brief Işın (Ray) ile geometrik şekiller arasında kesişim testi yapar.
 * @details Küre için analitik, Kutu için OBB (Oriented Bounding Box) testi uygular.
 */
bool NarrowPhase::raycast(const CollisionShape& shape, const Pose& transform, 
                          const Ray& ray, RaycastHit& hit) {
    return std::visit([&](auto&& arg) -> bool {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, SphereShape>) {
            Vector3 oc = ray.origin - transform.position;
            float b = oc.dot(ray.direction);
            float c = oc.lengthSquare() - (arg.radius * arg.radius);
            if (c > 0.0f && b > 0.0f) return false;
            float discriminant = (b * b) - c;
            if (discriminant < 0.0f) return false;
            float t = -b - std::sqrt(discriminant);
            if (t < 0.0f) t = 0.0f; 
            if (t > ray.maxDistance) return false;
            hit.hasHit = true; hit.distance = t;
            hit.position = ray.origin + ray.direction * t;
            hit.normal = (hit.position - transform.position).getNormalized();
            return true;
        }
        else if constexpr (std::is_same_v<T, BoxShape>) {
            Quaternion invRot = transform.orientation.getConjugate();
            Vector3 localOrigin = invRot * (ray.origin - transform.position);
            Vector3 localDir = invRot * ray.direction;
            Ray localRay(localOrigin, localDir, ray.maxDistance);
            AABB localAABB(-arg.halfExtents, arg.halfExtents);
            float t;
            if (localAABB.testRay(localRay, t)) {
                hit.hasHit = true; hit.distance = t;
                hit.position = ray.origin + ray.direction * t;
                Vector3 p = localOrigin + localDir * t;
                Vector3 n(0, 0, 0);
                float min = 1e10f;
                for(int i=0; i<3; ++i) {
                    float d = std::abs(std::abs(p[i]) - arg.halfExtents[i]);
                    if(d < min) { min = d; n = Vector3(0,0,0); n[i] = (p[i] > 0) ? 1.0f : -1.0f; }
                }
                hit.normal = transform.orientation * n;
                return true;
            }
            return false;
        }
        return false;
    }, shape.getVariant());
}

/**
 * @brief Standart GJK çarpışma testi.
 * @details Sadece çarpışma olup olmadığını hızlıca kontrol eder. EPA'nın çalışması için 
 *          gerekli olan başlangıç dörtyüzlüsünü (Simpleks) oluşturur.
 */
bool NarrowPhase::GJK(const CollisionShape& shapeA, const Pose& transformA,
                    const CollisionShape& shapeB, const Pose& transformB,
                    Simplex& outSimplex) {
    Vector3 direction = transformB.position - transformA.position;
    if (direction.lengthSquare() < 0.0001f) direction = Vector3(1.0f, 0.0f, 0.0f);

    outSimplex = Simplex();
    Vector3 support = getSupportPoint(shapeA, transformA, direction) - 
                      getSupportPoint(shapeB, transformB, direction * -1.0f);
    
    outSimplex.push_front(support);
    direction = support * -1.0f;

    for (int i = 0; i < 64; ++i) {
        support = getSupportPoint(shapeA, transformA, direction) - 
                  getSupportPoint(shapeB, transformB, direction * -1.0f);
        if (support.dot(direction) < 0.0f) return false;
        outSimplex.push_front(support);
        if (handleSimplex(outSimplex, direction)) return true;
    }
    return false;
}

/**
 * @brief GJK simpleks yapısını günceller ve bir sonraki arama yönünü belirler.
 */
bool NarrowPhase::handleSimplex(Simplex& simplex, Vector3& direction) {
    if (simplex.size() == 2) {
        Vector3 a = simplex[0], b = simplex[1];
        Vector3 ab = b - a, ao = a * -1.0f;
        direction = ab.cross(ao).cross(ab);
    }
    else if (simplex.size() == 3) {
        Vector3 a = simplex[0], b = simplex[1], c = simplex[2];
        Vector3 ab = b - a, ac = c - a, ao = a * -1.0f;
        Vector3 abc = ab.cross(ac);
        if (abc.cross(ac).dot(ao) > 0.0f) {
            if (ac.dot(ao) > 0.0f) { simplex.assign({a, c}); direction = ac.cross(ao).cross(ac); }
            else { if (ab.dot(ao) > 0.0f) { simplex.assign({a, b}); direction = ab.cross(ao).cross(ab); }
                   else { simplex.assign({a}); direction = ao; } }
        } else {
            if (ab.cross(abc).dot(ao) > 0.0f) {
                if (ab.dot(ao) > 0.0f) { simplex.assign({a, b}); direction = ab.cross(ao).cross(ab); }
                else { simplex.assign({a}); direction = ao; }
            } else {
                if (abc.dot(ao) > 0.0f) { direction = abc; }
                else { simplex.assign({a, c, b}); direction = abc * -1.0f; }
            }
        }
    }
    else {
        Vector3 a = simplex[0], b = simplex[1], c = simplex[2], d = simplex[3];
        Vector3 ab = b - a, ac = c - a, ad = d - a, ao = a * -1.0f;
        Vector3 abc = ab.cross(ac), acd = ac.cross(ad), adb = ad.cross(ab);
        if (abc.dot(ao) > 0.0f) { simplex.assign({a, b, c}); return handleSimplex(simplex, direction); }
        if (acd.dot(ao) > 0.0f) { simplex.assign({a, c, d}); return handleSimplex(simplex, direction); }
        if (adb.dot(ao) > 0.0f) { simplex.assign({a, d, b}); return handleSimplex(simplex, direction); }
        return true;
    }
    return false;
}

/**
 * @brief EPA (Expanding Polytope Algorithm) ile çarpışma detaylarını hesaplar.
 * @details Cisimlerin ne kadar iç içe girdiğini ve hangi yöne itilmeleri gerektiğini bulur. 
 *          Performans için bellek tahsisatı (allocation) yerine `thread_local` tamponlar kullanır.
 */
CollisionInfo NarrowPhase::EPA(const Simplex& simplex, 
                             const CollisionShape& shapeA, const Pose& transformA,
                             const CollisionShape& shapeB, const Pose& transformB) {
    thread_local std::vector<EPAVertex> vertices;
    thread_local std::vector<EPAFace> faces;
    thread_local std::vector<std::pair<size_t, size_t>> edges;

    vertices.clear(); faces.clear();
    if (vertices.capacity() < 96) vertices.reserve(96);
    if (faces.capacity() < 192) faces.reserve(192);
    if (edges.capacity() < 96) edges.reserve(96);

    for (int i = 0; i < simplex.size(); ++i) {
        Vector3 p = simplex[i];
        vertices.push_back({p, getSupportPoint(shapeA, transformA, p)});
    }

    auto getFace = [&](size_t i1, size_t i2, size_t i3) {
        Vector3 v1 = vertices[i1].minkowskiPoint, v2 = vertices[i2].minkowskiPoint, v3 = vertices[i3].minkowskiPoint;
        Vector3 n = (v2 - v1).cross(v3 - v1).getNormalized();
        float d = n.dot(v1);
        if (d < 0) { n = -n; d = -d; std::swap(i1, i2); }
        return EPAFace{i1, i2, i3, n, d};
    };

    faces.push_back(getFace(0, 1, 2)); faces.push_back(getFace(0, 2, 3));
    faces.push_back(getFace(0, 3, 1)); faces.push_back(getFace(1, 3, 2));

    EPAFace closestFace;
    for (int iter = 0; iter < 32; ++iter) {
        float minDistance = 1e10f; size_t closestFaceIndex = 0;
        for (size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].distance < minDistance) { minDistance = faces[i].distance; closestFaceIndex = i; }
        }
        closestFace = faces[closestFaceIndex];
        Vector3 searchDir = closestFace.normal;
        Vector3 pA = getSupportPoint(shapeA, transformA, searchDir);
        Vector3 p = pA - getSupportPoint(shapeB, transformB, searchDir * -1.0f);

        if (p.dot(searchDir) - closestFace.distance < 0.001f) break;

        vertices.push_back({p, pA});
        size_t newVIdx = vertices.size() - 1;
        edges.clear();
        auto addEdge = [&](size_t a, size_t b) {
            for (auto it = edges.begin(); it != edges.end(); ++it) {
                if (it->first == b && it->second == a) { edges.erase(it); return; }
            }
            edges.push_back({a, b});
        };

        for (size_t i = 0; i < faces.size(); ) {
            if (faces[i].normal.dot(p - vertices[faces[i].v1].minkowskiPoint) > 0) {
                addEdge(faces[i].v1, faces[i].v2); addEdge(faces[i].v2, faces[i].v3); addEdge(faces[i].v3, faces[i].v1);
                faces.erase(faces.begin() + i);
            } else i++;
        }
        for (auto& edge : edges) faces.push_back(getFace(edge.first, edge.second, newVIdx));
    }

    CollisionInfo info;
    info.hasCollision = true;
    info.normal = closestFace.normal;
    info.penetration = closestFace.distance;

    // Derin penetrasyonlarda normalin yönünü her zaman merkez farkına göre doğrula
    Vector3 centerSep = transformA.position - transformB.position;
    if (centerSep.lengthSquare() > 1e-16f && info.normal.dot(centerSep) < 0.0f) info.normal = -info.normal;

    Vector3 v1 = vertices[closestFace.v1].minkowskiPoint, v2 = vertices[closestFace.v2].minkowskiPoint, v3 = vertices[closestFace.v3].minkowskiPoint;
    Vector3 faceNormal = (v2 - v1).cross(v3 - v1);
    float areaSq = faceNormal.lengthSquare();
    Vector3 pOnFace = closestFace.normal * closestFace.distance;
    
    float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f;
    if (areaSq > 1e-12f) {
        w1 = faceNormal.dot((v2 - pOnFace).cross(v3 - pOnFace)) / areaSq;
        w2 = faceNormal.dot((v3 - pOnFace).cross(v1 - pOnFace)) / areaSq;
        w3 = 1.0f - w1 - w2;
    } else { w1 = 0.3333f; w2 = 0.3333f; w3 = 0.3334f; }

    info.contactPoint = (vertices[closestFace.v1].supportA * w1) + (vertices[closestFace.v2].supportA * w2) + (vertices[closestFace.v3].supportA * w3);
    return info;
}

} // namespace Baryon::collision