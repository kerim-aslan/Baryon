#pragma once
#include "../math/Vector3.hpp"
#include <algorithm>
#include <initializer_list>

/**
 * @file AABB.hpp
 * @brief Eksen Hizalı Sınırlayıcı Kutu (Axis-Aligned Bounding Box) yapısı.
 * @details Broad-phase (geniş faz) çarpışma algılamada kullanılan, kenarları koordinat 
 *          eksenlerine paralel olan ve hızlı kesişim testleri sağlayan temel prizma yapısıdır.
 */
namespace Baryon::collision {

// Döngüsel bağımlılığı önlemek için ön bildirim
struct Ray;

/**
 * @struct AABB
 * @brief Minimum ve maksimum noktalarla tanımlanan 3B sınırlayıcı kutu.
 */
struct AABB {
    Vector3 minBounds; ///< Minimum köşe noktası (sol-alt-arka).
    Vector3 maxBounds; ///< Maksimum köşe noktası (sağ-üst-ön).

    /**
     * @brief Sıfır boyutlu varsayılan AABB oluşturur.
     */
    constexpr AABB() = default;

    /**
     * @brief Belirtilen köşe noktalarıyla AABB oluşturur.
     * @param min Minimum köşe.
     * @param max Maksimum köşe.
     */
    constexpr AABB(const Vector3& min, const Vector3& max) : minBounds(min), maxBounds(max) {}

    /**
     * @brief İki AABB'nin kesişip kesişmediğini kontrol eder.
     * @param other Kesişim testi yapılacak diğer AABB.
     * @return Kesişiyorsa true, aksi halde false.
     */
    [[nodiscard]] constexpr bool testCollision(const AABB& other) const {
        if (maxBounds.x < other.minBounds.x || minBounds.x > other.maxBounds.x) return false;
        if (maxBounds.y < other.minBounds.y || minBounds.y > other.maxBounds.y) return false;
        if (maxBounds.z < other.minBounds.z || minBounds.z > other.maxBounds.z) return false;
        return true;
    }

    /**
     * @brief Bir ışının (Ray) bu AABB ile kesişip kesişmediğini test eder.
     * @param ray Test edilecek ışın.
     * @param[out] tMin Kesişim noktasının ışın üzerindeki mesafesi (t değeri).
     * @return Kesişiyorsa true, aksi halde false.
     */
    [[nodiscard]] constexpr bool testRay(const Ray& ray, float& tMin) const;

    /**
     * @brief Bu AABB'nin, başka bir AABB'yi tamamen kapsayıp kapsamadığını kontrol eder.
     * @param other İçerilme testi yapılacak AABB.
     * @return Tamamen kapsıyorsa true, aksi halde false.
     */
    [[nodiscard]] constexpr bool contains(const AABB& other) const {
        return (minBounds.x <= other.minBounds.x && maxBounds.x >= other.maxBounds.x) &&
               (minBounds.y <= other.minBounds.y && maxBounds.y >= other.maxBounds.y) &&
               (minBounds.z <= other.minBounds.z && maxBounds.z >= other.maxBounds.z);
    }

    /**
     * @brief Bu AABB'yi, diğer bir AABB'yi de içine alacak şekilde genişletir (birleştirir).
     * @param other Birleştirilecek AABB.
     */
    constexpr void merge(const AABB& other) {
        minBounds.x = std::min(minBounds.x, other.minBounds.x);
        minBounds.y = std::min(minBounds.y, other.minBounds.y);
        minBounds.z = std::min(minBounds.z, other.minBounds.z);
        maxBounds.x = std::max(maxBounds.x, other.maxBounds.x);
        maxBounds.y = std::max(maxBounds.y, other.maxBounds.y);
        maxBounds.z = std::max(maxBounds.z, other.maxBounds.z);
    }

    /**
     * @brief AABB'nin yüzey alanını hesaplar. (Ağaç yapılarında SAH maliyet hesabı için kullanılır).
     * @return Toplam yüzey alanı.
     */
    [[nodiscard]] constexpr float getSurfaceArea() const {
        Vector3 d = maxBounds - minBounds;
        return 2.0f * (d.x * d.y + d.y * d.z + d.x * d.z);
    }
};

} // namespace Baryon::collision

// Ray yapısının detayları için dahil ediliyor
#include "Ray.hpp"

namespace Baryon::collision {

/**
 * @brief Slab (dilim) yöntemi kullanarak ışın-AABB kesişim testini gerçekleştirir.
 */
[[nodiscard]] constexpr bool AABB::testRay(const Ray& ray, float& tMin) const {
    Vector3 invDir(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);

    float t1 = (minBounds.x - ray.origin.x) * invDir.x;
    float t2 = (maxBounds.x - ray.origin.x) * invDir.x;
    float t3 = (minBounds.y - ray.origin.y) * invDir.y;
    float t4 = (maxBounds.y - ray.origin.y) * invDir.y;
    float t5 = (minBounds.z - ray.origin.z) * invDir.z;
    float t6 = (maxBounds.z - ray.origin.z) * invDir.z;

    float tmin = std::max({std::min(t1, t2), std::min(t3, t4), std::min(t5, t6)});
    float tmax = std::min({std::max(t1, t2), std::max(t3, t4), std::max(t5, t6)});

    if (tmax < 0 || tmin > tmax) return false;
    tMin = tmin;
    return tmin < ray.maxDistance;
}

} // namespace Baryon::collision