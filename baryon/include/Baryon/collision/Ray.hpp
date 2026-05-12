#pragma once
#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"
#include <algorithm>

/**
 * @file Ray.hpp
 * @brief Işın (Ray) ve çarpışma sonucu (RaycastHit) yapıları.
 * @details Fizik dünyasında belirli bir yöne doğru hayali bir çizgi çekerek 
 *          çarpışma testleri (raycast) yapmayı sağlar. Silah atışları, 
 *          zemin algılama (ground check) ve görüş açısı (line of sight) 
 *          testleri gibi mekanikler için kullanılır.
 */
namespace Baryon::collision {

// Döngüsel bağımlılığı önlemek için AABB yapısının ön bildirimi
struct AABB;

/**
 * @struct Ray
 * @brief 3B uzayda belirli bir noktadan belirli bir yöne ilerleyen ışın.
 */
struct Ray {
    Vector3 origin;      ///< Işının başlangıç noktası (dünya koordinatlarında).
    Vector3 direction;   ///< Işının ilerleme yönü (otomatik olarak normalize edilir).
    float maxDistance;   ///< Işının ulaşabileceği maksimum menzil (birim cinsinden).

    /**
     * @brief Yeni bir ışın oluşturur.
     * @param o Başlangıç noktası.
     * @param d İlerleme yönü (verilen vektör otomatik olarak normalize edilir).
     * @param dist Maksimum menzil (varsayılan: 1000 birim).
     */
    Ray(Vector3 o, Vector3 d, float dist = 1000.0f) 
        : origin(o), direction(d.getNormalized()), maxDistance(dist) {}

    /**
     * @brief Işının uzayda taradığı tüm alanı kapsayan bir sınır kutusu (AABB) oluşturur.
     * @details Broad-phase (Geniş faz) testlerinde, ışının geçtiği bölgedeki potansiyel 
     *          cisimleri ağaç yapısından (DynamicAABBTree) hızlıca filtrelemek için kullanılır.
     * @return Işını kapsayan AABB.
     */
    [[nodiscard]] AABB computeAABB() const;
};

/**
 * @struct RaycastHit
 * @brief Işın sorgusunun (raycast) sonucunu barındıran yapı.
 * @details Işın bir cisme çarptığında, temas noktası ve çarpılan cisim 
 *          hakkındaki tüm detayları içerir.
 */
struct RaycastHit {
    bool hasHit{false};       ///< Çarpışma gerçekleşti mi? (Gerçekleşmediyse diğer veriler geçersizdir).
    Vector3 position;         ///< Çarpışmanın gerçekleştiği noktanın dünya koordinatları.
    Vector3 normal;           ///< Çarpılan yüzeyin normal vektörü (yüzeyin baktığı yön).
    float distance{0.0f};     ///< Işının başlangıç noktasından çarpışma noktasına kadar olan mesafe.
    ecs::Entity entity;       ///< Çarpılan varlığın (cismin) kimliği.
};

} // namespace Baryon::collision

#include "AABB.hpp"

namespace Baryon::collision {

/**
 * @brief Işının başlangıç ve bitiş noktalarını hesaplayarak ikisini de içine alan en küçük sınır kutusunu oluşturur.
 */
inline AABB Ray::computeAABB() const {
    Vector3 end = origin + (direction * maxDistance);
    return AABB(
        Vector3(std::min(origin.x, end.x), std::min(origin.y, end.y), std::min(origin.z, end.z)),
        Vector3(std::max(origin.x, end.x), std::max(origin.y, end.y), std::max(origin.z, end.z))
    );
}

} // namespace Baryon::collision