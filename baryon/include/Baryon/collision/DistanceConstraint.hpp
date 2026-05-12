#pragma once
#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"

/**
 * @file DistanceConstraint.hpp
 * @brief Mesafe kısıtlaması (Distance Constraint) yapısı.
 * @details İki cisim arasındaki mesafeyi belirli bir uzunlukta tutmaya çalışan 
 *          esnek veya katı bir bağlantıdır (eklem). Zincir, sarkaç, halat veya ip 
 *          gibi mekanizmaları modellemek için kullanılır.
 */
namespace Baryon::collision {

/**
 * @struct DistanceConstraint
 * @brief İki varlık (entity) arasındaki mesafe kısıtlamasının fiziksel özelliklerini tutar.
 */
struct DistanceConstraint {
    ecs::Entity entityA;       ///< Birinci varlık kimliği.
    ecs::Entity entityB;       ///< İkinci varlık kimliği.

    Vector3 localAnchorA;      ///< A cismi üzerindeki yerel bağlantı noktası.
    Vector3 localAnchorB;      ///< B cismi üzerindeki yerel bağlantı noktası.

    float targetDistance;      ///< Korunmaya çalışılan hedef mesafe (metre).
    
    /**
     * @brief Bağlantının sertlik (yay) katsayısı [0.0, 1.0] arasında olmalıdır.
     * @details 1.0 değeri esnemez (katı) bir bağlantı sağlarken, daha düşük 
     *          değerler paket lastiği veya yay gibi esnek davranmasına neden olur.
     */
    float stiffness{1.0f};     

    /**
     * @brief Sönümleme katsayısı.
     * @details Esnek bağlantılarda oluşan salınım ve titremeleri yavaşlatarak durdurur.
     */
    float damping{0.1f};       

    /**
     * @brief Yeni bir mesafe kısıtlaması oluşturur.
     * @param a Birinci varlık.
     * @param b İkinci varlık.
     * @param dist İki varlık arasındaki korunacak hedef mesafe.
     */
    DistanceConstraint(ecs::Entity a, ecs::Entity b, float dist)
        : entityA(a), entityB(b), targetDistance(dist) {}
};

} // namespace Baryon::collision