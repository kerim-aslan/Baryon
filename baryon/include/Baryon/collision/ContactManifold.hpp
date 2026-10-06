#pragma once

#include "../ecs/EntityManager.hpp"
#include "../math/Vector3.hpp"
#include <array>
#include <cmath>

namespace Baryon::collision {

/**
 * @struct ContactPoint
 * @brief İki cisim arasındaki tekil temas noktası geometrik ve fiziksel verileri.
 */
struct ContactPoint {
    Vector3 worldPosition{0.0f, 0.0f, 0.0f};
    Vector3 localPointA{0.0f, 0.0f, 0.0f};
    Vector3 localPointB{0.0f, 0.0f, 0.0f};
    float penetration{0.0f};

    float normalImpulse{0.0f};
    float tangentImpulse1{0.0f};
    float tangentImpulse2{0.0f};
    float tangentImpulse{0.0f};
    float splitImpulse{0.0f};
};

/**
 * @struct ContactManifold
 * @brief İki cisim arasındaki temas noktalarını (en fazla 4) gruplayan yapı.
 */
struct ContactManifold {
    ecs::Entity entityA;
    ecs::Entity entityB;

    Vector3 normal{0.0f, 0.0f, 0.0f};
    Vector3 tangent1{0.0f, 0.0f, 0.0f};
    Vector3 tangent2{0.0f, 0.0f, 0.0f};
    float penetration{0.0f};

    bool isTriggerPair{false};

    std::array<ContactPoint, 4> contacts;
    uint32_t contactCount{0};

    void computeTangents() {
        if (normal.lengthSquare() < 1e-10f) return;
        if (std::abs(normal.x) >= 0.57735f) {
            tangent1 = Vector3(normal.y, -normal.x, 0.0f).getNormalized();
        } else {
            tangent1 = Vector3(0.0f, normal.z, -normal.y).getNormalized();
        }
        tangent2 = normal.cross(tangent1);
    }

    ContactManifold(ecs::Entity a, ecs::Entity b)
        : entityA(a), entityB(b), normal(0, 0, 0), tangent1(0, 0, 0), tangent2(0, 0, 0), penetration(0), isTriggerPair(false) {}

    void addContact(const ContactPoint& cp) {
        if (contactCount < 4) {
            contacts[contactCount++] = cp;
        } else {
            contacts[3] = cp;
        }
    }
};

} // namespace Baryon::collision
