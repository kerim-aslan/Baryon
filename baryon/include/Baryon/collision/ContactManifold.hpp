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
    float velocityBias{0.0f};
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

    void inheritImpulses(const ContactManifold& oldManifold, float matchRadiusSq = 0.0004f) {
        bool usedOld[4] = {false, false, false, false};
        for (uint32_t cIdx = 0; cIdx < contactCount; ++cIdx) {
            auto& newCp = contacts[cIdx];
            float closestDistSq = matchRadiusSq;
            int bestMatch = -1;
            for (uint32_t oIdx = 0; oIdx < oldManifold.contactCount; ++oIdx) {
                if (oIdx < 4 && usedOld[oIdx]) continue;
                float distSqA = (oldManifold.contacts[oIdx].localPointA - newCp.localPointA).lengthSquare();
                float distSqB = (oldManifold.contacts[oIdx].localPointB - newCp.localPointB).lengthSquare();
                float distSq = std::min(distSqA, distSqB);
                if (distSq < closestDistSq) {
                    closestDistSq = distSq;
                    bestMatch = static_cast<int>(oIdx);
                }
            }
            if (bestMatch != -1) {
                size_t bmIdx = static_cast<size_t>(bestMatch);
                if (bmIdx < 4) usedOld[bmIdx] = true;
                newCp.normalImpulse = oldManifold.contacts[bmIdx].normalImpulse;
                newCp.tangentImpulse1 = oldManifold.contacts[bmIdx].tangentImpulse1;
                newCp.tangentImpulse2 = oldManifold.contacts[bmIdx].tangentImpulse2;
            }
        }
    }
};

} // namespace Baryon::collision
