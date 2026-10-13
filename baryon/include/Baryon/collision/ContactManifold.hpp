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
#include "../math/Pose.hpp"
#include "PhysicsConstants.hpp"
#include <array>
#include <cmath>
#include <algorithm>

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

    /**
     * @brief Dünya koordinatındaki temas noktası ve iki cismin pozundan yerel temas noktalarını
     *        hesaplayıp tüm itmeleri sıfırlayarak yeni bir temas noktası (ContactPoint) üretir.
     * @param worldPt Dünya uzayındaki temas noktası.
     * @param pen İçe geçme derinliği (penetration).
     * @param poseA Birinci cismin dünya pozu.
     * @param poseB İkinci cismin dünya pozu.
     * @return Hazırlanmış ContactPoint nesnesi.
     */
    static ContactPoint create(const Vector3& worldPt, float pen, const Pose& poseA, const Pose& poseB) {
        ContactPoint cp;
        cp.worldPosition = worldPt;
        cp.localPointA = poseA.orientation.getConjugate() * (worldPt - poseA.position);
        cp.localPointB = poseB.orientation.getConjugate() * (worldPt - poseB.position);
        cp.penetration = pen;
        cp.normalImpulse = 0.0f;
        cp.tangentImpulse1 = 0.0f;
        cp.tangentImpulse2 = 0.0f;
        cp.tangentImpulse = 0.0f;
        cp.splitImpulse = 0.0f;
        cp.velocityBias = 0.0f;
        return cp;
    }
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

    /**
     * @brief 4 noktadan oluşan temas poligonunun 2D teğet düzlemdeki izdüşüm alanını hesaplar.
     */
     static float calculateSupportArea(const std::array<Vector3, 4>& pts, const Vector3& t1, const Vector3& t2) {
        float u[4], v[4];
        float cu = 0.0f, cv = 0.0f;
        for (int i = 0; i < 4; ++i) {
            u[i] = pts[i].dot(t1);
            v[i] = pts[i].dot(t2);
            cu += u[i];
            cv += v[i];
        }
        cu *= 0.25f;
        cv *= 0.25f;

        int order[4] = {0, 1, 2, 3};
        float angles[4];
        for (int i = 0; i < 4; ++i) {
            angles[i] = std::atan2(v[i] - cv, u[i] - cu);
        }
        for (int i = 1; i < 4; ++i) {
            int keyIdx = order[i];
            float keyAngle = angles[keyIdx];
            int j = i - 1;
            while (j >= 0 && angles[order[j]] > keyAngle) {
                order[j + 1] = order[j];
                j = j - 1;
            }
            order[j + 1] = keyIdx;
        }

        float area = 0.0f;
        for (int i = 0; i < 4; ++i) {
            int curr = order[i];
            int next = order[(i + 1) % 4];
            area += (u[curr] * v[next] - u[next] * v[curr]);
        }
        return 0.5f * std::abs(area);
    }

    /**
     * @brief Mevcut 4 temas noktası ve yeni aday noktadan (toplam 5 nokta),
     *        en derin noktayı koruyup destek alanını (support polygon area)
     *        maksimize eden 4 temas noktasını seçer (Contact Reduction).
     */
    void reduceContacts(const ContactPoint& newPoint) {
        std::array<ContactPoint, 5> candidates = {
            contacts[0], contacts[1], contacts[2], contacts[3], newPoint
        };

        // 1. En derin temas noktasını bul (öncelikli destek noktası)
        int deepestIdx = 0;
        float maxPen = candidates[0].penetration;
        for (int i = 1; i < 5; ++i) {
            if (candidates[i].penetration > maxPen) {
                maxPen = candidates[i].penetration;
                deepestIdx = i;
            }
        }

        // 2. Teğet vektörlerini doğrula
        if (tangent1.lengthSquare() < 1e-6f || tangent2.lengthSquare() < 1e-6f) {
            computeTangents();
        }

        // 3. deepestIdx içeren 4'lü kombinasyonlar arasından alanı maksimize edeni seç
        int bestDropIdx = -1;
        float maxArea = -1.0f;

        for (int drop = 0; drop < 5; ++drop) {
            if (drop == deepestIdx) continue;

            std::array<Vector3, 4> subsetPts;
            int subCount = 0;
            for (int i = 0; i < 5; ++i) {
                if (i == drop) continue;
                subsetPts[subCount++] = candidates[i].worldPosition;
            }

            float area = calculateSupportArea(subsetPts, tangent1, tangent2);
            if (area > maxArea) {
                maxArea = area;
                bestDropIdx = drop;
            }
        }

        int writeIdx = 0;
        for (int i = 0; i < 5; ++i) {
            if (i == bestDropIdx) continue;
            contacts[writeIdx++] = candidates[i];
        }
        contactCount = 4;
    }

    void addContact(const ContactPoint& cp) {
        // Çok yakın nokta varsa güncelle
        for (uint32_t i = 0; i < contactCount; ++i) {
            if ((contacts[i].localPointA - cp.localPointA).lengthSquare() < physics::PhysicsConstants::WarmStartDistanceSqThreshold) {
                if (cp.penetration > contacts[i].penetration) {
                    contacts[i] = cp;
                }
                return;
            }
        }

        if (contactCount < 4) {
            contacts[contactCount++] = cp;
        } else {
            reduceContacts(cp);
        }
    }

    /**
     * @brief Önceki kareden kalan temas verilerinden (warm-starting) itmeleri eşleştirir ve devralır.
     * @param old Önceki karenin temas manifoldu.
     * @param maxDistSq Eşleştirme için maksimum karesel mesafe eşiği (varsayılan: 0.04f -> 20cm).
     */
    void inheritImpulses(const ContactManifold& old, float maxDistSq = physics::PhysicsConstants::WarmStartDistanceSqThreshold) {
        for (uint32_t i = 0; i < contactCount; ++i) {
            auto& newCp = contacts[i];
            float closestDistSq = maxDistSq;
            int bestMatch = -1;
            for (uint32_t j = 0; j < old.contactCount; ++j) {
                float distSq = (old.contacts[j].localPointA - newCp.localPointA).lengthSquare();
                if (distSq < closestDistSq) {
                    closestDistSq = distSq;
                    bestMatch = static_cast<int>(j);
                }
            }
            if (bestMatch != -1) {
                newCp.normalImpulse = old.contacts[bestMatch].normalImpulse;
                newCp.tangentImpulse1 = old.contacts[bestMatch].tangentImpulse1;
                newCp.tangentImpulse2 = old.contacts[bestMatch].tangentImpulse2;
            }
        }
    }
};

} // namespace Baryon::collision
