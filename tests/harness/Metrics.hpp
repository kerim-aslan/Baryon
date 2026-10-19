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

#include "Baryon/Simulator.hpp"
#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/collision/ContactManifold.hpp"

#include <vector>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <cmath>

/**
 * @file Metrics.hpp
 * @brief Simülasyon durumunu ve korunum yasalarını ölçen ortak metrik fonksiyonları.
 * @details Fizik motorunun doğrulanması ve regresyon takibi için enerji, momentum,
 *          maksimum penetrasyon derinliği ve FNV-1a durum hash'i hesaplar.
 */
namespace Baryon::harness {

/**
 * @brief 64-bit FNV-1a hash algoritması.
 * @param data Hashlenecek bayt dizisi.
 * @param size Bayt sayısı.
 * @param hash Başlangıç hash değeri (zincirleme için).
 * @return 64-bit güncel hash değeri.
 */
inline uint64_t fnv1a64(const void* data, size_t size, uint64_t hash = 0xcbf29ce484222325ULL) {
    const uint8_t* ptr = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= static_cast<uint64_t>(ptr[i]);
        hash *= 0x100000001b3ULL;
    }
    return hash;
}

/**
 * @brief 64-bit hash değerini onaltılık (hex) dizgiye dönüştürür.
 */
inline std::string hashToHexString(uint64_t hash) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << hash;
    return oss.str();
}

/**
 * @brief Kayan noktalı sayıları deterministik bit hash'i için normalize eder (-0.0 -> +0.0).
 */
inline float sanitizeFloat(float val) {
    if (std::abs(val) < 1e-12f) return 0.0f;
    return val;
}

/**
 * @brief Sahnedeki tüm dinamik cisimlerin toplam kinetik enerjisini hesaplar.
 * @details T = 1/2 * m * v^2 + 1/2 * w^T * I_world * w formülünü kullanır.
 * @param sim Fizik simülatörü referansı.
 * @return Toplam kinetik enerji (Joule).
 */
inline float computeKineticEnergy(const Simulator& sim) {
    const auto& registry = sim.getRegistry();
    const auto& posePool = registry.getComponentPool<Pose>();
    const auto& motionPool = registry.getComponentPool<Core::Motion>();
    const auto& massPool = registry.getComponentPool<Core::MassProps>();
    const auto& statePool = registry.getComponentPool<Core::BodyState>();

    float totalKinetic = 0.0f;
    const auto& entities = posePool.getAllEntities();

    for (auto entity : entities) {
        if (!statePool.hasComponent(entity) || !motionPool.hasComponent(entity) || !massPool.hasComponent(entity)) {
            continue;
        }

        const auto& state = statePool.getData(entity);
        if (state.type != Core::BodyType::Dynamic) {
            continue;
        }

        const auto& motion = motionPool.getData(entity);
        const auto& mass = massPool.getData(entity);
        const auto& pose = posePool.getData(entity);

        // Çizgisel kinetik enerji: 0.5 * m * v^2
        float linK = 0.5f * mass.mass * motion.linearVelocity.dot(motion.linearVelocity);

        // Açısal kinetik enerji: 0.5 * w^T * I_world * w
        Matrix3x3 R = pose.getOrientationMatrix();
        Matrix3x3 worldInertia = R * mass.localInertiaTensor * R.getTranspose();
        Vector3 Iw = worldInertia * motion.angularVelocity;
        float angK = 0.5f * motion.angularVelocity.dot(Iw);

        totalKinetic += (linK + angK);
    }

    return totalKinetic;
}

/**
 * @brief Sahnedeki tüm dinamik cisimlerin yerçekimi potansiyel enerjisini hesaplar.
 * @details U = sum( -m * (g . position) ) formülünü kullanır.
 * @param sim Fizik simülatörü referansı.
 * @param gravity Yerçekimi ivmesi vektörü (Varsayılan: 0, -9.81, 0).
 * @return Toplam potansiyel enerji (Joule).
 */
inline float computePotentialEnergy(const Simulator& sim, const Vector3& gravity = Vector3(0.0f, -9.81f, 0.0f)) {
    const auto& registry = sim.getRegistry();
    const auto& posePool = registry.getComponentPool<Pose>();
    const auto& massPool = registry.getComponentPool<Core::MassProps>();
    const auto& statePool = registry.getComponentPool<Core::BodyState>();

    float totalPotential = 0.0f;
    const auto& entities = posePool.getAllEntities();

    for (auto entity : entities) {
        if (!statePool.hasComponent(entity) || !massPool.hasComponent(entity)) {
            continue;
        }

        const auto& state = statePool.getData(entity);
        if (state.type != Core::BodyType::Dynamic) {
            continue;
        }

        const auto& pose = posePool.getData(entity);
        const auto& mass = massPool.getData(entity);

        // U = - m * (g . r)
        totalPotential += -mass.mass * gravity.dot(pose.position);
    }

    return totalPotential;
}

/**
 * @brief Sahnedeki toplam mekanik enerjiyi hesaplar (Kinetik + Potansiyel).
 */
inline float computeTotalEnergy(const Simulator& sim, const Vector3& gravity = Vector3(0.0f, -9.81f, 0.0f)) {
    return computeKineticEnergy(sim) + computePotentialEnergy(sim, gravity);
}

/**
 * @brief Sahnedeki tüm dinamik cisimlerin toplam çizgisel momentumunu hesaplar.
 * @details P = sum( m * v ).
 * @param sim Fizik simülatörü referansı.
 * @return Çizgisel momentum vektörü (kg*m/s).
 */
inline Vector3 computeLinearMomentum(const Simulator& sim) {
    const auto& registry = sim.getRegistry();
    const auto& motionPool = registry.getComponentPool<Core::Motion>();
    const auto& massPool = registry.getComponentPool<Core::MassProps>();
    const auto& statePool = registry.getComponentPool<Core::BodyState>();

    Vector3 totalMomentum(0.0f, 0.0f, 0.0f);
    const auto& entities = motionPool.getAllEntities();

    for (auto entity : entities) {
        if (!statePool.hasComponent(entity) || !massPool.hasComponent(entity)) {
            continue;
        }

        const auto& state = statePool.getData(entity);
        if (state.type != Core::BodyType::Dynamic) {
            continue;
        }

        const auto& motion = motionPool.getData(entity);
        const auto& mass = massPool.getData(entity);

        totalMomentum += motion.linearVelocity * mass.mass;
    }

    return totalMomentum;
}

/**
 * @brief Sahnedeki tüm dinamik cisimlerin orijine göre toplam açısal momentumunu hesaplar.
 * @details L = sum( r x (m * v) + I_world * w ).
 * @param sim Fizik simülatörü referansı.
 * @return Açısal momentum vektörü (kg*m^2/s).
 */
inline Vector3 computeAngularMomentum(const Simulator& sim) {
    const auto& registry = sim.getRegistry();
    const auto& posePool = registry.getComponentPool<Pose>();
    const auto& motionPool = registry.getComponentPool<Core::Motion>();
    const auto& massPool = registry.getComponentPool<Core::MassProps>();
    const auto& statePool = registry.getComponentPool<Core::BodyState>();

    Vector3 totalAngularMomentum(0.0f, 0.0f, 0.0f);
    const auto& entities = posePool.getAllEntities();

    for (auto entity : entities) {
        if (!statePool.hasComponent(entity) || !motionPool.hasComponent(entity) || !massPool.hasComponent(entity)) {
            continue;
        }

        const auto& state = statePool.getData(entity);
        if (state.type != Core::BodyType::Dynamic) {
            continue;
        }

        const auto& pose = posePool.getData(entity);
        const auto& motion = motionPool.getData(entity);
        const auto& mass = massPool.getData(entity);

        // Orbital açısal momentum: r x (m * v)
        Vector3 orbitalL = pose.position.cross(motion.linearVelocity * mass.mass);

        // Spin açısal momentum: I_world * w
        Matrix3x3 R = pose.getOrientationMatrix();
        Matrix3x3 worldInertia = R * mass.localInertiaTensor * R.getTranspose();
        Vector3 spinL = worldInertia * motion.angularVelocity;

        totalAngularMomentum += (orbitalL + spinL);
    }

    return totalAngularMomentum;
}

/**
 * @brief getContactsForEntity API'si üzerinden sahnedeki en büyük penetrasyon derinliğini hesaplar.
 * @param sim Fizik simülatörü referansı.
 * @return En büyük penetrasyon derinliği (metre).
 */
inline float computeMaxPenetration(const Simulator& sim) {
    const auto& registry = sim.getRegistry();
    const auto& entities = registry.getComponentPool<Pose>().getAllEntities();
    float maxPenetration = 0.0f;

    for (auto entity : entities) {
        auto contacts = sim.getContactsForEntity(entity);
        for (const auto& manifold : contacts) {
            if (manifold.isTriggerPair) {
                continue;
            }
            if (manifold.penetration > maxPenetration) {
                maxPenetration = manifold.penetration;
            }
            for (uint32_t c = 0; c < manifold.contactCount; ++c) {
                if (manifold.contacts[c].penetration > maxPenetration) {
                    maxPenetration = manifold.contacts[c].penetration;
                }
            }
        }
    }

    return maxPenetration;
}

/**
 * @brief Sahnedeki cisimlerin konum, yönelim ve hız bitlerinden 64-bit FNV-1a durum hash'i üretir.
 * @details Determinizm doğrulaması için entity kimliklerine göre deterministik sırada sıralama yapar.
 * @param sim Fizik simülatörü referansı.
 * @return 64-bit durum hash'i.
 */
inline uint64_t computeStateHash(const Simulator& sim) {
    const auto& registry = sim.getRegistry();
    const auto& posePool = registry.getComponentPool<Pose>();
    const auto& rawEntities = posePool.getAllEntities();

    // Deterministik sıralama (artan entity ID)
    std::vector<ecs::Entity> sortedEntities(rawEntities.begin(), rawEntities.end());
    std::sort(sortedEntities.begin(), sortedEntities.end(), [](const ecs::Entity& a, const ecs::Entity& b) {
        return a.id < b.id;
    });

    uint64_t hash = 0xcbf29ce484222325ULL;

    const auto& motionPool = registry.getComponentPool<Core::Motion>();

    for (const auto& entity : sortedEntities) {
        uint64_t id = entity.id;
        hash = fnv1a64(&id, sizeof(id), hash);

        const auto& pose = posePool.getData(entity);
        float px = sanitizeFloat(pose.position.x);
        float py = sanitizeFloat(pose.position.y);
        float pz = sanitizeFloat(pose.position.z);
        float ow = sanitizeFloat(pose.orientation.w);
        float ox = sanitizeFloat(pose.orientation.x);
        float oy = sanitizeFloat(pose.orientation.y);
        float oz = sanitizeFloat(pose.orientation.z);

        hash = fnv1a64(&px, sizeof(px), hash);
        hash = fnv1a64(&py, sizeof(py), hash);
        hash = fnv1a64(&pz, sizeof(pz), hash);
        hash = fnv1a64(&ow, sizeof(ow), hash);
        hash = fnv1a64(&ox, sizeof(ox), hash);
        hash = fnv1a64(&oy, sizeof(oy), hash);
        hash = fnv1a64(&oz, sizeof(oz), hash);

        if (motionPool.hasComponent(entity)) {
            const auto& motion = motionPool.getData(entity);
            float vx = sanitizeFloat(motion.linearVelocity.x);
            float vy = sanitizeFloat(motion.linearVelocity.y);
            float vz = sanitizeFloat(motion.linearVelocity.z);
            float wx = sanitizeFloat(motion.angularVelocity.x);
            float wy = sanitizeFloat(motion.angularVelocity.y);
            float wz = sanitizeFloat(motion.angularVelocity.z);

            hash = fnv1a64(&vx, sizeof(vx), hash);
            hash = fnv1a64(&vy, sizeof(vy), hash);
            hash = fnv1a64(&vz, sizeof(vz), hash);
            hash = fnv1a64(&wx, sizeof(wx), hash);
            hash = fnv1a64(&wy, sizeof(wy), hash);
            hash = fnv1a64(&wz, sizeof(wz), hash);
        }
    }

    return hash;
}

} // namespace Baryon::harness
