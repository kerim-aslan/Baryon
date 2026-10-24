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
 * @file Integrator.cpp
 * @brief Yarı-örtük Euler (Semi-implicit Euler) entegrasyon sistemi.
 * @details Kuvvet ve tork değerlerini, zaman adımı (dt) kullanarak hız ve 
 *          konum değişikliklerine dönüştürür. Kararlılığı artırmak için 
 *          hız ve konum güncellemeleri iki ayrı aşamada yapılır.
 */

#include "Baryon/systems/Integrator.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include <ranges>
#include <cmath>

namespace Baryon::systems {

Integrator::Integrator(Core::Registry& registry)
    : mRegistry(registry) {}

void Integrator::setAccelerationField(const Vector3& accelerationField) {
    mAccelerationField = accelerationField;
}

/**
 * @brief 1. AŞAMA: Hız Entegrasyonu (Çözücü öncesi çalışır).
 * @details Dinamik cisimlerin hızlarını dış etkenlere göre günceller:
 *
 *          1. Sönümleme (Damping): Hava sürtünmesini simüle ederek sonsuz ivmelenmeyi önler.
 *          2. Doğrusal İvme: Newton'un 2. Yasası (F=ma) ve global yerçekimi uygulanır.
 *          3. Açısal İvme: Tork değerleri, eylemsizlik tensörü kullanılarak açısal hıza çevrilir.
 *          4. Hassasiyet (Dead Zone): Enerji kaybı sonrası oluşan mikro titreşimleri (jitter) keser.
 *          5. Sıfırlama: Bir sonraki kare için kuvvet ve tork depoları boşaltılır.
 *
 * @param deltaTime Zaman adımı (saniye).
 */
void Integrator::integrateVelocities(float deltaTime) {
    auto& motionPool = mRegistry.getComponentPool<Core::Motion>();
    auto& motions = motionPool.getAllData();
    auto& motionEntities = motionPool.getAllEntities();

    for (auto&& [entity, motion] : std::views::zip(motionEntities, motions)) {
        if (!mRegistry.hasComponent<Core::BodyState>(entity) || !mRegistry.hasComponent<Core::MassProps>(entity)) continue;

        auto& state = mRegistry.getComponent<Core::BodyState>(entity);
        const auto& mass = mRegistry.getComponent<Core::MassProps>(entity);

        // Sadece hareketli (Dynamic) ve uyanık cisimler işlenir
        if (state.type != Core::BodyType::Dynamic || state.isSleeping) continue;

        // --- Doğrusal Hız Entegrasyonu ---
        // Sönümleme (Padé yaklaşımı ile hava direnci)
        float linDampFactor = 1.0f / (1.0f + motion.linearDamping * deltaTime);
        motion.linearVelocity *= linDampFactor;

        // Doğrusal ivme ve sönümleme
        Vector3 linearAcceleration = mAccelerationField + (motion.externalForce * mass.inverseMass);
        motion.linearVelocity += linearAcceleration * deltaTime;
        if (state.lockedAxes & Core::LockAxis::LinearX) motion.linearVelocity.x = 0.0f;
        if (state.lockedAxes & Core::LockAxis::LinearY) motion.linearVelocity.y = 0.0f;
        if (state.lockedAxes & Core::LockAxis::LinearZ) motion.linearVelocity.z = 0.0f;

        // --- Açısal Hız Entegrasyonu ---
        float angDampFactor = 1.0f / (1.0f + motion.angularDamping * deltaTime);
        motion.angularVelocity *= angDampFactor;

        // Açısal İvme: α = I⁻¹τ
        Vector3 angularAcceleration = mass.inverseInertiaTensor * motion.externalTorque;
        motion.angularVelocity += angularAcceleration * deltaTime;

        // Jiroskopik Terim (Gyroscopic Torque / Euler Denklemleri / Dzhanibekov Etkisi)
        if ((mEnableGyroscopicTorque || state.useGyroscopicTorque) && mRegistry.hasComponent<Pose>(entity)) {
            const auto& pose = mRegistry.getComponent<Pose>(entity);
            Matrix3x3 R = pose.getOrientationMatrix();
            Vector3 wBody = R.getTranspose() * motion.angularVelocity;

            // Çok küçük dönmelerde hesaplamayı atla
            if (wBody.lengthSquare() > 1e-6f) {
                // Euler hareket denklemleri: I * dw/dt + w x (I * w) = 0
                // dw/dt = I⁻¹ * ( (I * w) x w )
                auto computeEulerDeriv = [&](const Vector3& w) -> Vector3 {
                    Vector3 Iw = mass.localInertiaTensor * w;
                    Vector3 gyroTorque = Iw.cross(w); // Iw x w = - (w x Iw)
                    return mass.inverseLocalInertiaTensor * gyroTorque;
                };

                // 4. Dereceden Runge-Kutta (RK4) entegrasyonu
                Vector3 k1 = computeEulerDeriv(wBody);
                Vector3 k2 = computeEulerDeriv(wBody + k1 * (0.5f * deltaTime));
                Vector3 k3 = computeEulerDeriv(wBody + k2 * (0.5f * deltaTime));
                Vector3 k4 = computeEulerDeriv(wBody + k3 * deltaTime);

                Vector3 wBodyNew = wBody + (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (deltaTime / 6.0f);

                // Enerji korunum kuralı: Kinetik enerji kesinlikle artmamalıdır (E_new <= E_old)
                float E_old = 0.5f * wBody.dot(mass.localInertiaTensor * wBody);
                float E_new = 0.5f * wBodyNew.dot(mass.localInertiaTensor * wBodyNew);
                if (E_new > E_old && E_new > 1e-8f) {
                    wBodyNew *= std::sqrt(E_old / E_new);
                }

                motion.angularVelocity = R * wBodyNew;
            }
        }

        if (state.lockedAxes & Core::LockAxis::AngularX) motion.angularVelocity.x = 0.0f;
        if (state.lockedAxes & Core::LockAxis::AngularY) motion.angularVelocity.y = 0.0f;
        if (state.lockedAxes & Core::LockAxis::AngularZ) motion.angularVelocity.z = 0.0f;

        // Kuvvetleri sıfırla (Her karede kullanıcı tarafından yeniden uygulanmalıdır)
        motion.externalForce = Vector3(0.0f, 0.0f, 0.0f);
        motion.externalTorque = Vector3(0.0f, 0.0f, 0.0f);
    }
}

/**
 * @brief 2. AŞAMA: Konum ve Rotasyon Entegrasyonu (Çözücü sonrası çalışır).
 * @details Güncel hız ve Split Impulse değerlerini kullanarak cismin uzaydaki yerini belirler:
 *
 *          1. Konum Güncelleme: Mevcut pozisyona (v + v_split) eklenir.
 *          2. Rotasyon Güncelleme: (w + w_split) açısal hızına bağlı olarak kuaterniyon türevi entegre edilir.
 *          3. Normalizasyon: Sayısal hataların birikmesini önlemek için yönelim normalize edilir.
 *          4. Split Hızları Sıfırlama: Ayrık itmeler gerçek kinetik hızı kirletmez.
 *          5. Dünya Eylemsizliği: Cismin dönüşüne bağlı olarak önceden hesaplanan I_local⁻¹ dünya uzayına uyarlanır.
 *
 * @param deltaTime Zaman adımı (saniye).
 */
void Integrator::integratePositions(float deltaTime) {
    auto& motionPool = mRegistry.getComponentPool<Core::Motion>();
    auto& motions = motionPool.getAllData();
    auto& motionEntities = motionPool.getAllEntities();

    for (auto&& [entity, motion] : std::views::zip(motionEntities, motions)) {
        if (!mRegistry.hasComponent<Pose>(entity) || !mRegistry.hasComponent<Core::BodyState>(entity) || !mRegistry.hasComponent<Core::MassProps>(entity)) continue;

        auto& state = mRegistry.getComponent<Core::BodyState>(entity);
        if (state.type == Core::BodyType::Static || state.isSleeping) continue;

        auto& pose = mRegistry.getComponent<Pose>(entity);
        auto& mass = mRegistry.getComponent<Core::MassProps>(entity);

        // Doğrusal Hareket: p = p + (v + v_split)·dt
        if (!state.ccdIntegrated) {
            Vector3 effectiveLinVel = motion.linearVelocity + motion.splitLinearVelocity;
            if (state.lockedAxes & Core::LockAxis::LinearX) effectiveLinVel.x = 0.0f;
            if (state.lockedAxes & Core::LockAxis::LinearY) effectiveLinVel.y = 0.0f;
            if (state.lockedAxes & Core::LockAxis::LinearZ) effectiveLinVel.z = 0.0f;
            pose.position += effectiveLinVel * deltaTime;
        } else {
            state.ccdIntegrated = false;
        }

        // Açısal Hareket (Kuaterniyon türevi entegrasyonu)
        Vector3 effectiveAngVel = motion.angularVelocity + motion.splitAngularVelocity;
        if (state.lockedAxes & Core::LockAxis::AngularX) effectiveAngVel.x = 0.0f;
        if (state.lockedAxes & Core::LockAxis::AngularY) effectiveAngVel.y = 0.0f;
        if (state.lockedAxes & Core::LockAxis::AngularZ) effectiveAngVel.z = 0.0f;

        if (effectiveAngVel.lengthSquare() > 1e-12f) {
            Quaternion spin(
                effectiveAngVel.x,
                effectiveAngVel.y,
                effectiveAngVel.z,
                0.0f
            );
            Quaternion qDot = spin * pose.orientation;
            pose.orientation.x += qDot.x * deltaTime * 0.5f;
            pose.orientation.y += qDot.y * deltaTime * 0.5f;
            pose.orientation.z += qDot.z * deltaTime * 0.5f;
            pose.orientation.w += qDot.w * deltaTime * 0.5f;

            // Sayısal kararlılık için normalizasyon şarttır
            pose.orientation.normalize();
        }

        // Split hızları sıfırla (sadece bu karedeki geometrik ayrılma içindi)
        motion.splitLinearVelocity = Vector3(0.0f, 0.0f, 0.0f);
        motion.splitAngularVelocity = Vector3(0.0f, 0.0f, 0.0f);

        // Dünya Uzayı Eylemsizliği: I_world⁻¹ = R · I_local⁻¹ · Rᵀ
        Matrix3x3 R = pose.getOrientationMatrix();
        mass.inverseInertiaTensor = R * mass.inverseLocalInertiaTensor * R.getTranspose();
        if (state.lockedAxes & Core::LockAxis::AngularX) {
            mass.inverseInertiaTensor.rows[0] = Vector3(0.0f, 0.0f, 0.0f);
            mass.inverseInertiaTensor.rows[1].x = 0.0f;
            mass.inverseInertiaTensor.rows[2].x = 0.0f;
        }
        if (state.lockedAxes & Core::LockAxis::AngularY) {
            mass.inverseInertiaTensor.rows[1] = Vector3(0.0f, 0.0f, 0.0f);
            mass.inverseInertiaTensor.rows[0].y = 0.0f;
            mass.inverseInertiaTensor.rows[2].y = 0.0f;
        }
        if (state.lockedAxes & Core::LockAxis::AngularZ) {
            mass.inverseInertiaTensor.rows[2] = Vector3(0.0f, 0.0f, 0.0f);
            mass.inverseInertiaTensor.rows[0].z = 0.0f;
            mass.inverseInertiaTensor.rows[1].z = 0.0f;
        }
    }
}

} // namespace Baryon::systems