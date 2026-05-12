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
        // Sönümleme (Hava direnci)
        float linearDamping = 0.01f; 
        motion.linearVelocity *= (1.0f - linearDamping * deltaTime);

        // İvme hesaplama: a = F/m + g
        Vector3 linearAcceleration = mAccelerationField + (motion.externalForce * mass.inverseMass);
        motion.linearVelocity += linearAcceleration * deltaTime;

        // --- Açısal Hız Entegrasyonu ---
        float angularDamping = 1.0f; 
        motion.angularVelocity *= (1.0f - angularDamping * deltaTime);

        // Açısal İvme: α = I⁻¹τ
        Vector3 angularAcceleration = mass.inverseInertiaTensor * motion.externalTorque;
        motion.angularVelocity += angularAcceleration * deltaTime;

        // --- Hız Eşiği (Jitter Önleme) ---
        if (motion.linearVelocity.lengthSquare() < physics::PhysicsConstants::VelocityDeadZoneSq) {
            motion.linearVelocity = Vector3(0.0f, 0.0f, 0.0f);
        }
        if (motion.angularVelocity.lengthSquare() < physics::PhysicsConstants::VelocityDeadZoneSq) {
            motion.angularVelocity = Vector3(0.0f, 0.0f, 0.0f);
        }

        // Kuvvetleri sıfırla (Her karede kullanıcı tarafından yeniden uygulanmalıdır)
        motion.externalForce = Vector3(0.0f, 0.0f, 0.0f);
        motion.externalTorque = Vector3(0.0f, 0.0f, 0.0f);
    }
}

/**
 * @brief 2. AŞAMA: Konum ve Rotasyon Entegrasyonu (Çözücü sonrası çalışır).
 * @details Güncel hız değerlerini kullanarak cismin uzaydaki yerini belirler:
 *
 *          1. Konum Güncelleme: Mevcut pozisyona hız vektörü eklenir.
 *          2. Rotasyon Güncelleme: Açısal hıza bağlı olarak kuaterniyon türevi alınır.
 *          3. Normalizasyon: Sayısal hataların birikmesini önlemek için yönelim normalize edilir.
 *          4. Dünya Eylemsizliği: Cismin dönüşüne bağlı olarak eylemsizlik tensörü dünya uzayına uyarlanır.
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

        // Doğrusal Hareket: p = p + v·dt
        pose.position += motion.linearVelocity * deltaTime;

        // Açısal Hareket (Kuaterniyon türevi entegrasyonu)
        Quaternion spin(
            motion.angularVelocity.x,
            motion.angularVelocity.y,
            motion.angularVelocity.z,
            0.0f
        );
        Quaternion qDot = spin * pose.orientation;
        pose.orientation.x += qDot.x * deltaTime * 0.5f;
        pose.orientation.y += qDot.y * deltaTime * 0.5f;
        pose.orientation.z += qDot.z * deltaTime * 0.5f;
        pose.orientation.w += qDot.w * deltaTime * 0.5f;

        // Sayısal kararlılık için normalizasyon şarttır
        pose.orientation.normalize();

        // Dünya Uzayı Eylemsizliği: I_world⁻¹ = R · I_local⁻¹ · Rᵀ
        
        Matrix3x3 R = pose.getOrientationMatrix();
        mass.inverseInertiaTensor = R * mass.localInertiaTensor.getInverse() * R.getTranspose();
    }
}

} // namespace Baryon::systems