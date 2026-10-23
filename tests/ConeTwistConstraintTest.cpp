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

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/Core/Snapshot.hpp>
#include <Baryon/collision/ConeTwistConstraint.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief İki cisim arasındaki güncel burulma (twist) açısını hesaplayan yardımcı fonksiyon.
 */
float computeTwistAngle(const Pose& poseA, const Pose& poseB, const ConeTwistConstraint& constraint) {
    Vector3 uA = (poseA.orientation * constraint.twistAxisLocalA).getNormalized();
    Vector3 uB = (poseB.orientation * constraint.twistAxisLocalB).getNormalized();
    Vector3 uAvg = (uA + uB);
    if (uAvg.length() <= 1e-5f)
        return 0.0f;
    uAvg = uAvg.getNormalized();

    Vector3 a_A = constraint.twistAxisLocalA.getNormalized();
    Vector3 p_A1;
    if (std::abs(a_A.x) >= 0.57735f)
        p_A1 = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
    else
        p_A1 = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();

    Vector3 p_B1 = constraint.initialRelativeOrientation.getInverse() * p_A1;
    Vector3 p_worldA = poseA.orientation * p_A1;
    Vector3 p_worldB = poseB.orientation * p_B1;

    Vector3 pA_proj = (p_worldA - uAvg * p_worldA.dot(uAvg));
    Vector3 pB_proj = (p_worldB - uAvg * p_worldB.dot(uAvg));
    if (pA_proj.length() <= 1e-5f || pB_proj.length() <= 1e-5f)
        return 0.0f;

    pA_proj = pA_proj.getNormalized();
    pB_proj = pB_proj.getNormalized();

    float sinTwist = (pA_proj.cross(pB_proj)).dot(uAvg);
    float cosTwist = std::clamp(pA_proj.dot(pB_proj), -1.0f, 1.0f);
    return std::atan2(sinTwist, cosTwist);
}

} // namespace

/**
 * @brief 1. Test: Koni Salınım Açısı Sınırı Doğrulaması.
 * @details Salınım açısı kısıtlanan bir kol dış kuvvetle salındığında maksimum koni yarı-açısını aşmamalıdır.
 */
void testSwingConeLimit() {
    std::cout << "--- 1. Test: Koni Salinim Acisi Siniri Dogrulamasi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    // Sabit mafsal yuvası
    Pose anchorPose(Vector3(0.0f, 0.0f, 0.0f));
    Body base = sim.createBody(anchorPose, CollisionShape{SphereShape(0.1f)});
    (void)base.setBodyType(Core::BodyType::Static);
    (void)base.setCollisionMask(0);

    // Dinamik sarkaç kolu (X ekseni boyunca uzanan)
    Pose armPose(Vector3(1.0f, 0.0f, 0.0f));
    Body arm = sim.createBody(armPose, CollisionShape{BoxShape(Vector3(0.8f, 0.1f, 0.1f))});
    (void)arm.setBodyType(Core::BodyType::Dynamic);
    (void)arm.setMass(2.0f);
    (void)arm.setCollisionMask(0);

    // Koni salınım sınırı: 0.50 rad (~28.6 derece)
    const float swingLimit = 0.50f;
    sim.createConeTwistConstraint(base, arm,
                                  Vector3(0.0f, 0.0f, 0.0f),
                                  Vector3(-1.0f, 0.0f, 0.0f),
                                  Vector3(1.0f, 0.0f, 0.0f),
                                  Vector3(1.0f, 0.0f, 0.0f),
                                  swingLimit,
                                  -1.57f, 1.57f);

    // Z ekseni etrafında salınım başlat
    auto& motionArm = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity());
    motionArm.angularVelocity = Vector3(0.0f, 0.0f, 3.0f);

    const float dt = 1.0f / 60.0f;
    sim.setFixedTimeStep(dt);

    float maxObservedSwing = 0.0f;
    for (int step = 0; step < 120; ++step) {
        sim.step(dt);

        const auto& poseA = sim.getRegistry().getComponent<Pose>(base.getEntity());
        const auto& poseB = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        Vector3 uA = (poseA.orientation * Vector3(1.0f, 0.0f, 0.0f)).getNormalized();
        Vector3 uB = (poseB.orientation * Vector3(1.0f, 0.0f, 0.0f)).getNormalized();
        float cosTheta = std::clamp(uA.dot(uB), -1.0f, 1.0f);
        float currentSwing = std::acos(cosTheta);
        maxObservedSwing = std::max(maxObservedSwing, currentSwing);
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Hedef Koni Limiti    : " << swingLimit << " rad" << std::endl;
    std::cout << "  -> Olculen Maks Salinim : " << maxObservedSwing << " rad" << std::endl;

    CHECK_MSG(maxObservedSwing > 0.40f, "Kol sınır bölgesine ulaşabilmelidir");
    CHECK_MSG(maxObservedSwing <= swingLimit + 0.015f, "Koni salınım açısı limiti aşmamalıdır");
    std::cout << "  -> PASSED: Koni salinim acisi basariyla sinirlandirildi." << std::endl;
}

/**
 * @brief 2. Test: Eksenel Burulma (Twist) Sınırları Doğrulaması.
 * @details Eksen etrafında tork uygulandığında açı [lowerTwist, upperTwist] sınırlarına oturmalı ve aşmamalıdır.
 */
void testTwistLimit() {
    std::cout << "--- 2. Test: Eksenel Burulma Sinirlari Dogrulamasi ---" << std::endl;

    const float lowerTwist = -0.35f; // ~ -20 derece
    const float upperTwist = 0.50f;  // ~ +28.6 derece
    const float dt = 1.0f / 60.0f;

    // 2a: Pozitif burulma testi (Pozitif tork ile üst limite oturma)
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

        Body base = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.1f)});
        (void)base.setBodyType(Core::BodyType::Static);
        (void)base.setCollisionMask(0);

        Body arm = sim.createBody(Pose(Vector3(1.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.8f, 0.1f, 0.1f))});
        (void)arm.setBodyType(Core::BodyType::Dynamic);
        (void)arm.setMass(1.5f);
        (void)arm.setCollisionMask(0);

        sim.createConeTwistConstraint(base, arm,
                                      Vector3(0.0f, 0.0f, 0.0f),
                                      Vector3(-1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      0.2f, lowerTwist, upperTwist);

        sim.setFixedTimeStep(dt);

        // X ekseni boyunca sürekli pozitif tork uygula
        for (int step = 0; step < 120; ++step) {
            auto& motionArm = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity());
            motionArm.externalTorque = Vector3(4.0f, 0.0f, 0.0f);
            sim.step(dt);
        }

        const auto& poseA = sim.getRegistry().getComponent<Pose>(base.getEntity());
        const auto& poseB = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        const auto& ctc = sim.getRegistry().getComponent<ConeTwistConstraint>(base.getEntity());
        float finalTwist = computeTwistAngle(poseA, poseB, ctc);
        float limitPenetration = std::max(0.0f, finalTwist - upperTwist);

        std::cout << std::fixed << std::setprecision(5);
        std::cout << "  -> Pozitif Twist Limiti: " << upperTwist << " rad, Olculen: " << finalTwist
                  << " rad (Penetrasyon: " << limitPenetration << " rad)" << std::endl;
        CHECK_MSG(finalTwist >= upperTwist - 0.02f, "Pozitif burulma sınırına ulaşmış olmalıdır");
        CHECK_MSG(limitPenetration <= 0.010f, "Pozitif burulma üst sınır penetrasyonu <= 0.01 rad olmalıdır");
    }

    // 2b: Negatif burulma testi (Negatif tork ile alt limite oturma)
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

        Body base = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.1f)});
        (void)base.setBodyType(Core::BodyType::Static);
        (void)base.setCollisionMask(0);

        Body arm = sim.createBody(Pose(Vector3(1.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.8f, 0.1f, 0.1f))});
        (void)arm.setBodyType(Core::BodyType::Dynamic);
        (void)arm.setMass(1.5f);
        (void)arm.setCollisionMask(0);

        sim.createConeTwistConstraint(base, arm,
                                      Vector3(0.0f, 0.0f, 0.0f),
                                      Vector3(-1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      0.2f, lowerTwist, upperTwist);

        sim.setFixedTimeStep(dt);

        // X ekseni boyunca sürekli negatif tork uygula
        for (int step = 0; step < 120; ++step) {
            auto& motionArm = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity());
            motionArm.externalTorque = Vector3(-4.0f, 0.0f, 0.0f);
            sim.step(dt);
        }

        const auto& poseA = sim.getRegistry().getComponent<Pose>(base.getEntity());
        const auto& poseB = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        const auto& ctc = sim.getRegistry().getComponent<ConeTwistConstraint>(base.getEntity());
        float finalTwist = computeTwistAngle(poseA, poseB, ctc);
        float limitPenetration = std::max(0.0f, lowerTwist - finalTwist);

        std::cout << "  -> Negatif Twist Limiti: " << lowerTwist << " rad, Olculen: " << finalTwist
                  << " rad (Penetrasyon: " << limitPenetration << " rad)" << std::endl;
        CHECK_MSG(finalTwist <= lowerTwist + 0.02f, "Negatif burulma sınırına ulaşmış olmalıdır");
        CHECK_MSG(limitPenetration <= 0.010f, "Negatif burulma alt sınır penetrasyonu <= 0.01 rad olmalıdır");
    }

    std::cout << "  -> PASSED: Eksenel burulma alt ve ust sinirlari dogrulandi." << std::endl;
}

/**
 * @brief 3. Test: Ragdoll Kol Modeli ve Kararlılık (Yerçekimi & Ankor Drift Testi).
 * @details Statik omuz eklemine bağlı kol yerçekimi altında salınırken ankor noktası ayrılmamalıdır (drift < 1 mm).
 */
void testRagdollArmModel() {
    std::cout << "--- 3. Test: Ragdoll Kol Modeli ve Kararlilik Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Statik omuz
    Pose shoulderPose(Vector3(0.0f, 2.0f, 0.0f));
    Body shoulder = sim.createBody(shoulderPose, CollisionShape{SphereShape(0.15f)});
    (void)shoulder.setBodyType(Core::BodyType::Static);
    (void)shoulder.setCollisionMask(0);

    // Dinamik üst kol (Y ekseni boyunca uzanan silindir: halfHeight=0.40m, boy=0.80m)
    // Kütle merkezi (0, 1.35, 0). Üst uç (0, 1.75, 0) seviyesindedir.
    Pose armPose(Vector3(0.0f, 1.35f, 0.0f));
    Body upperArm = sim.createBody(armPose, CollisionShape{CylinderShape(0.08f, 0.40f)});
    (void)upperArm.setBodyType(Core::BodyType::Dynamic);
    (void)upperArm.setMass(3.5f);
    (void)upperArm.setCollisionMask(0);

    // Ankor noktası: Dünya uzayında (0.0, 1.75, 0.0)
    // Omuz yerel: (0, -0.25, 0), Kol yerel: (0, 0.40, 0)
    // Burulma ekseni: -Y ekseni (0, -1, 0)
    const float swingLimit = static_cast<float>(std::numbers::pi) / 3.0f; // 60 derece
    const float twistLimit = static_cast<float>(std::numbers::pi) / 4.0f; // 45 derece
    sim.createConeTwistConstraint(shoulder, upperArm,
                                  Vector3(0.0f, -0.25f, 0.0f),
                                  Vector3(0.0f, 0.40f, 0.0f),
                                  Vector3(0.0f, -1.0f, 0.0f),
                                  Vector3(0.0f, -1.0f, 0.0f),
                                  swingLimit,
                                  -twistLimit, twistLimit);

    // Başlangıçta 3B salınım başlatmak için hafif çapraz hız ver
    auto& motionArm = sim.getRegistry().getComponent<Core::Motion>(upperArm.getEntity());
    motionArm.linearVelocity = Vector3(1.5f, 0.0f, 0.8f);

    const float dt = 1.0f / 60.0f;
    sim.setFixedTimeStep(dt);

    float maxAnchorDrift = 0.0f;

    for (int step = 0; step < 300; ++step) { // 5 saniye
        sim.step(dt);

        const auto& poseA = sim.getRegistry().getComponent<Pose>(shoulder.getEntity());
        const auto& poseB = sim.getRegistry().getComponent<Pose>(upperArm.getEntity());
        const auto& motionB = sim.getRegistry().getComponent<Core::Motion>(upperArm.getEntity());

        Vector3 pA = poseA.position + poseA.orientation * Vector3(0.0f, -0.25f, 0.0f);
        Vector3 pB = poseB.position + poseB.orientation * Vector3(0.0f, 0.40f, 0.0f);
        float drift = (pA - pB).length();
        maxAnchorDrift = std::max(maxAnchorDrift, drift);

        // NaN / Inf kontrolü
        CHECK_MSG(!std::isnan(poseB.position.x) && !std::isinf(poseB.position.x), "Pozisyon NaN/Inf olamaz");
        CHECK_MSG(!std::isnan(motionB.linearVelocity.x) && !std::isinf(motionB.linearVelocity.x), "Hız NaN/Inf olamaz");
    }

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  -> Maksimum Ankor Drift : " << (maxAnchorDrift * 1000.0f) << " mm (Hedef: < 1.0 mm)" << std::endl;
    CHECK_MSG(maxAnchorDrift < 0.001f, "Ankor drift 1.0 mm altında kalmalıdır");
    std::cout << "  -> PASSED: Ragdoll kol modeli kararlilikla 300 adim kosturuldu." << std::endl;
}

/**
 * @brief 4. Test: Snapshot Determinizmi ve Kilitli Eklem (Degenerate Case) Testi.
 * @details Snapshot geri yükleme bit-düzeyi özdeşlik sağlamalı, [0, 0, 0] sınırları eklemi tam kilitlemelidir.
 */
void testSnapshotAndDegenerateLock() {
    std::cout << "--- 4. Test: Snapshot Determinizmi ve Kilitli Eklem Testi ---" << std::endl;

    // 4a: Snapshot Determinizmi
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        Body base = sim.createBody(Pose(Vector3(0.0f, 5.0f, 0.0f)), CollisionShape{SphereShape(0.1f)});
        (void)base.setBodyType(Core::BodyType::Static);
        (void)base.setCollisionMask(0);

        Body arm = sim.createBody(Pose(Vector3(1.0f, 5.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.8f, 0.1f, 0.1f))});
        (void)arm.setBodyType(Core::BodyType::Dynamic);
        (void)arm.setMass(2.0f);
        (void)arm.setCollisionMask(0);

        sim.createConeTwistConstraint(base, arm,
                                      Vector3(0.0f, 0.0f, 0.0f),
                                      Vector3(-1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      0.5f, -0.3f, 0.3f);

        const float dt = 1.0f / 60.0f;
        sim.setFixedTimeStep(dt);

        for (int i = 0; i < 30; ++i) {
            sim.step(dt);
        }

        // Snapshot kaydet
        Core::Snapshot snap = sim.saveSnapshot();

        for (int i = 0; i < 30; ++i) {
            sim.step(dt);
        }

        Pose poseAt60 = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        Vector3 linVelAt60 = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity()).linearVelocity;

        // Snapshot geri yükle ve tekrar koştur
        sim.restoreSnapshot(snap);
        for (int i = 0; i < 30; ++i) {
            sim.step(dt);
        }

        Pose poseRestored60 = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        Vector3 linVelRestored60 = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity()).linearVelocity;

        float posDiff = (poseAt60.position - poseRestored60.position).length();
        float velDiff = (linVelAt60 - linVelRestored60).length();

        CHECK_MSG(posDiff < 1e-5f, "Snapshot geri yükleme pozisyonu tam korumalı");
        CHECK_MSG(velDiff < 1e-5f, "Snapshot geri yükleme hızı tam korumalı");
        std::cout << "  -> Snapshot determinizmi dogrulandi (Pos farki: " << posDiff << " m)." << std::endl;
    }

    // 4b: Tam Kilitli Eklem (Degenerate: swing=0, twist=[0, 0])
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

        Body base = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.1f)});
        (void)base.setBodyType(Core::BodyType::Static);
        (void)base.setCollisionMask(0);

        Body arm = sim.createBody(Pose(Vector3(1.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.8f, 0.1f, 0.1f))});
        (void)arm.setBodyType(Core::BodyType::Dynamic);
        (void)arm.setMass(2.0f);
        (void)arm.setCollisionMask(0);

        // swing=0.0f, lowerTwist=0.0f, upperTwist=0.0f -> Tam kilit
        sim.createConeTwistConstraint(base, arm,
                                      Vector3(0.0f, 0.0f, 0.0f),
                                      Vector3(-1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      Vector3(1.0f, 0.0f, 0.0f),
                                      0.0f, 0.0f, 0.0f);

        auto& motionArm = sim.getRegistry().getComponent<Core::Motion>(arm.getEntity());
        motionArm.angularVelocity = Vector3(2.0f, 3.0f, -2.0f); // Rastgele dönme zorlaması

        const float dt = 1.0f / 60.0f;
        sim.setFixedTimeStep(dt);

        for (int i = 0; i < 60; ++i) {
            sim.step(dt);
        }

        const auto& poseB = sim.getRegistry().getComponent<Pose>(arm.getEntity());
        Vector3 uB = (poseB.orientation * Vector3(1.0f, 0.0f, 0.0f)).getNormalized();
        float swingAngle = std::acos(std::clamp(uB.dot(Vector3(1.0f, 0.0f, 0.0f)), -1.0f, 1.0f));

        CHECK_MSG(swingAngle < 0.02f, "Tam kilitli eklemde salınım açısı sıfıra yakın kalmalıdır");
        std::cout << "  -> Tam kilitli eklem salinim sapmasi: " << swingAngle << " rad (< 0.02 rad)." << std::endl;
    }

    std::cout << "  -> PASSED: Snapshot determinizmi ve kilitli eklem testleri basarili." << std::endl;
}

int main() {
    std::cout << "=================================================" << std::endl;
    std::cout << "  Baryon Fizik Motoru - ConeTwistConstraint Testi" << std::endl;
    std::cout << "=================================================" << std::endl;

    try {
        testSwingConeLimit();
        testTwistLimit();
        testRagdollArmModel();
        testSnapshotAndDegenerateLock();

        std::cout << "\n>>> CONETWIST CONSTRAINT TUM TESTLER BASARIYLA TAMAMLANDI! <<<" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "TEST BASARISIZ: " << e.what() << std::endl;
        return 1;
    }
}
