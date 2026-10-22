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
#include <Baryon/collision/PrismaticConstraint.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @brief 1. Test: Eksen Dışı Kayma ve Açısal Kilit Doğrulaması.
 * @details X ekseni boyunca kayan cisme Y ve Z eksenlerinde yan kuvvetler ve torklar uygulanır.
 *          Kabul Kriteri: Eksen dışı kayma < 1 mm, açısal dönme kilitli (< 0.1 derece).
 */
void testOffAxisDriftAndRotationLock() {
    std::cout << "--- 1. Test: Eksen Disi Kayma ve Acisal Kilit Dogrulamasi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0)); // Ağırlıksız ortam

    // Statik ray kılavuz gövdesi
    Body rail = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{BoxShape(Vector3(5.0f, 0.2f, 0.2f))});
    (void)rail.setBodyType(Core::BodyType::Static);

    // Kayan araba (slider)
    Body slider = sim.createBody(Pose(Vector3(0, 7, 0)), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
    (void)slider.setBodyType(Core::BodyType::Dynamic);
    (void)slider.setMass(2.0f);

    // X ekseni boyunca prizmatik eklem
    sim.createPrismaticConstraint(rail, slider, Vector3(0, 2, 0), Vector3(0, 0, 0), Vector3(1, 0, 0));

    // X ekseninde hız ver + eksen dışı saptırıcı kuvvet ve tork uygula
    (void)slider.setLinearVelocity(Vector3(3.0f, 0.0f, 0.0f));

    float maxOffAxisDrift = 0.0f;
    float maxAngleDeviation = 0.0f;

    for (int step = 0; step < 180; ++step) {
        // Her adımda Y ve Z doğrultusunda bozucu kuvvet ve tork uygula
        (void)slider.applyForce(Vector3(0.0f, 50.0f, -30.0f));
        (void)slider.applyTorque(Vector3(10.0f, -15.0f, 20.0f));

        sim.step(1.0f / 60.0f);

        const auto& poseS = sim.getRegistry().getComponent<Pose>(slider.getEntity());
        const auto& poseR = sim.getRegistry().getComponent<Pose>(rail.getEntity());
        const auto& pr = sim.getRegistry().getComponent<PrismaticConstraint>(rail.getEntity());

        // Eksen dışı öteleme hatası (Y ve Z mesafeleri)
        Vector3 rRail = poseR.orientation * pr.localAnchorA;
        Vector3 rSlider = poseS.orientation * pr.localAnchorB;
        Vector3 d = (poseS.position + rSlider) - (poseR.position + rRail);
        Vector3 u = (poseR.orientation * pr.localAxisA).getNormalized();
        Vector3 offAxisVec = d - u * d.dot(u);
        float offAxisDrift = offAxisVec.length();
        if (offAxisDrift > maxOffAxisDrift) {
            maxOffAxisDrift = offAxisDrift;
        }

        // Açısal sapma hesabı
        Quaternion qDiff = poseS.orientation * (poseR.orientation * pr.initialRelativeOrientation).getInverse();
        if (qDiff.w < 0.0f) qDiff = Quaternion(-qDiff.x, -qDiff.y, -qDiff.z, -qDiff.w);
        float angleRad = 2.0f * std::atan2(Vector3(qDiff.x, qDiff.y, qDiff.z).length(), qDiff.w);
        float angleDeg = angleRad * (180.0f / 3.14159265f);
        if (angleDeg > maxAngleDeviation) {
            maxAngleDeviation = angleDeg;
        }
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Maksimum Eksen Disi Kayma : " << (maxOffAxisDrift * 1000.0f) << " mm (Hedef: < 1.0 mm)" << std::endl;
    std::cout << "  -> Maksimum Acisal Sapma     : " << maxAngleDeviation << " derece (Hedef: < 0.1 derece)" << std::endl;

    CHECK_MSG(maxOffAxisDrift < 0.001f, "Prizmatik eklem eksen dışı kayması 1 mm'den az olmalı");
    CHECK_MSG(maxAngleDeviation < 0.1f, "Prizmatik eklemde dönme kilitli kalmalı (< 0.1 derece)");
}

/**
 * @brief 2. Test: Alt/Üst Sınırlar ve Yerçekimi Altında Limite Oturma.
 * @details Düşey Y ekseninde yerçekimi altında serbest bırakılan kutu, alt limite oturur.
 *          Kabul Kriteri: Limitte durur, limit içine sızmaz (< 2 mm), yerçekimi altında stabil kalır.
 */
void testLimitsAndGravityRest() {
    std::cout << "--- 2. Test: Alt/Ust Sinirlar ve Yercekimi Altinda Limite Oturma ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0)); // Düşey yerçekimi

    // Kılavuz mesnet
    Body post = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{BoxShape(Vector3(0.2f, 5.0f, 0.2f))});
    (void)post.setBodyType(Core::BodyType::Static);

    // Kayan asansör kabini
    Body elevator = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)elevator.setBodyType(Core::BodyType::Dynamic);
    (void)elevator.setMass(3.0f);

    // Y ekseni boyunca kayar eklem
    sim.createPrismaticConstraint(post, elevator, Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 1, 0));

    // Limitler: Alt limit -2.0 m, Üst limit +1.0 m
    const float lowerLimit = -2.0f;
    const float upperLimit = 1.0f;
    sim.setPrismaticLimits(post, lowerLimit, upperLimit);

    float maxPenetration = 0.0f;

    // 180 adım simüle et (3 saniye - yerçekimiyle düşüp limite oturması için yeterli)
    for (int step = 0; step < 180; ++step) {
        sim.step(1.0f / 60.0f);

        const auto& poseE = sim.getRegistry().getComponent<Pose>(elevator.getEntity());
        float relativeY = poseE.position.y - 10.0f;

        // Alt limit ihlali (penetrasyon)
        if (relativeY < lowerLimit) {
            float pen = lowerLimit - relativeY;
            if (pen > maxPenetration) {
                maxPenetration = pen;
            }
        }
    }

    const auto& finalPose = sim.getRegistry().getComponent<Pose>(elevator.getEntity());
    const auto& finalMotion = sim.getRegistry().getComponent<Core::Motion>(elevator.getEntity());
    float finalRelativeY = finalPose.position.y - 10.0f;
    float finalSpeed = finalMotion.linearVelocity.length();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Final Konum (Y)           : " << finalRelativeY << " m (Hedef: ~" << lowerLimit << " m)" << std::endl;
    std::cout << "  -> Maksimum Limit Ihlali     : " << (maxPenetration * 1000.0f) << " mm (Hedef: < 2.0 mm)" << std::endl;
    std::cout << "  -> Final Hiz (Kararlilik)    : " << finalSpeed << " m/s (Hedef: < 0.1 m/s)" << std::endl;

    CHECK_MSG(maxPenetration < 0.002f, "Asansör alt limite çarptığında limit içine 2 mm'den fazla sızmamalı");
    CHECK_MSG(std::abs(finalRelativeY - lowerLimit) < 0.005f, "Yerçekimi altında cisim alt limite oturmalı");
    CHECK_MSG(finalSpeed < 0.1f, "Limitte durduktan sonra cisim kararlı şekilde dinlenmeli");
}

/**
 * @brief 3. Test: Doğrusal Motor Testi (Linear Motor).
 * @details Yerçekimine karşı yukarı doğru iten motorun hedef hıza ulaşması ve kuvvet sınırını aşmaması.
 */
void testLinearMotor() {
    std::cout << "--- 3. Test: Dogrusal Motor Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body guide = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{BoxShape(Vector3(0.2f, 5.0f, 0.2f))});
    (void)guide.setBodyType(Core::BodyType::Static);

    Body car = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
    (void)car.setBodyType(Core::BodyType::Dynamic);
    (void)car.setMass(2.0f);

    sim.createPrismaticConstraint(guide, car, Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 1, 0));

    // Motor hızı: 2.0 m/s (yukarı), Maksimum motor kuvveti: 100 N (2 kg için mg=19.62 N olduğundan rahatça kaldırır)
    const float targetSpeed = 2.0f;
    sim.setPrismaticMotor(guide, targetSpeed, 100.0f);

    for (int i = 0; i < 90; ++i) {
        sim.step(1.0f / 60.0f);
    }

    const auto& motion = sim.getRegistry().getComponent<Core::Motion>(car.getEntity());
    float actualSpeed = motion.linearVelocity.y;
    float speedError = std::abs(actualSpeed - targetSpeed);

    std::cout << "  -> Hedef Motor Hizi : " << targetSpeed << " m/s" << std::endl;
    std::cout << "  -> Olculen Hiz      : " << actualSpeed << " m/s (Fark: " << speedError << " m/s)" << std::endl;

    CHECK_MSG(speedError < 0.05f, "Doğrusal motor yük altında hedef hıza ulaşmalı (±0.05 m/s)");
}

/**
 * @brief 4. Test: Snapshot Determinizmi ve Dejenere Durumlar.
 */
void testSnapshotAndDegenerate() {
    std::cout << "--- 4. Test: Snapshot Determinizmi ve Dejenere Durumlar ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body a = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{BoxShape(Vector3(1, 1, 1))});
    (void)a.setBodyType(Core::BodyType::Static);

    Body b = sim.createBody(Pose(Vector3(0, 2, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b.setBodyType(Core::BodyType::Dynamic);
    (void)b.setMass(1.0f);

    sim.createPrismaticConstraint(a, b, Vector3(0, 0, 0), Vector3(0, 0, 0), Vector3(0, 1, 0));
    sim.setPrismaticLimits(a, -1.0f, 3.0f);

    for (int i = 0; i < 30; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Core::Snapshot snap = sim.saveSnapshot();

    std::vector<Vector3> traj1;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj1.push_back(sim.getRegistry().getComponent<Pose>(b.getEntity()).position);
    }

    sim.restoreSnapshot(snap);

    std::vector<Vector3> traj2;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj2.push_back(sim.getRegistry().getComponent<Pose>(b.getEntity()).position);
    }

    for (size_t i = 0; i < 60; ++i) {
        CHECK_MSG(traj1[i].x == traj2[i].x &&
                  traj1[i].y == traj2[i].y &&
                  traj1[i].z == traj2[i].z,
                  "Snapshot geri yükleme deterministik olmalı");
    }

    std::cout << "  -> PASSED: Snapshot ve sınır durumları güvenli." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.4: PRISMATIC CONSTRAINT TESTLERI ===\n" << std::endl;

    testOffAxisDriftAndRotationLock();
    testLimitsAndGravityRest();
    testLinearMotor();
    testSnapshotAndDegenerate();

    return reportAndExit();
}
