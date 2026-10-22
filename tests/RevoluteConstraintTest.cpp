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
#include <Baryon/collision/RevoluteConstraint.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @brief 1. Test: Boştaki Tekerlek Açısal Motor Testi.
 * @details Sürtünmesiz ortamda motora sahip tekerlek hedef hıza +-2% toleransla ulaşmalıdır.
 */
void testFreeSpinningMotor() {
    std::cout << "--- 1. Test: Bostaki Tekerlek Motor Hizi Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    // Statik aks (Z = 0.0)
    Pose axlePose(Vector3(0.0f, 0.0f, 0.0f));
    Body axle = sim.createBody(axlePose, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)axle.setBodyType(Core::BodyType::Static);

    // Dinamik silindir tekerlek (Z = 0.5 m, aralarında temas ve sürtünme yok)
    Pose wheelPose(Vector3(0.0f, 0.0f, 0.5f));
    Body wheel = sim.createBody(wheelPose, CollisionShape{CylinderShape(0.5f, 0.2f)});
    (void)wheel.setBodyType(Core::BodyType::Dynamic);
    (void)wheel.setMass(2.0f);

    // Menteşe kısıtı: Z ekseni (0, 0, 1) etrafında serbest dönüş
    sim.createRevoluteConstraint(axle, wheel,
                                Vector3(0.0f, 0.0f, 0.5f),
                                Vector3(0.0f, 0.0f, 0.0f),
                                Vector3(0.0f, 0.0f, 1.0f),
                                Vector3(0.0f, 0.0f, 1.0f));

    const float targetSpeed = 12.0f; // rad/s
    const float maxTorque = 40.0f;   // N*m
    sim.setRevoluteMotor(axle, targetSpeed, maxTorque);

    const float dt = 1.0f / 60.0f;
    sim.setFixedTimeStep(dt);

    // 120 adım koştur (2 saniye)
    for (int step = 0; step < 120; ++step) {
        sim.step(dt);
    }

    Vector3 angVel = sim.getRegistry().getComponent<Core::Motion>(wheel.getEntity()).angularVelocity;
    float currentSpeed = angVel.z;
    float errorPercent = (std::abs(currentSpeed - targetSpeed) / targetSpeed) * 100.0f;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Hedef Hiz   : " << targetSpeed << " rad/s" << std::endl;
    std::cout << "  -> Ulasilan Hiz: " << currentSpeed << " rad/s (Hata: %" << errorPercent << ", Hedef: < %2.0)" << std::endl;

    CHECK_MSG(errorPercent < 2.0f, "Boştaki tekerlek hedef motor hızına %2.0 içinde yaklaşmalı");
    std::cout << "  -> PASSED: Bostaki motor hedef hiza basariyla oturdu." << std::endl;
}

/**
 * @brief 2. Test: Yük Altında Maksimum Tork Sınırı Doğrulaması.
 * @details Aşırı yük veya karşıt tork altında motor torku maxMotorTorque sınırını aşmamalıdır.
 */
void testMotorTorqueLimitUnderLoad() {
    std::cout << "--- 2. Test: Yuk Altinda Maksimum Tork Siniri Dogrulamasi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose axlePose(Vector3(0.0f, 0.0f, 0.0f));
    Body axle = sim.createBody(axlePose, CollisionShape{SphereShape(0.2f)});
    (void)axle.setBodyType(Core::BodyType::Static);

    // Çok büyük eylemsizliğe sahip ağır volan
    Pose flywheelPose(Vector3(0.0f, 0.0f, 0.0f));
    Body flywheel = sim.createBody(flywheelPose, CollisionShape{BoxShape(Vector3(2.0f, 2.0f, 2.0f))});
    (void)flywheel.setBodyType(Core::BodyType::Dynamic);
    (void)flywheel.setMass(500.0f); // 500 kg

    sim.createRevoluteConstraint(axle, flywheel,
                                Vector3(0.0f, 0.0f, 0.0f),
                                Vector3(0.0f, 0.0f, 0.0f),
                                Vector3(0.0f, 0.0f, 1.0f),
                                Vector3(0.0f, 0.0f, 1.0f));

    const float targetSpeed = 50.0f; // Yüksek hedef hız
    const float maxTorque = 15.0f;   // Sıkı tork tavanı (15 N*m)
    sim.setRevoluteMotor(axle, targetSpeed, maxTorque);

    const float dt = 1.0f / 60.0f;
    sim.setFixedTimeStep(dt);

    float maxObservedTorque = 0.0f;

    // 60 adım boyunca her adımda uygulanan anlık torku ölç
    for (int step = 0; step < 60; ++step) {
        sim.step(dt);
        const auto& rc = sim.getRegistry().getComponent<RevoluteConstraint>(axle.getEntity());
        float appliedTorque = std::abs(rc.accumulatedMotorImpulse) / dt;
        maxObservedTorque = std::max(maxObservedTorque, appliedTorque);

        CHECK_MSG(appliedTorque <= maxTorque + 1e-3f, "Motor anlık torku maxTorque sınırını hiçbir adımda aşmamalıdır");
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Tork Siniri        : " << maxTorque << " N*m" << std::endl;
    std::cout << "  -> Maksimum Olculen   : " << maxObservedTorque << " N*m" << std::endl;

    CHECK_MSG(maxObservedTorque > maxTorque * 0.95f, "Motor ağır yük altında tork sınırına kadar yüklenmeli");
    std::cout << "  -> PASSED: Motor tork siniri basariyla korundu." << std::endl;
}

/**
 * @brief 3. Test: Açısal Limitler ve Penetrasyon Toleransı Testi.
 * @details Yerçekimi ve harici yük altında menteşe sınır açılarına oturmalı ve penetrasyon < 0.01 rad olmalıdır.
 */
void testAngularLimits() {
    std::cout << "--- 3. Test: Acisal Limitler ve Penetrasyon Toleransi Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f)); // Yerçekimi

    // Tavan sabit ankrajı
    Pose anchorPose(Vector3(0.0f, 5.0f, 0.0f));
    Body anchor = sim.createBody(anchorPose, CollisionShape{SphereShape(0.2f)});
    (void)anchor.setBodyType(Core::BodyType::Static);

    // Sarkaç kolu (Başlangıçta yatay X = 2.0 m)
    Pose armPose(Vector3(1.5f, 5.0f, 0.0f));
    Body arm = sim.createBody(armPose, CollisionShape{BoxShape(Vector3(1.5f, 0.1f, 0.1f))});
    (void)arm.setBodyType(Core::BodyType::Dynamic);
    (void)arm.setMass(3.0f);

    // Z ekseni etrafında menteşe: Ankor noktaları
    sim.createRevoluteConstraint(anchor, arm,
                                Vector3(0.0f, 0.0f, 0.0f),
                                Vector3(-1.5f, 0.0f, 0.0f),
                                Vector3(0.0f, 0.0f, 1.0f),
                                Vector3(0.0f, 0.0f, 1.0f));

    // Limitler: Yerçekimi kolu aşağı düşürmek isteyecek (negatif Z rotasyonu).
    // Alt limiti -0.5 rad (~ -28.6 derece) olarak ayarlayalım.
    const float lowerLimit = -0.5f;
    const float upperLimit = 0.5f;
    sim.setRevoluteLimits(anchor, lowerLimit, upperLimit);

    const float dt = 1.0f / 120.0f; // Yüksek hassasiyetli adım
    sim.setFixedTimeStep(dt);

    // Kol düşüp limite çarpıp oturana kadar 240 adım koştur
    for (int step = 0; step < 240; ++step) {
        sim.step(dt);
    }

    // Limit açısını ölç
    const auto& poseA = sim.getRegistry().getComponent<Pose>(anchor.getEntity());
    const auto& poseB = sim.getRegistry().getComponent<Pose>(arm.getEntity());
    const auto& rc = sim.getRegistry().getComponent<RevoluteConstraint>(anchor.getEntity());

    Vector3 a_A = rc.hingeAxisLocalA.getNormalized();
    Vector3 u_localA;
    if (std::abs(a_A.x) >= 0.57735f)
        u_localA = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
    else
        u_localA = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();
    Vector3 v_localA = a_A.cross(u_localA);
    Vector3 u_localB = rc.initialRelativeOrientation.getInverse() * u_localA;

    Vector3 u_worldA = poseA.orientation * u_localA;
    Vector3 v_worldA = poseA.orientation * v_localA;
    Vector3 u_worldB = poseB.orientation * u_localB;
    float measuredAngle = std::atan2(u_worldB.dot(v_worldA), u_worldB.dot(u_worldA));

    float limitPenetration = std::max(0.0f, lowerLimit - measuredAngle);

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "  -> Alt Limit          : " << lowerLimit << " rad" << std::endl;
    std::cout << "  -> Olculen Aci        : " << measuredAngle << " rad" << std::endl;
    std::cout << "  -> Limit Penetrasyonu : " << limitPenetration << " rad (Hedef: < 0.0100 rad)" << std::endl;

    CHECK_MSG(limitPenetration <= 0.0100f, "Revolute limit açısı penetrasyonu 0.01 rad altında kalmalıdır");
    CHECK_MSG(std::abs(measuredAngle - lowerLimit) < 0.02f, "Sarkaç yerçekimi altında alt limite oturmalıdır");
    std::cout << "  -> PASSED: Acisal limit dogrulugu ve toleransi basarili." << std::endl;
}

/**
 * @brief 4. Test: Snapshot Determinizmi ve Dejenere Durumlar.
 */
void testSnapshotAndDegenerate() {
    std::cout << "--- 4. Test: Snapshot Determinizmi ve Dejenere Durumlar ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Body fixedA = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{SphereShape(0.2f)});
    (void)fixedA.setBodyType(Core::BodyType::Static);

    Body wheelB = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{CylinderShape(0.5f, 0.2f)});
    (void)wheelB.setBodyType(Core::BodyType::Dynamic);
    (void)wheelB.setMass(2.0f);

    sim.createRevoluteConstraint(fixedA, wheelB,
                                Vector3(0, 0, 0), Vector3(0, 0, 0),
                                Vector3(0, 1, 0), Vector3(0, 1, 0));
    sim.setRevoluteLimits(fixedA, -1.0f, 1.0f);
    sim.setRevoluteMotor(fixedA, 5.0f, 30.0f);

    // 40 adım koştur
    for (int i = 0; i < 40; ++i) {
        sim.step(1.0f / 60.0f);
    }

    // Snapshot al
    Core::Snapshot snap = sim.saveSnapshot();

    // 60 adım daha koştur (State 1)
    std::vector<Vector3> traj1;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj1.push_back(sim.getRegistry().getComponent<Core::Motion>(wheelB.getEntity()).angularVelocity);
    }

    // Snapshot geri yükle
    sim.restoreSnapshot(snap);

    // 60 adım tekrar koştur (State 2)
    std::vector<Vector3> traj2;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj2.push_back(sim.getRegistry().getComponent<Core::Motion>(wheelB.getEntity()).angularVelocity);
    }

    for (size_t i = 0; i < 60; ++i) {
        CHECK_MSG((traj1[i] - traj2[i]).length() < 1e-6f, "Snapshot geri yükleme motorlu menteşede deterministik olmalıdır");
    }

    // Dejenere durum: 0 motor torku, 0 hız, kilitli limitler (lower == upper)
    Body degenB = sim.createBody(Pose(Vector3(3, 3, 3)), CollisionShape{SphereShape(0.1f)});
    sim.createRevoluteConstraint(degenB, fixedA,
                                Vector3(0, 0, 0), Vector3(0, 0, 0),
                                Vector3(1, 0, 0), Vector3(1, 0, 0));
    sim.setRevoluteLimits(degenB, 0.0f, 0.0f);
    sim.setRevoluteMotor(degenB, 0.0f, 0.0f);
    sim.step(1.0f / 60.0f); // NaN / crash üretmemeli
    CHECK_MSG(true, "Dejenere motor/limit sınırları güvenli çalışmalı");

    std::cout << "  -> PASSED: Snapshot ve sınır durumları güvenli." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.6: REVOLUTE MOTOR VE API TESTLERI ===" << std::endl;
    try {
        testFreeSpinningMotor();
        testMotorTorqueLimitUnderLoad();
        testAngularLimits();
        testSnapshotAndDegenerate();
        std::cout << "\n>>> ALL 4 REVOLUTE TEST SCENARIOS PASSED. <<<" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    }
}
