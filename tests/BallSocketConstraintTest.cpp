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
#include <Baryon/collision/BallSocketConstraint.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @brief 1. Test: Sarkaç Sabit Noktası Drift, Enerji Korunumu ve Periyot Doğrulaması.
 * @details 10 saniye boyunca (600 adım):
 *          - Pivot noktası sapması < 1 mm
 *          - Sönümsüz modda mekanik enerji kaybı < %2
 *          - Küçük açılı salınım periyodu T = 2π√(L/g) ± %3
 */
void testPendulumHarmonicMotion() {
    std::cout << "--- 1. Test: Sarkac Drift, Enerji Korunumu ve Salinim Periyodu ---" << std::endl;

    Simulator sim;
    const float g = 9.81f;
    sim.setAccelerationField(Vector3(0, -g, 0));

    const float L = 2.0f; // Sarkaç ip/çubuk boyu
    const Vector3 pivotPos(0.0f, 10.0f, 0.0f);

    // Sabit mafsal yuvası
    Body anchor = sim.createBody(Pose(pivotPos), CollisionShape{SphereShape(0.1f)});
    (void)anchor.setBodyType(Core::BodyType::Static);

    // Küçük açı: 10 derece (~0.1745 rad)
    const float theta0 = 10.0f * (static_cast<float>(std::numbers::pi) / 180.0f);
    const float x0 = L * std::sin(theta0);
    const float y0 = pivotPos.y - L * std::cos(theta0);
    const Vector3 bobPos(x0, y0, 0.0f);

    // Dynamic sarkaç ucu (bob)
    const float massBob = 1.0f;
    const float radiusBob = 0.05f;
    Body bob = sim.createBody(Pose(bobPos), CollisionShape{SphereShape(radiusBob)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(massBob);

    // Sönümsüz ortam ayarı
    auto& motionBob = sim.getRegistry().getComponent<Core::Motion>(bob.getEntity());
    motionBob.linearDamping = 0.0f;
    motionBob.angularDamping = 0.0f;

    // Ball-Socket mafsalı bağla
    sim.createBallSocketConstraint(anchor, bob, pivotPos);

    const float dt = 1.0f / 60.0f;
    const int totalSteps = 600; // 10 saniye

    // Başlangıç mekanik enerjisi: E = Ek + Ep (Ep = m * g * (y - y_bottom))
    const float yBottom = pivotPos.y - L;
    auto computeEnergy = [&](const Vector3& pos, const Vector3& linVel, const Vector3& angVel, const Matrix3x3& invInertia) {
        float h = pos.y - yBottom;
        float ep = massBob * g * h;
        float ekLin = 0.5f * massBob * linVel.dot(linVel);
        // Küre için I = 2/5 * m * r^2
        float iScalar = 0.4f * massBob * radiusBob * radiusBob;
        float ekAng = 0.5f * iScalar * angVel.dot(angVel);
        (void)invInertia;
        return ep + ekLin + ekAng;
    };

    const float initialEnergy = computeEnergy(bobPos, Vector3(0, 0, 0), Vector3(0, 0, 0), Matrix3x3::identity());

    float maxPivotDrift = 0.0f;
    float finalEnergy = initialEnergy;

    // Periyot tespiti için x pozisyonunun sıfırdan geçiş zamanları (negatiften pozitife)
    std::vector<float> zeroCrossings;
    float prevX = bobPos.x;

    for (int step = 1; step <= totalSteps; ++step) {
        sim.step(dt);

        const auto& poseB = sim.getRegistry().getComponent<Pose>(bob.getEntity());
        const auto& motionB = sim.getRegistry().getComponent<Core::Motion>(bob.getEntity());
        const auto& massB = sim.getRegistry().getComponent<Core::MassProps>(bob.getEntity());
        const auto& bs = sim.getRegistry().getComponent<BallSocketConstraint>(anchor.getEntity());

        // Pivot drift hesabı
        Vector3 worldAnchorA = pivotPos;
        Vector3 worldAnchorB = poseB.position + poseB.orientation * bs.localAnchorB;
        float drift = (worldAnchorA - worldAnchorB).length();
        if (drift > maxPivotDrift) {
            maxPivotDrift = drift;
        }

        // Enerji hesabı
        finalEnergy = computeEnergy(poseB.position, motionB.linearVelocity, motionB.angularVelocity, massB.inverseInertiaTensor);

        // Sıfır geçişi (negatiften pozitife yükselen kenar)
        float currX = poseB.position.x;
        if (prevX < 0.0f && currX >= 0.0f) {
            // Lineer interpolasyon ile hassas geçiş anı
            float fraction = -prevX / (currX - prevX);
            float crossingTime = (static_cast<float>(step - 1) + fraction) * dt;
            zeroCrossings.push_back(crossingTime);
        }
        prevX = currX;
    }

    // Teorik periyot: T = 2 * pi * sqrt(L / g)
    const float theoreticalPeriod = 2.0f * static_cast<float>(std::numbers::pi) * std::sqrt(L / g);

    float measuredPeriod = 0.0f;
    if (zeroCrossings.size() >= 2) {
        float totalPeriodTime = zeroCrossings.back() - zeroCrossings.front();
        measuredPeriod = totalPeriodTime / static_cast<float>(zeroCrossings.size() - 1);
    }

    float periodErrorPercent = std::abs(measuredPeriod - theoreticalPeriod) / theoreticalPeriod * 100.0f;
    float energyLossPercent = std::abs(finalEnergy - initialEnergy) / initialEnergy * 100.0f;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> 10 sn Sonuclari:" << std::endl;
    std::cout << "     Maksimum Pivot Drift  : " << (maxPivotDrift * 1000.0f) << " mm (Hedef: < 1.0 mm)" << std::endl;
    std::cout << "     Mekanik Enerji Kaybi  : %" << energyLossPercent << " (Hedef: < %2.0)" << std::endl;
    std::cout << "     Teorik Periyot        : " << theoreticalPeriod << " s" << std::endl;
    std::cout << "     Olcülen Periyot       : " << measuredPeriod << " s (Fark: %" << periodErrorPercent << ", Hedef: < %3.0)" << std::endl;

    CHECK_MSG(maxPivotDrift < 0.001f, "10 saniye sonunda sarkaç sabit noktası 1 mm'den az sapmalı");
    CHECK_MSG(energyLossPercent < 2.0f, "Sönümsüz modda 10 saniyelik mekanik enerji kaybı %2'den az olmalı");
    CHECK_MSG(periodErrorPercent < 3.0f, "Küçük açı salınım periyodu teorik 2pi*sqrt(L/g) değerine %3 toleransla uymalı");
}

/**
 * @brief 2. Test: Serbest Açısal Dönme Doğrulaması (3 Açısal DOF Serbest).
 * @details Cisme 3 eksende açısal darbe uygulanır. Mafsal kilitlenmeden dönerken
 *          ankor noktası sabit kalmalıdır.
 */
void testFreeAngularRotation() {
    std::cout << "--- 2. Test: Serbest Acisal Donme (3-DOF Serbest) ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0)); // Ağırlıksız ortam

    const Vector3 pivot(0, 5, 0);
    Body anchor = sim.createBody(Pose(pivot), CollisionShape{SphereShape(0.1f)});
    (void)anchor.setBodyType(Core::BodyType::Static);
    (void)anchor.setTrigger(true);

    Body spinner = sim.createBody(Pose(pivot), CollisionShape{SphereShape(0.2f)});
    (void)spinner.setBodyType(Core::BodyType::Dynamic);
    (void)spinner.setMass(2.0f);

    auto& motion = sim.getRegistry().getComponent<Core::Motion>(spinner.getEntity());
    motion.linearDamping = 0.0f;
    motion.angularDamping = 0.0f;

    sim.createBallSocketConstraint(anchor, spinner, pivot);

    // 3 eksende açısal hız ver
    (void)spinner.setAngularVelocity(Vector3(3.0f, 4.0f, 5.0f));

    float maxPivotDrift = 0.0f;
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);

        const auto& poseS = sim.getRegistry().getComponent<Pose>(spinner.getEntity());
        const auto& bs = sim.getRegistry().getComponent<BallSocketConstraint>(anchor.getEntity());
        Vector3 worldAnchorB = poseS.position + poseS.orientation * bs.localAnchorB;
        float drift = (pivot - worldAnchorB).length();
        if (drift > maxPivotDrift) {
            maxPivotDrift = drift;
        }
    }

    const auto& finalMotion = sim.getRegistry().getComponent<Core::Motion>(spinner.getEntity());
    float angSpeed = finalMotion.angularVelocity.length();

    std::cout << "  -> Donme sonrasi acisal hiz buyuklugu: " << angSpeed << " rad/s (Beklenen: ~7.07)" << std::endl;
    std::cout << "  -> Maksimum mafsal kaymasi            : " << (maxPivotDrift * 1000.0f) << " mm" << std::endl;

    CHECK_MSG(maxPivotDrift < 0.001f, "Hızlı açısal dönüşte mafsal noktası 1 mm'den az sapmalı");
    CHECK_MSG(angSpeed > 5.0f, "Açısal hız mafsal tarafından kilitlenmemeli (serbest dönmeli)");
}

/**
 * @brief 3. Test: Snapshot ve Geri Yükleme Determinizmi.
 */
void testSnapshotDeterminism() {
    std::cout << "--- 3. Test: BallSocket Snapshot Determinizmi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body anchor = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.2f)});
    (void)anchor.setBodyType(Core::BodyType::Static);

    Body bob = sim.createBody(Pose(Vector3(1.0f, 9.0f, 0.5f)), CollisionShape{SphereShape(0.2f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(1.5f);

    sim.createBallSocketConstraint(anchor, bob, Vector3(0, 10, 0));

    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Core::Snapshot snap = sim.saveSnapshot();

    std::vector<Vector3> trajectory1;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        trajectory1.push_back(sim.getRegistry().getComponent<Pose>(bob.getEntity()).position);
    }

    sim.restoreSnapshot(snap);

    std::vector<Vector3> trajectory2;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        trajectory2.push_back(sim.getRegistry().getComponent<Pose>(bob.getEntity()).position);
    }

    for (size_t i = 0; i < 60; ++i) {
        CHECK_MSG(trajectory1[i].x == trajectory2[i].x &&
                  trajectory1[i].y == trajectory2[i].y &&
                  trajectory1[i].z == trajectory2[i].z,
                  "Snapshot sonrasi yorunge bit-for-bit eslesmeli");
    }

    std::cout << "  -> PASSED: Snapshot geri yukleme deterministik." << std::endl;
}

/**
 * @brief 4. Test: Dejenere ve Sınır Durumlar.
 */
void testDegenerateCases() {
    std::cout << "--- 4. Test: Dejenere ve Sinir Durumlar ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Çakışık konumlu iki cisim
    Body a = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{SphereShape(0.2f)});
    Body b = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{SphereShape(0.2f)});
    sim.createBallSocketConstraint(a, b, Vector3(0, 5, 0));

    for (int i = 0; i < 30; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 posA = sim.getRegistry().getComponent<Pose>(a.getEntity()).position;
    CHECK_MSG(!std::isnan(posA.x) && !std::isnan(posA.y) && !std::isnan(posA.z),
              "Cakisik konumlarda NaN uretilmemeli");

    std::cout << "  -> PASSED: Dejenere durumlar guvenli." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.3: BALL-SOCKET CONSTRAINT TESTLERI ===\n" << std::endl;

    testPendulumHarmonicMotion();
    testFreeAngularRotation();
    testSnapshotDeterminism();
    testDegenerateCases();

    return reportAndExit();
}
