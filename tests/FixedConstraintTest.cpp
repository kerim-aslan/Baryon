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
#include <Baryon/collision/FixedConstraint.hpp>
#include <iostream>
#include <cmath>
#include <numbers>

using namespace Baryon;
using namespace Baryon::collision;

// Test 1: 10 saniye boyunca yük altında (Yerçekimi ve Cantilever Torku) drift doğrulaması
void testCantileverLoadDrift() {
    std::cout << "--- 1. Test: 10 Saniye Yuk Altinda Sabit Eklem Drift Dogrulamasi ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // A cismi: Statik ankraj mesnedi
    Pose poseA(Vector3(0.0f, 10.0f, 0.0f), Quaternion::identity());
    Body anchor = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    // B cismi: Dinamik konsol kiriş (cantilever beam) - 1.5m yana uzanır, 2 kg kütle
    Pose poseB(Vector3(1.5f, 10.0f, 0.0f), Quaternion::identity());
    Body beam = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.5f, 0.1f, 0.1f))});
    (void)beam.setBodyType(Core::BodyType::Dynamic);
    (void)beam.setMass(2.0f);

    // İki cismi aralarındaki mesafeyi koruyarak sabit eklemle kaynakla
    sim.createFixedConstraint(anchor, beam);

    Vector3 initialRelPos = poseB.position - poseA.position;
    Quaternion initialRelRot = poseA.orientation.getInverse() * poseB.orientation;

    float maxLinearDrift = 0.0f;
    float maxAngularDrift = 0.0f;

    // 10 saniye boyunca simülasyon (60 Hz -> 600 adım)
    const float dt = 1.0f / 60.0f;
    const int totalSteps = 600;

    for (int step = 0; step < totalSteps; ++step) {
        sim.step(dt);

        Vector3 curPosA = sim.getRegistry().getComponent<Pose>(anchor.getEntity()).position;
        Vector3 curPosB = sim.getRegistry().getComponent<Pose>(beam.getEntity()).position;
        Quaternion curRotA = sim.getRegistry().getComponent<Pose>(anchor.getEntity()).orientation;
        Quaternion curRotB = sim.getRegistry().getComponent<Pose>(beam.getEntity()).orientation;

        // Çizgisel sapma (bağıl konum hatası)
        Vector3 curRelPos = curPosB - curPosA;
        float linearDrift = (curRelPos - initialRelPos).length();
        if (linearDrift > maxLinearDrift) {
            maxLinearDrift = linearDrift;
        }

        // Açısal sapma (bağıl yönelim hatası)
        Quaternion expectedRotB = curRotA * initialRelRot;
        Quaternion rotDiff = curRotB * expectedRotB.getInverse();
        if (rotDiff.w < 0.0f) {
            rotDiff.w = -rotDiff.w;
            rotDiff.x = -rotDiff.x;
            rotDiff.y = -rotDiff.y;
            rotDiff.z = -rotDiff.z;
        }
        float clampedW = std::clamp(rotDiff.w, -1.0f, 1.0f);
        float angularDriftRad = 2.0f * std::acos(clampedW);
        if (angularDriftRad > maxAngularDrift) {
            maxAngularDrift = angularDriftRad;
        }
    }

    float maxAngularDriftDeg = maxAngularDrift * (180.0f / static_cast<float>(std::numbers::pi));

    std::cout << "  -> 10 sn (600 adim) sonuclari:" << std::endl;
    std::cout << "     Maksimum Cizgisel Drift : " << (maxLinearDrift * 1000.0f) << " mm (Hedef: < 1.0 mm)" << std::endl;
    std::cout << "     Maksimum Acisal Drift   : " << maxAngularDriftDeg << " derece (Hedef: < 0.5 derece)" << std::endl;

    CHECK_MSG(maxLinearDrift < 0.001f, "10 sn boyunca cizgisel drift 1 mm'nin altinda kalmalidir");
    CHECK_MSG(maxAngularDriftDeg < 0.5f, "10 sn boyunca acisal drift 0.5 derecenin altinda kalmalidir");
}

// Test 2: İki dinamik cismin birlikte serbest takla atması (Dynamic - Dynamic)
void testTwoDynamicBodiesTumbling() {
    std::cout << "\n--- 2. Test: Iki Dinamik Cismin Birlikte Takla Atmasi ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Pose poseA(Vector3(0.0f, 15.0f, 0.0f), Quaternion::identity());
    Body b1 = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.3f, 0.3f, 0.3f))});
    (void)b1.setBodyType(Core::BodyType::Dynamic);
    (void)b1.setMass(1.0f);

    Pose poseB(Vector3(0.6f, 15.0f, 0.0f), Quaternion::identity());
    Body b2 = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.3f, 0.3f, 0.3f))});
    (void)b2.setBodyType(Core::BodyType::Dynamic);
    (void)b2.setMass(1.5f);

    sim.createFixedConstraint(b1, b2);

    // Cisme dönme ve yatay hız ver
    (void)b1.setLinearVelocity(Vector3(3.0f, 2.0f, 0.0f));
    (void)b1.setAngularVelocity(Vector3(0.0f, 0.0f, 5.0f));

    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 pA = sim.getRegistry().getComponent<Pose>(b1.getEntity()).position;
    Vector3 pB = sim.getRegistry().getComponent<Pose>(b2.getEntity()).position;
    float dist = (pB - pA).length();

    float diffFromExpected = std::abs(dist - 0.6f);
    std::cout << "  -> Dinamik takla sonrasi mesafe: " << dist << " m (Beklenen: 0.6 m, Sapma: " << (diffFromExpected * 1000.0f) << " mm)" << std::endl;

    CHECK_MSG(diffFromExpected < 0.001f, "Dinamik takla sirasinda sabit eklem mesafesi 1 mm'den az sapmalidir");
}

// Test 3: Snapshot ve Determinizm Doğrulaması
void testSnapshotDeterminism() {
    std::cout << "\n--- 3. Test: FixedConstraint Snapshot Determinizmi ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Body anchor = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    Body beam = sim.createBody(Pose(Vector3(1, 10, 0)), CollisionShape{BoxShape(Vector3(0.3f, 0.1f, 0.1f))});
    (void)beam.setBodyType(Core::BodyType::Dynamic);
    (void)beam.setMass(3.0f);

    sim.createFixedConstraint(anchor, beam);

    for (int i = 0; i < 30; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Core::Snapshot snap = sim.saveSnapshot();

    std::vector<Vector3> traj1;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj1.push_back(sim.getRegistry().getComponent<Pose>(beam.getEntity()).position);
    }

    sim.restoreSnapshot(snap);

    std::vector<Vector3> traj2;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        traj2.push_back(sim.getRegistry().getComponent<Pose>(beam.getEntity()).position);
    }

    for (size_t i = 0; i < 60; ++i) {
        CHECK_MSG(traj1[i].x == traj2[i].x && traj1[i].y == traj2[i].y && traj1[i].z == traj2[i].z,
                  "FixedConstraint snapshot sonrasi trajektori birebir ayni olmalidir");
    }

    std::cout << "  -> PASSED: Snapshot geri yukleme deterministik." << std::endl;
}

// Test 4: Dejenere ve Uç Durumlar
void testDegenerateFixedJoint() {
    std::cout << "\n--- 4. Test: Dejenere ve Sinir Durumlar ---" << std::endl;
    Simulator sim;

    // Aynı konumdaki iki cisim (coincident)
    Body b1 = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{SphereShape(0.2f)});
    Body b2 = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{SphereShape(0.2f)});
    sim.createFixedConstraint(b1, b2);

    for (int i = 0; i < 10; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 p1 = sim.getRegistry().getComponent<Pose>(b1.getEntity()).position;
    Vector3 p2 = sim.getRegistry().getComponent<Pose>(b2.getEntity()).position;
    CHECK_MSG(!std::isnan(p1.x) && !std::isnan(p2.x), "Ayni konumdaki cisimlerde NaN olmamalidir");

    std::cout << "  -> PASSED: Dejenere durumlar guvenli." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.2: FIXED CONSTRAINT (WELD JOINT) TESTLERI ===\n" << std::endl;

    testCantileverLoadDrift();
    testTwoDynamicBodiesTumbling();
    testSnapshotDeterminism();
    testDegenerateFixedJoint();

    return reportAndExit();
}
