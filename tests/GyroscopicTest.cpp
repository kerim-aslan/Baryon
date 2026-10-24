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
 * @file GyroscopicTest.cpp
 * @brief Jiroskopik Terim (Gyroscopic Precession / Dzhanibekov Etkisi) testleri (Faz 4.3).
 * @details Serbest uzayda asimetrik dönen katı cisimler için Euler hareket denklemlerinin
 *          (w x (I*w)) açısal momentum korunumu ve enerji artışı olmaksızın doğrulanması.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief Jiroskopik tork yapılandırma ve gövde API'lerinin doğrulanması.
 */
void testGyroscopicSettingsAPI() {
    std::cout << "[GyroscopicTest] Ayar API Testi...\n";
    Simulator sim;

    CHECK(!sim.isGyroscopicTorqueEnabled());
    sim.setGyroscopicTorque(true);
    CHECK(sim.isGyroscopicTorqueEnabled());
    CHECK(sim.getSettings().enableGyroscopicTorque);

    sim.setGyroscopicTorque(false);
    CHECK(!sim.isGyroscopicTorqueEnabled());

    Pose p0(Vector3(0.0f, 0.0f, 0.0f));
    Body body = sim.createBody(p0, CollisionShape{BoxShape(Vector3(0.2f, 1.0f, 0.5f))});
    (void)body.setMass(2.0f);

    auto hasGyro = body.hasGyroscopicTorque();
    CHECK(hasGyro.has_value() && !hasGyro.value());

    auto res = body.setGyroscopicTorque(true);
    CHECK(res.has_value());

    hasGyro = body.hasGyroscopicTorque();
    CHECK(hasGyro.has_value() && hasGyro.value());

    std::cout << "  -> Ayar API testi basarili.\n";
}

/**
 * @brief Sönümsüz Serbest Dönen Çubukta Açısal Momentum ve Enerji Korunumu Testi (Faz 4.3).
 * @details Sönümsüz ve yerçekimsiz ortamda serbest dönen asimetrik bir çubukta:
 *          1. Açısal momentum vektörünün büyüklüğü sapması < %1 olmalıdır.
 *          2. Kinetik enerji kesinlikle artmamalıdır (E_t <= E_0).
 */
void testFreeSpinningAsymmetricRod() {
    std::cout << "[GyroscopicTest] Serbest Donen Asimetrik Cubuk (Momentum ve Enerji Korunumu)...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Yerçekimsiz uzay

    // Asimetrik kutu/çubuk: Ix != Iy != Iz
    // halfExtents = (0.1, 1.0, 0.4) m
    Pose rodPose(Vector3(0.0f, 0.0f, 0.0f));
    Body rod = sim.createBody(rodPose, CollisionShape{BoxShape(Vector3(0.1f, 1.0f, 0.4f))});
    (void)rod.setMass(2.0f);
    (void)rod.setDamping(0.0f, 0.0f); // Sıfır sönümleme (enerji korunumu için)
    (void)rod.setGyroscopicTorque(true);

    Matrix3x3 I_local = rod.getLocalInertiaTensor().value();

    // Başlangıç açısal hızı (üç eksende de bileşeni olan hızlı 3D dönüş)
    Vector3 w0(6.0f, 15.0f, 4.0f);
    (void)rod.setAngularVelocity(w0);

    // Başlangıç açısal momentumu ve dönme kinetik enerjisi
    Vector3 L0 = I_local * w0;
    float L_mag0 = L0.length();
    float E_initial = 0.5f * w0.dot(L0);

    std::cout << "  -> Baslangic Açısal Momentum Buyuklugu: " << L_mag0 << " kg*m^2/s\n";
    std::cout << "  -> Baslangic Donme Kinetik Enerjisi: " << E_initial << " J\n";

    const float dt = 1.0f / 60.0f;
    float maxMomentumDevPct = 0.0f;
    float maxEnergyIncrease = 0.0f;

    // 600 adım (10 saniye boyunca serbest jiroskopik dönüş)
    for (int step = 0; step < 600; ++step) {
        sim.step(dt);

        Vector3 w_t = rod.getAngularVelocity().value();
        Pose pose_t = rod.getPose().value();
        Matrix3x3 R_t = pose_t.getOrientationMatrix();

        // Dünya uzayındaki açısal momentum: L_world = R * (I_local * (R^T * w))
        Vector3 w_body = R_t.getTranspose() * w_t;
        Vector3 L_body = I_local * w_body;
        Vector3 L_world = R_t * L_body;
        float L_mag_t = L_world.length();

        float momentumDevPct = std::abs(L_mag_t - L_mag0) / L_mag0 * 100.0f;
        maxMomentumDevPct = std::max(maxMomentumDevPct, momentumDevPct);

        float E_t = 0.5f * w_body.dot(L_body);
        if (E_t > E_initial) {
            maxEnergyIncrease = std::max(maxEnergyIncrease, E_t - E_initial);
        }
    }

    std::cout << "  -> 600 adimda Maksimum Momentum Sapmasi: %" << maxMomentumDevPct << "\n";
    std::cout << "  -> 600 adimda Maksimum Enerji Artisi: " << maxEnergyIncrease << " J\n";

    // 1. Kriter: Açısal momentum sapması < %1 (hedef tam sağlanmalı)
    CHECK(maxMomentumDevPct < 1.0f);

    // 2. Kriter: Enerji artışı olmamalıdır (sayısal tolerans 1e-4 J)
    CHECK(maxEnergyIncrease < 1e-4f);

    std::cout << "  -> Serbest donen cubuk momentum ve enerji testi basarili.\n";
}

/**
 * @brief Dzhanibekov Etkisi (Ara Eksen Taklası / Tennis Racket Instability) Testi.
 * @details Asimetrik bir cisim (I1 < I2 < I3) ara eylemsizlik ekseni etrafında
 *          döndürüldüğünde jiroskopik tork etkisiyle periyodik takla atar (açısal hızın işareti döner).
 */
void testDzhanibekovEffect() {
    std::cout << "[GyroscopicTest] Dzhanibekov Etkisi (Ara Eksen Donus Kararsizligi)...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    // Kutu boyutları: halfExtents = (0.1, 0.8, 0.3)
    // Iy < Iz < Ix -> Ara eksen Z eksenidir!
    Pose pose(Vector3(0.0f, 0.0f, 0.0f));
    Body body = sim.createBody(pose, CollisionShape{BoxShape(Vector3(0.1f, 0.8f, 0.3f))});
    (void)body.setMass(2.0f);
    (void)body.setDamping(0.0f, 0.0f);
    (void)body.setGyroscopicTorque(true);

    Matrix3x3 I = body.getLocalInertiaTensor().value();
    float Ixx = I.rows[0].x;
    float Iyy = I.rows[1].y;
    float Izz = I.rows[2].z;
    std::cout << "  -> Eylemsizlik Momentleri: Ixx=" << Ixx << ", Iyy=" << Iyy << ", Izz=" << Izz << "\n";
    CHECK(Iyy < Izz && Izz < Ixx); // Z ekseninin ara eksen olduğunu doğrula

    // Ara eksen (Z) etrafında baskın dönüş, X ekseninde küçük pertürbasyon
    Vector3 initialW(0.05f, 0.0f, 18.0f);
    (void)body.setAngularVelocity(initialW);

    const float dt = 1.0f / 60.0f;
    bool reachedNegative = false;
    bool flippedBack = false;

    // 600 adım boyunca ara eksen dönüşünü takip et
    for (int step = 0; step < 600; ++step) {
        sim.step(dt);
        Pose curPose = body.getPose().value();
        Vector3 curW = body.getAngularVelocity().value();
        Vector3 wBody = curPose.getOrientationMatrix().getTranspose() * curW;

        if (wBody.z < -10.0f) {
            reachedNegative = true;
        }
        if (reachedNegative && wBody.z > 10.0f) {
            flippedBack = true;
            break;
        }
    }

    std::cout << "  -> Ara eksen isaret donusu (Dzhanibekov taklasi): " 
              << (reachedNegative ? "Gerceklesti (w_z < -10)" : "Basarisiz")
              << ", Geri donus: " << (flippedBack ? "Tamamlandi (w_z > +10)" : "Beklemede") << "\n";
    CHECK(reachedNegative);
    CHECK(flippedBack);
    std::cout << "  -> Dzhanibekov etkisi testi basarili.\n";
}

/**
 * @brief Snapshot ve Geri Yükleme Determinizm Testi.
 */
void testSnapshotDeterminism() {
    std::cout << "[GyroscopicTest] Snapshot Determinizm Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose pose(Vector3(0.0f, 0.0f, 0.0f));
    Body body = sim.createBody(pose, CollisionShape{BoxShape(Vector3(0.2f, 1.2f, 0.5f))});
    (void)body.setMass(2.0f);
    (void)body.setDamping(0.0f, 0.0f);
    (void)body.setGyroscopicTorque(true);
    (void)body.setAngularVelocity(Vector3(8.0f, 12.0f, 5.0f));

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 20; ++i) sim.step(dt);

    auto snapshot = sim.saveSnapshot();

    // 30 adım ilerlet
    for (int i = 0; i < 30; ++i) sim.step(dt);
    Vector3 w1 = body.getAngularVelocity().value();
    Quaternion q1 = body.getOrientation().value();

    // Snapshot geri yükle ve tekrar 30 adım ilerlet
    sim.restoreSnapshot(snapshot);
    for (int i = 0; i < 30; ++i) sim.step(dt);
    Vector3 w2 = body.getAngularVelocity().value();
    Quaternion q2 = body.getOrientation().value();

    CHECK(std::abs(w1.x - w2.x) < 1e-5f);
    CHECK(std::abs(w1.y - w2.y) < 1e-5f);
    CHECK(std::abs(w1.z - w2.z) < 1e-5f);
    CHECK(std::abs(q1.x - q2.x) < 1e-5f);
    CHECK(std::abs(q1.y - q2.y) < 1e-5f);
    CHECK(std::abs(q1.z - q2.z) < 1e-5f);
    CHECK(std::abs(q1.w - q2.w) < 1e-5f);

    std::cout << "  -> Snapshot determinizm testi basarili.\n";
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "   Jiroskopik Terim Testleri (Faz 4.3)  \n";
    std::cout << "========================================\n";

    testGyroscopicSettingsAPI();
    testFreeSpinningAsymmetricRod();
    testDzhanibekovEffect();
    testSnapshotDeterminism();

    std::cout << "\n[TUM JIROSKOPIK TERIM TESTLERI BASARIYLA TAMAMLANDI!]\n";
    return 0;
}
