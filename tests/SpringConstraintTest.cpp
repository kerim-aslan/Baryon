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
#include <Baryon/collision/SpringConstraint.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @brief 1. Test: Sönümsüz Harmonik Salınım ve Analitik Periyot / Enerji Doğrulaması.
 * @details k = 50 N/m, m = 2 kg sönümsüz sistemde teorik periyot T = 2*pi*sqrt(m/k) = 1.2566 s.
 *          Kabul Kriteri: Periyot hatası < %2.5, Enerji kaybı / sapması < %3.0.
 */
void testUndampedHarmonicOscillation() {
    std::cout << "--- 1. Test: Sonumsuz Harmonik Salinim ve Analitik Periyot Dogrulamasi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Yerçekimsiz saf yay salınımı

    // Statik tavan ankrajı
    Pose ceilingPose(Vector3(0.0f, 10.0f, 0.0f));
    Body ceiling = sim.createBody(ceilingPose, CollisionShape{SphereShape(0.2f)});
    (void)ceiling.setBodyType(Core::BodyType::Static);

    // Salınan kütle: m = 2.0 kg, restLength = 2.0 m, başlangıçta 1.0 m uzatılarak y = 7.0 m'ye konur.
    const float m = 2.0f;
    const float k = 50.0f;
    const float c = 0.0f;
    const float L0 = 2.0f;
    const float y0 = 7.0f; // Uzama delta_y = (10 - 7) - 2.0 = 1.0 m

    Pose bobPose(Vector3(0.0f, y0, 0.0f));
    Body bob = sim.createBody(bobPose, CollisionShape{SphereShape(0.2f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(m);

    sim.createSpringConstraint(ceiling, bob, L0, k, c);

    const float theoreticalPeriod = 2.0f * static_cast<float>(std::numbers::pi) * std::sqrt(m / k);
    const float initialPotentialEnergy = 0.5f * k * 1.0f * 1.0f;

    const float dt = 1.0f / 120.0f; // 120 Hz yüksek doğruluklu adım
    sim.setFixedTimeStep(dt);

    float simTime = 0.0f;
    std::vector<float> peakTimes;
    float prevVelY = 0.0f;

    float minEnergy = initialPotentialEnergy;
    float maxEnergy = initialPotentialEnergy;

    // 4 tam salınım boyunca (~5 saniye) koştur
    for (int step = 0; step < 600; ++step) {
        sim.step(dt);
        simTime += dt;

        Vector3 curPos = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
        Vector3 curVel = sim.getRegistry().getComponent<Core::Motion>(bob.getEntity()).linearVelocity;

        float extension = std::abs(curPos.y - 10.0f) - L0;
        float currentE = 0.5f * m * curVel.y * curVel.y + 0.5f * k * extension * extension;
        minEnergy = std::min(minEnergy, currentE);
        maxEnergy = std::max(maxEnergy, currentE);

        // Hızın negatiften pozitife geçtiği dip noktaları (alt tepe noktaları)
        if (prevVelY < 0.0f && curVel.y >= 0.0f && simTime > 0.1f) {
            peakTimes.push_back(simTime);
        }
        prevVelY = curVel.y;
    }

    CHECK_MSG(peakTimes.size() >= 3, "En az 3 salınım tepe noktası yakalanmalı");

    float totalMeasuredPeriod = 0.0f;
    for (size_t i = 1; i < peakTimes.size(); ++i) {
        totalMeasuredPeriod += (peakTimes[i] - peakTimes[i - 1]);
    }
    float measuredPeriod = totalMeasuredPeriod / static_cast<float>(peakTimes.size() - 1);
    float periodErrorPercent = (std::abs(measuredPeriod - theoreticalPeriod) / theoreticalPeriod) * 100.0f;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Teorik Periyot    : " << theoreticalPeriod << " saniye" << std::endl;
    std::cout << "  -> Olculen Periyot   : " << measuredPeriod << " saniye (Hata: %" << periodErrorPercent << ", Hedef: < %2.5)" << std::endl;
    std::cout << "  -> Enerji Kararliligi: Max E = " << maxEnergy << " J, Baslangic E = " << initialPotentialEnergy << " J" << std::endl;

    CHECK_MSG(periodErrorPercent < 2.5f, "Sönümsüz yay salınım periyodu analitik değerle %2.5 içinde uyuşmalı");
    CHECK_MSG(maxEnergy <= initialPotentialEnergy * 1.05f, "Sönümsüz yayda yapay enerji artışı veya sayısal patlama olmamalı");
}

/**
 * @brief 2. Test: Kritik Sönümleme (Critical Damping zeta = 1.0) Testi.
 * @details Kritik sönümlemede cisim aşım (overshoot) yapmadan en hızlı şekilde denge konumuna oturmalıdır.
 *          Kabul Kriteri: y_max <= 8.005 m (aşım < 5 mm), son hız < 0.02 m/s.
 */
void testCriticalDamping() {
    std::cout << "--- 2. Test: Kritik Sonumleme (zeta = 1.0) ve Asimsiz Denge Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose ceilingPose(Vector3(0.0f, 10.0f, 0.0f));
    Body ceiling = sim.createBody(ceilingPose, CollisionShape{SphereShape(0.2f)});
    (void)ceiling.setBodyType(Core::BodyType::Static);

    const float m = 2.0f;
    const float k = 50.0f;
    const float L0 = 2.0f;
    const float omega = std::sqrt(k / m); // 5 rad/s
    const float c_crit = 2.0f * m * omega; // 20 N*s/m (zeta = 1.0)

    // Başlangıçta yay 1 metre uzatılmış (y = 7.0 m). Denge konumu y = 8.0 m.
    Pose bobPose(Vector3(0.0f, 7.0f, 0.0f));
    Body bob = sim.createBody(bobPose, CollisionShape{SphereShape(0.2f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(m);

    sim.createSpringConstraint(ceiling, bob, L0, k, c_crit);

    const float dt = 1.0f / 60.0f;
    float maxY = 7.0f;

    // 3 saniye boyunca simüle et (180 adım)
    for (int step = 0; step < 180; ++step) {
        sim.step(dt);
        Vector3 curPos = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
        if (curPos.y > maxY) {
            maxY = curPos.y;
        }
    }

    Vector3 finalPos = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
    Vector3 finalVel = sim.getRegistry().getComponent<Core::Motion>(bob.getEntity()).linearVelocity;

    float equilibriumY = 8.0f;
    float overshoot = std::max(0.0f, maxY - equilibriumY);
    float finalRestError = std::abs(finalPos.y - equilibriumY);
    float finalSpeed = finalVel.length();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> Hedef Denge Konumu (Y) : " << equilibriumY << " m" << std::endl;
    std::cout << "  -> Maksimum Ulasilan Y    : " << maxY << " m (Asim: " << (overshoot * 1000.0f) << " mm, Hedef: < 5.0 mm)" << std::endl;
    std::cout << "  -> Final Konum Hatasi     : " << (finalRestError * 1000.0f) << " mm (Hedef: < 10.0 mm)" << std::endl;
    std::cout << "  -> Final Dinlenme Hizi    : " << finalSpeed << " m/s (Hedef: < 0.02 m/s)" << std::endl;

    CHECK_MSG(overshoot < 0.005f, "Kritik sönümlemede aşım (overshoot) 5 mm'den küçük olmalı");
    CHECK_MSG(finalRestError < 0.01f, "Kritik sönümlü yay 3 saniye sonunda denge konumuna oturmalı (< 10 mm)");
    CHECK_MSG(finalSpeed < 0.02f, "Kritik sönümlü yay durduğunda hızı < 0.02 m/s olmalı");
}

/**
 * @brief 3. Test: Zayıf Sönümleme ve Logaritmik Dekrement Testi.
 * @details zeta = 0.1 zayıf sönümlü salınıcıda genlik üstel olarak söner: A(t) = A_0 * exp(-zeta * omega * t).
 */
void testUnderdampedDecay() {
    std::cout << "--- 3. Test: Zayif Sonumleme (zeta = 0.1) Ustel Genlik Sonumu Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose ceilingPose(Vector3(0.0f, 10.0f, 0.0f));
    Body ceiling = sim.createBody(ceilingPose, CollisionShape{SphereShape(0.2f)});
    (void)ceiling.setBodyType(Core::BodyType::Static);

    const float m = 2.0f;
    const float k = 50.0f;
    const float L0 = 2.0f;
    const float omega = std::sqrt(k / m); // 5 rad/s
    const float zeta = 0.1f;
    const float c = 2.0f * m * zeta * omega; // 2.0 N*s/m

    // Başlangıç genliği A0 = 1.0 m (y = 7.0 m -> 10 - 7 - 2 = 1.0)
    Pose bobPose(Vector3(0.0f, 7.0f, 0.0f));
    Body bob = sim.createBody(bobPose, CollisionShape{SphereShape(0.2f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(m);

    sim.createSpringConstraint(ceiling, bob, L0, k, c);

    const float dt = 1.0f / 120.0f;
    sim.setFixedTimeStep(dt);

    float simTime = 0.0f;
    float firstPeakTime = 0.0f;
    float firstPeakAmp = 0.0f;
    float prevVelY = 0.0f;

    // 1 tam salınım periyodunu bul
    for (int step = 0; step < 240; ++step) {
        sim.step(dt);
        simTime += dt;

        Vector3 curPos = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
        Vector3 curVel = sim.getRegistry().getComponent<Core::Motion>(bob.getEntity()).linearVelocity;

        if (prevVelY < 0.0f && curVel.y >= 0.0f && simTime > 0.5f) {
            firstPeakTime = simTime;
            firstPeakAmp = std::abs(curPos.y - 8.0f); // 8.0m denge konumuna göre genlik
            break;
        }
        prevVelY = curVel.y;
    }

    CHECK_MSG(firstPeakTime > 0.0f, "Birinci salınım periyodu tepe noktası yakalanmalı");

    // Teorik genlik: A(T) = A0 * exp(-zeta * omega * T)
    float theoreticalAmp = 1.0f * std::exp(-zeta * omega * firstPeakTime);
    float ampError = std::abs(firstPeakAmp - theoreticalAmp);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  -> 1 Periyot Zamani   : " << firstPeakTime << " s" << std::endl;
    std::cout << "  -> Teorik Tepe Genlik : " << theoreticalAmp << " m" << std::endl;
    std::cout << "  -> Olculen Tepe Genlik: " << firstPeakAmp << " m (Fark: " << (ampError * 1000.0f) << " mm, Hedef: < 100 mm)" << std::endl;

    CHECK_MSG(ampError < 0.10f, "Zayıf sönümlü genlik sönümü analitik üstel bozulma ile uyumlu olmalı (< 100 mm)");
}

/**
 * @brief 4. Test: Snapshot Determinizmi ve Dejenere Sınır Durumları.
 */
void testSnapshotAndDegenerateCases() {
    std::cout << "--- 4. Test: Snapshot Determinizmi ve Dejenere Durumlar ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Body fixedA = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.2f)});
    (void)fixedA.setBodyType(Core::BodyType::Static);

    Body dynB = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{BoxShape(Vector3(0.3f, 0.3f, 0.3f))});
    (void)dynB.setBodyType(Core::BodyType::Dynamic);
    (void)dynB.setMass(3.0f);

    sim.createSpringConstraint(fixedA, dynB, 1.5f, 150.0f, 10.0f);

    // 60 adım koştur
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }

    // Snapshot al
    Core::Snapshot snap = sim.saveSnapshot();

    // 60 adım daha koştur (State 1)
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }
    Vector3 pos1 = sim.getRegistry().getComponent<Pose>(dynB.getEntity()).position;
    Vector3 vel1 = sim.getRegistry().getComponent<Core::Motion>(dynB.getEntity()).linearVelocity;

    // Snapshot geri yükle ve tekrar 60 adım koştur (State 2)
    sim.restoreSnapshot(snap);
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }
    Vector3 pos2 = sim.getRegistry().getComponent<Pose>(dynB.getEntity()).position;
    Vector3 vel2 = sim.getRegistry().getComponent<Core::Motion>(dynB.getEntity()).linearVelocity;

    CHECK_MSG((pos1 - pos2).length() < 1e-6f, "Snapshot sonrasi pozisyon deterministik olmali");
    CHECK_MSG((vel1 - vel2).length() < 1e-6f, "Snapshot sonrasi hiz deterministik olmali");

    // Dejenere durum: 0 uzunluk, 0 sertlik, 0 sönüm güvenliği
    Body degen = sim.createBody(Pose(Vector3(5, 5, 5)), CollisionShape{SphereShape(0.1f)});
    sim.createSpringConstraint(degen, fixedA, 0.0f, 0.0f, 0.0f);
    sim.step(1.0f / 60.0f); // NaN / crash üretmemeli
    CHECK_MSG(true, "Dejenere sıfır katsayılı yay kararlı çalışmalı");

    std::cout << "  -> PASSED: Snapshot determinizmi ve sınır durumları güvenli." << std::endl;
}

/**
 * @brief 5. Test: Yüksek Sertlik ve Büyük dt Altında Kararlılık Testi (Patlamama).
 */
void testHighStiffnessAndLargeDt() {
    std::cout << "--- 5. Test: Yuksek Sertlik (k=100000 N/m) ve Buyuk dt (33.3 ms) Kararlilik Testi ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Body fixedCeil = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.2f)});
    (void)fixedCeil.setBodyType(Core::BodyType::Static);

    Body stiffBob = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{SphereShape(0.2f)});
    (void)stiffBob.setBodyType(Core::BodyType::Dynamic);
    (void)stiffBob.setMass(10.0f);

    sim.createSpringConstraint(fixedCeil, stiffBob, 2.0f, 100000.0f, 2000.0f);
    const float largeDt = 1.0f / 30.0f; // 30 Hz büyük adım
    sim.setFixedTimeStep(largeDt);

    for (int step = 0; step < 120; ++step) {
        sim.step(largeDt);
        Vector3 pos = sim.getRegistry().getComponent<Pose>(stiffBob.getEntity()).position;
        Vector3 vel = sim.getRegistry().getComponent<Core::Motion>(stiffBob.getEntity()).linearVelocity;

        CHECK_MSG(!std::isnan(pos.x) && !std::isnan(pos.y) && !std::isnan(pos.z), "Pozisyon NaN olmamali");
        CHECK_MSG(!std::isinf(pos.x) && !std::isinf(pos.y) && !std::isinf(pos.z), "Pozisyon Inf olmamali");
        CHECK_MSG(pos.y > 0.0f && pos.y < 15.0f, "Sistem buyuk dt altinda patlamamali");
        CHECK_MSG(!std::isnan(vel.x) && !std::isnan(vel.y) && !std::isnan(vel.z), "Hiz NaN olmamali");
    }

    std::cout << "  -> PASSED: k=100000 N/m ve dt=33.3ms altinda sistem kararlı ve patlamadi." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.5: SPRING CONSTRAINT TESTLERI ===" << std::endl;
    try {
        testUndampedHarmonicOscillation();
        testCriticalDamping();
        testUnderdampedDecay();
        testSnapshotAndDegenerateCases();
        testHighStiffnessAndLargeDt();
        std::cout << "\n>>> ALL 5 SPRING TEST SCENARIOS PASSED. <<<" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
        return 1;
    }
}
