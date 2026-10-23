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
 * @file DOFLockTest.cpp
 * @brief Baryon fizik motoru eksen kilitleme (DOF Lock) testleri.
 * @details Cisimlerin serbestlik derecelerinin (DoF) doğrusal ve açısal bazda
 *          kilitlenmesi, 2B fizik modu, devrilmeyen karakterler, 1-DoF ray/asansör
 *          mekanizmaları ve snapshot determinizmi test edilir.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief 2B Fizik Modu Testi (LockAxis::Lock2D).
 * @details XY düzleminde hareket eden cisimlerin Z ekseni ötelemesi ile X ve Y
 *          rotasyonlarının tam olarak kilitlendiğini doğrular.
 */
void test2DPhysicsMode() {
    std::cout << "[DOFLockTest] 2B Fizik Modu (Lock2D) Testi...\n";
    Simulator sim;
    const float dt = 1.0f / 60.0f;

    // Statik zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, CollisionShape{BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 2B kilitli kutu 1
    Pose p1Init(Vector3(-1.0f, 5.0f, 0.0f));
    Body b1 = sim.createBody(p1Init, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b1.setMass(2.0f);
    b1.setLockedAxes(Core::LockAxis::Lock2D);

    // 2B kilitli kutu 2 (açılı bir çarpışma oluşturmak üzere yerleştirilmiş)
    Pose p2Init(Vector3(0.0f, 2.0f, 0.0f));
    Body b2 = sim.createBody(p2Init, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b2.setMass(2.0f);
    b2.setLockedAxes(Core::LockAxis::Lock2D);

    // Başlangıçta Z bileşenine de kasti bir yanal itme verelim
    (void)b1.applyLinearImpulse(Vector3(1.0f, 0.0f, 5.0f));

    // 180 adım koştur
    for (int step = 0; step < 180; ++step) {
        sim.step(dt);

        Vector3 p1 = b1.getPosition().value();
        Vector3 p2 = b2.getPosition().value();
        Quaternion q1 = b1.getOrientation().value();
        Quaternion q2 = b2.getOrientation().value();
        Vector3 v1 = b1.getLinearVelocity().value();
        Vector3 v2 = b2.getLinearVelocity().value();

        // Z ötelemesi kesinlikle 0 olmalı
        CHECK(std::abs(p1.z) < 1e-5f);
        CHECK(std::abs(p2.z) < 1e-5f);
        CHECK(std::abs(v1.z) < 1e-5f);
        CHECK(std::abs(v2.z) < 1e-5f);

        // Pitch (X) ve Roll (Y) rotasyonları tam sıfır kalmalı (kuaterniyon x ve y sıfır)
        CHECK(std::abs(q1.x) < 1e-5f);
        CHECK(std::abs(q1.y) < 1e-5f);
        CHECK(std::abs(q2.x) < 1e-5f);
        CHECK(std::abs(q2.y) < 1e-5f);
    }

    std::cout << "  -> 2B Fizik Modu testi basarili.\n";
}

/**
 * @brief Dik Duran Karakter / Devrilmeyen Kutu Testi (AngularX | AngularZ).
 * @details Cisim yalnızca dikey (yaw, Y ekseni etrafında) dönebilir; pitch ve roll kilitlidir.
 *          Köşeden darbe aldığında devrilmez.
 */
void testUprightCharacter() {
    std::cout << "[DOFLockTest] Dik Duran Karakter (AngularX | AngularZ) Testi...\n";
    Simulator sim;
    const float dt = 1.0f / 60.0f;

    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, CollisionShape{BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // İnce uzun dik kutu (karakter gövdesi gibi)
    Pose charPose(Vector3(0.0f, 2.0f, 0.0f));
    Body character = sim.createBody(charPose, CollisionShape{BoxShape(Vector3(0.4f, 1.0f, 0.4f))});
    (void)character.setMass(5.0f);
    // X ve Z açısal dönüşleri kilitli (sadece Y etrafında dönüş - yaw serbest)
    character.setLockedAxes(Core::LockAxis::AngularX | Core::LockAxis::AngularZ);

    // Güçlü devirici açısal itme ve torklar uygulayalım
    (void)character.applyAngularImpulse(Vector3(50.0f, 10.0f, 30.0f));
    (void)character.applyTorque(Vector3(100.0f, 20.0f, 100.0f));

    for (int step = 0; step < 120; ++step) {
        sim.step(dt);
        Quaternion q = character.getOrientation().value();

        // Pitch ve Roll kilitli olmalı: kuaterniyonda x ve z bileşenleri 0 kalmalı
        CHECK(std::abs(q.x) < 1e-5f);
        CHECK(std::abs(q.z) < 1e-5f);
    }

    std::cout << "  -> Dik Karakter testi basarili.\n";
}

/**
 * @brief 1-DoF Dikey Asansör Testi (LinearX | LinearZ | AngularAll).
 * @details Yalnızca Y ekseninde serbestçe kayabilen asansör kabini.
 *          Yanal çarpışmalar ve rüzgar kuvvetleri uygulandığında dahi X ve Z koordinatları değişmez.
 */
void testVerticalSlider() {
    std::cout << "[DOFLockTest] 1-DoF Dikey Asansor Testi...\n";
    Simulator sim;
    const float dt = 1.0f / 60.0f;

    Pose elevPose(Vector3(5.0f, 10.0f, -3.0f));
    Body elevator = sim.createBody(elevPose, CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    (void)elevator.setMass(10.0f);
    // Yalnızca LinearY serbest: LinearX, LinearZ ve tüm açısal eksenler kilitli
    elevator.setLockedAxes(Core::LockAxis::LinearX | Core::LockAxis::LinearZ | Core::LockAxis::AngularAll);

    // Şiddetli yanal itme ve açısal torklar verelim
    (void)elevator.applyLinearImpulse(Vector3(200.0f, 0.0f, -150.0f));
    (void)elevator.applyTorque(Vector3(50.0f, 50.0f, 50.0f));

    for (int step = 0; step < 60; ++step) {
        sim.step(dt);
        Vector3 p = elevator.getPosition().value();
        Vector3 v = elevator.getLinearVelocity().value();
        Vector3 w = elevator.getAngularVelocity().value();

        // X ve Z kesinlikle başlangıç konumunda kalmalı
        CHECK(std::abs(p.x - 5.0f) < 1e-5f);
        CHECK(std::abs(p.z - (-3.0f)) < 1e-5f);
        CHECK(std::abs(v.x) < 1e-5f);
        CHECK(std::abs(v.z) < 1e-5f);

        // Tüm açısal hızlar sıfır olmalı
        CHECK(w.length() < 1e-5f);

        // Y ekseninde ise yerçekimiyle serbestçe düşüyor olmalı
        CHECK(p.y < 10.0f);
    }

    std::cout << "  -> 1-DoF Dikey Asansor testi basarili.\n";
}

/**
 * @brief Dinamik Kilit Açma ve Kapama (Runtime Toggle) Testi.
 * @details Simülasyon devam ederken eksen kilitlerinin dinamik olarak değiştirilmesi.
 */
void testDynamicLockToggle() {
    std::cout << "[DOFLockTest] Dinamik Kilit Degistirme Testi...\n";
    Simulator sim;
    const float dt = 1.0f / 60.0f;

    Pose bodyPose(Vector3(0.0f, 10.0f, 0.0f));
    Body body = sim.createBody(bodyPose, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)body.setMass(2.0f);

    // 1. Aşama: Tamamen kilitli (All)
    body.setLockedAxes(Core::LockAxis::All);
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    CHECK(std::abs(body.getPosition().value().y - 10.0f) < 1e-5f);

    // 2. Aşama: Y ekseni kilidini aç
    body.setLockedAxes(Core::LockAxis::LinearX | Core::LockAxis::LinearZ | Core::LockAxis::AngularAll);
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    float droppedY = body.getPosition().value().y;
    CHECK(droppedY < 9.5f);

    // 3. Aşama: Tekrar Y eksenini kilitle (havada asılı kalmalı)
    body.setLockedAxes(Core::LockAxis::All);
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    CHECK(std::abs(body.getPosition().value().y - droppedY) < 1e-4f);

    std::cout << "  -> Dinamik Kilit Degistirme testi basarili.\n";
}

/**
 * @brief Snapshot ve Determinizm Testi.
 * @details Eksen kilidi durumunun ve kilitli hareket simülasyonunun snapshot kaydı
 *          ve geri yüklemesiyle deterministik olarak korunduğunu doğrular.
 */
void testSnapshotDeterminism() {
    std::cout << "[DOFLockTest] Snapshot ve Determinizm Testi...\n";
    Simulator sim;
    const float dt = 1.0f / 60.0f;

    Pose bodyPose(Vector3(1.0f, 5.0f, 2.0f));
    Body b = sim.createBody(bodyPose, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b.setMass(3.0f);
    b.setLockedAxes(Core::LockAxis::LinearZ | Core::LockAxis::AngularX | Core::LockAxis::AngularY);

    for (int i = 0; i < 20; ++i) {
        sim.step(dt);
    }

    // Durumu kaydet
    auto snapshot = sim.saveSnapshot();

    // 30 adım daha ilerlet
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    Vector3 pA = b.getPosition().value();

    // Geri yükle ve tekrar 30 adım ilerlet
    sim.restoreSnapshot(snapshot);
    CHECK(b.getLockedAxes() == (Core::LockAxis::LinearZ | Core::LockAxis::AngularX | Core::LockAxis::AngularY));

    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    Vector3 pB = b.getPosition().value();

    CHECK(std::abs(pA.x - pB.x) < 1e-5f);
    CHECK(std::abs(pA.y - pB.y) < 1e-5f);
    CHECK(std::abs(pA.z - pB.z) < 1e-5f);

    std::cout << "  -> Snapshot ve Determinizm testi basarili.\n";
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "Baryon Fizik Motoru - Eksen Kilitleme Testleri (Faz 3.9)\n";
    std::cout << "========================================\n";

    test2DPhysicsMode();
    testUprightCharacter();
    testVerticalSlider();
    testDynamicLockToggle();
    testSnapshotDeterminism();

    std::cout << "Tum Eksen Kilitleme (DOF Lock) testleri basariyla gecti!\n";
    return 0;
}
