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
 * @file GravityScaleTest.cpp
 * @brief Gövde bazlı yerçekimi ölçeklendirme (gravityScale) ve enerji korunumu testleri (Faz 4.5).
 * @details Cisimlerin bireysel gravityScale çarpanları, yerçekimsiz süzülme (gravityScale=0),
 *          ters yerçekimi ve sıfır sönümlemede (damping=0) enerji korunumunu doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @brief 1. Test: gravityScale ve Sönümleme API Getter / Setter Doğrulaması.
 */
void testGravityScaleAPI() {
    std::cout << "[GravityScaleTest] 1. API ve Varsayilan Deger Testi...\n";
    Simulator sim;

    Body b = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{SphereShape(0.5f)});
    (void)b.setBodyType(Core::BodyType::Dynamic);

    // Varsayılan değerler
    CHECK(b.getGravityScale().has_value());
    CHECK(std::abs(b.getGravityScale().value() - 1.0f) < 1e-5f);
    CHECK(b.getLinearDamping().has_value());
    CHECK(std::abs(b.getLinearDamping().value() - 0.05f) < 1e-5f);
    CHECK(b.getAngularDamping().has_value());
    CHECK(std::abs(b.getAngularDamping().value() - 0.05f) < 1e-5f);

    // Değiştirme
    (void)b.setGravityScale(2.5f);
    CHECK(std::abs(b.getGravityScale().value() - 2.5f) < 1e-5f);

    (void)b.setLinearDamping(0.2f);
    CHECK(std::abs(b.getLinearDamping().value() - 0.2f) < 1e-5f);

    (void)b.setAngularDamping(0.3f);
    CHECK(std::abs(b.getAngularDamping().value() - 0.3f) < 1e-5f);

    (void)b.setDamping(0.1f, 0.4f);
    CHECK(std::abs(b.getLinearDamping().value() - 0.1f) < 1e-5f);
    CHECK(std::abs(b.getAngularDamping().value() - 0.4f) < 1e-5f);

    std::cout << "  -> API getter/setter testi basarili.\n";
}

/**
 * @brief 2. Test: gravityScale = 0 Serbest Süzülme (Free Float) Testi.
 * @details Global yerçekimi altında gravityScale = 0 olan bir cisim dikey yönde kesinlikle düşmemelidir.
 */
void testZeroGravityScaleFreeFloat() {
    std::cout << "[GravityScaleTest] 2. gravityScale=0 Serbest Suzulme (Free Float) Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // b1: Normal yerçekimi (gravityScale = 1.0)
    Body b1 = sim.createBody(Pose(Vector3(-2.0f, 10.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)b1.setBodyType(Core::BodyType::Dynamic);
    (void)b1.setMass(1.0f);
    (void)b1.setGravityScale(1.0f);

    // b2: Ağırlıksız süzülen cisim (gravityScale = 0.0)
    Body b2 = sim.createBody(Pose(Vector3(0.0f, 10.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)b2.setBodyType(Core::BodyType::Dynamic);
    (void)b2.setMass(1.0f);
    (void)b2.setGravityScale(0.0f);
    (void)b2.setDamping(0.0f, 0.0f);
    (void)b2.setLinearVelocity(Vector3(2.0f, 0.0f, 0.0f)); // Yatay hız

    // 120 adım (2 saniye) simüle et
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; ++i) {
        sim.step(dt);

        Vector3 pos2 = b2.getPosition().value();
        Vector3 vel2 = b2.getLinearVelocity().value();

        // b2 dikey yönde zerre kadar kıpırdamamalıdır
        CHECK(std::abs(pos2.y - 10.0f) < 1e-5f);
        CHECK(std::abs(vel2.y) < 1e-5f);
        // Yatay hızını sürtünmesiz korumalıdır
        CHECK(std::abs(vel2.x - 2.0f) < 1e-5f);
    }

    Vector3 pos1 = b1.getPosition().value();
    // b1 serbest düşüş yapıp aşağı inmiş olmalıdır
    CHECK(pos1.y < 0.0f);

    std::cout << "  -> gravityScale=0 serbest suzulme basariyla dogrulandi.\n";
}

/**
 * @brief 3. Test: Ters ve Çift Yerçekimi (gravityScale = -1.0 ve 2.0) Testi.
 */
void testInvertedAndScaledGravity() {
    std::cout << "[GravityScaleTest] 3. Olcekli ve Ters Yercekimi Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -10.0f, 0));

    // b_inv: Ters yerçekimi (-1.0) -> yukarı ivmelenmeli
    Body b_inv = sim.createBody(Pose(Vector3(-5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)b_inv.setBodyType(Core::BodyType::Dynamic);
    (void)b_inv.setMass(1.0f);
    (void)b_inv.setGravityScale(-1.0f);
    (void)b_inv.setDamping(0.0f, 0.0f);

    // b_double: 2x yerçekimi (2.0) -> iki kat hızlı düşmeli
    Body b_double = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)b_double.setBodyType(Core::BodyType::Dynamic);
    (void)b_double.setMass(1.0f);
    (void)b_double.setGravityScale(2.0f);
    (void)b_double.setDamping(0.0f, 0.0f);

    // 1 saniye (60 adım) simüle et
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {
        sim.step(dt);
    }

    // t = 1.0s:
    // b_inv: v_y = +10 m/s
    // b_double: v_y = -20 m/s
    Vector3 v_inv = b_inv.getLinearVelocity().value();
    Vector3 v_double = b_double.getLinearVelocity().value();

    CHECK(std::abs(v_inv.y - 10.0f) < 0.1f);
    CHECK(std::abs(v_double.y - (-20.0f)) < 0.1f);

    std::cout << "  -> Ters ve olcekli yercekimi basarili.\n";
}

/**
 * @brief 4. Test: damping = 0 Enerji Korunumu (Energy Conservation) Testi.
 * @details Sürtünmesiz ortamda (damping = 0, yerçekimi yok) serbest hareket eden cismin
 *          kinetik enerjisi E = 0.5 * m * v^2 yüzlerce adım boyunca tam korunmalıdır.
 */
void testZeroDampingEnergyConservation() {
    std::cout << "[GravityScaleTest] 4. damping=0 Enerji Korunumu Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0)); // Yerçekimsiz uzay

    Body ball = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{SphereShape(0.5f)});
    (void)ball.setBodyType(Core::BodyType::Dynamic);
    (void)ball.setMass(2.0f);
    (void)ball.setDamping(0.0f, 0.0f); // Sıfır sönümleme
    (void)ball.setLinearVelocity(Vector3(3.0f, 4.0f, 0.0f)); // v = 5 m/s

    float initialSpeedSq = ball.getLinearVelocity().value().lengthSquare();
    float initialEnergy = 0.5f * 2.0f * initialSpeedSq; // E = 25.0 J

    const float dt = 1.0f / 60.0f;
    for (int step = 0; step < 300; ++step) {
        sim.step(dt);
    }

    float finalSpeedSq = ball.getLinearVelocity().value().lengthSquare();
    float finalEnergy = 0.5f * 2.0f * finalSpeedSq;

    float energyDrift = std::abs(finalEnergy - initialEnergy);
    std::cout << "  Baslangic Enerjisi: " << initialEnergy << " J | Son Enerji: " << finalEnergy
              << " J | Fark: " << energyDrift << " J\n";

    CHECK_MSG(energyDrift < 1e-4f, "damping=0 durumunda kinetik enerji tam korunmalidir (fark < 1e-4 J)");
    std::cout << "  -> Enerji korunumu basariyla dogrulandi.\n";
}

int main() {
    std::cout << "=== BARYON FAZ 4.5 GRAVITYSCALE VE ENERJI KORUNUMU TESTLERI ===\n";
    testGravityScaleAPI();
    testZeroGravityScaleFreeFloat();
    testInvertedAndScaledGravity();
    testZeroDampingEnergyConservation();
    return reportAndExit();
}
