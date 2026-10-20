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
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @file CylinderShapeTest.cpp
 * @brief Faz 2.1 Silindir Geometrisi (CylinderShape) Doğrulama Testi.
 * @details Silindirin atalet tensörü, dik durma kararlılığı, eğik düzlemde yuvarlanma ivmesi
 *          (a ≈ g sinθ / 1.5), çarpışma çiftleri ve ışın kesişim doğruluğunu sınar.
 */

// 1. Analitik Atalet Tensörü Testi
void testCylinderInertiaTensor() {
    std::cout << "\n--- 1. Cylinder Inertia Tensor Test ---" << std::endl;
    Simulator sim;

    float r = 0.5f;
    float halfH = 1.0f; // h = 2.0m
    float mass = 2.0f;
    float h = halfH * 2.0f;

    // Teorik:
    // Iyy = 0.5 * m * r^2 = 0.5 * 2 * 0.25 = 0.25
    // Ixx = Izz = (1/12) * m * (3*r^2 + h^2) = (2/12) * (3*0.25 + 4) = (1/6) * 4.75 ≈ 0.7916667
    float expectedIyy = 0.5f * mass * r * r;
    float expectedIxx = (1.0f / 12.0f) * mass * (3.0f * r * r + h * h);

    Body cyl = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{CylinderShape(r, halfH)});
    (void)cyl.setMass(mass);

    Matrix3x3 I = cyl.getLocalInertiaTensor().value();
    std::cout << "Cylinder Ixx: " << I[0].x << " (Expected: " << expectedIxx << ")" << std::endl;
    std::cout << "Cylinder Iyy: " << I[1].y << " (Expected: " << expectedIyy << ")" << std::endl;
    std::cout << "Cylinder Izz: " << I[2].z << " (Expected: " << expectedIxx << ")" << std::endl;

    CHECK_MSG(std::abs(I[0].x - expectedIxx) < 1e-4f, "Ixx analitik formulle eslesmelidir");
    CHECK_MSG(std::abs(I[1].y - expectedIyy) < 1e-4f, "Iyy analitik formulle eslesmelidir");
    CHECK_MSG(std::abs(I[2].z - expectedIxx) < 1e-4f, "Izz analitik formulle eslesmelidir");
}

// 2. Dik Durma ve Dinlenme Yüksekliği Testi
void testCylinderUprightResting() {
    std::cout << "\n--- 2. Cylinder Upright Resting & Sleep Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin: Üst yüzeyi Y = 0.0 (Merkez (0, -0.5, 0), halfExtents (20, 0.5, 20))
    Body ground = sim.createBody(Pose(Vector3(0.0f, -0.5f, 0.0f)), CollisionShape{BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // Silindir: r = 0.6, halfH = 0.8 -> Zemine oturduğunda merkez Y = 0.8 olmalıdır
    float halfH = 0.8f;
    Body cyl = sim.createBody(Pose(Vector3(0.0f, 2.0f, 0.0f)), CollisionShape{CylinderShape(0.6f, halfH)});
    (void)cyl.setMass(2.0f);
    (void)cyl.setMaterial(0.5f, 0.1f); // Düşük sekmeli

    // 240 adım (4 saniye) simüle et
    for (int i = 0; i < 240; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 pos = cyl.getPosition().value();
    std::cout << "Cylinder Final Y: " << pos.y << " m (Expected: " << halfH << " m)" << std::endl;
    CHECK_MSG(std::abs(pos.y - halfH) < 0.02f, "Silindir zeminde dik ve dogru yukseklikte dinlenmelidir");
    CHECK_MSG(cyl.isSleeping().value(), "Silindir hareketini tamamlayip uyku moduna gecmelidir");
}

// 3. Eğimli Düzlemde Yuvarlanma İvmesi Testi (a ≈ g sinθ / 1.5)
void testCylinderRollingOnIncline() {
    std::cout << "\n--- 3. Cylinder Rolling on Incline Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Eğim açısı: 30 derece (Z ekseni etrafında dönüş)
    float theta = 30.0f * (3.14159265f / 180.0f);
    Quaternion rampRot(Vector3(0.0f, 0.0f, 1.0f), -theta); // Sağa aşağı eğimli

    // Eğimli rampa: Kutu genişliği boyunca
    Body ramp = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f), rampRot), CollisionShape{BoxShape(Vector3(20.0f, 0.5f, 5.0f))});
    ramp.setBodyType(Core::BodyType::Static);
    (void)ramp.setMaterial(0.20f, 0.0f); // Kaymayı önlemek için yeterli sürtünme (mu > tan(theta)/3 = 0.192)

    // Silindir: Yan yatırılmış (Z ekseni boyunca, yani yerel Y ekseni dünya Z eksenine döndürülmüş)
    // Böylece silindirin dairesel kesiti XY düzlemindedir ve rampa boyunca Z etrafında yuvarlanır.
    float r = 0.5f;
    float halfH = 0.5f;
    Quaternion cylOrientation(Vector3(1.0f, 0.0f, 0.0f), 3.14159265f * 0.5f);

    // Rampanın üst yüzeyinde bir noktaya yerleştir
    // Rampa yüzeyi merkezden 0.5 yukarıda + silindir yarıçapı 0.5 = rampa normali boyunca 1.0 birim yukarı
    Vector3 rampNormal = rampRot * Vector3(0.0f, 1.0f, 0.0f);
    Vector3 startPos = Vector3(0.0f, 0.0f, 0.0f) + rampNormal * (0.5f + r);

    Body cyl = sim.createBody(Pose(startPos, cylOrientation), CollisionShape{CylinderShape(r, halfH)});
    (void)cyl.setMass(2.0f);
    (void)cyl.setMaterial(0.20f, 0.0f);

    // Teorik ivme: a = g * sin(theta) / 1.5
    float a_theory = (9.81f * std::sin(theta)) / 1.5f;

    // 60 adım (1.0 saniye) simüle et
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 linVel = cyl.getLinearVelocity().value();
    float speed = linVel.length();
    // v = a * t = a * 1.0
    std::cout << "Measured Speed after 1.0s: " << speed << " m/s (Theory: " << a_theory << " m/s)" << std::endl;

    // Şartnamedeki kural: ±%10 tolerans
    float diffPercent = std::abs(speed - a_theory) / a_theory * 100.0f;
    std::cout << "Rolling Acceleration Error: " << diffPercent << " %" << std::endl;
    CHECK_MSG(diffPercent < 10.0f, "Kati silindir yuvarlanma ivmesi teorik degere %10 icinde uymaktadir");
}

// 4. Silindir-Küre, Silindir-Kutu ve Silindir-Silindir Çarpışma Testi
void testCylinderCollisionPairs() {
    std::cout << "\n--- 4. Cylinder Collision Pairs Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Sıfır yerçekimi

    // 4a. Silindir - Küre karşılıklı çarpışma (esnek çarpışma ile geri sekme)
    Body cyl = sim.createBody(Pose(Vector3(-2.0f, 0.0f, 0.0f)), CollisionShape{CylinderShape(0.5f, 1.0f)});
    (void)cyl.setMass(1.0f);
    (void)cyl.setMaterial(0.0f, 1.0f);
    (void)cyl.setLinearVelocity(Vector3(5.0f, 0.0f, 0.0f));

    Body sph = sim.createBody(Pose(Vector3(2.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)sph.setMass(1.0f);
    (void)sph.setMaterial(0.0f, 1.0f);
    (void)sph.setLinearVelocity(Vector3(-5.0f, 0.0f, 0.0f));

    // 30 adım koştur
    for (int i = 0; i < 30; ++i) {
        sim.step(1.0f / 60.0f);
    }

    // Çarpışıp geri sekmeliler
    CHECK_MSG(cyl.getLinearVelocity().value().x < 0.0f, "Silindir carpmadan sonra geri sekmelidir");
    CHECK_MSG(sph.getLinearVelocity().value().x > 0.0f, "Kure carpmadan sonra geri sekmelidir");

    // 4b. Silindir - Silindir dik kesişim (Edge-Edge temas kararlılığı)
    Simulator sim2;
    sim2.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Body ground = sim2.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(10, 0.5f, 10))});
    ground.setBodyType(Core::BodyType::Static);

    // Alt silindir: X boyunca yatık
    Quaternion qX(Vector3(0, 0, 1), 3.14159265f * 0.5f);
    Body cylBottom = sim2.createBody(Pose(Vector3(0, 0.5f, 0), qX), CollisionShape{CylinderShape(0.5f, 2.0f)});
    (void)cylBottom.setMass(5.0f);

    // Üst silindir: Z boyunca yatık (alt silindire dik)
    Quaternion qZ(Vector3(1, 0, 0), 3.14159265f * 0.5f);
    Body cylTop = sim2.createBody(Pose(Vector3(0, 1.6f, 0), qZ), CollisionShape{CylinderShape(0.5f, 2.0f)});
    (void)cylTop.setMass(2.0f);

    // 120 adım koştur, patlama/uçma (NaN/Inf) olmamalı
    bool isStable = true;
    for (int i = 0; i < 120; ++i) {
        sim2.step(1.0f / 60.0f);
        Vector3 p = cylTop.getPosition().value();
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.y > 10.0f) {
            isStable = false;
            break;
        }
    }
    CHECK_MSG(isStable, "Dik eksenli silindirler edge-edge temasinda kararli kalmalidir");
}

// 5. Silindir Işın Kesişimi (Raycast) Testi
void testCylinderRaycast() {
    std::cout << "\n--- 5. Cylinder Raycast Test ---" << std::endl;
    Simulator sim;

    // Silindir: merkez (10, 0, 0), r = 1.0, halfH = 2.0 (Y aralığı [-2, +2])
    Body cyl = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{CylinderShape(1.0f, 2.0f)});
    cyl.setBodyType(Core::BodyType::Static);

    // Test 5a: Yan kavisli yüzeye atılan ışın (Orijinden +X yönüne)
    // Beklenen temas: X = 9.0 (10 - 1.0), mesafe = 9.0m, normal = (-1, 0, 0)
    RaycastHit hitSide = sim.raycast(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(hitSide.hasHit, "Kavisli yan yuzeye atilan isin carpmalidir");
    std::cout << "Side Raycast Hit Dist: " << hitSide.distance << " m (Expected: 9.0 m)" << std::endl;
    CHECK_MSG(std::abs(hitSide.distance - 9.0f) < 0.02f, "Yan yuzey temas mesafesi 9.0m olmalidir");
    CHECK_MSG(hitSide.normal.x < -0.9f, "Yan yuzey normali sola (-X) bakmalidir");

    // Test 5b: Üst kapağa dikey atılan ışın ((10, 10, 0) noktasından -Y yönüne)
    // Üst kapak Y = 2.0. Başlangıç Y = 10. Beklenen mesafe = 8.0m, normal = (0, 1, 0)
    RaycastHit hitTop = sim.raycast(Vector3(10.0f, 10.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f), 20.0f);
    CHECK_MSG(hitTop.hasHit, "Ust kapaga atilan isin carpmalidir");
    std::cout << "Top Cap Raycast Hit Dist: " << hitTop.distance << " m (Expected: 8.0 m)" << std::endl;
    CHECK_MSG(std::abs(hitTop.distance - 8.0f) < 0.02f, "Ust kapak temas mesafesi 8.0m olmalidir");
    CHECK_MSG(hitTop.normal.y > 0.9f, "Ust kapak normali yukari (+Y) bakmalidir");

    // Test 5c: Teğet geçen ışın (X=10, Y=0, Z=1.5 -> silindir yarıçapı 1.0 olduğundan ıskalar)
    RaycastHit hitMiss = sim.raycast(Vector3(0.0f, 0.0f, 1.5f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(!hitMiss.hasHit, "Silindirin yanindan teget gecen isin iskalayip carpmamalidir");
}

// 6. Dejenere Girdi Testi
void testCylinderDegenerate() {
    std::cout << "\n--- 6. Cylinder Degenerate Inputs Test ---" << std::endl;
    Simulator sim;

    // Sıfır yarıçap ve yükseklik
    Body b1 = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{CylinderShape(0.0f, 0.0f)});
    (void)b1.setMass(1.0f);

    // 10 adım koştur, NaN veya çökme olmamalı
    for (int i = 0; i < 10; ++i) {
        sim.step(1.0f / 60.0f);
    }
    Vector3 p = b1.getPosition().value();
    CHECK_MSG(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z), "Dejenere silindir sonlu sayi uretmelidir");
}

int main() {
    std::cout << "=== BARYON CYLINDER SHAPE TEST SUITE (PHASE 2.1) ===\n" << std::endl;

    testCylinderInertiaTensor();
    testCylinderUprightResting();
    testCylinderRollingOnIncline();
    testCylinderCollisionPairs();
    testCylinderRaycast();
    testCylinderDegenerate();

    return reportAndExit();
}
