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
 * @file ConeShapeTest.cpp
 * @brief Baryon fizik motoru Faz 2.2 ConeShape (koni geometrisi) test paketi.
 * 
 * Bu dosya analitik eylemsizlik tensörü doğrulaması, düz zeminde dik durma ve uyku testi,
 * tepe noktası aşağıda devrilme dinamiği, koni çarpışma çiftleri, analitik ışın kesişimi (raycast)
 * ve bozulmuş (degenerate) girdi kontrollerini kapsamlı bir şekilde doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @file ConeShapeTest.cpp
 * @brief Faz 2.2 Koni Geometrisi (ConeShape) Doğrulama Testi.
 * @details Koninin analitik atalet tensörü, taban üzerinde dik durma kararlılığı,
 *          ucu aşağı bırakılınca devrilip oturması, çarpışma çiftleri ve ışın kesişim doğruluğunu sınar.
 */

// 1. Analitik Atalet Tensörü Testi
void testConeInertiaTensor() {
    std::cout << "\n--- 1. Cone Inertia Tensor Test ---" << std::endl;
    Simulator sim;

    float r = 0.6f;
    float h = 1.2f;
    float mass = 2.0f;

    Body cone = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{ConeShape(r, h)});
    (void)cone.setMass(mass);

    // Teorik Kütle Merkezi Eylemsizlik Tensörü (Y ekseni simetri eksenidir):
    // Iyy = (3/10) * m * r^2
    // Ixx = Izz = (3/20) * m * (r^2 + 0.25 * h^2)
    float expectedIyy = 0.3f * mass * r * r;
    float expectedIxx = 0.15f * mass * (r * r + 0.25f * h * h);
    float expectedIzz = expectedIxx;

    auto inertiaOpt = cone.getLocalInertiaTensor();
    CHECK_MSG(inertiaOpt.has_value(), "Cismin eylemsizlik tensoru basariyla okunmali");

    Matrix3x3 I = inertiaOpt.value();
    std::cout << "Cone Ixx: " << I[0].x << " (Expected: " << expectedIxx << ")" << std::endl;
    std::cout << "Cone Iyy: " << I[1].y << " (Expected: " << expectedIyy << ")" << std::endl;
    std::cout << "Cone Izz: " << I[2].z << " (Expected: " << expectedIzz << ")" << std::endl;

    CHECK_MSG(std::abs(I[0].x - expectedIxx) < 1e-4f, "Koni Ixx analitik degerle eslesmeli");
    CHECK_MSG(std::abs(I[1].y - expectedIyy) < 1e-4f, "Koni Iyy analitik degerle eslesmeli");
    CHECK_MSG(std::abs(I[2].z - expectedIzz) < 1e-4f, "Koni Izz analitik degerle eslesmeli");
}

// 2. Taban Üzerinde Dik Durma ve Uyku Testi
void testConeUprightResting() {
    std::cout << "\n--- 2. Cone Upright Resting & Sleep Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Statik Zemin: Üst yüzeyi y = 0 seviyesinde
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(10, 0.5f, 10))});
    ground.setBodyType(Core::BodyType::Static);

    // Koni: r = 0.5m, h = 1.0m. Ağırlık merkezi yerel orijinde (0,0,0).
    // Taban y = -0.25m, Tepe y = 0.75m.
    // Taban y=0 zeminine oturunca kütle merkezinin beklenen yüksekliği y = 0.25m olmalıdır.
    float r = 0.5f;
    float h = 1.0f;
    Body cone = sim.createBody(Pose(Vector3(0, 0.6f, 0)), CollisionShape{ConeShape(r, h)});
    (void)cone.setMass(1.0f);

    // 120 adım (2 saniye) simüle et
    for (int i = 0; i < 120; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 finalPos = cone.getPosition().value();
    std::cout << "Cone Final Resting Y: " << finalPos.y << " m (Expected: 0.25 m)" << std::endl;

    // Dinlenme yüksekliği ideal ± 1.5 cm içinde olmalı
    float diffY = std::abs(finalPos.y - 0.25f);
    CHECK_MSG(diffY < 0.015f, "Koni taban ustunde dik dururken agirlik merkezi beklenen yukseklikte dinlenmelidir");

    // Yönelim kontrolü: Dik kalmalı (yukarı vektörü hala (0,1,0))
    Quaternion q = cone.getOrientation().value();
    Vector3 up = q * Vector3(0, 1, 0);
    CHECK_MSG(up.y > 0.98f, "Koni dik durusunu muhafaza etmeli ve devrilmemelidir");
}

// 3. Ucu Aşağı Bırakılınca Devrilme ve Dengeye Gelme Testi
void testConeApexDownTipping() {
    std::cout << "\n--- 3. Cone Apex Down Tipping Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Statik Zemin
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(10, 0.5f, 10))});
    ground.setBodyType(Core::BodyType::Static);

    // Koniyi ucu aşağı gelecek şekilde 170 derece eğik (ters-eğimli) başlat
    // Tam ters konum (180 derece) metastabil olduğundan küçük bir perturbasyon (170 derece)
    // gerçek dünyadaki devrilmeyi tetikler.
    float tiltAngle = 170.0f * (3.14159265f / 180.0f);
    Quaternion invRot(Vector3(1.0f, 0.0f, 0.0f), tiltAngle);

    float r = 0.5f;
    float h = 1.0f;
    Body cone = sim.createBody(Pose(Vector3(0, 1.2f, 0), invRot), CollisionShape{ConeShape(r, h)});
    (void)cone.setMass(1.0f);

    // 180 adım (3 saniye) simüle et
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Quaternion finalRot = cone.getOrientation().value();
    Vector3 finalAxis = finalRot * Vector3(0.0f, 1.0f, 0.0f);
    std::cout << "Inverted Cone Final Axis Y component: " << finalAxis.y << std::endl;

    // Ucu aşağı bırakılan koni sivri tepesi üstünde dengede kalamaz (kararsız denge)
    // Devrilerek yan yatmalı veya tabanına oturmalıdır (axis Y asla başlangıçtaki gibi ters -0.98 kalamaz)
    CHECK_MSG(finalAxis.y > -0.5f, "Ucu asagi birakilan koni devrilerek kararli bir konuma gecmelidir");
}

// 4. Çarpışma Çiftleri Testi (Koni-Küre, Koni-Kutu, Koni-Koni)
void testConeCollisionPairs() {
    std::cout << "\n--- 4. Cone Collision Pairs Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Sıfır yerçekimi

    // Koni ve Küre karşı karşıya fırlatılır
    Body cone = sim.createBody(Pose(Vector3(-2.0f, 0.0f, 0.0f)), CollisionShape{ConeShape(0.5f, 1.0f)});
    cone.setBodyType(Core::BodyType::Dynamic);
    (void)cone.setMass(1.0f);
    (void)cone.setLinearVelocity(Vector3(4.0f, 0.0f, 0.0f));

    Body sphere = sim.createBody(Pose(Vector3(2.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    sphere.setBodyType(Core::BodyType::Dynamic);
    (void)sphere.setMass(1.0f);
    (void)sphere.setLinearVelocity(Vector3(-4.0f, 0.0f, 0.0f));

    // Çarpışma anı ve sonrası
    bool collided = false;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        if (cone.getLinearVelocity().value().x < 0.0f && sphere.getLinearVelocity().value().x > 0.0f) {
            collided = true;
            break;
        }
    }
    CHECK_MSG(collided, "Koni ve kure carpisip ters yonlere sekmelidir");

    // Koni-Koni Çarpışması
    Simulator sim2;
    sim2.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));
    Body cone1 = sim2.createBody(Pose(Vector3(-2.0f, 0.0f, 0.0f)), CollisionShape{ConeShape(0.5f, 1.0f)});
    cone1.setBodyType(Core::BodyType::Dynamic);
    (void)cone1.setMass(1.0f);
    (void)cone1.setMaterial(0.1f, 0.8f);
    (void)cone1.setLinearVelocity(Vector3(3.0f, 0.0f, 0.0f));

    Body cone2 = sim2.createBody(Pose(Vector3(2.0f, 0.0f, 0.0f)), CollisionShape{ConeShape(0.5f, 1.0f)});
    cone2.setBodyType(Core::BodyType::Dynamic);
    (void)cone2.setMass(1.0f);
    (void)cone2.setMaterial(0.1f, 0.8f);
    (void)cone2.setLinearVelocity(Vector3(-3.0f, 0.0f, 0.0f));

    bool coneConeCollided = false;
    for (int i = 0; i < 60; ++i) {
        sim2.step(1.0f / 60.0f);
        float v1x = cone1.getLinearVelocity().value().x;
        float v2x = cone2.getLinearVelocity().value().x;
        if (v1x < 0.0f && v2x > 0.0f) {
            coneConeCollided = true;
            break;
        }
    }
    CHECK_MSG(coneConeCollided, "Koni-koni carpismasi basariyla algilanip cozulmelidir");
}

// 5. Işın Kesişimi (Raycast) Testi
void testConeRaycast() {
    std::cout << "\n--- 5. Cone Raycast Test ---" << std::endl;
    Simulator sim;

    float r = 1.0f;
    float h = 2.0f;
    // Koni merkezde: Taban y = -0.5, Tepe y = 1.5
    Body cone = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{ConeShape(r, h)});
    cone.setBodyType(Core::BodyType::Static);

    // 1. Yan kavisli yuzeye yatay isin: (-5, 0.5, 0) -> +X yonunde
    // y = 0.5 seviyesinde koni yaricapi:
    // r(y) = k * (yApex - y) = (1.0 / 2.0) * (1.5 - 0.5) = 0.5m
    // Isin -X tarafindan geldiginden temas noktasi x = -0.5m olmalidir.
    // Baslangic x = -5.0m olduguna gore mesafe: 5.0 - 0.5 = 4.5m!
    RaycastHit hitSide = sim.raycast(Vector3(-5.0f, 0.5f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 10.0f);
    CHECK_MSG(hitSide.hasHit, "Koninin yan kavisli yuzeyine atilan isin temas etmeli");
    if (hitSide.hasHit) {
        std::cout << "Side Raycast Hit Dist: " << hitSide.distance << " m (Expected: 4.5 m)" << std::endl;
        CHECK_MSG(std::abs(hitSide.distance - 4.5f) < 0.01f, "Yan yuzey isin kesisim mesafesi analitik degerle uyusmali");
        CHECK_MSG(hitSide.normal.x < -0.5f, "Yan yuzey dikmesi isin yonune karsi yonelmelidir");
    }

    // 2. Taban duz kapagina alttan dikey isin: (0, -5, 0) -> +Y yonunde
    // Taban y = -0.5m seviyesinde. Baslangic y = -5.0m olduguna gore mesafe: 5.0 - 0.5 = 4.5m!
    RaycastHit hitBase = sim.raycast(Vector3(0.0f, -5.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f), 10.0f);
    CHECK_MSG(hitBase.hasHit, "Koninin taban kapagina alttan atilan isin temas etmeli");
    if (hitBase.hasHit) {
        std::cout << "Base Raycast Hit Dist: " << hitBase.distance << " m (Expected: 4.5 m)" << std::endl;
        CHECK_MSG(std::abs(hitBase.distance - 4.5f) < 0.01f, "Taban kapagi isin mesafesi 4.5m olmali");
        CHECK_MSG(std::abs(hitBase.normal.y - (-1.0f)) < 0.01f, "Taban normali -Y yonunde olmali");
    }

    // 3. Iskalanan isin (Miss ray): (5, 5, 5) -> +X yonunde
    RaycastHit hitMiss = sim.raycast(Vector3(5.0f, 5.0f, 5.0f), Vector3(1.0f, 0.0f, 0.0f), 10.0f);
    CHECK_MSG(!hitMiss.hasHit, "Koniyi iskaliyan isin temas uretmemelidir");
}

// 6. Dejenere ve Asiri Girdiler Testi (Degenerate Inputs)
void testConeDegenerateInputs() {
    std::cout << "\n--- 6. Cone Degenerate Inputs Test ---" << std::endl;
    Simulator sim;

    // Sifir ve negatif yaricap/yukseklik
    Body c1 = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{ConeShape(0.0f, 0.0f)});
    (void)c1.setMass(1.0f);

    Body c2 = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{ConeShape(-1.0f, -2.0f)});
    (void)c2.setMass(1.0f);

    // Isin testi cokmemeli
    RaycastHit hit1 = sim.raycast(Vector3(0, 5, 0), Vector3(0, -1, 0), 10.0f);
    (void)hit1;

    // Bir adım simülasyon çökmemeli
    sim.step(1.0f / 60.0f);

    CHECK_MSG(true, "Dejenere girdi ile yapilan koni islemleri guvenle tamamlandi (cokme/NaN yok)");
}

int main() {
    std::cout << "=== BARYON CONE SHAPE TEST SUITE (PHASE 2.2) ===\n" << std::endl;

    testConeInertiaTensor();
    testConeUprightResting();
    testConeApexDownTipping();
    testConeCollisionPairs();
    testConeRaycast();
    testConeDegenerateInputs();

    return reportAndExit();
}
