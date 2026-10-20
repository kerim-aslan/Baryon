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
#include <Baryon/collision/Sweep.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @file SweepQueryTest.cpp
 * @brief Faz 1.3 Süpürme / Şekil Fırlatma (Sweep Queries / Shape Cast) Doğrulama Testi.
 * @details sweepSphere, sweepBox, sweepCapsule, sweepShape fonksiyonlarının
 *          analitik mesafe, temas noktası, yüzey normali ve katman filtreleme doğruluğunu sınar.
 */

// 1. Küre Süpürme Analitik Mesafe ve Normal Testi
void testSphereSweepDistanceAndNormal() {
    std::cout << "\n--- 1. Sphere Sweep Distance and Normal Test ---" << std::endl;
    Simulator sim;

    // Hedef küre: merkez (10, 0, 0), yarıçap 1.0
    Body target = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    target.setBodyType(Core::BodyType::Static);

    // Süpürülen küre: başlangıç (0, 0, 0), yarıçap 0.5, yön (1, 0, 0), maks mesafe 20
    // Analitik temas mesafesi: 10 - (0.5 + 1.0) = 8.5 m
    // Analitik temas noktası: Hedef yüzey üzerinde (9.0, 0, 0)
    // Analitik normal: Süpürülen cisme karşı dışa bakan normal (-1, 0, 0)
    SweepHit hit = sim.sweepSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f, Vector3(1.0f, 0.0f, 0.0f), 20.0f);

    CHECK_MSG(hit.hasHit, "Kure supurmesi hedefi vurmalidir");
    CHECK_MSG(hit.entity.id == target.getEntity().id, "Vurulan varlik hedef kure olmalidir");
    std::cout << "Sphere Sweep Hit Dist: " << hit.distance << " m (Expected: 8.5 m)" << std::endl;
    CHECK_MSG(std::abs(hit.distance - 8.5f) < 0.05f, "Supurme mesafesi 8.5m olmalidir");

    std::cout << "Sphere Sweep Hit Point: (" << hit.point.x << ", " << hit.point.y << ", " << hit.point.z << ")" << std::endl;
    CHECK_MSG(std::abs(hit.point.x - 9.0f) < 0.05f, "Temas noktasi X=9.0 olmalidir");
    CHECK_MSG(std::abs(hit.normal.x - (-1.0f)) < 0.05f, "Normal X=-1.0 olmalidir");
    CHECK_MSG(std::abs(hit.fraction - (8.5f / 20.0f)) < 0.01f, "Fraction orani dogru olmalidir");
}

// 2. Kutu Süpürme (sweepBox) Testi
void testBoxSweepDistanceAndNormal() {
    std::cout << "\n--- 2. Box Sweep Distance and Normal Test ---" << std::endl;
    Simulator sim;

    // Hedef kutu: merkez (6, 0, 0), yarım boyutlar (1, 2, 2) -> sol yüzeyi X = 5.0
    Body target = sim.createBody(Pose(Vector3(6.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 2.0f, 2.0f))});
    target.setBodyType(Core::BodyType::Static);

    // Süpürülen kutu: başlangıç (0, 0, 0), yarım boyutlar (0.5, 0.5, 0.5) -> sağ yüzeyi X = +0.5
    // Yön: (1, 0, 0), Maks: 15
    // Analitik temas mesafesi: 5.0 - 0.5 = 4.5 m
    SweepHit hit = sim.sweepBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), Quaternion(), Vector3(1.0f, 0.0f, 0.0f), 15.0f);

    CHECK_MSG(hit.hasHit, "Kutu supurmesi hedefi vurmalidir");
    std::cout << "Box Sweep Hit Dist: " << hit.distance << " m (Expected: 4.5 m)" << std::endl;
    CHECK_MSG(std::abs(hit.distance - 4.5f) < 0.05f, "Kutu temas mesafesi 4.5m olmalidir");
    CHECK_MSG(std::abs(hit.point.x - 5.0f) < 0.05f, "Kutu temas noktasi X=5.0 olmalidir");
    CHECK_MSG(hit.normal.x < -0.9f, "Kutu temas normali sola (-X) bakmalidir");
}

// 3. Kapsül Süpürme (sweepCapsule) Testi
void testCapsuleSweep() {
    std::cout << "\n--- 3. Capsule Sweep Test ---" << std::endl;
    Simulator sim;

    // Hedef duvar: merkez (8, 0, 0), yarım boyut (0.5, 5, 5) -> sol yüzey X = 7.5
    Body wall = sim.createBody(Pose(Vector3(8.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.5f, 5.0f, 5.0f))});
    wall.setBodyType(Core::BodyType::Static);

    // Dikey kapsül: merkez (0, 0, 0), r = 0.5, h = 2.0
    // Yön: (1, 0, 0)
    // Kapsülün en sağ ucu X = 0.5
    // Beklenen temas mesafesi: 7.5 - 0.5 = 7.0 m
    SweepHit hit = sim.sweepCapsule(Vector3(0.0f, 0.0f, 0.0f), 0.5f, 2.0f, Quaternion(), Vector3(1.0f, 0.0f, 0.0f), 20.0f);

    CHECK_MSG(hit.hasHit, "Kapsul supurmesi duvari vurmalidir");
    std::cout << "Capsule Sweep Hit Dist: " << hit.distance << " m (Expected: 7.0 m)" << std::endl;
    CHECK_MSG(std::abs(hit.distance - 7.0f) < 0.05f, "Kapsul temas mesafesi 7.0m olmalidir");
}

// 4. Iskalanma ve Teğet Geçme (Miss / Grazing) Testi
void testSweepMiss() {
    std::cout << "\n--- 4. Sweep Miss & Grazing Test ---" << std::endl;
    Simulator sim;

    // Hedef kutu: merkez (5, 5, 0), yarım boyut (1, 1, 1) -> Y aralığı [4, 6]
    Body target = sim.createBody(Pose(Vector3(5.0f, 5.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    (void)target;

    // Süpürme: merkez (0, 0, 0), yarıçap 0.5, yön (1, 0, 0), Y=0 doğrultusunda geçer (Y=0, hedef Y=[4,6] -> temas yok)
    SweepHit hit = sim.sweepSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f, Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(!hit.hasHit, "Uzakta teget gecen supurme carpmamalidir");
    CHECK_MSG(hit.distance == 20.0f, "Iskalamada mesafe maxDistance kalmalidir");

    // Süpürme hedef yönünde ama maxDistance hedefe ulaşmaya yetmiyor
    SweepHit hitShort = sim.sweepSphere(Vector3(0.0f, 5.0f, 0.0f), 0.5f, Vector3(1.0f, 0.0f, 0.0f), 2.0f);
    CHECK_MSG(!hitShort.hasHit, "Yetersiz maxDistance carpmamalidir (Mesafe 2m < 3.5m)");
}

// 5. Başlangıçta Çakışma (t = 0 İlk Temas)
void testInitialOverlapSweep() {
    std::cout << "\n--- 5. Initial Overlap (t=0) Sweep Test ---" << std::endl;
    Simulator sim;

    // Hedef küre (0, 0, 0) r = 1.0
    Body target = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)target;

    // Süpürülen küre merkez (0.5, 0, 0) r = 1.0 -> Başlangıçta zaten iç içeler!
    SweepHit hit = sim.sweepSphere(Vector3(0.5f, 0.0f, 0.0f), 1.0f, Vector3(1.0f, 0.0f, 0.0f), 10.0f);
    CHECK_MSG(hit.hasHit, "Baslangicta ic ice olan supurme temas bildirmelidir");
    CHECK_MSG(hit.distance == 0.0f, "Baslangictaki temasta distance 0 olmalidir");
    CHECK_MSG(hit.fraction == 0.0f, "Baslangictaki temasta fraction 0 olmalidir");
}

// 6. Bileşik Şekil (CompoundShape) Süpürme Testi
void testCompoundShapeSweep() {
    std::cout << "\n--- 6. Compound Shape Sweep Test ---" << std::endl;
    Simulator sim;

    // Dambıl: sol küre (-3, 0, 0) r=1, sağ küre (+3, 0, 0) r=1
    // Ortası Y=0, X=0 boştur!
    std::vector<CompoundChild> children;
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(-3.0f, 0.0f, 0.0f)));
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(3.0f, 0.0f, 0.0f)));
    CompoundShape dumbbell(std::move(children));

    Body dumbbellBody = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{dumbbell});
    dumbbellBody.setBodyType(Core::BodyType::Static);

    // Test 6a: Dambılın ortasındaki boşluktan geçen küre (Z boyunca (0, 0, -10) -> (0, 0, +10))
    // AABB kapsasa bile boşluktan geçmeli ve ıskalamalıdır!
    SweepHit hitGap = sim.sweepSphere(Vector3(0.0f, 0.0f, -10.0f), 0.5f, Vector3(0.0f, 0.0f, 1.0f), 20.0f);
    CHECK_MSG(!hitGap.hasHit, "Dambilin boslugundan gecen kure carpmamalidir");

    // Test 6b: Dambılın sağ küresine doğru fırlatılan küre ((3, 0, -10) -> Z boyunca)
    // Sağ küre merkezi (3, 0, 0) r=1.0. Süpürülen r=0.5.
    // Beklenen temas mesafesi: 10 - (1.0 + 0.5) = 8.5 m
    SweepHit hitRight = sim.sweepSphere(Vector3(3.0f, 0.0f, -10.0f), 0.5f, Vector3(0.0f, 0.0f, 1.0f), 20.0f);
    CHECK_MSG(hitRight.hasHit, "Dambilin sag kuresine atilan kure carpmalidir");
    std::cout << "Compound Sweep Hit Dist: " << hitRight.distance << " m (Expected: 8.5 m)" << std::endl;
    CHECK_MSG(std::abs(hitRight.distance - 8.5f) < 0.05f, "Dambil sag kure temas mesafesi 8.5m olmalidir");
}

// 7. Katman Maskeleme (layerMask) ile Sıralı Hedef Seçimi
void testSweepLayerMasking() {
    std::cout << "\n--- 7. Sweep Layer Masking Test ---" << std::endl;
    Simulator sim;

    // İki engel aynı hat üzerinde peş peşe:
    // Engel 1 (Önde): X = 5, Katman 0x0001
    Body front = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)front.setCollisionLayer(0x0001);
    front.setBodyType(Core::BodyType::Static);

    // Engel 2 (Arkada): X = 10, Katman 0x0002
    Body back = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)back.setCollisionLayer(0x0002);
    back.setBodyType(Core::BodyType::Static);

    // Sorgu A: layerMask = 0x0001 -> Öndeki engeli vurur (X=5, mesafe 5 - 1.5 = 3.5)
    SweepHit hit1 = sim.sweepSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f, Vector3(1.0f, 0.0f, 0.0f), 20.0f, 0x0001);
    CHECK_MSG(hit1.hasHit && hit1.entity.id == front.getEntity().id, "Mask 0x0001 ondeki engeli vurmalidir");
    CHECK_MSG(std::abs(hit1.distance - 3.5f) < 0.05f, "Ondeki engelin mesafesi 3.5m olmalidir");

    // Sorgu B: layerMask = 0x0002 -> Öndeki engeli atlar/delip geçer ve arkadaki engeli vurur! (X=10, mesafe 10 - 1.5 = 8.5)
    SweepHit hit2 = sim.sweepSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f, Vector3(1.0f, 0.0f, 0.0f), 20.0f, 0x0002);
    CHECK_MSG(hit2.hasHit && hit2.entity.id == back.getEntity().id, "Mask 0x0002 arkadaki engeli vurmalidir (ondekini gormezden gelmeli)");
    CHECK_MSG(std::abs(hit2.distance - 8.5f) < 0.05f, "Arkadaki engelin mesafesi 8.5m olmalidir");
}

int main() {
    std::cout << "=== BARYON SWEEP QUERY TEST SUITE (PHASE 1.3) ===\n" << std::endl;

    testSphereSweepDistanceAndNormal();
    testBoxSweepDistanceAndNormal();
    testCapsuleSweep();
    testSweepMiss();
    testInitialOverlapSweep();
    testCompoundShapeSweep();
    testSweepLayerMasking();

    return reportAndExit();
}
