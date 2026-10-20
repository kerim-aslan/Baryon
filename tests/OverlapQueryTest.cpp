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
#include <algorithm>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

/**
 * @file OverlapQueryTest.cpp
 * @brief Faz 1.2 Çakışma / Hacim Sorguları (Overlap Queries) Doğrulama Testi.
 * @details overlapSphere, overlapBox, overlapShape, katman maskeleme ve 
 *          bileşik şekil hassasiyetini sınar.
 */

static bool containsEntity(const std::vector<ecs::Entity>& vec, ecs::Entity e) {
    return std::find_if(vec.begin(), vec.end(), [e](ecs::Entity it) {
        return it.id == e.id;
    }) != vec.end();
}

// 1. Küre Hacim Sorgusu (overlapSphere)
void testOverlapSphere() {
    std::cout << "\n--- 1. Sphere Overlap Query Test ---" << std::endl;
    Simulator sim;

    // Merkezde (0,0,0) yarıçapı 1 olan küre
    Body s1 = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    // (2.5, 0, 0) konumunda yarıçapı 1 olan küre (merkezler arası mesafe 2.5)
    Body s2 = sim.createBody(Pose(Vector3(2.5f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    // (10, 0, 0) konumunda uzak küre
    Body s3 = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});

    // Sorgu 1: (0,0,0) merkezli, r = 0.5 küre -> s1 içinde kalır, sadece s1 kesişmeli
    auto res1 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f);
    CHECK_MSG(res1.size() == 1, "r=0.5 sorgusu sadece 1 nesne bulmali");
    CHECK_MSG(containsEntity(res1, s1.getEntity()), "s1 bulunmali");

    // Sorgu 2: (0,0,0) merkezli, r = 1.6 küre -> s1 (r=1) ve s2 (pos=2.5, r=1 -> min mesafe 1.5) ile kesişir (1.6 > 1.5)
    auto res2 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 1.6f);
    CHECK_MSG(res2.size() == 2, "r=1.6 sorgusu s1 ve s2'yi bulmali");
    CHECK_MSG(containsEntity(res2, s1.getEntity()) && containsEntity(res2, s2.getEntity()), "s1 ve s2 bulunmali");
    CHECK_MSG(!containsEntity(res2, s3.getEntity()), "s3 bulunmamali");

    // Sorgu 3: (0,0,0) merkezli, r = 1.4 küre -> s2 ile min mesafe 1.5, bu yüzden kesişmemeli
    auto res3 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 1.4f);
    CHECK_MSG(res3.size() == 1, "r=1.4 sorgusu sadece s1'i bulmali");
    CHECK_MSG(containsEntity(res3, s1.getEntity()), "s1 bulunmali");
}

// 2. Yönelimli Kutu Hacim Sorgusu (overlapBox / OBB)
void testOverlapBoxOBB() {
    std::cout << "\n--- 2. Oriented Box (OBB) Overlap Query Test ---" << std::endl;
    Simulator sim;

    // (2, 2, 0) noktasında 0.2 yarıçaplı küçük bir küre
    Body probe = sim.createBody(Pose(Vector3(2.0f, 2.0f, 0.0f)), CollisionShape{SphereShape(0.2f)});

    // Kutu: yarım boyutlar (2, 0.5, 0.5), merkez (0,0,0)
    // Döndürülmemiş hali: X [-2, 2], Y [-0.5, 0.5] -> (2, 2, 0) dışındadır!
    auto resUnrotated = sim.overlapBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(2.0f, 0.5f, 0.5f));
    CHECK_MSG(!containsEntity(resUnrotated, probe.getEntity()), "Dondurulmemis kutu (2,2,0)'daki nesneyi kesmemeli");

    // Şimdi aynı kutuyu Z ekseni etrafında 45 derece (pi/4) döndürürsek:
    // Köşegen doğrultusu (1, 1, 0) yönünde uzanır ve (2, 2, 0) doğrultusunu kapsar!
    // 45 derece rotasyon kuaterniyonu:
    Quaternion rot45(Vector3(0.0f, 0.0f, 1.0f), 3.14159265f * 0.25f);
    auto resRotated = sim.overlapBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(3.0f, 0.5f, 0.5f), rot45);
    CHECK_MSG(containsEntity(resRotated, probe.getEntity()), "45 derece dondurulmus OBB kutu nesneyi kesmeli");
}

// 3. Bileşik Şekil (CompoundShape) Sorgusu
void testOverlapCompoundShape() {
    std::cout << "\n--- 3. Compound Shape Overlap Query Test ---" << std::endl;
    Simulator sim;

    // Dambıl şeklinde bileşik cisim:
    // İki küre: (-3, 0, 0) r=1 ve (+3, 0, 0) r=1
    // Ortada hiçbir şey yok! (0,0,0) boştur.
    std::vector<CompoundChild> children;
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(-3.0f, 0.0f, 0.0f)));
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(3.0f, 0.0f, 0.0f)));
    CompoundShape dumbbell(std::move(children));

    Body dumbbellBody = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{dumbbell});

    // Sorgu 1: AABB'si dambılı kapsayan ama tam ortadaki boşlukta duran küçük küre: (0, 0, 0), r = 0.5
    // AABB testi pozitif çıksa bile dar faz (NarrowPhase) kesişim OLMADIĞINI tespit etmelidir!
    auto resCenter = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 0.5f);
    CHECK_MSG(resCenter.empty(), "Dambilin ortasindaki boslukta kesisim olmamali (AABB yalanci pozitif elenmeli)");

    // Sorgu 2: Sol küreyi kesen sorgu: (-3, 0, 0), r = 0.5
    auto resLeft = sim.overlapSphere(Vector3(-3.0f, 0.0f, 0.0f), 0.5f);
    CHECK_MSG(resLeft.size() == 1 && containsEntity(resLeft, dumbbellBody.getEntity()), "Sol kure ile kesisim saptanmali");

    // Sorgu 3: overlapShape ile başka bir CompoundShape'in dambıl ile sorgulanması
    std::vector<CompoundChild> queryChildren;
    queryChildren.emplace_back(CollisionShape{SphereShape(0.5f)}, Pose(Vector3(3.0f, 0.0f, 0.0f)));
    CompoundShape queryComp(std::move(queryChildren));
    auto resComp = sim.overlapShape(CollisionShape{queryComp}, Pose(Vector3(0.0f, 0.0f, 0.0f)));
    CHECK_MSG(containsEntity(resComp, dumbbellBody.getEntity()), "Bilesik sorgu dambili saptamali");
}

// 4. Katman Maskeleme (layerMask) Filtreleme Testi
void testLayerMaskFiltering() {
    std::cout << "\n--- 4. Layer Mask Filtering Test ---" << std::endl;
    Simulator sim;

    // (0,0,0) konumunda 3 farklı katmanda 3 kutu
    Body b1 = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    (void)b1.setCollisionLayer(0x0001); // Katman 1 (Düşmanlar)

    Body b2 = sim.createBody(Pose(Vector3(0.5f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    (void)b2.setCollisionLayer(0x0002); // Katman 2 (Dostlar)

    Body b3 = sim.createBody(Pose(Vector3(0.0f, 0.5f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    (void)b3.setCollisionLayer(0x0004); // Katman 3 (Tetikleyiciler)

    // Mask 0x0001 -> Yalnızca b1
    auto res1 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 2.0f, 0x0001);
    CHECK_MSG(res1.size() == 1 && containsEntity(res1, b1.getEntity()), "Mask 0x0001 sadece b1 vermeli");

    // Mask 0x0002 -> Yalnızca b2
    auto res2 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 2.0f, 0x0002);
    CHECK_MSG(res2.size() == 1 && containsEntity(res2, b2.getEntity()), "Mask 0x0002 sadece b2 vermeli");

    // Mask 0x0003 (Katman 1 | 2) -> b1 ve b2
    auto res3 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 2.0f, 0x0003);
    CHECK_MSG(res3.size() == 2, "Mask 0x0003 b1 ve b2 vermeli");
    CHECK_MSG(containsEntity(res3, b1.getEntity()) && containsEntity(res3, b2.getEntity()), "b1 ve b2 icermeli");

    // Mask 0x0008 (Hiçbiri eşleşmiyor)
    auto res4 = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 2.0f, 0x0008);
    CHECK_MSG(res4.empty(), "Eslesmeyen mask bos donmeli");

    // Varsayılan Mask (0xFFFFFFFF) -> Hepsi
    auto resAll = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 2.0f);
    CHECK_MSG(resAll.size() == 3, "Varsayilan mask 3 nesneyi de vermeli");
}

// 5. Statik ve Tetikleyici (Trigger) Varlıkların Algılanması
void testStaticAndTriggerDetection() {
    std::cout << "\n--- 5. Static & Trigger Overlap Detection Test ---" << std::endl;
    Simulator sim;

    Body staticBody = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    staticBody.setBodyType(Core::BodyType::Static);

    Body triggerBody = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)triggerBody.setTrigger(true);

    auto res = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 1.5f);
    CHECK_MSG(res.size() == 2, "Hem statik hem de tetikleyici govde sorguda cikmali");
    CHECK_MSG(containsEntity(res, staticBody.getEntity()), "Statik govde bulunmali");
    CHECK_MSG(containsEntity(res, triggerBody.getEntity()), "Tetikleyici govde bulunmali");
}

// 6. Body Nesnesi Döndüren Kolaylık Fonksiyonları (overlapSphereBodies vb.)
void testOverlapBodiesConvenience() {
    std::cout << "\n--- 6. Overlap Bodies Convenience API Test ---" << std::endl;
    Simulator sim;

    Body b1 = sim.createBody(Pose(Vector3(1.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    Body b2 = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)b2;

    auto bodies = sim.overlapSphereBodies(Vector3(1.0f, 0.0f, 0.0f), 1.0f);
    CHECK_MSG(bodies.size() == 1, "overlapSphereBodies 1 nesne dondurmeli");
    CHECK_MSG(bodies[0].getEntity().id == b1.getEntity().id, "Dondurulen Body b1 olmali");
    CHECK_MSG(bodies[0].getPosition().value().x == 1.0f, "Body API uzerinden pozisyon okunabilmeli");
}

// 7. Dejenere / Sınır Durum Güvenliği (Zero Radius, Empty World)
void testDegenerateEdgeCases() {
    std::cout << "\n--- 7. Degenerate Edge Cases Test ---" << std::endl;
    Simulator sim;

    // Boş dünyada sorgu çökmemeli
    auto resEmpty = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 10.0f);
    CHECK_MSG(resEmpty.empty(), "Bos dunyada sorgu bos dondurmeli");

    // Cisim varken sıfır veya negatif yarıçap
    Body b = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)b;
    auto resZeroR = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), 0.0f);
    CHECK_MSG(resZeroR.empty(), "Sifir yaricapli kure sorgusu guvenle bos donmeli");

    auto resNegR = sim.overlapSphere(Vector3(0.0f, 0.0f, 0.0f), -5.0f);
    CHECK_MSG(resNegR.empty(), "Negatif yaricapli kure sorgusu guvenle bos donmeli");
}

int main() {
    std::cout << "=== BARYON OVERLAP QUERY TEST SUITE (PHASE 1.2) ===\n" << std::endl;

    testOverlapSphere();
    testOverlapBoxOBB();
    testOverlapCompoundShape();
    testLayerMaskFiltering();
    testStaticAndTriggerDetection();
    testOverlapBodiesConvenience();
    testDegenerateEdgeCases();

    return reportAndExit();
}
