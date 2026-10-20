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
 * @file RaycastAllTest.cpp
 * @brief Faz 1.4 Çoklu Işın Kesişimi (raycastAll) Doğrulama Testi.
 * @details raycastAll fonksiyonunun ışının kestiği tüm nesneleri mesafeye göre
 *          küçükten büyüğe sıralı döndürmesini, katman maskesini ve geriye dönük uyumluluğunu sınar.
 */

// 1. Çoklu İsabet ve Mesafeye Göre Sıralama Testi
void testRaycastAllOrderAndDistances() {
    std::cout << "\n--- 1. Multi-hit Ordering and Distance Test ---" << std::endl;
    Simulator sim;

    // Aynı hat üzerinde X ekseni boyunca 3 kutu:
    // Kutu 1: X = 5, yarım boyut (1, 1, 1) -> Sol yüzey X = 4.0
    Body b1 = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b1.setBodyType(Core::BodyType::Static);

    // Kutu 2: X = 10, yarım boyut (1, 1, 1) -> Sol yüzey X = 9.0
    Body b2 = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b2.setBodyType(Core::BodyType::Static);

    // Kutu 3: X = 15, yarım boyut (1, 1, 1) -> Sol yüzey X = 14.0
    Body b3 = sim.createBody(Pose(Vector3(15.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b3.setBodyType(Core::BodyType::Static);

    // Orijinden (0, 0, 0) +X yönüne ışın fırlat
    auto hits = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 30.0f);

    CHECK_MSG(hits.size() == 3, "Isin 3 kutunun ucunu de delip gecmelidir");
    if (hits.size() == 3) {
        CHECK_MSG(hits[0].entity.id == b1.getEntity().id, "Ilk isabet en yakindaki b1 olmalidir");
        CHECK_MSG(hits[1].entity.id == b2.getEntity().id, "Ikinci isabet ortadaki b2 olmalidir");
        CHECK_MSG(hits[2].entity.id == b3.getEntity().id, "Ucuncu isabet en uzaktaki b3 olmalidir");

        std::cout << "Hit 0 Dist: " << hits[0].distance << " m (Expected: 4.0 m)" << std::endl;
        std::cout << "Hit 1 Dist: " << hits[1].distance << " m (Expected: 9.0 m)" << std::endl;
        std::cout << "Hit 2 Dist: " << hits[2].distance << " m (Expected: 14.0 m)" << std::endl;

        CHECK_MSG(std::abs(hits[0].distance - 4.0f) < 0.01f, "b1 temas mesafesi 4.0m olmalidir");
        CHECK_MSG(std::abs(hits[1].distance - 9.0f) < 0.01f, "b2 temas mesafesi 9.0m olmalidir");
        CHECK_MSG(std::abs(hits[2].distance - 14.0f) < 0.01f, "b3 temas mesafesi 14.0m olmalidir");

        CHECK_MSG(hits[0].distance < hits[1].distance, "Sonuclar mesafeye gore kucukten buyuge sirali olmalidir (1<2)");
        CHECK_MSG(hits[1].distance < hits[2].distance, "Sonuclar mesafeye gore kucukten buyuge sirali olmalidir (2<3)");
    }
}

// 2. Katman Maskeleme (layerMask) Filtreleme Testi
void testRaycastAllLayerMask() {
    std::cout << "\n--- 2. Layer Mask Filtering Test ---" << std::endl;
    Simulator sim;

    // Kutu 1: X = 5, Katman 0x0001
    Body b1 = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)b1.setCollisionLayer(0x0001);
    b1.setBodyType(Core::BodyType::Static);

    // Kutu 2: X = 10, Katman 0x0002
    Body b2 = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)b2.setCollisionLayer(0x0002);
    b2.setBodyType(Core::BodyType::Static);

    // Kutu 3: X = 15, Katman 0x0001
    Body b3 = sim.createBody(Pose(Vector3(15.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    (void)b3.setCollisionLayer(0x0001);
    b3.setBodyType(Core::BodyType::Static);

    // Sorgu A: layerMask = 0x0001 -> b1 ve b3 donmeli, b2 atlanmali
    auto hitsA = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 30.0f, 0x0001);
    CHECK_MSG(hitsA.size() == 2, "Mask 0x0001 sadece 2 nesneyi dondurmelidir");
    if (hitsA.size() == 2) {
        CHECK_MSG(hitsA[0].entity.id == b1.getEntity().id, "Ilk isabet b1 olmalidir");
        CHECK_MSG(hitsA[1].entity.id == b3.getEntity().id, "Ikinci isabet b3 olmalidir");
    }

    // Sorgu B: layerMask = 0x0002 -> yalnizca b2 donmeli
    auto hitsB = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 30.0f, 0x0002);
    CHECK_MSG(hitsB.size() == 1, "Mask 0x0002 yalnizca ortadaki b2 nesnesini dondurmelidir");
    if (hitsB.size() == 1) {
        CHECK_MSG(hitsB[0].entity.id == b2.getEntity().id, "Isabet b2 olmalidir");
    }
}

// 3. Maksimum Mesafe (maxDist) Sınırı Testi
void testRaycastAllMaxDistance() {
    std::cout << "\n--- 3. Max Distance Clamping Test ---" << std::endl;
    Simulator sim;

    Body b1 = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    b1.setBodyType(Core::BodyType::Static);
    Body b2 = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    b2.setBodyType(Core::BodyType::Static);
    Body b3 = sim.createBody(Pose(Vector3(20.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    b3.setBodyType(Core::BodyType::Static);

    // maxDist = 12.0f: b1 (mesafe 4) ve b2 (mesafe 9) dahil olmali, b3 (mesafe 19) menzil disi kalmali
    auto hits = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 12.0f);
    CHECK_MSG(hits.size() == 2, "maxDist=12.0f ile yalnizca ilk 2 nesne vurulmalidir");
    if (hits.size() == 2) {
        CHECK_MSG(hits[0].entity.id == b1.getEntity().id, "Ilk nesne b1 olmalidir");
        CHECK_MSG(hits[1].entity.id == b2.getEntity().id, "Ikinci nesne b2 olmalidir");
    }
}

// 4. Iskalamada Boş Sonuç Testi
void testRaycastAllMiss() {
    std::cout << "\n--- 4. Miss / Empty Result Test ---" << std::endl;
    Simulator sim;

    Body b = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b.setBodyType(Core::BodyType::Static);

    // Ters yone atilan isin (-X)
    auto hits = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(-1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(hits.empty(), "Ters yone atilan isin bos vektor dondurmelidir");
}

// 5. Geriye Dönük Uyumluluk (raycast == raycastAll[0]) Testi
void testRaycastAllCompatibility() {
    std::cout << "\n--- 5. Backward Compatibility (raycast vs raycastAll[0]) Test ---" << std::endl;
    Simulator sim;

    Body b1 = sim.createBody(Pose(Vector3(6.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    b1.setBodyType(Core::BodyType::Static);
    Body b2 = sim.createBody(Pose(Vector3(12.0f, 0.0f, 0.0f)), CollisionShape{SphereShape(1.0f)});
    b2.setBodyType(Core::BodyType::Static);

    RaycastHit singleHit = sim.raycast(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    auto allHits = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);

    CHECK_MSG(singleHit.hasHit, "Tekil raycast temas bildirmelidir");
    CHECK_MSG(!allHits.empty(), "Coklu raycastAll en az 1 eleman dondurmelidir");
    if (singleHit.hasHit && !allHits.empty()) {
        CHECK_MSG(singleHit.entity.id == allHits[0].entity.id, "raycast sonucu raycastAll'un ilk elemaniyla ayni varlik olmalidir");
        CHECK_MSG(std::abs(singleHit.distance - allHits[0].distance) < 1e-4f, "Mesafe degerleri birebir eslesmelidir");
        CHECK_MSG(std::abs(singleHit.position.x - allHits[0].position.x) < 1e-4f, "Temas noktalari birebir eslesmelidir");
    }
}

// 6. Bileşik Şekil (CompoundShape) raycastAll Testi
void testRaycastAllCompoundShape() {
    std::cout << "\n--- 6. Compound Shape raycastAll Test ---" << std::endl;
    Simulator sim;

    std::vector<CompoundChild> children;
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(0.0f, 0.0f, -3.0f)));
    children.emplace_back(CollisionShape{SphereShape(1.0f)}, Pose(Vector3(0.0f, 0.0f, 3.0f)));
    CompoundShape dumbbell(std::move(children));

    Body dumbbellBody = sim.createBody(Pose(Vector3(10.0f, 0.0f, 0.0f)), CollisionShape{dumbbell});
    dumbbellBody.setBodyType(Core::BodyType::Static);

    // Z=3 cizgisi boyunca Z kuresine atilan isin
    auto hitsZ3 = sim.raycastAll(Vector3(0.0f, 0.0f, 3.0f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(hitsZ3.size() == 1, "Z=3 kuresine atilan isin dambili vurmalidir");
    if (!hitsZ3.empty()) {
        std::cout << "Dumbbell Hit Dist: " << hitsZ3[0].distance << " m (Expected: 9.0 m)" << std::endl;
        CHECK_MSG(std::abs(hitsZ3[0].distance - 9.0f) < 0.05f, "Dambil temas mesafesi 9.0m olmalidir (10 - 1)");
    }

    // Ortadaki bosluktan (Z=0) atilan isin dambili iskalayacaktir
    auto hitsZ0 = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 20.0f);
    CHECK_MSG(hitsZ0.empty(), "Dambilin ortasindaki bosluktan gecen isin iskalayip bos donmelidir");
}

// 7. Dejenere Girdi Testi
void testRaycastAllDegenerate() {
    std::cout << "\n--- 7. Degenerate Inputs Test ---" << std::endl;
    Simulator sim;

    Body b = sim.createBody(Pose(Vector3(5.0f, 0.0f, 0.0f)), CollisionShape{BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b.setBodyType(Core::BodyType::Static);

    // Sifir mesafe
    auto h1 = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 0.0f);
    CHECK_MSG(h1.empty(), "maxDist=0 bos donmelidir");

    // Negatif mesafe
    auto h2 = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), -10.0f);
    CHECK_MSG(h2.empty(), "Negatif maxDist bos donmelidir");

    // Sifir uzunluklu yon
    auto h3 = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), 10.0f);
    CHECK_MSG(h3.empty(), "Sifir yon bos donmelidir");

    // Sifir katman maskesi
    auto h4 = sim.raycastAll(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), 10.0f, 0x00000000);
    CHECK_MSG(h4.empty(), "Sifir katman maskesi bos donmelidir");
}

int main() {
    std::cout << "=== BARYON RAYCAST ALL TEST SUITE (PHASE 1.4) ===\n" << std::endl;

    testRaycastAllOrderAndDistances();
    testRaycastAllLayerMask();
    testRaycastAllMaxDistance();
    testRaycastAllMiss();
    testRaycastAllCompatibility();
    testRaycastAllCompoundShape();
    testRaycastAllDegenerate();

    return reportAndExit();
}
