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
 * @file HeightfieldShapeTest.cpp
 * @brief Baryon fizik motoru Faz 2.3 HeightfieldShape (arazi geometrisi) test paketi.
 * 
 * Bu dosya duz heightfield zemin davranisi ve resting height/sleep dogrulamasi,
 * egimli arazide kure yuvarlanmasi, hucre sinirlarinda diken/sekme (ghost-collision) onleme,
 * 256x256 hucreli arazide 200 kure ile performans olcumu, analitik isin kesisimi (raycast)
 * ve bozulmus (degenerate) girdi guvenligi kontrollerini icerir.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <chrono>

using namespace Baryon;
using namespace Baryon::collision;

// 1. Düz Heightfield = Düz Zemin Davranışı ve Uyku Testi
void testFlatHeightfieldResting() {
    std::cout << "\n--- 1. Flat Heightfield Resting & Sleep Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // 10x10 duz arazi, cellSize = 1.0m, toplam genislik 9x9m, yukseklik y = 0.0m
    int nx = 10;
    int nz = 10;
    float cellSize = 1.0f;
    std::vector<float> heights(static_cast<size_t>(nx * nz), 0.0f);

    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{HeightfieldShape(nx, nz, cellSize, heights, true)});
    terrain.setBodyType(Core::BodyType::Static);

    // Kutu: yarim boyut 0.5m (tam boyut 1x1x1m), kutle 1.0kg. Baslangic y = 2.0m
    Body box = sim.createBody(Pose(Vector3(0.0f, 2.0f, 0.0f)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    box.setBodyType(Core::BodyType::Dynamic);
    (void)box.setMass(1.0f);

    // Kure: yaricap 0.5m, kutle 1.0kg. Baslangic y = 2.0m, x = 2.0m
    Body sphere = sim.createBody(Pose(Vector3(2.0f, 2.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    sphere.setBodyType(Core::BodyType::Dynamic);
    (void)sphere.setMass(1.0f);

    // Simulasyon: 5 saniye (300 adim)
    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }

    float boxY = box.getPosition().value().y;
    float sphereY = sphere.getPosition().value().y;
    std::cout << "Box Final Resting Y: " << boxY << " m (Beklenen: 0.5 m)" << std::endl;
    std::cout << "Sphere Final Resting Y: " << sphereY << " m (Beklenen: 0.5 m)" << std::endl;

    // Duz zemin dinlenme yuksekligi kontrolu (+/- 0.02m slop payi ile)
    CHECK_MSG(std::abs(boxY - 0.5f) <= 0.02f, "Kutu duz heightfield uzerinde 0.5m yukseklikte durmalidir");
    CHECK_MSG(std::abs(sphereY - 0.5f) <= 0.02f, "Kure duz heightfield uzerinde 0.5m yukseklikte durmalidir");

    // Cisimler durulmali ve uykuya gecebilmelidir
    CHECK(box.isSleeping().value());
    CHECK(sphere.isSleeping().value());
}

// 2. Eğimli Arazide Küre Yuvarlanma Testi
void testSlopedTerrainRolling() {
    std::cout << "\n--- 2. Sloped Terrain Rolling Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // X ekseni boyunca egimli arazi: y(x) = -0.2f * x
    int nx = 20;
    int nz = 10;
    float cellSize = 1.0f;
    std::vector<float> heights(static_cast<size_t>(nx * nz), 0.0f);
    float totalW = static_cast<float>(nx - 1) * cellSize;
    float halfW = totalW * 0.5f;

    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            float vx = static_cast<float>(i) * cellSize - halfW;
            heights[static_cast<size_t>(j * nx + i)] = -0.2f * vx; // Eğimli rampa
        }
    }

    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{HeightfieldShape(nx, nz, cellSize, heights, true)});
    terrain.setBodyType(Core::BodyType::Static);

    // Kureyi yuksek tarafa koyalim (x = -5.0m -> y = +1.0m)
    float startX = -5.0f;
    float startY = -0.2f * startX + 0.55f;
    Body sphere = sim.createBody(Pose(Vector3(startX, startY, 0.0f)), CollisionShape{SphereShape(0.5f)});
    sphere.setBodyType(Core::BodyType::Dynamic);
    (void)sphere.setMass(1.0f);
    (void)sphere.setMaterial(0.3f, 0.0f);

    // 240 adim kostur (4 saniye)
    for (int i = 0; i < 240; ++i) {
        sim.step(1.0f / 60.0f);
    }

    float finalX = sphere.getPosition().value().x;
    float finalVx = sphere.getLinearVelocity().value().x;
    std::cout << "Sphere Final X: " << finalX << " (Start: " << startX << "), Vx: " << finalVx << std::endl;

    // Kure egim asagi dogru (+X yonunde) ivmelenmeli ve ilerlemelidir
    CHECK_MSG(finalX > startX + 1.0f, "Kure egimli arazide asagi dogru ilerlemelidir");
    CHECK_MSG(finalVx > 0.5f, "Kure egimli arazide pozitif yatay hiza ulasmalidir");
}

// 3. Ghost-Collision (Hücre Sınırı Diken/Sekme Önleme) Testi
void testGhostCollisionAtCellBorders() {
    std::cout << "\n--- 3. Ghost Collision / Internal Edge Filtering Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // 20x5 duz arazi, cellSize = 1.0m
    int nx = 20;
    int nz = 5;
    float cellSize = 1.0f;
    std::vector<float> heights(static_cast<size_t>(nx * nz), 0.0f);

    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{HeightfieldShape(nx, nz, cellSize, heights, true)});
    terrain.setBodyType(Core::BodyType::Static);

    // Kure tam zemin seviyesinde yatay yuksek hizla kaydirilir (vx = 10.0 m/s)
    Body sphere = sim.createBody(Pose(Vector3(-8.0f, 0.5f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    sphere.setBodyType(Core::BodyType::Dynamic);
    (void)sphere.setMass(1.0f);
    (void)sphere.setMaterial(0.0f, 0.0f); // Surtunmesiz kayma
    (void)sphere.setLinearVelocity(Vector3(10.0f, 0.0f, 0.0f));

    // Kure hucre sinirlarini teker teker asarken dikey ziplama / diken yasamamali
    float maxVy = 0.0f;
    float maxY = 0.5f;

    for (int i = 0; i < 90; ++i) {
        sim.step(1.0f / 60.0f);
        float vy = sphere.getLinearVelocity().value().y;
        float y = sphere.getPosition().value().y;
        if (std::abs(vy) > maxVy) maxVy = std::abs(vy);
        if (y > maxY) maxY = y;
    }

    std::cout << "Max Vertical Speed (Vy) across borders: " << maxVy << " m/s, Max Y: " << maxY << " m" << std::endl;
    // Duz arazide sinirlardan gecerken kure sekip ziplamamali (max Vy < 0.25 m/s ve maxY < 0.55m)
    CHECK_MSG(maxVy < 0.25f, "Hucre sinirlarindan gecerken kurede dikey sicrama (ghost collision) olmamalidir");
    CHECK_MSG(std::abs(maxY - 0.5f) <= 0.05f, "Kure zemin duzleminde kalmalidir");
}

// 4. Performans ve Stres Testi (256x256 Izgarada 200 Kure)
void testHeightfieldPerformanceBenchmark() {
    std::cout << "\n--- 4. Performance Benchmark: 256x256 Grid with 200 Spheres ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // 256x256 buyuk arazi = 65,536 tepe noktasi, 130,050 ucgen!
    int nx = 256;
    int nz = 256;
    float cellSize = 2.0f;
    std::vector<float> heights(static_cast<size_t>(nx * nz), 0.0f);

    // Hafif dalgali arazi: sin(x) * cos(z)
    for (int j = 0; j < nz; ++j) {
        for (int i = 0; i < nx; ++i) {
            float x = static_cast<float>(i) * 0.1f;
            float z = static_cast<float>(j) * 0.1f;
            heights[static_cast<size_t>(j * nx + i)] = std::sin(x) * std::cos(z) * 1.5f;
        }
    }

    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{HeightfieldShape(nx, nz, cellSize, heights, true)});
    terrain.setBodyType(Core::BodyType::Static);

    // 200 adet kure rastgele/duzenli dagitilir
    const int numSpheres = 200;
    for (int k = 0; k < numSpheres; ++k) {
        float x = static_cast<float>(k % 20 - 10) * 4.0f;
        float z = static_cast<float>(k / 20 - 5) * 4.0f;
        Body sp = sim.createBody(Pose(Vector3(x, 10.0f + static_cast<float>(k % 5) * 0.5f, z)), CollisionShape{SphereShape(0.5f)});
        sp.setBodyType(Core::BodyType::Dynamic);
        (void)sp.setMass(1.0f);
    }

    // 60 adim kostur ve toplam / adim basina sureyi olc
    auto start = std::chrono::high_resolution_clock::now();
    for (int step = 0; step < 60; ++step) {
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgStepMs = totalMs / 60.0;
    std::cout << "256x256 Grid (130k triangles) with 200 active spheres:" << std::endl;
    std::cout << "Total Time (60 steps): " << totalMs << " ms | Avg Step Time: " << avgStepMs << " ms" << std::endl;

    // O(1) grid arama sayesinde adim suresi makul olmali (< 35ms adim basina)
    CHECK_MSG(avgStepMs < 35.0, "256x256 arazide 200 kure adim suresi kabul edilebilir sinirlarda olmalidir (< 35 ms)");
}

// 5. Işın Kesişimi (Raycast) ve Overlap Sorguları Testi
void testHeightfieldRaycastAndOverlap() {
    std::cout << "\n--- 5. Heightfield Raycast & Overlap Test ---" << std::endl;
    Simulator sim;

    int nx = 10;
    int nz = 10;
    float cellSize = 2.0f;
    std::vector<float> heights(static_cast<size_t>(nx * nz), 0.0f);

    // Merkezdeki hucreyi tepe yapalim (x=4,5, z=4,5)
    heights[static_cast<size_t>(4 * nx + 4)] = 2.0f;
    heights[static_cast<size_t>(4 * nx + 5)] = 2.0f;
    heights[static_cast<size_t>(5 * nx + 4)] = 2.0f;
    heights[static_cast<size_t>(5 * nx + 5)] = 2.0f;

    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{HeightfieldShape(nx, nz, cellSize, heights, true)});
    terrain.setBodyType(Core::BodyType::Static);

    // Yukaridan duz zemin alanina dik isin: (x = -4, z = -4, y = 5) -> yon: (0, -1, 0)
    RaycastHit hitFlat = sim.raycast(Vector3(-4.0f, 5.0f, -4.0f), Vector3(0.0f, -1.0f, 0.0f), 10.0f);
    CHECK_MSG(hitFlat.hasHit, "Duz zemin alanina atilan isin temas etmelidir");
    CHECK_MSG(std::abs(hitFlat.distance - 5.0f) <= 0.01f, "Isin mesafesi tam 5.0m olmalidir");
    CHECK_MSG(std::abs(hitFlat.position.y - 0.0f) <= 0.01f, "Isin carpma noktasi Y = 0.0m olmalidir");
    CHECK_MSG(std::abs(hitFlat.normal.y - 1.0f) <= 0.01f, "Isin carpma normali +Y yonunde olmalidir");

    // Overlap sorgusu: zemin uzerindeki bir kure ile test
    auto entities = sim.overlapSphere(Vector3(-4.0f, 0.3f, -4.0f), 0.5f);
    CHECK_MSG(!entities.empty(), "Zemin ile kesisim alanindaki overlapSphere araziyi bulmalidir");
}

// 6. Dejenere ve Uç Değerler Testi
void testHeightfieldDegenerateInputs() {
    std::cout << "\n--- 6. Heightfield Degenerate Inputs Test ---" << std::endl;
    Simulator sim;

    // nx < 2, nz < 2, cellSize <= 0, NaN yukseklikler
    std::vector<float> emptyHeights;
    HeightfieldShape hfDegenerate(0, -5, -2.0f, emptyHeights, true);

    CHECK(hfDegenerate.getWidthSamples() >= 2);
    CHECK(hfDegenerate.getLengthSamples() >= 2);
    CHECK(hfDegenerate.getCellSize() > 0.0f);
    CHECK(!hfDegenerate.getHeights().empty());

    // NaN iceren yukseklik dizisi
    std::vector<float> nanHeights = {0.0f, NAN, INFINITY, 1.0f};
    HeightfieldShape hfNan(2, 2, 1.0f, nanHeights, true);
    for (float h : hfNan.getHeights()) {
        CHECK(!std::isnan(h));
        CHECK(!std::isinf(h));
    }

    // Degenerate girdiyle govde olusturulup simulasyon adimi atilabilmelidir (cokme olmamali)
    Body terrain = sim.createBody(Pose(Vector3(0.0f, 0.0f, 0.0f)), CollisionShape{hfNan});
    terrain.setBodyType(Core::BodyType::Static);
    sim.step(1.0f / 60.0f);
}

int main() {
    std::cout << "=== BARYON HEIGHTFIELD SHAPE TEST SUITE (PHASE 2.3) ===" << std::endl;

    testFlatHeightfieldResting();
    testSlopedTerrainRolling();
    testGhostCollisionAtCellBorders();
    testHeightfieldPerformanceBenchmark();
    testHeightfieldRaycastAndOverlap();
    testHeightfieldDegenerateInputs();

    return reportAndExit();
}
