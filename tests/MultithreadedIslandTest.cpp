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
 * @file MultithreadedIslandTest.cpp
 * @brief Çok iş parçacıklı ada (island) çözücüsü ve determinizm testleri (Faz 5.1).
 * @details std::jthread havuzu altında 1 iş parçacığı ile N iş parçacığı arasında
 *          bit seviyesinde durum hash determinizmini ve 10k cisimde hızlanma faktörünü doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <atomic>

using namespace Baryon;
using namespace Baryon::collision;

// 64-bit FNV-1a Hash Fonksiyonu
static inline uint64_t fnv1a64(const void* data, size_t size, uint64_t hash = 14695981039346656037ULL) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

static uint64_t computeWorldStateHash(const std::vector<Body>& bodies) {
    uint64_t hash = 14695981039346656037ULL;
    for (const auto& b : bodies) {
        Vector3 pos = b.getPosition().value();
        Quaternion ori = b.getOrientation().value();
        Vector3 linVel = b.getLinearVelocity().value();
        Vector3 angVel = b.getAngularVelocity().value();

        hash = fnv1a64(&pos.x, sizeof(float), hash);
        hash = fnv1a64(&pos.y, sizeof(float), hash);
        hash = fnv1a64(&pos.z, sizeof(float), hash);

        hash = fnv1a64(&ori.x, sizeof(float), hash);
        hash = fnv1a64(&ori.y, sizeof(float), hash);
        hash = fnv1a64(&ori.z, sizeof(float), hash);
        hash = fnv1a64(&ori.w, sizeof(float), hash);

        hash = fnv1a64(&linVel.x, sizeof(float), hash);
        hash = fnv1a64(&linVel.y, sizeof(float), hash);
        hash = fnv1a64(&linVel.z, sizeof(float), hash);

        hash = fnv1a64(&angVel.x, sizeof(float), hash);
        hash = fnv1a64(&angVel.y, sizeof(float), hash);
        hash = fnv1a64(&angVel.z, sizeof(float), hash);
    }
    return hash;
}

/**
 * @brief 1. Test: 1 İş Parçacığı vs N İş Parçacığı Determinizm Doğrulaması.
 * @details 20 adet birbirinden bağımsız kule/ada içeren senaryoda hem 1 thread hem 8 thread
 *          ile 300 adım koşturulduğunda ulaşılan durum hash'i bit seviyesinde birebir aynı olmalıdır.
 */
void testMultithreadedDeterminism() {
    std::cout << "[MultithreadedIslandTest] 1. 1 Thread vs 8 Thread Determinizm Testi...\n";

    auto runSimulation = [](uint32_t threadCount) -> uint64_t {
        Simulator sim;
        SimulationSettings settings = sim.getSettings();
        settings.threadCount = threadCount;
        sim.setSettings(settings);
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        std::vector<Body> dynamicBodies;

        // 20 ayrı bağımsız ada: her adada 1 zemin ve üst üste 3 kutu
        for (int i = 0; i < 20; ++i) {
            float xOffset = static_cast<float>(i) * 10.0f;

            // Zemin
            Body ground = sim.createBody(Pose(Vector3(xOffset, -0.5f, 0.0f)),
                                         CollisionShape{BoxShape(Vector3(4.0f, 0.5f, 4.0f))});
            (void)ground.setBodyType(Core::BodyType::Static);

            // 3 katlı kutu kulesi
            for (int k = 0; k < 3; ++k) {
                float yPos = 0.5f + static_cast<float>(k) * 1.05f;
                Body box = sim.createBody(Pose(Vector3(xOffset, yPos, 0.0f)),
                                          CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
                (void)box.setBodyType(Core::BodyType::Dynamic);
                (void)box.setMass(1.0f);
                (void)box.setMaterial(0.5f, 0.2f);
                dynamicBodies.push_back(box);
            }
        }

        const float dt = 1.0f / 60.0f;
        for (int step = 0; step < 300; ++step) {
            sim.step(dt);
        }

        return computeWorldStateHash(dynamicBodies);
    };

    uint64_t hashSingle = runSimulation(1);
    uint64_t hashMulti = runSimulation(8);

    std::cout << "  Single-threaded (1T) Hash: 0x" << std::hex << hashSingle << std::dec << "\n";
    std::cout << "  Multi-threaded  (8T) Hash: 0x" << std::hex << hashMulti << std::dec << "\n";

    CHECK_MSG(hashSingle != 0, "Durum hash sifir olmamalidir");
    CHECK_MSG(hashSingle == hashMulti, "1 is parcacigi ve 8 is parcaciginda durum hash'i bit seviyesinde ayni olmalidir");
    std::cout << "  -> Determinizm basariyla dogrulandi.\n";
}

/**
 * @brief 2. Test: 10k Cisim Hızlanma Testi (1T vs 8T).
 * @details 10,000 cisimden oluşan çok adalı sahnede 8 çekirdekte hızlanma (speedup) ölçülür.
 */
void testScaleAndSpeedup10k() {
    std::cout << "[MultithreadedIslandTest] 2. 10,000 Cisim Hizlanma (Speedup) Testi...\n";

    struct BenchResult {
        double totalStepMs{0.0};
        double totalSolverMs{0.0};
    };

    auto benchmarkPiles = [](uint32_t threadCount, int totalBodies, int steps) -> BenchResult {
        Simulator sim;
        sim.getRegistry().setMaxCapacity(static_cast<uint32_t>(totalBodies + 2000));
        SimulationSettings settings = sim.getSettings();
        settings.threadCount = threadCount;
        settings.velocityIterations = 15;
        settings.positionIterations = 4;
        settings.enableSleeping = false; // Sürekli çözücü yükünü ölçmek için uyku kapalı
        sim.setSettings(settings);
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        int bodiesPerIsland = 10;
        int numIslands = totalBodies / bodiesPerIsland;

        for (int i = 0; i < numIslands; ++i) {
            float xOffset = static_cast<float>(i % 100) * 5.0f;
            float zOffset = static_cast<float>(i / 100) * 5.0f;

            Body ground = sim.createBody(Pose(Vector3(xOffset, -0.5f, zOffset)),
                                         CollisionShape{BoxShape(Vector3(1.0f, 0.5f, 1.0f))});
            (void)ground.setBodyType(Core::BodyType::Static);

            for (int k = 0; k < bodiesPerIsland; ++k) {
                float yPos = 0.5f + static_cast<float>(k) * 0.99f;
                Body box = sim.createBody(Pose(Vector3(xOffset, yPos, zOffset)),
                                          CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
                (void)box.setBodyType(Core::BodyType::Dynamic);
                (void)box.setMass(1.0f);
            }
        }

        const float dt = 1.0f / 60.0f;
        // Isınma adımı: temas manifoldlarını oluştur
        sim.step(dt);

        double accumulatedSolverMs = 0.0;
        auto start = std::chrono::high_resolution_clock::now();
        for (int s = 0; s < steps; ++s) {
            sim.step(dt);
            accumulatedSolverMs += sim.getLastSolverTimeMs();
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        return { elapsed.count(), accumulatedSolverMs };
    };

    const int totalBodies = 10000;
    const int benchmarkSteps = 15;

    std::cout << "  10,000 cisim hazırlanıyor, 1 iş parçacığı ölçülüyor...\n";
    BenchResult res1T = benchmarkPiles(1, totalBodies, benchmarkSteps);
    std::cout << "  1 Thread  -> Toplam Adım: " << res1T.totalStepMs << " ms ("
              << (res1T.totalStepMs / benchmarkSteps) << " ms/adim) | Cozucu: " 
              << res1T.totalSolverMs << " ms (" << (res1T.totalSolverMs / benchmarkSteps) << " ms/adim)\n";

    std::cout << "  8 iş parçacığı ölçülüyor...\n";
    BenchResult res8T = benchmarkPiles(8, totalBodies, benchmarkSteps);
    std::cout << "  8 Threads -> Toplam Adım: " << res8T.totalStepMs << " ms ("
              << (res8T.totalStepMs / benchmarkSteps) << " ms/adim) | Cozucu: " 
              << res8T.totalSolverMs << " ms (" << (res8T.totalSolverMs / benchmarkSteps) << " ms/adim)\n";

    double solverSpeedup = (res8T.totalSolverMs > 0.0) ? (res1T.totalSolverMs / res8T.totalSolverMs) : 1.0;
    double stepSpeedup = res1T.totalStepMs / res8T.totalStepMs;

    std::cout << "  -> 10k Cisim Ada Cozucusu Hizlanma Faktoru (Solver Speedup): " << solverSpeedup << "x\n";
    std::cout << "  -> 10k Cisim Toplam Adim Hizlanma Faktoru (Step Speedup):   " << stepSpeedup << "x\n";

    CHECK_MSG(solverSpeedup >= 2.5, "8 is parcaciginda cok adali ada cozucusu (island solver) en az 2.5x-3x hizlanma saglamalidir");
}

/**
 * @brief 3. Test: Çoklu Eklem Kırılma Eşzamanlılığı Testi.
 * @details Paralel koşan adalar içindeki eklemler aynı anda kırıldığında veri yarışının (data race)
 *          olmadığı ve tüm bildirimlerin eksiksiz alındığı doğrulanır.
 */
void testConcurrentJointBreakage() {
    std::cout << "[MultithreadedIslandTest] 3. Coklu Eklem Kirilma Eszamanlilik Testi...\n";

    Simulator sim;
    SimulationSettings settings = sim.getSettings();
    settings.threadCount = 8;
    sim.setSettings(settings);

    std::atomic<int> breakCount{0};
    sim.setJointBreakCallback([&](ecs::Entity, ecs::Entity, float) {
        breakCount.fetch_add(1, std::memory_order_relaxed);
    });

    // 16 ayrı adada gerilim altında kırılacak eklemler oluştur
    for (int i = 0; i < 16; ++i) {
        float x = static_cast<float>(i) * 10.0f;
        Body b1 = sim.createBody(Pose(Vector3(x, 0, 0)), CollisionShape{SphereShape(0.5f)});
        (void)b1.setBodyType(Core::BodyType::Static);

        Body b2 = sim.createBody(Pose(Vector3(x + 2.0f, 0, 0)), CollisionShape{SphereShape(0.5f)});
        (void)b2.setBodyType(Core::BodyType::Dynamic);
        (void)b2.setMass(1.0f);

        sim.createDistanceConstraint(b1, b2, 2.0f);
        sim.setConstraintBreakImpulse<DistanceConstraint>(b1, 1.0f); // Düşük kırılma eşiği

        // Zıt yönde şiddetli itme uygula
        (void)b2.applyLinearImpulse(Vector3(100.0f, 0, 0));
    }

    sim.step(1.0f / 60.0f);

    std::cout << "  Kırılan eklem sayısı: " << breakCount.load() << " / 16\n";
    CHECK_MSG(breakCount.load() == 16, "16 eklemin tamami eszamanli olarak guvenle kirilmalidir");
    std::cout << "  -> Coklu eklem kirilma eszamanliligi basariyla dogrulandi.\n";
}

int main() {
    std::cout << "=== BARYON FAZ 5.1 COK IS PARCACIKLI ADA COZUCUSU TESTLERI ===\n";
    testMultithreadedDeterminism();
    testScaleAndSpeedup10k();
    testConcurrentJointBreakage();
    return reportAndExit();
}
