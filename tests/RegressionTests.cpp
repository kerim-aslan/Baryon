#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

// Bağımsız N ada için simülasyon adım süresini ölçer (ms cinsinden)
double measureIslandsTimeMs(int N, int stepCount = 5) {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Her ada için birbirinden izole static zemin ve dynamic kutu oluştur
    for (int i = 0; i < N; ++i) {
        float x = static_cast<float>(i % 100) * 10.0f;
        float z = static_cast<float>(i / 100) * 10.0f;

        Body ground = sim.createBody(Pose(Vector3(x, 0.0f, z)), CollisionShape{BoxShape(Vector3(1.0f, 0.2f, 1.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        Body box = sim.createBody(Pose(Vector3(x, 0.6f, z)), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
    }

    // Isınma adımı (temasların oluşması ve manifoldların kurulması)
    sim.step(1.0f / 60.0f);

    auto start = std::chrono::high_resolution_clock::now();
    for (int s = 0; s < stepCount; ++s) {
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    return std::chrono::duration<double, std::milli>(end - start).count();
}

double getMedianRunTime(int N, int runs = 3, int stepCount = 5) {
    std::vector<double> times;
    times.reserve(runs);
    for (int r = 0; r < runs; ++r) {
        double t = measureIslandsTimeMs(N, stepCount);
        times.push_back(t);
        std::cout << "  [N=" << N << "] Run " << (r + 1) << ": " << t << " ms" << std::endl;
    }
    std::sort(times.begin(), times.end());
    return times[runs / 2];
}

// ==================== N6 BENCHMARK WORKLOADS ====================

// (a) Workload A: Hepsi Uyanık
void benchmarkAllAwake(int N = 1000, int steps = 10, bool keepAwakeActive = false) {
    std::cout << "\n--- N6.2 (a) Workload: All Awake (N=" << N << ") ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    std::vector<Body> boxes;
    boxes.reserve(N);

    for (int i = 0; i < N; ++i) {
        float x = static_cast<float>(i % 100) * 10.0f;
        float z = static_cast<float>(i / 100) * 10.0f;

        Body ground = sim.createBody(Pose(Vector3(x, 0.0f, z)), CollisionShape{BoxShape(Vector3(1.0f, 0.2f, 1.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        Body box = sim.createBody(Pose(Vector3(x, 0.5f, z)), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
        boxes.push_back(box);
    }

    // 120 adım ilerle: keepAwakeActive false ise hepsi uyur
    for (int s = 0; s < 120; ++s) {
        if (keepAwakeActive) {
            for (auto& b : boxes) {
                b.setLinearVelocity(Vector3(0.01f * (s % 2 == 0 ? 1.0f : -1.0f), 0.0f, 0.0f));
            }
        }
        sim.step(1.0f / 60.0f);
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int s = 0; s < steps; ++s) {
        if (keepAwakeActive) {
            for (auto& b : boxes) {
                b.setLinearVelocity(Vector3(0.01f * (s % 2 == 0 ? 1.0f : -1.0f), 0.0f, 0.0f));
            }
        }
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    int awakeCount = 0;
    for (auto& b : boxes) {
        if (!b.isSleeping().value()) awakeCount++;
    }

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double msPerStep = totalMs / steps;

    std::cout << "  Awake: " << awakeCount << " / " << N 
              << " | Total: " << totalMs << " ms (" << msPerStep << " ms/adim)" << std::endl;

    CHECK_MSG(awakeCount == N, "Benchmark'ta tum cisimler uyanik olmali (awake == N)");
}

// (b) Workload B: Hepsi Uyuyor
void benchmarkAllAsleep(int N = 1000, int steps = 10) {
    std::cout << "\n--- N6.2 (b) Workload: All Asleep (N=" << N << ") ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    std::vector<Body> boxes;
    boxes.reserve(N);

    for (int i = 0; i < N; ++i) {
        float x = static_cast<float>(i % 100) * 10.0f;
        float z = static_cast<float>(i / 100) * 10.0f;

        Body ground = sim.createBody(Pose(Vector3(x, 0.0f, z)), CollisionShape{BoxShape(Vector3(1.0f, 0.2f, 1.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        Body box = sim.createBody(Pose(Vector3(x, 0.5f, z)), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
        boxes.push_back(box);
    }

    // Cisimlerin dinlenip uykuya geçmesi için 120 adım simüle et
    for (int s = 0; s < 120; ++s) {
        sim.step(1.0f / 60.0f);
    }

    int awakeCountBefore = 0;
    for (auto& b : boxes) {
        if (!b.isSleeping().value()) awakeCountBefore++;
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int s = 0; s < steps; ++s) {
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    int awakeCountAfter = 0;
    for (auto& b : boxes) {
        if (!b.isSleeping().value()) awakeCountAfter++;
    }

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double msPerStep = totalMs / steps;

    std::cout << "  Awake: " << awakeCountAfter << " / " << N 
              << " | Total: " << totalMs << " ms (" << msPerStep << " ms/adim)" << std::endl;

    CHECK_MSG(awakeCountAfter == 0, "Benchmark'ta tum cisimler uyuyor olmali (awake == 0)");
}

// (c) Workload C: 10'lu Kutu Kuleleri
void benchmarkTowerStacking(int towerCount = 50, int boxesPerTower = 10, int steps = 20) {
    int totalBoxes = towerCount * boxesPerTower;
    std::cout << "\n--- N6.2 (c) Workload: 10'lu Kutu Kuleleri (" << towerCount 
              << " kule x " << boxesPerTower << " kutu = " << totalBoxes << " kutu) ---" << std::endl;

    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    std::vector<Body> dynamicBoxes;
    dynamicBoxes.reserve(totalBoxes);

    for (int t = 0; t < towerCount; ++t) {
        float tx = static_cast<float>(t % 10) * 5.0f;
        float tz = static_cast<float>(t / 10) * 5.0f;

        Body ground = sim.createBody(Pose(Vector3(tx, -0.5f, tz)), CollisionShape{BoxShape(Vector3(2.0f, 0.5f, 2.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        for (int b = 0; b < boxesPerTower; ++b) {
            float y = 0.5f + static_cast<float>(b) * 1.0f;
            Body box = sim.createBody(Pose(Vector3(tx, y, tz)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
            (void)box.setBodyType(Core::BodyType::Dynamic);
            dynamicBoxes.push_back(box);
        }
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int s = 0; s < steps; ++s) {
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();

    int awakeCount = 0;
    bool hasNaN = false;
    bool towersStanding = true;

    for (size_t i = 0; i < dynamicBoxes.size(); ++i) {
        auto& b = dynamicBoxes[i];
        if (!b.isSleeping().value()) awakeCount++;
        Vector3 pos = b.getPosition().value();
        if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) {
            hasNaN = true;
        }
        int towerIdx = static_cast<int>(i / boxesPerTower);
        float expectedX = static_cast<float>(towerIdx % 10) * 5.0f;
        float expectedZ = static_cast<float>(towerIdx / 10) * 5.0f;
        // Kule patlayıp uçmamış olmalı (X ve Z en fazla 1.5m kaymış olabilir)
        if (std::abs(pos.x - expectedX) > 2.0f || std::abs(pos.z - expectedZ) > 2.0f || pos.y < -1.0f) {
            towersStanding = false;
        }
    }

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double msPerStep = totalMs / steps;

    std::cout << "  Awake: " << awakeCount << " / " << totalBoxes 
              << " | Total: " << totalMs << " ms (" << msPerStep << " ms/adim)" << std::endl;

    CHECK_MSG(!hasNaN, "Kule kutularinda NaN olmamali");
    CHECK_MSG(towersStanding, "Kuleler stabil kalmali ve patlamamali");
}

int main() {
    std::cout << "=== Baryon N5/N6 Regression & Scaling Ratio Benchmark ===" << std::endl;

    std::cout << "Measuring N = 1000 islands (3 runs)..." << std::endl;
    double median1000 = getMedianRunTime(1000, 3, 5);
    std::cout << "--> Median N=1000: " << median1000 << " ms" << std::endl;

    std::cout << "Measuring N = 4000 islands (3 runs)..." << std::endl;
    double median4000 = getMedianRunTime(4000, 3, 5);
    std::cout << "--> Median N=4000: " << median4000 << " ms" << std::endl;

    double ratio = median4000 / median1000;
    std::cout << "\nScaling Ratio T(4000) / T(1000) = " << ratio << " (Max allowed: 5.5, Linear: 4.0)\n" << std::endl;

    CHECK_MSG(ratio <= 5.5, "Olceklenme orani <= 5.5 olmali (olculen: " + std::to_string(ratio) + ")");

    // N6 Benchmarks: keepAwakeActive true ile uyanik tutuyoruz (GREEN)
    benchmarkAllAwake(1000, 10, true);
    benchmarkAllAsleep(1000, 10);
    benchmarkTowerStacking(50, 10, 20);

    return reportAndExit();
}
