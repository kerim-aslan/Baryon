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
 * @file BenchmarkTest.cpp
 * @brief Baryon Faz 5.3 Benchmark Senaryolari Kosucusu ve Performans Dogrulamasi.
 * @details 1k, 10k, 50k cisim, kule stabilitesi ve 200 halkali zincir senaryolarini
 *          simule eder; ms/adim, kare basina temas sayisi, kinetik/potansiyel enerjiyi
 *          out.csv dosyasina kaydeder.
 */

#include "TestCommon.hpp"
#include "harness/Metrics.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <Baryon/collision/DistanceConstraint.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>

using namespace Baryon;
using namespace Baryon::collision;

struct BenchmarkResult {
    std::string scenario;
    int step;
    double stepTimeMs;
    size_t activeContacts;
    size_t bodyCount;
    float kineticEnergy;
    float potentialEnergy;
};

static std::vector<BenchmarkResult> gResults;

static void writeCsv(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open CSV output: " << filename << std::endl;
        return;
    }

    ofs << "scenario,step,step_time_ms,active_contacts,body_count,kinetic_energy,potential_energy\n";
    for (const auto& r : gResults) {
        ofs << r.scenario << ","
            << r.step << ","
            << std::fixed << std::setprecision(4) << r.stepTimeMs << ","
            << r.activeContacts << ","
            << r.bodyCount << ","
            << std::setprecision(3) << r.kineticEnergy << ","
            << r.potentialEnergy << "\n";
    }
    std::cout << "[Benchmark] CSV kaydi tamamlandi: " << filename
              << " (" << gResults.size() << " kayit)" << std::endl;
}

static void runScenario(
    const std::string& scenarioName,
    Simulator& sim,
    int totalSteps,
    size_t bodyCount,
    float dt = 1.0f / 60.0f)
{
    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "[Benchmark] Baslatiliyor: " << scenarioName
              << " | Cisim: " << bodyCount << " | Adim: " << totalSteps << std::endl;

    double totalTimeMs = 0.0;
    size_t peakContacts = 0;

    for (int step = 0; step < totalSteps; ++step) {
        auto tStart = std::chrono::high_resolution_clock::now();
        sim.step(dt);
        auto tEnd = std::chrono::high_resolution_clock::now();

        double elapsedMs = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
        totalTimeMs += elapsedMs;

        size_t contacts = sim.getActiveContactCount();
        if (contacts > peakContacts) peakContacts = contacts;

        float ke = harness::computeKineticEnergy(sim);
        float pe = harness::computePotentialEnergy(sim);

        gResults.push_back({
            scenarioName,
            step + 1,
            elapsedMs,
            contacts,
            bodyCount,
            ke,
            pe
        });

        // Sayısal stabilite doğrulaması
        CHECK_MSG(!std::isnan(ke) && !std::isinf(ke), "Kinetik enerji NaN veya Inf olamaz!");
        CHECK_MSG(!std::isnan(pe) && !std::isinf(pe), "Potansiyel enerji NaN veya Inf olamaz!");
    }

    double avgMs = totalTimeMs / static_cast<double>(totalSteps);
    std::cout << "[Benchmark] Tamamlandi: " << scenarioName
              << " | Ortalama: " << std::fixed << std::setprecision(2) << avgMs << " ms/adim"
              << " | Toplam: " << std::setprecision(1) << totalTimeMs << " ms"
              << " | Tepe Kontak: " << peakContacts << std::endl;
}

// 1. 1,000 Cisim Serbest Dusme ve Carpisma
static void benchmark1kBodies() {
    Simulator sim;
    SimulationSettings settings;
    settings.threadCount = 4;
    sim.setSettings(settings);

    // Statik zemin
    Body ground = sim.createBody(Pose(Vector3(0.0f, -0.5f, 0.0f)), CollisionShape{BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 10x10x10 = 1,000 dinamik kutu
    size_t count = 0;
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            for (int z = 0; z < 10; ++z) {
                Vector3 pos(
                    (static_cast<float>(x) - 4.5f) * 1.2f,
                    1.0f + static_cast<float>(y) * 1.2f,
                    (static_cast<float>(z) - 4.5f) * 1.2f
                );
                sim.createBody(Pose(pos), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
                count++;
            }
        }
    }

    runScenario("1k_bodies", sim, 50, count);
}

// 2. 10,000 Cisim Grid Yigini
static void benchmark10kBodies() {
    Simulator sim;
    SimulationSettings settings;
    settings.threadCount = 8;
    sim.setSettings(settings);
    sim.getRegistry().setMaxCapacity(15000);

    // Statik zemin
    Body ground = sim.createBody(Pose(Vector3(0.0f, -0.5f, 0.0f)), CollisionShape{BoxShape(Vector3(100.0f, 0.5f, 100.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 25x16x25 = 10,000 dinamik kutu
    size_t count = 0;
    for (int x = 0; x < 25; ++x) {
        for (int y = 0; y < 16; ++y) {
            for (int z = 0; z < 25; ++z) {
                Vector3 pos(
                    (static_cast<float>(x) - 12.0f) * 1.5f,
                    1.0f + static_cast<float>(y) * 1.5f,
                    (static_cast<float>(z) - 12.0f) * 1.5f
                );
                sim.createBody(Pose(pos), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
                count++;
            }
        }
    }

    runScenario("10k_bodies", sim, 20, count);
}

// 3. 50,000 Cisim Genis Alan Stres Testi
static void benchmark50kBodies() {
    Simulator sim;
    SimulationSettings settings;
    settings.threadCount = 8;
    sim.setSettings(settings);
    sim.getRegistry().setMaxCapacity(60000);

    // Statik zemin
    Body ground = sim.createBody(Pose(Vector3(0.0f, -1.0f, 0.0f)), CollisionShape{BoxShape(Vector3(300.0f, 1.0f, 300.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 50x20x50 = 50,000 dinamik kutu
    size_t count = 0;
    for (int x = 0; x < 50; ++x) {
        for (int y = 0; y < 20; ++y) {
            for (int z = 0; z < 50; ++z) {
                Vector3 pos(
                    (static_cast<float>(x) - 24.5f) * 2.0f,
                    1.5f + static_cast<float>(y) * 2.0f,
                    (static_cast<float>(z) - 24.5f) * 2.0f
                );
                sim.createBody(Pose(pos), CollisionShape{BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
                count++;
            }
        }
    }

    runScenario("50k_bodies", sim, 10, count);
}

// 4. Kule Stabilitesi (Tower Stability - 20 Katli Kutu Kulesi)
static void benchmarkTowerStability() {
    Simulator sim;
    SimulationSettings settings;
    settings.threadCount = 4;
    sim.setSettings(settings);

    // Statik zemin
    Body ground = sim.createBody(Pose(Vector3(0.0f, -0.5f, 0.0f)), CollisionShape{BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 20 katlı kule
    size_t count = 0;
    const float boxH = 0.5f;
    for (int level = 0; level < 20; ++level) {
        float y = 0.5f + static_cast<float>(level) * (boxH * 2.0f);
        // Kat başına 2 kutu
        sim.createBody(Pose(Vector3(-0.3f, y, 0.0f)), CollisionShape{BoxShape(Vector3(0.25f, boxH, 0.5f))});
        sim.createBody(Pose(Vector3(0.3f, y, 0.0f)), CollisionShape{BoxShape(Vector3(0.25f, boxH, 0.5f))});
        count += 2;
    }

    runScenario("tower_stability", sim, 60, count);
}

// 5. 200 Halkali Zincir (200-Link Chain)
static void benchmarkChain200() {
    Simulator sim;
    SimulationSettings settings;
    settings.threadCount = 4;
    sim.setSettings(settings);
    sim.getRegistry().setMaxCapacity(1000);

    const int linkCount = 200;
    const float linkDist = 0.5f;

    // Tepe ankor (statik)
    Body prevBody = sim.createBody(Pose(Vector3(0.0f, 105.0f, 0.0f)), CollisionShape{SphereShape(0.2f)});
    (void)prevBody.setBodyType(Core::BodyType::Static);

    for (int i = 0; i < linkCount; ++i) {
        float y = 105.0f - static_cast<float>(i + 1) * linkDist;
        Body link = sim.createBody(Pose(Vector3(0.0f, y, 0.0f)), CollisionShape{SphereShape(0.15f)});
        (void)link.setMass(0.5f);

        sim.createDistanceConstraint(prevBody, link, linkDist);
        prevBody = link;
    }

    runScenario("chain_200", sim, 60, static_cast<size_t>(linkCount + 1));
}

int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "=== Baryon Faz 5.3: Benchmark Senaryolari (1k, 10k, 50k, Kule, Zincir) ===" << std::endl;
    std::cout << "==========================================================" << std::endl;

    benchmark1kBodies();
    benchmarkTowerStability();
    benchmarkChain200();
    benchmark10kBodies();
    benchmark50kBodies();

    // CSV Çıktısı oluştur
    writeCsv("benchmark_results.csv");
    writeCsv("out.csv");

    std::cout << "\n==========================================================" << std::endl;
    std::cout << "=== [BenchmarkTest] TUM SENARYOLAR BASARIYLA TAMAMLANDI ===" << std::endl;
    std::cout << "==========================================================" << std::endl;

    return reportAndExit();
}
