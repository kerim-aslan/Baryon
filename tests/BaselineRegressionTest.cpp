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
#include "harness/BaselineScenarios.hpp"
#include "harness/HarnessRunner.hpp"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>

using namespace Baryon::harness;

int main() {
    std::cout << "=== BARYON BASELINE REGRESSION TEST (PHASE 0) ===\n" << std::endl;

    // 0. baseline_v1.json dosyasının mevcudiyeti kontrolü
#ifdef BASELINE_FILE_PATH
    CHECK_MSG(std::filesystem::exists(BASELINE_FILE_PATH), "baseline_v1.json dosyasi mevcut olmali");
#endif

    // 1. Senaryo: slide_friction (Kutu, µ=0.5, v0=5 m/s zeminde kayar)
    {
        std::cout << "--- 1. Baseline: slide_friction ---" << std::endl;
        SlideFrictionScenario scen;
        HarnessOptions opt;
        opt.scenarioName = scen.getName();
        opt.verbose = false;

        HarnessResult res = HarnessRunner::run(scen, opt);
        double measuredX = res.customMetrics["final_x"];
        double expectedX = 2.52814;
        double errorPct = std::abs(measuredX - expectedX) / expectedX * 100.0;

        std::cout << "  Measured final_x: " << measuredX << " m | Baseline: " << expectedX 
                  << " m | Diff: " << errorPct << "%" << std::endl;

        CHECK_MSG(res.passed, res.message);
        CHECK_MSG(errorPct <= 2.0, "slide_friction final_x degeri baseline ile +-2% icinde eslesmeli");
    }

    // 2. Senaryo: sphere_restitution (Küre, e=0.8, 5 m'den düşer, 2. tepe ≈ 3.10 m)
    {
        std::cout << "\n--- 2. Baseline: sphere_restitution ---" << std::endl;
        SphereRestitutionScenario scen;
        HarnessOptions opt;
        opt.scenarioName = scen.getName();
        opt.verbose = false;

        HarnessResult res = HarnessRunner::run(scen, opt);
        double measuredPeak1 = res.customMetrics["peak_1"];
        double expectedPeak1 = 3.11028;
        double errorPct = std::abs(measuredPeak1 - expectedPeak1) / expectedPeak1 * 100.0;

        std::cout << "  Measured peak_1: " << measuredPeak1 << " m | Baseline: " << expectedPeak1 
                  << " m | Diff: " << errorPct << "%" << std::endl;

        CHECK_MSG(res.passed, res.message);
        CHECK_MSG(errorPct <= 2.0, "sphere_restitution peak_1 degeri baseline ile +-2% icinde eslesmeli");
    }

    // 3. Senaryo: distance_joint (Mesafe eklemi, uzunluk 2.0m, drift)
    {
        std::cout << "\n--- 3. Baseline: distance_joint ---" << std::endl;
        DistanceJointScenario scen;
        HarnessOptions opt;
        opt.scenarioName = scen.getName();
        opt.verbose = false;

        HarnessResult res = HarnessRunner::run(scen, opt);
        double maxDist = res.customMetrics["max_distance"];
        double minDist = res.customMetrics["min_distance"];
        double drift = maxDist - 2.0;

        std::cout << "  Min dist: " << minDist << " m | Max dist: " << maxDist 
                  << " m | Drift: " << drift << " m" << std::endl;

        CHECK_MSG(res.passed, res.message);
        CHECK_MSG(drift <= 0.013, "distance_joint drift degeri <= 0.013 m olmali");
    }

    // 4. Senaryo: heavy_on_light_stack (100:1 kütle oranlı 2 kutu yığını)
    {
        std::cout << "\n--- 4. Baseline: heavy_on_light_stack ---" << std::endl;
        HeavyOnLightStackScenario scen;
        HarnessOptions opt;
        opt.scenarioName = scen.getName();
        opt.verbose = false;

        HarnessResult res = HarnessRunner::run(scen, opt);
        double bottomY = res.customMetrics["bottom_y"];
        double topY = res.customMetrics["top_y"];
        double expectedBottom = 0.48229;
        double expectedTop = 1.47138;

        double errBottom = std::abs(bottomY - expectedBottom) / expectedBottom * 100.0;
        double errTop = std::abs(topY - expectedTop) / expectedTop * 100.0;

        std::cout << "  Alt kutu y: " << bottomY << " (Baseline: " << expectedBottom << ", Diff: " << errBottom << "%)" << std::endl;
        std::cout << "  Ust kutu y: " << topY << " (Baseline: " << expectedTop << ", Diff: " << errTop << "%)" << std::endl;

        CHECK_MSG(res.passed, res.message);
        CHECK_MSG(errBottom <= 2.0, "heavy_on_light_stack alt kutu y +-2% icinde olmali");
        CHECK_MSG(errTop <= 2.0, "heavy_on_light_stack ust kutu y +-2% icinde olmali");
    }

    // 5. Senaryo: ccd_bullet (CCD, 500 m/s küre, 0.1 m kalınlıklı duvar)
    {
        std::cout << "\n--- 5. Baseline: ccd_bullet ---" << std::endl;
        CcdBulletScenario scen;
        HarnessOptions opt;
        opt.scenarioName = scen.getName();
        opt.verbose = false;

        HarnessResult res = HarnessRunner::run(scen, opt);
        double measuredX = res.customMetrics["final_x"];
        double expectedX = 9.86118;
        double errorPct = std::abs(measuredX - expectedX) / expectedX * 100.0;

        std::cout << "  Measured bullet X: " << measuredX << " m | Baseline: " << expectedX 
                  << " m | Diff: " << errorPct << "%" << std::endl;

        CHECK_MSG(res.passed, res.message);
        CHECK_MSG(measuredX <= 9.95, "CCD bullet duvarin icinden gecmemeli (x <= 9.95)");
        CHECK_MSG(errorPct <= 2.0, "ccd_bullet x degeri baseline ile +-2% icinde eslesmeli");
    }

    // 6. Senaryo: determinism_40 (Determinizm: 40 cisim, 600 adım, 2 koşu)
    {
        std::cout << "\n--- 6. Baseline: determinism_40 ---" << std::endl;
        Determinism40Scenario scen1;
        Determinism40Scenario scen2;

        HarnessOptions opt;
        opt.scenarioName = scen1.getName();
        opt.verbose = false;

        std::cout << "  Running Run 1 (600 steps)..." << std::endl;
        HarnessResult res1 = HarnessRunner::run(scen1, opt);
        uint64_t hash1 = scen1.getFinalHash();

        std::cout << "  Running Run 2 (600 steps, independent instance)..." << std::endl;
        HarnessResult res2 = HarnessRunner::run(scen2, opt);
        uint64_t hash2 = scen2.getFinalHash();

        std::cout << "  Run 1 State Hash: " << hashToHexString(hash1) << std::endl;
        std::cout << "  Run 2 State Hash: " << hashToHexString(hash2) << std::endl;

        CHECK_MSG(hash1 != 0, "State hash sifir olmamali");
        CHECK_MSG(hash1 == hash2, "Determinizm: Run 1 ve Run 2 durum hashleri birebir ayni olmali");
    }

    return reportAndExit();
}
