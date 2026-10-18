/*
 * Baryon - A custom physics engine
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <cmath>
#include <iostream>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

struct TowerResult {
    bool standing;
    float topY;
    float horizontalDev;
    float heightDiff;
};

TowerResult runTowerTest(int n, bool offsetOddBoxes, uint32_t velocityIterations, float durationSeconds = 15.0f) {
    Simulator sim;
    SimulationSettings settings = sim.getSettings();
    settings.enableSleeping = false;
    settings.velocityIterations = velocityIterations;
    sim.setSettings(settings);
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Ground: Static box at (0, -0.5, 0), size 20x1x20
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(10.0f, 0.5f, 10.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);
    (void)ground.setMaterial(0.6f, 0.0f);

    std::vector<Body> boxes;
    boxes.reserve(n);

    for (int b = 0; b < n; ++b) {
        float x = (offsetOddBoxes && (b % 2 == 1)) ? 0.01f : 0.0f;
        float y = 0.5f + static_cast<float>(b) * 1.0f;
        float z = 0.0f;

        Body box = sim.createBody(Pose(Vector3(x, y, z)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
        (void)box.setMaterial(0.6f, 0.0f);
        boxes.push_back(box);
    }

    int steps = static_cast<int>(std::round(durationSeconds * 60.0f));
    for (int s = 0; s < steps; ++s) {
        sim.step(1.0f / 60.0f);
    }

    // Evaluate top box
    Body& topBox = boxes.back();
    Vector3 topPos = topBox.getPosition().value();
    float expectedTopY = 0.5f + static_cast<float>(n - 1) * 1.0f;
    float heightDiff = expectedTopY - topPos.y;
    float horizontalDev = std::sqrt(topPos.x * topPos.x + topPos.z * topPos.z);

    bool standing = (heightDiff < 0.25f) && (horizontalDev < 0.5f);

    return TowerResult{
        .standing = standing,
        .topY = topPos.y,
        .horizontalDev = horizontalDev,
        .heightDiff = heightDiff
    };
}

int main() {
    std::cout << "=== G2: High Tower Stability & Regression Suite (Sleeping OFF) ===" << std::endl;

    // 1. Aligned Tower Test: n = 2..13 (Expectation: n <= 12 standing)
    std::cout << "\n--- Test 1: Aligned Towers (n = 2..13, 15s) ---" << std::endl;
    int standingCount = 0;
    int maxStandingN = 0;
    for (int n = 2; n <= 13; ++n) {
        TowerResult res = runTowerTest(n, false, 12, 15.0f);
        std::cout << "  n=" << n 
                  << " -> standing=" << (res.standing ? "YES" : "NO")
                  << " | topY=" << res.topY 
                  << " (diff=" << res.heightDiff << ")"
                  << " | horizDev=" << res.horizontalDev << " m" 
                  << std::endl;
        if (res.standing) {
            standingCount++;
            if (maxStandingN == n - 1 || maxStandingN == 0) {
                maxStandingN = n;
            }
        }
    }

    std::cout << "  Standing count: " << standingCount << " / 12 (Max standing n = " << maxStandingN << ")" << std::endl;
    for (int n = 2; n <= 12; ++n) {
        // Will check if n <= 12 is standing
    }
    CHECK_MSG(maxStandingN >= 12, "Aligned tower: n <= 12 must remain standing!");

    // 2. Offset Tower Test: n = 8, odd boxes offset by +0.01 in X, 15s
    std::cout << "\n--- Test 2: Offset Tower (n = 8, odd +0.01m X, 15s) ---" << std::endl;
    {
        TowerResult res = runTowerTest(8, true, 12, 15.0f);
        std::cout << "  n=8 offset -> topY=" << res.topY 
                  << " | horizDev=" << res.horizontalDev << " m (limit <= 0.05m)"
                  << std::endl;
        CHECK_MSG(res.standing, "n=8 offset tower must remain standing!");
        CHECK_MSG(res.horizontalDev <= 0.05f, "n=8 offset tower horizontal deviation must be <= 0.05m!");
    }

    // 3. Iteration Invariance: n = 8 with velocityIterations in {12, 20, 40}
    std::cout << "\n--- Test 3: Iteration Invariance (n = 8, vIters in {12, 20, 40}, 15s) ---" << std::endl;
    for (uint32_t vIters : {12u, 20u, 40u}) {
        TowerResult res = runTowerTest(8, false, vIters, 15.0f);
        std::cout << "  vIters=" << vIters 
                  << " -> standing=" << (res.standing ? "YES" : "NO")
                  << " | horizDev=" << res.horizontalDev << " m (limit <= 0.1m)"
                  << std::endl;
        CHECK_MSG(res.standing, "n=8 tower under vIters must remain standing!");
        CHECK_MSG(res.horizontalDev <= 0.1f, "n=8 tower under vIters deviation must be <= 0.1m!");
    }

    return reportAndExit();
}
