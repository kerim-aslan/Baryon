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

struct DropMetrics {
    float minY;
    float apexRatio;
};

static DropMetrics runDrop(float H, float e, bool useCCD) {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Ground: top surface at y = 0
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)),
                                 CollisionShape{BoxShape(Vector3(100.0f, 0.5f, 100.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);
    (void)ground.setMaterial(0.0f, e);

    // Falling sphere: R = 0.5. At contact with ground, center y should be 0.5.
    Body ball = sim.createBody(Pose(Vector3(0, H + 0.5f, 0)),
                               CollisionShape{SphereShape(0.5f)});
    (void)ball.setBodyType(Core::BodyType::Dynamic);
    (void)ball.setMass(1.0f);
    (void)ball.setMaterial(0.0f, e);
    (void)ball.setCCD(useCCD);

    float minY = H + 0.5f;
    float maxPostBounceY = 0.0f;
    bool bounced = false;

    int maxSteps = static_cast<int>(std::ceil(std::sqrt(2.0f * H / 9.81f) * 2.5f * 60.0f)) + 60;
    if (maxSteps < 120) maxSteps = 120;

    for (int step = 0; step < maxSteps; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = ball.getPosition().value();
        Vector3 vel = ball.getLinearVelocity().value();

        if (pos.y < minY) {
            minY = pos.y;
        }

        if (!bounced) {
            if (vel.y > 0.05f) {
                bounced = true;
            }
        } else {
            if (pos.y > maxPostBounceY) {
                maxPostBounceY = pos.y;
            }
            if (maxPostBounceY > 0.52f && vel.y < -0.1f) {
                break;
            }
        }
    }

    float bounceHeight = maxPostBounceY - 0.5f;
    float apexRatio = (H > 0.0f) ? (bounceHeight / H) : 0.0f;

    return DropMetrics{minY, apexRatio};
}

int main() {
    std::cout << "=== G3: CCD Bounce In Air Test (Restitution Drop Minimum Y) ===" << std::endl;

    const std::vector<float> heights = {1.0f, 3.0f, 8.0f, 15.0f};

    // Test e = 1.0
    std::cout << "\n--- Test e = 1.0 Drops ---" << std::endl;
    for (float H : heights) {
        DropMetrics off = runDrop(H, 1.0f, false);
        DropMetrics on = runDrop(H, 1.0f, true);

        float yDiff = on.minY - off.minY;
        std::cout << "  H=" << H << "m | e=1.0: "
                  << "CCD=OFF minY=" << off.minY << ", apex=" << off.apexRatio
                  << " | CCD=ON minY=" << on.minY << ", apex=" << on.apexRatio
                  << " | diff(ON-OFF)=" << yDiff << " m"
                  << std::endl;

        // Peak ratio check [0.85, 1.05]
        CHECK_MSG(on.apexRatio >= 0.85f && on.apexRatio <= 1.05f, "CCD ON apex ratio must be in [0.85, 1.05]");

        // Mid-air bounce check: CCD ON minY must be within 0.02m of CCD OFF minY
        CHECK_MSG(std::abs(yDiff) <= 0.02f,
                  ("H=" + std::to_string(H) + "m e=1.0: CCD ON minY (" + std::to_string(on.minY) + 
                   ") must be within 0.02m of CCD OFF (" + std::to_string(off.minY) + ")").c_str());
    }

    // Test e = 0.0
    std::cout << "\n--- Test e = 0.0 Drops ---" << std::endl;
    for (float H : heights) {
        DropMetrics off = runDrop(H, 0.0f, false);
        DropMetrics on = runDrop(H, 0.0f, true);

        float yDiff = on.minY - off.minY;
        std::cout << "  H=" << H << "m | e=0.0: "
                  << "CCD=OFF minY=" << off.minY
                  << " | CCD=ON minY=" << on.minY
                  << " | diff(ON-OFF)=" << yDiff << " m"
                  << std::endl;

        CHECK_MSG(std::abs(yDiff) <= 0.02f,
                  ("H=" + std::to_string(H) + "m e=0.0: CCD ON minY (" + std::to_string(on.minY) + 
                   ") must be within 0.02m of CCD OFF (" + std::to_string(off.minY) + ")").c_str());
    }

    return reportAndExit();
}
