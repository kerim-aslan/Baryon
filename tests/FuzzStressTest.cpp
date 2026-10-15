#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <random>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

struct BodyTracking {
    Body body;
    bool wasInside{true};
    bool escapedOverTop{false};
};

bool checkNaN(const Vector3& v) {
    return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z) ||
           std::isinf(v.x) || std::isinf(v.y) || std::isinf(v.z);
}

void runFuzzTestForSeed(uint32_t seed, int bodyCount = 200, int steps = 2000) {
    std::cout << "\n>>> Running Fuzz Stress Test for Seed " << seed << " (" << bodyCount << " bodies, " << steps << " steps)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Arena: Zemin (40x40) ve 4 Duvar (Yukseklik = 8.5m)
    // Zemin: ust y = 0.0
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // Duvarlar: Yarı genişlik 4.25 -> alt 0, üst 8.5
    Body wallN = sim.createBody(Pose(Vector3(0, 4.25f, 20.0f)), CollisionShape{BoxShape(Vector3(20.0f, 4.25f, 0.5f))});
    (void)wallN.setBodyType(Core::BodyType::Static);

    Body wallS = sim.createBody(Pose(Vector3(0, 4.25f, -20.0f)), CollisionShape{BoxShape(Vector3(20.0f, 4.25f, 0.5f))});
    (void)wallS.setBodyType(Core::BodyType::Static);

    Body wallE = sim.createBody(Pose(Vector3(20.0f, 4.25f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 4.25f, 20.0f))});
    (void)wallE.setBodyType(Core::BodyType::Static);

    Body wallW = sim.createBody(Pose(Vector3(-20.0f, 4.25f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 4.25f, 20.0f))});
    (void)wallW.setBodyType(Core::BodyType::Static);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> distXZ(-14.0f, 14.0f);
    std::uniform_real_distribution<float> distY(1.0f, 15.0f);
    std::uniform_real_distribution<float> distVel(-3.0f, 3.0f);
    std::uniform_real_distribution<float> distSize(0.25f, 0.5f);
    std::uniform_int_distribution<int> distShape(0, 3);

    std::vector<BodyTracking> trackedBodies;
    trackedBodies.reserve(bodyCount);

    for (int i = 0; i < bodyCount; ++i) {
        Vector3 pos(distXZ(rng), distY(rng), distXZ(rng));
        Pose pose(pos);

        int shapeType = distShape(rng);
        CollisionShape shape = CollisionShape{SphereShape(0.4f)};
        if (shapeType == 0) {
            shape = CollisionShape{SphereShape(distSize(rng))};
        } else if (shapeType == 1) {
            float s = distSize(rng);
            shape = CollisionShape{BoxShape(Vector3(s, s, s))};
        } else if (shapeType == 2) {
            shape = CollisionShape{CapsuleShape(distSize(rng) * 0.6f, distSize(rng) * 1.5f)};
        } else {
            // Küçük konveks çokyüzlü
            std::vector<Vector3> verts;
            float r = distSize(rng);
            for (int v = 0; v < 8; ++v) {
                verts.push_back(Vector3(
                    (v & 1 ? r : -r) * 0.8f,
                    (v & 2 ? r : -r) * 0.8f,
                    (v & 4 ? r : -r) * 0.8f
                ));
            }
            shape = CollisionShape{ConvexHullShape(verts)};
        }

        Body b = sim.createBody(pose, shape);
        (void)b.setBodyType(Core::BodyType::Dynamic);
        (void)b.setMass(1.0f + distSize(rng) * 2.0f);
        (void)b.setLinearVelocity(Vector3(distVel(rng), distVel(rng), distVel(rng)));
        (void)b.setAngularVelocity(Vector3(distVel(rng), distVel(rng), distVel(rng)));

        trackedBodies.push_back({b, true, false});
    }

    int nanCount = 0;
    int groundTunnelCount = 0;
    int wallTunnelCount = 0;

    for (int step = 0; step < steps; ++step) {
        sim.step(1.0f / 60.0f);

        for (auto& tb : trackedBodies) {
            Vector3 pos = tb.body.getPosition().value();
            Vector3 vel = tb.body.getLinearVelocity().value();
            Vector3 w = tb.body.getAngularVelocity().value();

            if (checkNaN(pos) || checkNaN(vel) || checkNaN(w)) {
                nanCount++;
            }

            bool inArenaBounds = (std::abs(pos.x) <= 19.5f && std::abs(pos.z) <= 19.5f);

            // Kriter 2: Zeminden geçen (y < -0.6 ve arena içinde) = 0
            if (inArenaBounds && pos.y < -0.6f) {
                groundTunnelCount++;
            }

            // Kriter 3: Duvardan geçen (dışarı ilk çıkışta y < 8.5) = 0
            if (tb.wasInside && !inArenaBounds) {
                if (pos.y < 8.5f) {
                    wallTunnelCount++;
                } else {
                    tb.escapedOverTop = true;
                }
                tb.wasInside = false;
            }
        }

        if (nanCount > 0 || groundTunnelCount > 0 || wallTunnelCount > 0) {
            std::cout << "[ABORT] Seed " << seed << " failed at step " << step 
                      << " | NaNs: " << nanCount 
                      << " | Ground Tunnels: " << groundTunnelCount 
                      << " | Wall Tunnels: " << wallTunnelCount << std::endl;
            break;
        }
    }

    std::cout << "Seed " << seed << " Result -> NaNs: " << nanCount 
              << " | Ground Tunnels: " << groundTunnelCount 
              << " | Wall Tunnels: " << wallTunnelCount << std::endl;

    CHECK_MSG(nanCount == 0, ("Seed " + std::to_string(seed) + " NaN sayisi 0 olmali").c_str());
    CHECK_MSG(groundTunnelCount == 0, ("Seed " + std::to_string(seed) + " zeminden gecen cisim olmamali").c_str());
    CHECK_MSG(wallTunnelCount == 0, ("Seed " + std::to_string(seed) + " duvardan gecen cisim olmamali").c_str());
}

int main() {
    std::cout << "=== Baryon V5 Random Fuzz Stress Test (5 Seeds) ===" << std::endl;

    const std::vector<uint32_t> seeds = {42, 101, 777, 1337, 2026};
    for (uint32_t s : seeds) {
        runFuzzTestForSeed(s, 200, 2000);
    }

    return reportAndExit();
}
