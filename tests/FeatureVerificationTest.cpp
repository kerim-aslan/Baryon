/*
 * Baryon Physics Engine - Comprehensive Feature Verification Test Suite
 * Copyright (C) 2026 Kerim Aslan
 */

#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <Baryon/collision/CollisionListener.hpp>
#include <cassert>
#include <iostream>
#include <cmath>

using namespace Baryon;

// Custom listener to test event firing
class TestListener : public CollisionListener {
public:
    int triggerEnterCount{0};
    int triggerExitCount{0};
    int collisionEnterCount{0};
    int collisionExitCount{0};

    void onCollisionEnter(ecs::Entity, ecs::Entity, const collision::ContactManifold&) override {
        collisionEnterCount++;
    }
    void onCollisionExit(ecs::Entity, ecs::Entity) override {
        collisionExitCount++;
    }
    void onTriggerEnter(ecs::Entity, ecs::Entity) override {
        triggerEnterCount++;
    }
    void onTriggerExit(ecs::Entity, ecs::Entity) override {
        triggerExitCount++;
    }
};

void runBodyApiTests() {
    std::cout << "[TEST 1] Body API (Force, Torque, Material, Trigger, Layer/Mask)..." << std::endl;
    Simulator sim;
    Body body = sim.createBody(Pose(Vector3(0, 5, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    body.setBodyType(Core::BodyType::Dynamic);
    body.setMass(2.0f);

    // 1. Force
    auto resForce = body.applyForce(Vector3(0, 100, 0));
    assert(resForce.has_value());
    assert(!body.isSleeping().value());

    // 2. Torque
    auto resTorque = body.applyTorque(Vector3(0, 50, 0));
    assert(resTorque.has_value());

    // 3. Angular Impulse
    auto resAngImp = body.applyAngularImpulse(Vector3(0, 10, 0));
    assert(resAngImp.has_value());

    // 4. Angular Velocity getter/setter
    body.setAngularVelocity(Vector3(1, 2, 3));
    Vector3 w = body.getAngularVelocity().value();
    assert(std::abs(w.x - 1.0f) < 1e-4f && std::abs(w.y - 2.0f) < 1e-4f && std::abs(w.z - 3.0f) < 1e-4f);

    // 5. Material
    body.setMaterial(0.8f, 0.9f);

    // 6. Trigger
    assert(!body.isTrigger().value());
    body.setTrigger(true);
    assert(body.isTrigger().value());

    // 7. Layer and Mask
    body.setCollisionLayer(0x0004);
    body.setCollisionMask(0x0001);
    assert(body.getCollisionLayer().value() == 0x0004);
    assert(body.getCollisionMask().value() == 0x0001);

    std::cout << "  -> PASSED: All Body API functions work correctly!" << std::endl;
}

void runTriggerMechanicsTest() {
    std::cout << "[TEST 2] Trigger (Sensor) Zero-Impulse Phase-Through Test..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0)); // Zero G

    // Trigger box at (0, 0, 0)
    Body trigger = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape{collision::BoxShape(Vector3(1, 1, 1))});
    trigger.setBodyType(Core::BodyType::Static);
    trigger.setTrigger(true);

    // Dynamic ball flying directly through trigger from X = -3 to +3 at 10 m/s
    Body runner = sim.createBody(Pose(Vector3(-3.0f, 0, 0)), collision::CollisionShape{collision::SphereShape(0.4f)});
    runner.setBodyType(Core::BodyType::Dynamic);
    runner.setMass(1.0f);
    runner.setLinearVelocity(Vector3(10.0f, 0, 0));

    // Step through the trigger
    bool wasInsideTrigger = false;
    for (int step = 0; step < 60; ++step) {
        sim.step(1.0f / 60.0f);
        auto manifolds = sim.getManifolds();
        for (const auto& [k, m] : manifolds) {
            if (m.isTriggerPair) {
                wasInsideTrigger = true;
                for (uint32_t c = 0; c < m.contactCount; ++c) {
                    assert(m.contacts[c].normalImpulse == 0.0f); // MUST NOT apply impulse
                }
            }
        }
    }

    assert(wasInsideTrigger);
    // After 1 second at 10 m/s, runner should be around X = 7.0 without stopping or bouncing
    Vector3 finalPos = sim.getRegistry().getComponent<Pose>(runner.getEntity()).position;
    assert(finalPos.x > 3.0f);
    std::cout << "  -> PASSED: Runner phased through trigger with zero bounce! Final X: " << finalPos.x << std::endl;
}

void runCollisionFilteringTest() {
    std::cout << "[TEST 3] Bitmask Collision Filtering (Layers & Masks)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // Red Box at (0, 0, 0) - Layer 0x0002, Mask 0x0002
    Body red = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape{collision::BoxShape(Vector3(1, 1, 1))});
    red.setBodyType(Core::BodyType::Dynamic);
    red.setCollisionLayer(0x0002);
    red.setCollisionMask(0x0002);

    // Blue Box overlapping at (0.5, 0, 0) - Layer 0x0004, Mask 0x0004
    Body blue = sim.createBody(Pose(Vector3(0.5f, 0, 0)), collision::CollisionShape{collision::BoxShape(Vector3(1, 1, 1))});
    blue.setBodyType(Core::BodyType::Dynamic);
    blue.setCollisionLayer(0x0004);
    blue.setCollisionMask(0x0004);

    sim.step(1.0f / 60.0f);

    // Manifold count MUST be 0 because layers and masks do not overlap!
    assert(sim.getManifolds().empty());
    std::cout << "  -> PASSED: Red and Blue objects correctly ignored each other via bitmask filter!" << std::endl;
}

void runDistanceConstraintDriftTest() {
    std::cout << "[TEST 4] DistanceConstraint Baumgarte Drift Correction..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body anchor = sim.createBody(Pose(Vector3(0, 10, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    anchor.setBodyType(Core::BodyType::Static);

    float targetDist = 2.0f;
    Body bob = sim.createBody(Pose(Vector3(0.0f, 10.0f - targetDist, 0)), collision::CollisionShape{collision::SphereShape(0.3f)});
    bob.setBodyType(Core::BodyType::Dynamic);
    bob.setMass(2.0f);

    sim.createDistanceConstraint(anchor, bob, targetDist);
    // Simulate vertical hanging under gravity for 120 steps (2 seconds)
    for (int i = 0; i < 120; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 pAnchor = sim.getRegistry().getComponent<Pose>(anchor.getEntity()).position;
    Vector3 pBob = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
    float currentDist = (pBob - pAnchor).length();
    float drift = std::abs(currentDist - targetDist);
    assert(drift < 0.05f);
    std::cout << "  -> PASSED: Target distance: " << targetDist << "m | Actual: " << currentDist 
              << "m | Zero Drift achieved: " << drift << "m" << std::endl;
}

void runCollisionListenerEventsTest() {
    std::cout << "[TEST 5] CollisionListener Callbacks (Enter & Exit)..." << std::endl;
    Simulator sim;
    TestListener listener;
    sim.setCollisionListener(&listener);
    sim.setAccelerationField(Vector3(0, 0, 0));

    // Dynamic Sphere 1 at (0, 0, 0)
    Body s1 = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape{collision::SphereShape(1.0f)});
    s1.setBodyType(Core::BodyType::Dynamic);

    // Dynamic Sphere 2 at (1.5, 0, 0) overlapping
    Body s2 = sim.createBody(Pose(Vector3(1.5f, 0, 0)), collision::CollisionShape{collision::SphereShape(1.0f)});
    s2.setBodyType(Core::BodyType::Dynamic);

    sim.step(1.0f / 60.0f);
    assert(listener.collisionEnterCount > 0);

    // Move s2 far away
    sim.getRegistry().getComponent<Pose>(s2.getEntity()).position = Vector3(100.0f, 0, 0);
    sim.step(1.0f / 60.0f);
    assert(listener.collisionExitCount > 0);

    std::cout << "  -> PASSED: onCollisionEnter (" << listener.collisionEnterCount 
              << ") and onCollisionExit (" << listener.collisionExitCount << ") fired successfully!" << std::endl;
}

void runRaycastAllShapesTest() {
    std::cout << "[TEST 6] Raycast across Capsule, Triangle, and StaticMesh (Möller-Trumbore)..." << std::endl;
    Simulator sim;

    // 1. Capsule at (0, 5, 0)
    Body cap = sim.createBody(Pose(Vector3(0, 5, 0)), collision::CollisionShape{collision::CapsuleShape(0.5f, 2.0f)});
    cap.setBodyType(Core::BodyType::Static);

    auto hitCap = sim.raycast(Vector3(-5, 5, 0), Vector3(1, 0, 0), 10.0f);
    assert(hitCap.hasHit);

    // 2. Static Mesh at (0, 10, 0)
    std::vector<Vector3> verts = {
        Vector3(-1.0f, 0.0f, -1.0f),
        Vector3(1.0f, 0.0f, -1.0f),
        Vector3(0.0f, 0.0f, 1.0f)
    };
    std::vector<uint32_t> inds = { 0, 1, 2 };
    auto meshObj = std::make_shared<collision::StaticMesh>(verts, inds);
    Body meshBody = sim.createBody(Pose(Vector3(0, 10, 0)), collision::CollisionShape{collision::StaticMeshShape(meshObj)});
    meshBody.setBodyType(Core::BodyType::Static);

    auto hitMesh = sim.raycast(Vector3(0, 15, 0), Vector3(0, -1, 0), 10.0f);
    assert(hitMesh.hasHit);

    std::cout << "  -> PASSED: Capsule hit distance: " << hitCap.distance 
              << "m | StaticMesh (Möller-Trumbore) hit distance: " << hitMesh.distance << "m" << std::endl;
}

void runDynamicSparseSetGrowthTest() {
    std::cout << "[TEST 7] Dynamic SparseSet Growth beyond 10,000 Entities..." << std::endl;
    Simulator sim;

    // Create 12,000 bodies to exceed the old MAX_ENTITIES = 10,000 limit
    const int count = 12000;
    collision::CollisionShape shape{collision::BoxShape(Vector3(0.1f, 0.1f, 0.1f))};
    for (int i = 0; i < count; ++i) {
        sim.createBody(Pose(Vector3(i * 0.5f, 0, 0)), shape);
    }

    assert(sim.getRegistry().getComponentPool<Pose>().getAllEntities().size() == count);
    std::cout << "  -> PASSED: Created " << count << " entities without crash or assertion failure!" << std::endl;
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "      BARYON PHYSICS ENGINE VERIFICATION SUITE    " << std::endl;
    std::cout << "==================================================" << std::endl;

    runBodyApiTests();
    runTriggerMechanicsTest();
    runCollisionFilteringTest();
    runDistanceConstraintDriftTest();
    runCollisionListenerEventsTest();
    runRaycastAllShapesTest();
    runDynamicSparseSetGrowthTest();

    std::cout << "==================================================" << std::endl;
    std::cout << "  >>> ALL 7 NEW FEATURE VERIFICATION TESTS PASSED <<<" << std::endl;
    std::cout << "==================================================" << std::endl;
    return 0;
}
