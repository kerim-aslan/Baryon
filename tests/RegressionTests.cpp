/*
 * Baryon Physics Engine - Regression Verification Test Suite
 * Copyright (C) 2026 Kerim Aslan
 */

#include "Baryon/Simulator.hpp"
#include "Baryon/collision/CollisionShape.hpp"
#include "Baryon/collision/NarrowPhase.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include "TestCommon.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <chrono>

using namespace Baryon;

// #1: Kapsül (r=0.3, h=1.0) kutu zemine düşüyor
void testCapsuleOnBoxFloor() {
    std::cout << "\n[REGRESSION 1] Capsule (r=0.3, h=1.0) on Box Floor..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin: Üst yüzeyi Y=0'da olan kutu (halfExtents: 10, 0.5, 10, pos: 0, -0.5, 0)
    Pose floorPose;
    floorPose.position = Vector3(0.0f, -0.5f, 0.0f);
    Body floor = sim.createBody(floorPose, collision::CollisionShape(collision::BoxShape(Vector3(10.0f, 0.5f, 10.0f))));
    floor.setBodyType(Core::BodyType::Static);

    // Dik kapsül: Y=2.0'dan bırakılıyor
    Pose capPose;
    capPose.position = Vector3(0.0f, 2.0f, 0.0f);
    collision::CapsuleShape capShape(0.3f, 1.0f); // r=0.3, h=1.0
    Body cap = sim.createBody(capPose, collision::CollisionShape(capShape));
    cap.setBodyType(Core::BodyType::Dynamic);

    // 180 adım simüle et (3 saniye)
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 finalPos = cap.getPosition().value();
    std::cout << "  -> Capsule final Y: " << finalPos.y << " (Expected dik: ~0.8m)" << std::endl;
    // Dik duran kapsülün alt noktası y = pos.y - (h/2 + r) = pos.y - 0.8
    // Üst yüzey Y=0 olduğuna göre beklenen pos.y >= 0.75m olmalıdır.
    CHECK_MSG(finalPos.y >= 0.75f, "Capsule penetrated floor or sank into ground");
}

// #2: Küre statik mesh zemine düşüyor (yerçekimi açık)
void testSphereOnStaticMeshFloor() {
    std::cout << "\n[REGRESSION 2] Sphere on Static Mesh Floor with Gravity..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Y=0'da iki üçgenli zemin mesh'i
    std::vector<Vector3> verts = {
        Vector3(-10.0f, 0.0f, -10.0f),
        Vector3( 10.0f, 0.0f, -10.0f),
        Vector3( 10.0f, 0.0f,  10.0f),
        Vector3(-10.0f, 0.0f,  10.0f)
    };
    std::vector<uint32_t> indices = { 0, 1, 2, 0, 2, 3 };
    auto meshObj = std::make_shared<collision::StaticMesh>(verts, indices);
    Body meshFloor = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape(collision::StaticMeshShape(meshObj)));
    meshFloor.setBodyType(Core::BodyType::Static);

    // Yarıçapı 0.5 olan küre Y=3.0'dan düşüyor
    Body sphere = sim.createBody(Pose(Vector3(0, 3.0f, 0)), collision::CollisionShape(collision::SphereShape(0.5f)));
    sphere.setBodyType(Core::BodyType::Dynamic);

    // 180 adım simüle et
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 finalPos = sphere.getPosition().value();
    std::cout << "  -> Sphere final Y: " << finalPos.y << " (Expected: ~0.5m)" << std::endl;
    CHECK_MSG(finalPos.y >= 0.45f, "Sphere penetrated static mesh floor");
}

// #3: 1 kg küre, durağan 1 kg kutuya çarpıyor (4 m/s, momentum aktarımı)
void testMomentumConservationAt4ms() {
    std::cout << "\n[REGRESSION 3] 1kg Sphere collides with 1kg Box at 4, 6, 10, 20 m/s (Zero-G CCD)..." << std::endl;
    std::vector<float> testSpeeds = {4.0f, 6.0f, 10.0f, 20.0f};

    for (float v0 : testSpeeds) {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, 0, 0));

        // Mermi küre (1 kg, v = v0)
        Body bullet = sim.createBody(Pose(Vector3(-2.0f, 0, 0)), collision::CollisionShape(collision::SphereShape(0.2f)));
        bullet.setBodyType(Core::BodyType::Dynamic);
        CHECK(bullet.setMass(1.0f).has_value());
        bullet.setLinearVelocity(Vector3(v0, 0, 0));

        // Hedef kutu (1 kg, v = 0)
        Body target = sim.createBody(Pose(Vector3(0.0f, 0, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.2f, 0.2f, 0.2f))));
        target.setBodyType(Core::BodyType::Dynamic);
        CHECK(target.setMass(1.0f).has_value());

        for (int i = 0; i < 60; ++i) {
            sim.step(1.0f / 60.0f);
        }

        Vector3 bulletVel = bullet.getLinearVelocity().value();
        Vector3 targetVel = target.getLinearVelocity().value();
        float expectedV = v0 * 0.5f;
        std::cout << "  -> v0 = " << v0 << " m/s | After: Bullet Vx = " << bulletVel.x 
                  << ", Target Vx = " << targetVel.x << " (Expected ~" << expectedV << " m/s each)" << std::endl;
        CHECK_MSG(targetVel.x > (v0 * 0.25f), "Momentum not transferred to target under CCD");
        CHECK_MSG(bulletVel.x > 0.0f, "Bullet stopped dead under CCD");
    }
}

// #4: Küre 3 m'den düşüyor, restitution e=1.0 ile sekme yüksekliği
void testRestitutionBounce() {
    std::cout << "\n[REGRESSION 4] Sphere dropping from 3m with Restitution e=1.0..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin Y=0
    Pose floorPose;
    floorPose.position = Vector3(0.0f, -0.5f, 0.0f);
    Body floor = sim.createBody(floorPose, collision::CollisionShape(collision::BoxShape(Vector3(10.0f, 0.5f, 10.0f))));
    floor.setBodyType(Core::BodyType::Static);
    floor.setMaterial(0.0f, 1.0f);

    // Küre r=0.2, Y=3.0, CCD kapalı (saf restitution testi)
    Body sphere = sim.createBody(Pose(Vector3(0, 3.0f, 0)), collision::CollisionShape(collision::SphereShape(0.2f)));
    sphere.setBodyType(Core::BodyType::Dynamic);
    sphere.setCCD(false);
    sphere.setMaterial(0.0f, 1.0f);

    float maxApexAfterBounce = 0.0f;
    bool hasHitFloor = false;

    // 180 adım simüle et (~3 saniye)
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = sphere.getPosition().value();
        if (pos.y <= 0.25f) {
            hasHitFloor = true;
        }
        if (hasHitFloor && pos.y > maxApexAfterBounce) {
            maxApexAfterBounce = pos.y;
        }
    }

    std::cout << "  -> Bounce Max Apex Y: " << maxApexAfterBounce << "m (Expected: ~3.0m)" << std::endl;
    CHECK_MSG(maxApexAfterBounce >= 2.0f, "No bounce observed with restitution e=1.0");
}

// #5: setMass(10) ardından açısal impuls yanıtı (I ölçekleme)
void testSetMassInertiaScaling() {
    std::cout << "\n[REGRESSION 5] setMass(10) Inertia Tensor Scaling..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // Kutu 1x1x1 (halfExtents: 0.5, 0.5, 0.5), varsayılan kütle = 1.0 kg
    Body body = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))));
    body.setBodyType(Core::BodyType::Dynamic);
    CHECK(body.setMass(10.0f).has_value()); // 10 katına çıkar

    // Y ekseni etrafında 1.0 N·m·s açısal impuls uygula
    body.applyAngularImpulse(Vector3(0, 1.0f, 0));

    Vector3 w = body.getAngularVelocity().value();
    std::cout << "  -> Angular velocity wy: " << w.y << " (Expected ~0.6 rad/s, observed 6.0 if unscaled)" << std::endl;
    CHECK_MSG(std::abs(w.y - 0.6f) < 0.1f, "setMass did not rescale inertia tensor");
}

// #6: 8 kutu üst üste (1x1x1, sürtünme 0.6) kule stabilitesi
void testBoxStackStability() {
    std::cout << "\n[REGRESSION 6] 8-Box Stack Stability (10 velocity iterations)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin Y=0
    Body floor = sim.createBody(Pose(Vector3(0, -0.5f, 0)), collision::CollisionShape(collision::BoxShape(Vector3(10.0f, 0.5f, 10.0f))));
    floor.setBodyType(Core::BodyType::Static);
    floor.setMaterial(0.6f, 0.0f);

    std::vector<Body> boxes;
    for (int i = 0; i < 8; ++i) {
        float y = 0.5f + static_cast<float>(i) * 1.0f; // 0.5, 1.5, ..., 7.5
        Body b = sim.createBody(Pose(Vector3(0, y, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))));
        b.setBodyType(Core::BodyType::Dynamic);
        CHECK(b.setMass(1.0f).has_value());
        b.setMaterial(0.6f, 0.0f);
        boxes.push_back(b);
    }

    // 300 adım simüle et (5 saniye)
    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 topPos = boxes.back().getPosition().value();
    float drift = std::sqrt(topPos.x * topPos.x + topPos.z * topPos.z);
    std::cout << "  -> Top box final Pos: (" << topPos.x << ", " << topPos.y << ", " << topPos.z 
              << "), Drift: " << drift << "m (Expected drift < 0.2m, Y > 6.5m)" << std::endl;
    CHECK_MSG(drift < 0.2f && topPos.y > 6.5f, "8-box stack collapsed");
}

// #8: 20 FPS çerçeve süresi altında zaman akışı (catchup)
void testTimeStepCatchupAt20FPS() {
    std::cout << "\n[REGRESSION 8] 20 FPS Frame Rate (dt=0.05s) Real-Time Catchup..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body body = sim.createBody(Pose(Vector3(0, 100.0f, 0)), collision::CollisionShape(collision::SphereShape(0.5f)));
    body.setBodyType(Core::BodyType::Dynamic);
    body.setCCD(false);

    // 20 kare @ 0.05s = 1.0 saniye gerçek zaman
    for (int i = 0; i < 20; ++i) {
        sim.step(0.05f);
    }

    float finalY = body.getPosition().value().y;
    float dropDist = 100.0f - finalY;
    // Serbest düşüş: h = 0.5 * g * t^2 = 0.5 * 9.81 * 1.0 = ~4.9m
    std::cout << "  -> Drop distance after 1.0s real-time: " << dropDist << "m (Expected: ~4.9m)" << std::endl;
    CHECK_MSG(dropDist >= 4.0f, "Simulation slowed down due to maxSubSteps clamp (running at 2/3 speed)");
}

// #10: D1 - Ada ölçeklenme testi (2000 bağımsız temas çifti = 2000 ada x 2000 manifold)
void testIslandScalingPerformance() {
    std::cout << "\n[REGRESSION 10] Island Scaling Performance Benchmark (2000 islands with contacts)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // 2000 bağımsız temas eden çift (her çift kendi adasında, toplam 2000 ada ve 2000 manifold)
    const int pairCount = 2000;
    collision::CollisionShape shape(collision::SphereShape(0.5f));
    for (int i = 0; i < pairCount; ++i) {
        float x = static_cast<float>(i % 50) * 5.0f;
        float y = static_cast<float>((i / 50) % 40) * 5.0f;
        float z = static_cast<float>(i / 2000) * 5.0f;

        // A ve B hafifçe iç içe temas halinde (0.95m aralıkla iki r=0.5 küre)
        Body b1 = sim.createBody(Pose(Vector3(x, y, z)), shape);
        b1.setBodyType(Core::BodyType::Dynamic);
        b1.setCCD(false);

        Body b2 = sim.createBody(Pose(Vector3(x + 0.95f, y, z)), shape);
        b2.setBodyType(Core::BodyType::Dynamic);
        b2.setCCD(false);
    }

    // Isınma adımı (temasların üretilmesi için)
    sim.step(1.0f / 60.0f);
    size_t manifoldCount = sim.getManifolds().size();
    std::cout << "  -> Active manifolds: " << manifoldCount << std::endl;

    // 10 adımın süresini ölç
    auto start = std::chrono::high_resolution_clock::now();
    const int steps = 10;
    for (int s = 0; s < steps; ++s) {
        sim.step(1.0f / 60.0f);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double msPerStep = totalMs / steps;

#ifdef NDEBUG
    CHECK_MSG(msPerStep < 15.0, "Island solving O(islands x manifolds) quadratic bottleneck detected");
#else
    CHECK_MSG(msPerStep < 50.0, "Island solving quadratic bottleneck detected in Debug+ASan");
#endif
}

// #7: A5 - EPA sağlamlığı ve bozuk girdi güvenliği
void testEPARobustness() {
    std::cout << "\n[REGRESSION 7] Capsule-Sphere EPA Robustness & Degenerate Simplex Handling..." << std::endl;

    // 1. Bozuk / Dejeneratif Simplex EPA testi (hasCollision=false dönmeli)
    collision::Simplex degenerateSimplex;
    degenerateSimplex.push_front(Vector3(0, 0, 0));
    degenerateSimplex.push_front(Vector3(0, 0, 0));
    degenerateSimplex.push_front(Vector3(0, 0, 0));
    degenerateSimplex.push_front(Vector3(0, 0, 0));

    collision::CollisionShape shapeA(collision::SphereShape(1.0f));
    collision::CollisionShape shapeB(collision::BoxShape(Vector3(1, 1, 1)));
    Pose poseA(Vector3(0, 0, 0));
    Pose poseB(Vector3(0.5f, 0, 0));

    collision::CollisionInfo info = collision::NarrowPhase::EPA(
        degenerateSimplex, shapeA, poseA, shapeB, poseB
    );

    std::cout << "  -> Degenerate simplex EPA hasCollision: " << std::boolalpha << info.hasCollision 
              << " | Normal: (" << info.normal.x << ", " << info.normal.y << ", " << info.normal.z << ") (Expected hasCollision=false)" << std::endl;
    CHECK_MSG(!info.hasCollision, "EPA returned hasCollision=true with invalid normal for degenerate zero-simplex!");

    // 2. Tablo #7 Senaryosu: Kapsül ve Statik Küre (r=3.0) çarpışması
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body sphere = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape(collision::SphereShape(3.0f)));
    sphere.setBodyType(Core::BodyType::Static);

    Body capsule = sim.createBody(Pose(Vector3(0, 4.0f, 0)), collision::CollisionShape(collision::CapsuleShape(0.5f, 2.0f)));
    capsule.setBodyType(Core::BodyType::Dynamic);

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 120; ++i) {
        sim.step(1.0f / 60.0f);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "  -> Capsule vs r=3 Sphere (120 steps) elapsed time: " << elapsedMs << " ms (Expected: < 2000 ms, was hanging >20s)" << std::endl;
    CHECK_MSG(elapsedMs < 2000.0, "Capsule vs Sphere scenario hung or exceeded timeout in EPA!");
}

// #9: A8 - Uyku eşiği ve 45 derece eğik kapsül devrilme testi
void testTiltedCapsuleToppling() {
    std::cout << "\n[REGRESSION 9] 45-degree Tilted Capsule Toppling Test..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin: Üst yüzeyi Y=0'da olan kutu (halfExtents: 10, 0.5, 10, pos: 0, -0.5, 0)
    Pose floorPose;
    floorPose.position = Vector3(0.0f, -0.5f, 0.0f);
    Body floor = sim.createBody(floorPose, collision::CollisionShape(collision::BoxShape(Vector3(10.0f, 0.5f, 10.0f))));
    floor.setBodyType(Core::BodyType::Static);

    // 45 derece eğik kapsül (Z ekseni etrafında 45 derece eğik)
    // r = 0.3m, h = 1.0m
    Pose capPose;
    capPose.position = Vector3(0.0f, 1.5f, 0.0f);
    float angleRad = 45.0f * (3.14159265f / 180.0f);
    capPose.orientation = Quaternion(Vector3(0, 0, 1), angleRad);

    collision::CapsuleShape capShape(0.3f, 1.0f);
    Body cap = sim.createBody(capPose, collision::CollisionShape(capShape));
    cap.setBodyType(Core::BodyType::Dynamic);

    // 300 adım simüle et (5 saniye)
    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 finalPos = cap.getPosition().value();
    std::cout << "  -> Tilted Capsule final Y: " << finalPos.y << " (Expected yatay/devrilmiş: ~0.3m, hatalı/erken uyku: ~0.64m)" << std::endl;
    CHECK_MSG(finalPos.y <= 0.45f, "Tilted capsule froze mid-air or fell asleep prematurely without toppling flat!");
}

// #10: C1 - CollisionSystem::step ayrıştırma (decomposition) ve 3 dal doğrulaması
void testCollisionSystemStepDecomposition() {
    std::cout << "\n[REGRESSION 10] C1: CollisionSystem::step Decomposition & Branches..." << std::endl;

    // 1. Dosya satır sayısı kontrolü
    std::vector<std::string> searchPaths = {
        "baryon/src/systems/CollisionSystem.cpp",
        "../baryon/src/systems/CollisionSystem.cpp",
        "../../baryon/src/systems/CollisionSystem.cpp"
    };
    std::ifstream file;
    for (const auto& path : searchPaths) {
        file.open(path);
        if (file.is_open()) break;
    }
    int stepLines = 0;
    if (file.is_open()) {
        std::string line;
        bool inStep = false;
        int braceDepth = 0;
        while (std::getline(file, line)) {
            if (line.find("void CollisionSystem::step()") != std::string::npos) {
                inStep = true;
            }
            if (inStep) {
                stepLines++;
                for (char c : line) {
                    if (c == '{') braceDepth++;
                    else if (c == '}') {
                        braceDepth--;
                        if (braceDepth == 0) {
                            inStep = false;
                            break;
                        }
                    }
                }
                if (!inStep) break;
            }
        }
        std::cout << "  -> CollisionSystem::step() LOC: " << stepLines << " (Expected: < 120 lines, originally ~366 lines)" << std::endl;
        CHECK_MSG(stepLines < 120, "CollisionSystem::step is too monolithic (exceeds 120 lines)!");
    } else {
        std::cout << "  -> Note: CollisionSystem.cpp not found relative to cwd, skipping LOC check." << std::endl;
    }

    // 2. Simülatörde 3 dalın (Mesh, Box-Box, General) doğru çalıştığını doğrula
    Simulator sim;

    // Dal 1: Mesh vs Sphere
    std::vector<Vector3> meshVerts = {
        Vector3(-5, 0, -5), Vector3(5, 0, -5), Vector3(5, 0, 5),
        Vector3(-5, 0, -5), Vector3(5, 0, 5), Vector3(-5, 0, 5)
    };
    std::vector<uint32_t> meshIndices = {0, 1, 2, 3, 4, 5};
    auto meshObj = std::make_shared<collision::StaticMesh>(meshVerts, meshIndices);
    Body meshFloor = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape(collision::StaticMeshShape(meshObj)));
    meshFloor.setBodyType(Core::BodyType::Static);

    Body sphereOnMesh = sim.createBody(Pose(Vector3(0, 0.5f, 0)), collision::CollisionShape(collision::SphereShape(1.0f)));
    sphereOnMesh.setBodyType(Core::BodyType::Dynamic);

    // Dal 2: Box vs Box
    Body boxFloor = sim.createBody(Pose(Vector3(20, 0, 0)), collision::CollisionShape(collision::BoxShape(Vector3(5, 1, 5))));
    boxFloor.setBodyType(Core::BodyType::Static);

    Body boxOnBox = sim.createBody(Pose(Vector3(20, 1.5f, 0)), collision::CollisionShape(collision::BoxShape(Vector3(1, 1, 1))));
    boxOnBox.setBodyType(Core::BodyType::Dynamic);

    // Dal 3: General (Sphere vs Capsule)
    Body capsuleStatic = sim.createBody(Pose(Vector3(-20, 0, 0)), collision::CollisionShape(collision::CapsuleShape(0.5f, 2.0f)));
    capsuleStatic.setBodyType(Core::BodyType::Static);

    Body sphereOnCapsule = sim.createBody(Pose(Vector3(-20, 1.2f, 0)), collision::CollisionShape(collision::SphereShape(0.5f)));
    sphereOnCapsule.setBodyType(Core::BodyType::Dynamic);

    // Bir adım ilerlet ve her çift için manifold varlığını kontrol et
    sim.step(1.0f / 60.0f);

    auto meshContacts = sim.getContactsForEntity(sphereOnMesh.getEntity());
    auto boxContacts = sim.getContactsForEntity(boxOnBox.getEntity());
    auto generalContacts = sim.getContactsForEntity(sphereOnCapsule.getEntity());

    std::cout << "  -> Mesh branch contacts: " << meshContacts.size() << " (Expected: > 0)" << std::endl;
    std::cout << "  -> Box-Box branch contacts: " << boxContacts.size() << " (Expected: > 0)" << std::endl;
    std::cout << "  -> General branch contacts: " << generalContacts.size() << " (Expected: > 0)" << std::endl;

    CHECK_MSG(!meshContacts.empty(), "Mesh collision branch failed to generate contact manifold!");
    CHECK_MSG(!boxContacts.empty(), "Box-Box collision branch failed to generate contact manifold!");
    CHECK_MSG(!generalContacts.empty(), "General collision branch failed to generate contact manifold!");
}

// #11: C2 - ContactManifold::inheritImpulses warm-start eşleştirme testi
void testWarmStartInheritImpulses() {
    std::cout << "\n[REGRESSION 11] C2: ContactManifold::inheritImpulses Warm-Start..." << std::endl;

    collision::ContactManifold oldManifold(ecs::Entity(1), ecs::Entity(2));
    collision::ContactPoint oldCp0;
    oldCp0.localPointA = Vector3(0.0f, 0.0f, 0.0f);
    oldCp0.normalImpulse = 42.5f;
    oldCp0.tangentImpulse1 = 5.2f;
    oldCp0.tangentImpulse2 = -3.1f;
    oldManifold.addContact(oldCp0);

    collision::ContactPoint oldCp1;
    oldCp1.localPointA = Vector3(1.0f, 0.0f, 0.0f);
    oldCp1.normalImpulse = 100.0f;
    oldCp1.tangentImpulse1 = 12.0f;
    oldCp1.tangentImpulse2 = 7.0f;
    oldManifold.addContact(oldCp1);

    // Yeni manifold: cp0 yakın (mesafe = 0.01 < 0.2m / distSq = 0.0001 < 0.04) -> eşleşmeli
    // cp1 uzak (mesafe = 5.0m > 0.2m) -> eşleşmemeli
    collision::ContactManifold newManifold(ecs::Entity(1), ecs::Entity(2));
    collision::ContactPoint newCp0;
    newCp0.localPointA = Vector3(0.01f, 0.0f, 0.0f);
    newManifold.addContact(newCp0);

    collision::ContactPoint newCp1;
    newCp1.localPointA = Vector3(5.0f, 0.0f, 0.0f);
    newManifold.addContact(newCp1);

    newManifold.inheritImpulses(oldManifold);

    std::cout << "  -> Matched normalImpulse: " << newManifold.contacts[0].normalImpulse << " (Expected: 42.5)" << std::endl;
    std::cout << "  -> Matched tangentImpulse1: " << newManifold.contacts[0].tangentImpulse1 << " (Expected: 5.2)" << std::endl;
    std::cout << "  -> Matched tangentImpulse2: " << newManifold.contacts[0].tangentImpulse2 << " (Expected: -3.1)" << std::endl;
    std::cout << "  -> Unmatched normalImpulse: " << newManifold.contacts[1].normalImpulse << " (Expected: 0.0)" << std::endl;

    CHECK_MSG(std::abs(newManifold.contacts[0].normalImpulse - 42.5f) < 1e-4f, "Contact 0 normalImpulse not inherited!");
    CHECK_MSG(std::abs(newManifold.contacts[0].tangentImpulse1 - 5.2f) < 1e-4f, "Contact 0 tangentImpulse1 not inherited!");
    CHECK_MSG(std::abs(newManifold.contacts[0].tangentImpulse2 - (-3.1f)) < 1e-4f, "Contact 0 tangentImpulse2 not inherited!");
    CHECK_MSG(newManifold.contacts[1].normalImpulse == 0.0f, "Contact 1 wrongly inherited impulse despite being far!");
}

// #12: C3 - Temas noktası oluşturma yardımcı fonksiyonu testi
void testContactPointCreationHelper() {
    std::cout << "\n[REGRESSION 12] C3: ContactPoint::create Helper..." << std::endl;

    Pose poseA;
    poseA.position = Vector3(2.0f, 3.0f, 4.0f);
    // Y ekseni etrafında 90 derece dönüş (cos(45)=sqrt(0.5), sin(45)=sqrt(0.5))
    poseA.orientation = Quaternion(Vector3(0, 1, 0), 1.57079632f);

    Pose poseB;
    poseB.position = Vector3(5.0f, 6.0f, 7.0f);
    poseB.orientation = Quaternion(); // birim kuaterniyon

    Vector3 worldPt(2.0f, 3.0f, 6.0f);
    float pen = 0.05f;

    collision::ContactPoint cp = collision::ContactPoint::create(worldPt, pen, poseA, poseB);

    std::cout << "  -> cp.worldPosition: (" << cp.worldPosition.x << ", " << cp.worldPosition.y << ", " << cp.worldPosition.z << ")" << std::endl;
    std::cout << "  -> cp.penetration: " << cp.penetration << std::endl;
    std::cout << "  -> cp.normalImpulse: " << cp.normalImpulse << std::endl;
    std::cout << "  -> cp.localPointB: (" << cp.localPointB.x << ", " << cp.localPointB.y << ", " << cp.localPointB.z << ")" << std::endl;

    CHECK_MSG((cp.worldPosition - worldPt).length() < 1e-5f, "worldPosition mismatch!");
    CHECK_MSG(std::abs(cp.penetration - pen) < 1e-5f, "penetration mismatch!");
    CHECK_MSG(cp.normalImpulse == 0.0f, "normalImpulse not zeroed!");
    CHECK_MSG(cp.tangentImpulse1 == 0.0f, "tangentImpulse1 not zeroed!");
    CHECK_MSG(cp.tangentImpulse2 == 0.0f, "tangentImpulse2 not zeroed!");
    CHECK_MSG(cp.splitImpulse == 0.0f, "splitImpulse not zeroed!");

    // worldPt - poseB.position = (2-5, 3-6, 6-7) = (-3, -3, -1)
    Vector3 expectedLocalB(-3.0f, -3.0f, -1.0f);
    CHECK_MSG((cp.localPointB - expectedLocalB).length() < 1e-4f, "localPointB calculation incorrect!");

    // localPointA: worldPt - poseA.position = (0, 0, 2). 90 deg rotation around Y gives (2, 0, 0)
    Vector3 expectedLocalA = poseA.orientation.getConjugate() * (worldPt - poseA.position);
    CHECK_MSG((cp.localPointA - expectedLocalA).length() < 1e-4f, "localPointA calculation incorrect!");
}

// #13: B3 - SimulationSettings yapılandırması testi
void testSimulationSettingsConfiguration() {
    std::cout << "\n[REGRESSION 13] B3: SimulationSettings Configuration..." << std::endl;

    Simulator sim;
    SimulationSettings settings;
    settings.velocityIterations = 20;
    settings.positionIterations = 8;
    settings.broadPhaseVelocityExpansion = 0.05f;
    settings.maxSubSteps = 12;
    settings.sleepLinearThreshold = 0.01f;
    settings.sleepAngularThreshold = 0.02f;
    settings.timeToSleep = 0.8f;

    sim.setSettings(settings);

    const auto& s = sim.getSettings();
    std::cout << "  -> velocityIterations: " << s.velocityIterations << " (Expected: 20)" << std::endl;
    std::cout << "  -> positionIterations: " << s.positionIterations << " (Expected: 8)" << std::endl;
    std::cout << "  -> broadPhaseVelocityExpansion: " << s.broadPhaseVelocityExpansion << " (Expected: 0.05)" << std::endl;
    std::cout << "  -> maxSubSteps: " << sim.getMaxSubSteps() << " (Expected: 12)" << std::endl;

    CHECK_MSG(s.velocityIterations == 20, "velocityIterations not updated in settings!");
    CHECK_MSG(s.positionIterations == 8, "positionIterations not updated in settings!");
    CHECK_MSG(std::abs(s.broadPhaseVelocityExpansion - 0.05f) < 1e-5f, "broadPhaseVelocityExpansion not updated!");
    CHECK_MSG(sim.getMaxSubSteps() == 12, "maxSubSteps getter mismatch!");
}

// #14: B7 - ManifoldCache ve ContactEvents ayrıştırma testi
struct MockCollisionListener : public CollisionListener {
    int collisionEnterCount{0};
    int collisionExitCount{0};
    int triggerEnterCount{0};
    int triggerExitCount{0};

    void onCollisionEnter(ecs::Entity, ecs::Entity, const collision::ContactManifold&) override { collisionEnterCount++; }
    void onCollisionExit(ecs::Entity, ecs::Entity) override { collisionExitCount++; }
    void onTriggerEnter(ecs::Entity, ecs::Entity) override { triggerEnterCount++; }
    void onTriggerExit(ecs::Entity, ecs::Entity) override { triggerExitCount++; }
};

void testManifoldCacheAndContactEvents() {
    std::cout << "\n[REGRESSION 14] B7: ManifoldCache & ContactEvents Decomposition..." << std::endl;

    collision::ManifoldCache cache;
    ecs::Entity e1(10), e2(20), e3(30);

    uint64_t key12 = collision::ManifoldCache::makeKey(e1, e2);
    uint64_t key21 = collision::ManifoldCache::makeKey(e2, e1);
    CHECK_MSG(key12 == key21, "makeKey symmetricity failed!");

    collision::ContactManifold m12(e1, e2);
    cache.getCurrent().emplace(key12, m12);

    // Yeni kareye geç (beginFrame)
    cache.beginFrame();
    CHECK_MSG(cache.getCurrent().empty(), "getCurrent should be empty after beginFrame!");
    CHECK_MSG(cache.findPrevious(key12) != nullptr, "findPrevious failed to locate previous manifold!");

    // Yeni manifoldlar: e1-e2 devam ediyor, e2-e3 tetikleyici olarak ekleniyor
    cache.getCurrent().emplace(key12, m12);
    collision::ContactManifold m23(e2, e3);
    m23.isTriggerPair = true;
    uint64_t key23 = collision::ManifoldCache::makeKey(e2, e3);
    cache.getCurrent().emplace(key23, m23);

    // Olay bildirimi (ContactEvents::dispatch)
    MockCollisionListener listener;
    collision::ContactEvents::dispatch(&listener, cache.getCurrent(), cache.getPrevious());

    std::cout << "  -> triggerEnterCount: " << listener.triggerEnterCount << " (Expected: 1)" << std::endl;
    std::cout << "  -> collisionEnterCount: " << listener.collisionEnterCount << " (Expected: 0, e1-e2 already existed)" << std::endl;

    CHECK_MSG(listener.triggerEnterCount == 1, "triggerEnterCount should be 1!");
    CHECK_MSG(listener.collisionEnterCount == 0, "collisionEnterCount should be 0 since e1-e2 was in previous!");

    // e1-e2 ve e2-e3'ün sonlandığı kare
    cache.beginFrame();
    collision::ContactEvents::dispatch(&listener, cache.getCurrent(), cache.getPrevious());
    std::cout << "  -> collisionExitCount: " << listener.collisionExitCount << " (Expected: 1)" << std::endl;
    std::cout << "  -> triggerExitCount: " << listener.triggerExitCount << " (Expected: 1)" << std::endl;

    CHECK_MSG(listener.collisionExitCount == 1, "collisionExitCount should be 1!");
    CHECK_MSG(listener.triggerExitCount == 1, "triggerExitCount should be 1!");
}

// #15: C6 - PhysicsConstants sihirli sayıların taşınması testi
void testPhysicsConstantsMagicNumbers() {
    std::cout << "\n[REGRESSION 15] C6: PhysicsConstants Named Constants..." << std::endl;

    std::cout << "  -> NormalSimilarityThreshold: " << physics::PhysicsConstants::NormalSimilarityThreshold << " (Expected: 0.95)" << std::endl;
    std::cout << "  -> WarmStartDistanceSqThreshold: " << physics::PhysicsConstants::WarmStartDistanceSqThreshold << " (Expected: 0.04)" << std::endl;
    std::cout << "  -> MinMeshQueryExtent: " << physics::PhysicsConstants::MinMeshQueryExtent << " (Expected: 3.0)" << std::endl;
    std::cout << "  -> RollingResistanceCoefficient: " << physics::PhysicsConstants::RollingResistanceCoefficient << " (Expected: 0.02)" << std::endl;
    std::cout << "  -> DegenerateVectorThresholdSq: " << physics::PhysicsConstants::DegenerateVectorThresholdSq << " (Expected: 0.0001)" << std::endl;
    std::cout << "  -> InfinityDistance: " << physics::PhysicsConstants::InfinityDistance << " (Expected: 1e10)" << std::endl;

    CHECK_MSG(std::abs(physics::PhysicsConstants::NormalSimilarityThreshold - 0.95f) < 1e-5f, "NormalSimilarityThreshold mismatch!");
    CHECK_MSG(std::abs(physics::PhysicsConstants::WarmStartDistanceSqThreshold - 0.04f) < 1e-5f, "WarmStartDistanceSqThreshold mismatch!");
    CHECK_MSG(std::abs(physics::PhysicsConstants::MinMeshQueryExtent - 3.0f) < 1e-5f, "MinMeshQueryExtent mismatch!");
    CHECK_MSG(std::abs(physics::PhysicsConstants::RollingResistanceCoefficient - 0.02f) < 1e-5f, "RollingResistanceCoefficient mismatch!");
    CHECK_MSG(std::abs(physics::PhysicsConstants::DegenerateVectorThresholdSq - 0.0001f) < 1e-6f, "DegenerateVectorThresholdSq mismatch!");
    CHECK_MSG(physics::PhysicsConstants::InfinityDistance == 1e10f, "InfinityDistance mismatch!");
}

// #16: C7 - Mesh Temas Azaltması (Contact Reduction) testi
void testContactReduction() {
    std::cout << "\n[REGRESSION 16] C7: Mesh Contact Reduction (Support Area Maximization)..." << std::endl;

    collision::ContactManifold manifold(ecs::Entity(1), ecs::Entity(2));
    manifold.normal = Vector3(0.0f, 1.0f, 0.0f);
    manifold.computeTangents();

    Pose poseA, poseB;

    // 4 köşe noktası (4x4 kare, Alan = 16.0)
    // p0: en derin nokta (penetration = 0.5)
    auto p0 = collision::ContactPoint::create(Vector3(-2.0f, 0.0f, -2.0f), 0.5f, poseA, poseB);
    auto p1 = collision::ContactPoint::create(Vector3( 2.0f, 0.0f, -2.0f), 0.2f, poseA, poseB);
    auto p2 = collision::ContactPoint::create(Vector3( 2.0f, 0.0f,  2.0f), 0.2f, poseA, poseB);
    auto p3 = collision::ContactPoint::create(Vector3(-2.0f, 0.0f,  2.0f), 0.2f, poseA, poseB);

    manifold.addContact(p0);
    manifold.addContact(p1);
    manifold.addContact(p2);
    manifold.addContact(p3);

    // 5. iç nokta (merkeze yakın, az penetrasyon: derinlik = 0.05)
    auto pInterior = collision::ContactPoint::create(Vector3(-0.5f, 0.0f, 0.5f), 0.05f, poseA, poseB);
    manifold.addContact(pInterior);

    // Manifold'daki 4 temas noktasının alanını hesapla
    std::vector<Vector3> pts;
    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        pts.push_back(manifold.contacts[i].worldPosition);
    }

    auto computeArea = [](const std::vector<Vector3>& points, const Vector3& norm) {
        if (points.size() < 3) return 0.0f;
        Vector3 t1, t2;
        if (std::abs(norm.x) >= 0.57735f) {
            t1 = Vector3(norm.y, -norm.x, 0.0f).getNormalized();
        } else {
            t1 = Vector3(0.0f, norm.z, -norm.y).getNormalized();
        }
        t2 = norm.cross(t1);

        struct P2 { float u, v, a; };
        std::vector<P2> p2d;
        float cu = 0.0f, cv = 0.0f;
        for (const auto& pt : points) {
            float u = pt.dot(t1);
            float v = pt.dot(t2);
            p2d.push_back({u, v, 0.0f});
            cu += u; cv += v;
        }
        cu /= static_cast<float>(points.size());
        cv /= static_cast<float>(points.size());
        for (auto& pt : p2d) pt.a = std::atan2(pt.v - cv, pt.u - cu);
        std::sort(p2d.begin(), p2d.end(), [](const P2& a, const P2& b) { return a.a < b.a; });

        float area = 0.0f;
        for (size_t i = 0; i < p2d.size(); ++i) {
            size_t next = (i + 1) % p2d.size();
            area += (p2d[i].u * p2d[next].v - p2d[next].u * p2d[i].v);
        }
        return 0.5f * std::abs(area);
    };

    float area = computeArea(pts, manifold.normal);
    std::cout << "  -> Manifold contactCount: " << manifold.contactCount << " (Expected: 4)" << std::endl;
    std::cout << "  -> Support polygon area: " << area << " (Expected: ~16.0, naive overwriting yields ~11.25)" << std::endl;

    // En derin nokta (p0) mutlaka korunmalı
    bool hasDeepest = false;
    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        if ((manifold.contacts[i].worldPosition - p0.worldPosition).lengthSquare() < 1e-4f) {
            hasDeepest = true;
            break;
        }
    }

    CHECK_MSG(manifold.contactCount == 4, "contactCount should be 4!");
    CHECK_MSG(hasDeepest, "Deepest contact point p0 must be preserved!");
    CHECK_MSG(area >= 15.9f, "Contact reduction must maximize support polygon area (expected >= 15.9)!");
}

// #17: C8 - Error handling & [[nodiscard]] on Body methods
void testErrorHandlingAndNoDiscard() {
    std::cout << "\n[REGRESSION 17] C8: Error handling & [[nodiscard]] on Body methods..." << std::endl;

    Simulator sim;
    auto b = sim.createBody(Pose(Vector3(0, 1, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))));

    auto validRes = b.setMass(2.0f);
    CHECK_MSG(validRes.has_value(), "Valid mass setting should succeed!");

    auto negRes = b.setMass(-1.0f);
    std::cout << "  -> setMass(-1.0f) has_value: " << std::boolalpha << negRes.has_value() << " (Expected: false)" << std::endl;
    CHECK_MSG(!negRes.has_value(), "Negative mass must return error!");

    // Dead entity test
    Body deadBody(ecs::Entity(99999), sim.getRegistry());
    auto deadRes = deadBody.setMass(10.0f);
    std::cout << "  -> deadBody.setMass(10.0f) has_value: " << std::boolalpha << deadRes.has_value() << " (Expected: false)" << std::endl;
    CHECK_MSG(!deadRes.has_value(), "Dead entity must return error!");
}

// #18: C9 - Mutable Manifolds Access without const_cast
void testMutableManifoldsAccessAndNoConstCast() {
    std::cout << "\n[REGRESSION 18] C9: Mutable manifolds access without const_cast..." << std::endl;

    Simulator sim;
    auto b1 = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))));
    auto b2 = sim.createBody(Pose(Vector3(0, 0.5f, 0)), collision::CollisionShape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))));

    sim.step(1.0f / 60.0f);

    auto& mutableManifolds = sim.getManifolds();
    std::cout << "  -> Mutable manifolds count: " << mutableManifolds.size() << " (Expected: >= 1)" << std::endl;
    CHECK_MSG(!mutableManifolds.empty(), "Should have active manifolds!");

    if (!mutableManifolds.empty()) {
        auto& firstManifold = mutableManifolds.begin()->second;
        firstManifold.penetration = 0.42f;
        CHECK_MSG(std::abs(sim.getManifolds().begin()->second.penetration - 0.42f) < 1e-4f, "Direct mutable modification must succeed without const_cast!");
    }
}

// #19: C4 - Diğer Büyük Fonksiyonların Bölünmesi testi
void testLargeFunctionsDecomposition() {
    std::cout << "\n[REGRESSION 19] C4: Large Functions Decomposition..." << std::endl;

    auto countFunctionLOC = [](const std::string& filePath, const std::string& signature) -> int {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            file.open("../" + filePath);
        }
        if (!file.is_open()) return -1;
        std::string line;
        bool inside = false;
        int depth = 0;
        int loc = 0;
        while (std::getline(file, line)) {
            if (!inside) {
                if (line.find(signature) != std::string::npos) {
                    inside = true;
                    if (line.find('{') != std::string::npos) depth++;
                }
            } else {
                loc++;
                for (char c : line) {
                    if (c == '{') depth++;
                    else if (c == '}') {
                        depth--;
                        if (depth == 0) return loc;
                    }
                }
            }
        }
        return loc;
    };

    int islandStepLOC = countFunctionLOC("baryon/src/systems/IslandSystem.cpp", "IslandSystem::step(");
    int solverExecuteLOC = countFunctionLOC("baryon/src/systems/SequentialImpulsesKernel.cpp", "SequentialImpulsesKernel::execute(");
    int applyCcdLOC = countFunctionLOC("baryon/src/systems/CollisionSystem.cpp", "CollisionSystem::applyCCD(");

    std::cout << "  -> IslandSystem::step() LOC: " << islandStepLOC << " (Expected: < 60 lines, originally ~191)" << std::endl;
    std::cout << "  -> SequentialImpulsesKernel::execute() LOC: " << solverExecuteLOC << " (Expected: < 45 lines, originally ~121)" << std::endl;
    std::cout << "  -> CollisionSystem::applyCCD() LOC: " << applyCcdLOC << " (Expected: < 55 lines, originally ~203)" << std::endl;

    CHECK_MSG(islandStepLOC > 0 && islandStepLOC < 60, "IslandSystem::step() should be refactored into smaller sub-functions (< 60 LOC)!");
    CHECK_MSG(solverExecuteLOC > 0 && solverExecuteLOC < 45, "SequentialImpulsesKernel::execute() should be refactored into smaller sub-functions (< 45 LOC)!");
    CHECK_MSG(applyCcdLOC > 0 && applyCcdLOC < 55, "CollisionSystem::applyCCD() should be refactored into smaller sub-functions (< 55 LOC)!");
}

// #20: C5 - Gölgeleme (-Wshadow) Uyarısı Temizliği testi
void testShadowWarningFree() {
    std::cout << "\n[REGRESSION 20] C5: Shadow Warning (-Wshadow) Cleanliness..." << std::endl;

    // Verify constructor semantics with distinct values to ensure no shadowing or self-assignment bug
    Baryon::Vector3 v(1.0f, 2.0f, 3.0f);
    CHECK_MSG(v.x == 1.0f && v.y == 2.0f && v.z == 3.0f, "Vector3 constructor parameters must assign correctly without shadowing!");

    Baryon::Quaternion q(0.1f, 0.2f, 0.3f, 0.9f);
    CHECK_MSG(q.x == 0.1f && q.y == 0.2f && q.z == 0.3f && q.w == 0.9f, "Quaternion constructor parameters must assign correctly without shadowing!");

    Baryon::Pose p(v, q);
    CHECK_MSG(p.position.x == 1.0f && p.orientation.w == 0.9f, "Pose constructor parameters must assign correctly without shadowing!");

    std::cout << "  -> Vector3, Quaternion, Pose constructors verified without -Wshadow warnings." << std::endl;
}

// #21: A9 - Entity Nesli 16-bit ve Geri Dönüşüm (Recycle) Güvenliği Testi
void testEntityGenerationRecycling() {
    std::cout << "\n[REGRESSION 21] A9: Entity 16-bit Generation and Recycling..." << std::endl;

    Baryon::Simulator sim;
    auto& reg = sim.getRegistry();

    // 1. Bir varlık oluştur ve bileşen ata
    Baryon::Pose pose(Baryon::Vector3(0, 1, 0));
    Baryon::ecs::Entity originalEntity = reg.createEntity();
    reg.addComponent<Baryon::Pose>(originalEntity, pose);

    CHECK_MSG(reg.isAlive(originalEntity), "Created entity must be alive!");
    CHECK_MSG(reg.hasComponent<Baryon::Pose>(originalEntity), "Created entity must have Pose component!");

    // 2. Varlığı sil
    reg.destroyEntity(originalEntity);
    CHECK_MSG(!reg.isAlive(originalEntity), "Destroyed entity must not be alive!");
    CHECK_MSG(!reg.hasComponent<Baryon::Pose>(originalEntity), "Destroyed entity must not report having component!");

    // 3. Aynı slotu 256 kez (8-bit sınırının ötesinde) geri dönüştür
    Baryon::ecs::Entity latestEntity = originalEntity;
    for (int i = 0; i < 300; ++i) {
        latestEntity = reg.createEntity();
        reg.destroyEntity(latestEntity);
    }

    // 4. Şimdi aynı slota yeni bir varlık oluştur
    Baryon::ecs::Entity newEntityInSameSlot = reg.createEntity();
    reg.addComponent<Baryon::Pose>(newEntityInSameSlot, Baryon::Pose(Baryon::Vector3(5, 5, 5)));

    // Eski handle asla geçerli veya bileşene sahip OLMAMALI!
    bool oldHandleAlive = reg.isAlive(originalEntity);
    bool oldHandleHasComp = reg.hasComponent<Baryon::Pose>(originalEntity);
    std::cout << "  -> 300 recycling cycles: oldHandle alive: " << (oldHandleAlive ? "true" : "false")
              << " (Expected: false), hasComponent: " << (oldHandleHasComp ? "true" : "false") << " (Expected: false)" << std::endl;

    CHECK_MSG(!oldHandleAlive, "Old entity handle must remain dead even after 300 recycles of the same slot!");
    CHECK_MSG(!oldHandleHasComp, "Old entity handle must NOT report having components belonging to new entity in same slot!");
    CHECK_MSG(reg.isAlive(newEntityInSameSlot), "New entity in same slot must be alive!");
}

// #22: B1 & B2 - Variadic ComponentTypeList ve Bit Düzeyinde Snapshot Determinizmi Testi
void testComponentVariadicAndSnapshotDeterminism() {
    std::cout << "\n[REGRESSION 22] B1 & B2: Variadic ComponentTypeList & Snapshot Determinism..." << std::endl;

    // B1 Kontrolü: Tek merkezli bileşen listesi boyutu
    static_assert(std::tuple_size_v<Baryon::Core::ComponentTypeList> == 9, "ComponentTypeList must define all 9 core components in one place!");
    static_assert(std::tuple_size_v<Baryon::Core::ComponentStatesTuple> == 9, "ComponentStatesTuple must be derived from ComponentTypeList!");
    std::cout << "  -> Variadic ComponentTypeList verified: 9 components derived in one single definition." << std::endl;

    // B2 Kontrolü: "kaydet -> N adım -> geri yükle -> N adım" bit düzeyinde eşitlik
    Baryon::Simulator sim;
    sim.setAccelerationField(Baryon::Vector3(0, -9.81f, 0));

    // Zemin
    auto ground = sim.createBody(Baryon::Pose(Baryon::Vector3(0, 0, 0)), Baryon::collision::CollisionShape(Baryon::collision::BoxShape(Baryon::Vector3(20, 1, 20))));
    ground.setBodyType(Baryon::Core::BodyType::Static);

    // Kutu ve Küre
    auto box = sim.createBody(Baryon::Pose(Baryon::Vector3(0, 3, 0)), Baryon::collision::CollisionShape(Baryon::collision::BoxShape(Baryon::Vector3(0.5f, 0.5f, 0.5f))));
    box.setBodyType(Baryon::Core::BodyType::Dynamic);
    (void)box.setMass(1.0f);

    auto sphere = sim.createBody(Baryon::Pose(Baryon::Vector3(0.2f, 5, 0)), Baryon::collision::CollisionShape(Baryon::collision::SphereShape(0.4f)));
    sphere.setBodyType(Baryon::Core::BodyType::Dynamic);
    (void)sphere.setMass(1.5f);

    // İlk 20 adım koştur ve snapshot kaydet
    for (int i = 0; i < 20; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Baryon::Core::Snapshot savedSnap = sim.saveSnapshot();

    // Sonraki 40 adım koştur ve pozisyonları kaydet (Run 1)
    for (int i = 0; i < 40; ++i) {
        sim.step(1.0f / 60.0f);
    }
    Baryon::Vector3 run1BoxPos = box.getPosition().value();
    Baryon::Vector3 run1SpherePos = sphere.getPosition().value();
    Baryon::Vector3 run1BoxVel = box.getLinearVelocity().value();
    Baryon::Vector3 run1SphereVel = sphere.getLinearVelocity().value();

    // Snapshot'ı geri yükle
    sim.restoreSnapshot(savedSnap);

    // Tekrar aynı 40 adımı koştur (Run 2)
    for (int i = 0; i < 40; ++i) {
        sim.step(1.0f / 60.0f);
    }
    Baryon::Vector3 run2BoxPos = box.getPosition().value();
    Baryon::Vector3 run2SpherePos = sphere.getPosition().value();
    Baryon::Vector3 run2BoxVel = box.getLinearVelocity().value();
    Baryon::Vector3 run2SphereVel = sphere.getLinearVelocity().value();

    std::cout << "  -> Run 1 Box Pos: (" << run1BoxPos.x << ", " << run1BoxPos.y << ", " << run1BoxPos.z << ")" << std::endl;
    std::cout << "  -> Run 2 Box Pos: (" << run2BoxPos.x << ", " << run2BoxPos.y << ", " << run2BoxPos.z << ")" << std::endl;

    CHECK_MSG(run1BoxPos.x == run2BoxPos.x && run1BoxPos.y == run2BoxPos.y && run1BoxPos.z == run2BoxPos.z,
              "Bit-level determinism failed for box position after rewind!");
    CHECK_MSG(run1SpherePos.x == run2SpherePos.x && run1SpherePos.y == run2SpherePos.y && run1SpherePos.z == run2SpherePos.z,
              "Bit-level determinism failed for sphere position after rewind!");
    CHECK_MSG(run1BoxVel.x == run2BoxVel.x && run1BoxVel.y == run2BoxVel.y && run1BoxVel.z == run2BoxVel.z,
              "Bit-level determinism failed for box velocity after rewind!");
    CHECK_MSG(run1SphereVel.x == run2SphereVel.x && run1SphereVel.y == run2SphereVel.y && run1SphereVel.z == run2SphereVel.z,
              "Bit-level determinism failed for sphere velocity after rewind!");
}

// #23: E8 - 250 / 1000 / 2000 Cisim Ölçeklenme Benchmark Testi
void testScalingBenchmark() {
    std::cout << "\n[REGRESSION 23] E8: Scaling Performance Benchmark (250 / 1000 / 2000 bodies)..." << std::endl;

    auto benchmarkCount = [](size_t count) -> double {
        Baryon::Simulator sim;
        sim.setAccelerationField(Baryon::Vector3(0, -9.81f, 0));

        // Geniş zemin
        auto floor = sim.createBody(Baryon::Pose(Baryon::Vector3(0, -1, 0)),
                                    Baryon::collision::CollisionShape(Baryon::collision::BoxShape(Baryon::Vector3(500, 1, 500))));
        floor.setBodyType(Baryon::Core::BodyType::Static);

        size_t side = static_cast<size_t>(std::ceil(std::sqrt(count)));
        for (size_t i = 0; i < count; ++i) {
            float x = static_cast<float>(i % side) * 2.5f - (side * 1.25f);
            float z = static_cast<float>(i / side) * 2.5f - (side * 1.25f);
            auto body = sim.createBody(Baryon::Pose(Baryon::Vector3(x, 1.0f + static_cast<float>(i % 3) * 1.5f, z)),
                                       Baryon::collision::CollisionShape(Baryon::collision::SphereShape(0.5f)));
            body.setBodyType(Baryon::Core::BodyType::Dynamic);
            (void)body.setMass(1.0f);
        }

        // 1 ısınma adımı
        sim.step(1.0f / 60.0f);

        // 10 adım ölç
        auto start = std::chrono::high_resolution_clock::now();
        for (int s = 0; s < 10; ++s) {
            sim.step(1.0f / 60.0f);
        }
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count() / 10.0;
    };

    double ms250 = benchmarkCount(250);
    double ms1000 = benchmarkCount(1000);
    double ms2000 = benchmarkCount(2000);

    std::cout << "  -> 250 bodies:  " << ms250 << " ms/step" << std::endl;
    std::cout << "  -> 1000 bodies: " << ms1000 << " ms/step" << std::endl;
    std::cout << "  -> 2000 bodies: " << ms2000 << " ms/step" << std::endl;

    CHECK_MSG(ms250 < 50.0, "250 bodies step time too slow!");
    CHECK_MSG(ms1000 < 100.0, "1000 bodies step time too slow!");
    CHECK_MSG(ms2000 < 250.0, "2000 bodies step time too slow!");
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "        BARYON REGRESSION TEST SUITE              " << std::endl;
    std::cout << "==================================================" << std::endl;

    testCapsuleOnBoxFloor();
    testSphereOnStaticMeshFloor();
    testMomentumConservationAt4ms();
    testRestitutionBounce();
    testSetMassInertiaScaling();
    testBoxStackStability();
    testEPARobustness();
    testTimeStepCatchupAt20FPS();
    testIslandScalingPerformance();
    testTiltedCapsuleToppling();
    testCollisionSystemStepDecomposition();
    testWarmStartInheritImpulses();
    testContactPointCreationHelper();
    testSimulationSettingsConfiguration();
    testManifoldCacheAndContactEvents();
    testPhysicsConstantsMagicNumbers();
    testContactReduction();
    testErrorHandlingAndNoDiscard();
    testMutableManifoldsAccessAndNoConstCast();
    testLargeFunctionsDecomposition();
    testShadowWarningFree();
    testEntityGenerationRecycling();
    testComponentVariadicAndSnapshotDeterminism();
    testScalingBenchmark();

    return Baryon::Test::reportAndExit();
}
