#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

// 1. ConvexHull küp zeminde dinlenme testi (y ≈ 0.5)
void testConvexHullCubeOnGround() {
    std::cout << "\n--- V4.1: ConvexHull Cube on Ground Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(50, 0.5f, 50))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // 1x1x1 birim küpün 8 köşesi
    std::vector<Vector3> cubeVertices = {
        Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, -0.5f, -0.5f),
        Vector3(0.5f,  0.5f, -0.5f), Vector3(-0.5f,  0.5f, -0.5f),
        Vector3(-0.5f, -0.5f,  0.5f), Vector3(0.5f, -0.5f,  0.5f),
        Vector3(0.5f,  0.5f,  0.5f), Vector3(-0.5f,  0.5f,  0.5f)
    };
    Body hull = sim.createBody(Pose(Vector3(0, 2.0f, 0)), CollisionShape{ConvexHullShape(cubeVertices)});
    (void)hull.setBodyType(Core::BodyType::Dynamic);

    for (int i = 0; i < 200; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 pos = hull.getPosition().value();
    std::cout << "ConvexHull Cube resting y: " << pos.y << " (Expected ~0.50)" << std::endl;
    CHECK_MSG(pos.y >= 0.47f && pos.y <= 0.53f, "ConvexHull kup zemin ustunde y in [0.47, 0.53] olmali");
}

// 2. Döndürülmüş Statik Mesh çarpışma testi
void testRotatedMeshCollision() {
    std::cout << "\n--- V4.2: Rotated Static Mesh Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // 20x20 iki üçgenden oluşan düzlem mesh
    std::vector<Vector3> verts = {
        Vector3(-10, 0, -10), Vector3(10, 0, -10), Vector3(10, 0, 10),
        Vector3(-10, 0, -10), Vector3(10, 0, 10), Vector3(-10, 0, 10)
    };
    std::vector<uint32_t> indices = {0, 2, 1, 3, 5, 4};
    auto meshPtr = std::make_shared<StaticMesh>(verts, indices);

    // Mesh'i 30 derece Z ekseni etrafında döndür
    float angle = 30.0f * (3.14159265f / 180.0f);
    Quaternion rot(Vector3(0, 0, 1), angle);
    Pose meshPose(Vector3(0, 0, 0), rot);

    Body meshBody = sim.createBody(meshPose, CollisionShape{StaticMeshShape(meshPtr)});
    (void)meshBody.setBodyType(Core::BodyType::Static);

    // Küre mesh yüzeyi üzerine düşsün
    Body sphere = sim.createBody(Pose(Vector3(2.0f, 3.0f, 0.0f)), CollisionShape{SphereShape(0.5f)});
    (void)sphere.setBodyType(Core::BodyType::Dynamic);

    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 pos = sphere.getPosition().value();
    Vector3 vel = sphere.getLinearVelocity().value();
    std::cout << "Sphere on rotated mesh pos: (" << pos.x << ", " << pos.y << ", " << pos.z 
              << ") vel: (" << vel.x << ", " << vel.y << ", " << vel.z << ")" << std::endl;
    // Küre eğimli zeminden aşağı kaymalı ve zemini delip düşmemeli
    CHECK_MSG(pos.y > 0.0f, "Kure dondurulmus mesh'i delip asagi dusmemeli");
    CHECK_MSG(vel.x < -0.5f, "Kure egimli mesh uzerinde asagi dogru kaymali");
}

// 3. A/B sırası değişmiş kapsül-zemin simetrisi
void testCapsuleGroundEntityOrderSymmetry() {
    std::cout << "\n--- V4.3: Capsule Ground Entity Order Symmetry ---" << std::endl;

    auto runSimWithOrder = [](bool groundFirst) -> float {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));

        if (groundFirst) {
            Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(20, 0.5f, 20))});
            (void)ground.setBodyType(Core::BodyType::Static);
            Body cap = sim.createBody(Pose(Vector3(0, 2.0f, 0)), CollisionShape{CapsuleShape(0.3f, 1.0f)});
            (void)cap.setBodyType(Core::BodyType::Dynamic);

            for (int i = 0; i < 180; ++i) {
                sim.step(1.0f / 60.0f);
            }
            return cap.getPosition().value().y;
        } else {
            Body cap = sim.createBody(Pose(Vector3(0, 2.0f, 0)), CollisionShape{CapsuleShape(0.3f, 1.0f)});
            (void)cap.setBodyType(Core::BodyType::Dynamic);
            Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(20, 0.5f, 20))});
            (void)ground.setBodyType(Core::BodyType::Static);

            for (int i = 0; i < 180; ++i) {
                sim.step(1.0f / 60.0f);
            }
            return cap.getPosition().value().y;
        }
    };

    float yGroundFirst = runSimWithOrder(true);
    float yCapFirst = runSimWithOrder(false);

    std::cout << "Resting Y (Ground first): " << yGroundFirst << std::endl;
    std::cout << "Resting Y (Capsule first): " << yCapFirst << std::endl;

    CHECK_MSG(std::abs(yGroundFirst - yCapFirst) < 0.01f, "Entity sirasi degisse bile dinlenme konumu ayni olmali");
    // r=0.3, h=1.0 -> toplam yarım boy = 0.8
    CHECK_MSG(yGroundFirst >= 0.77f && yGroundFirst <= 0.83f, "Kapsul zeminde y in [0.77, 0.83] olmali");
}

// 4. Dikdörtgen kutu ve kapsül açısal hız (omega) yanıtı
void testAngularVelocityResponse() {
    std::cout << "\n--- V4.4: Rectangular Box & Capsule Omega Response ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // Kutu: yarım boyutlar (0.5, 1.0, 1.5)
    Body box = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{BoxShape(Vector3(0.5f, 1.0f, 1.5f))});
    (void)box.setBodyType(Core::BodyType::Dynamic);
    (void)box.setMass(2.0f);

    Vector3 angImpulse(1.0f, 2.0f, 3.0f);
    (void)box.applyAngularImpulse(angImpulse);

    Vector3 wBox = box.getAngularVelocity().value();
    Matrix3x3 invIBox = box.getLocalInertiaTensor().value().getInverse();
    Vector3 expectedWBox = invIBox * angImpulse;

    std::cout << "Box w: (" << wBox.x << ", " << wBox.y << ", " << wBox.z << ")" << std::endl;
    std::cout << "Expected w: (" << expectedWBox.x << ", " << expectedWBox.y << ", " << expectedWBox.z << ")" << std::endl;

    CHECK_MSG((wBox - expectedWBox).length() < 1e-4f, "Box acisal hiz yaniti invI * angImpulse ile eslesmeli");

    // Kapsül: r=0.4, h=1.5
    Body cap = sim.createBody(Pose(Vector3(5, 0, 0)), CollisionShape{CapsuleShape(0.4f, 1.5f)});
    (void)cap.setBodyType(Core::BodyType::Dynamic);
    (void)cap.setMass(3.0f);

    (void)cap.applyAngularImpulse(angImpulse);
    Vector3 wCap = cap.getAngularVelocity().value();
    Matrix3x3 invICap = cap.getLocalInertiaTensor().value().getInverse();
    Vector3 expectedWCap = invICap * angImpulse;

    std::cout << "Capsule w: (" << wCap.x << ", " << wCap.y << ", " << wCap.z << ")" << std::endl;
    std::cout << "Expected w: (" << expectedWCap.x << ", " << expectedWCap.y << ", " << expectedWCap.z << ")" << std::endl;

    CHECK_MSG((wCap - expectedWCap).length() < 1e-4f, "Kapsul acisal hiz yaniti invI * angImpulse ile eslesmeli");
}

int main() {
    testConvexHullCubeOnGround();
    testRotatedMeshCollision();
    testCapsuleGroundEntityOrderSymmetry();
    testAngularVelocityResponse();
    return reportAndExit();
}
