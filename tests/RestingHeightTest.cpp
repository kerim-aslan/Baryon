#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <string>

using namespace Baryon;
using namespace Baryon::collision;

enum class DynamicType {
    Sphere,
    Box,
    UprightCapsule,
    HorizontalCapsule,
    ConvexHullCube
};

enum class GroundType {
    BoxGround,
    MeshGround
};

struct RestingResult {
    std::string name;
    float restingY;
    float idealY;
    float errorCm;
    bool passed;
};

RestingResult testPairRestingHeight(DynamicType dType, GroundType gType, const std::string& name, float idealY) {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // 1. Zemin oluştur (y = 0 üst yüzey)
    if (gType == GroundType::BoxGround) {
        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(50, 0.5f, 50))});
        (void)ground.setBodyType(Core::BodyType::Static);
    } else {
        std::vector<Vector3> verts = {
            Vector3(-20, 0, -20), Vector3(20, 0, -20), Vector3(20, 0, 20),
            Vector3(-20, 0, -20), Vector3(20, 0, 20), Vector3(-20, 0, 20)
        };
        std::vector<uint32_t> indices = {0, 2, 1, 3, 5, 4}; // Upward normal
        auto meshPtr = std::make_shared<StaticMesh>(verts, indices);
        Body ground = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{StaticMeshShape(meshPtr)});
        (void)ground.setBodyType(Core::BodyType::Static);
    }

    // 2. Dinamik cisim oluştur
    auto createDynamic = [&]() -> Body {
        if (dType == DynamicType::Sphere) {
            return sim.createBody(Pose(Vector3(0, 1.5f, 0)), CollisionShape{SphereShape(0.5f)});
        } else if (dType == DynamicType::Box) {
            return sim.createBody(Pose(Vector3(0, 1.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        } else if (dType == DynamicType::UprightCapsule) {
            return sim.createBody(Pose(Vector3(0, 1.5f, 0)), CollisionShape{CapsuleShape(0.3f, 1.0f)});
        } else if (dType == DynamicType::HorizontalCapsule) {
            Quaternion rot(Vector3(0, 0, 1), 3.14159265f * 0.5f); // 90 derece yatır
            return sim.createBody(Pose(Vector3(0, 1.5f, 0), rot), CollisionShape{CapsuleShape(0.3f, 1.0f)});
        } else {
            std::vector<Vector3> cubeVertices = {
                Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, -0.5f, -0.5f),
                Vector3(0.5f,  0.5f, -0.5f), Vector3(-0.5f,  0.5f, -0.5f),
                Vector3(-0.5f, -0.5f,  0.5f), Vector3(0.5f, -0.5f,  0.5f),
                Vector3(0.5f,  0.5f,  0.5f), Vector3(-0.5f,  0.5f,  0.5f)
            };
            return sim.createBody(Pose(Vector3(0, 1.5f, 0)), CollisionShape{ConvexHullShape(cubeVertices)});
        }
    };
    Body body = createDynamic();
    (void)body.setBodyType(Core::BodyType::Dynamic);

    // 3. 240 adım simüle et (tam dinlenme)
    for (int step = 0; step < 240; ++step) {
        sim.step(1.0f / 60.0f);
    }

    float restingY = body.getPosition().value().y;
    float diff = restingY - idealY;
    float errorCm = diff * 100.0f;
    bool passed = (std::abs(diff) <= 0.015f); // ideal +- 1.5 cm

    return {name, restingY, idealY, errorCm, passed};
}

int main() {
    std::cout << "=== Baryon N3 Resting Height Diagnostic Test ===" << std::endl;

    std::vector<std::tuple<DynamicType, GroundType, std::string, float>> testCases = {
        {DynamicType::Sphere,            GroundType::BoxGround,  "Sphere on BoxGround",           0.50f},
        {DynamicType::Sphere,            GroundType::MeshGround, "Sphere on MeshGround",          0.50f},
        {DynamicType::Box,               GroundType::BoxGround,  "Box on BoxGround",              0.50f},
        {DynamicType::Box,               GroundType::MeshGround, "Box on MeshGround",             0.50f},
        {DynamicType::UprightCapsule,    GroundType::BoxGround,  "UprightCapsule on BoxGround",   0.80f},
        {DynamicType::UprightCapsule,    GroundType::MeshGround, "UprightCapsule on MeshGround",  0.80f},
        {DynamicType::HorizontalCapsule, GroundType::BoxGround,  "HorizontalCapsule on BoxGround",0.30f},
        {DynamicType::HorizontalCapsule, GroundType::MeshGround, "HorizontalCapsule on MeshGround",0.30f},
        {DynamicType::ConvexHullCube,    GroundType::BoxGround,  "ConvexHull on BoxGround",       0.50f},
        {DynamicType::ConvexHullCube,    GroundType::MeshGround, "ConvexHull on MeshGround",      0.50f}
    };

    for (const auto& [dType, gType, name, ideal] : testCases) {
        RestingResult res = testPairRestingHeight(dType, gType, name, ideal);
        std::cout << "[" << (res.passed ? "PASS" : "FAIL") << "] " 
                  << res.name << " -> Resting Y: " << res.restingY 
                  << " (Ideal: " << res.idealY << ", Diff: " << res.errorCm << " cm)" << std::endl;
        CHECK_MSG(res.passed, (res.name + " dinlenme yuksekligi ideal +- 1.5 cm olmali (olculen diff: " + std::to_string(res.errorCm) + " cm)").c_str());
    }

    return reportAndExit();
}
