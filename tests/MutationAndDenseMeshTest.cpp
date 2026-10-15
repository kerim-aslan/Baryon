#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

// 1. Yoğun mesh (>= 100 üçgen) oluşturucu
static std::shared_ptr<StaticMesh> createDensePlaneMesh(int gridW = 10, int gridH = 10, float size = 10.0f) {
    std::vector<Vector3> verts;
    std::vector<uint32_t> indices;

    float halfSize = size * 0.5f;
    float dx = size / static_cast<float>(gridW);
    float dz = size / static_cast<float>(gridH);

    for (int j = 0; j <= gridH; ++j) {
        float z = -halfSize + static_cast<float>(j) * dz;
        for (int i = 0; i <= gridW; ++i) {
            float x = -halfSize + static_cast<float>(i) * dx;
            verts.push_back(Vector3(x, 0.0f, z));
        }
    }

    for (int j = 0; j < gridH; ++j) {
        for (int i = 0; i < gridW; ++i) {
            uint32_t row1 = static_cast<uint32_t>(j * (gridW + 1));
            uint32_t row2 = static_cast<uint32_t>((j + 1) * (gridW + 1));

            uint32_t v0 = row1 + static_cast<uint32_t>(i);
            uint32_t v1 = row1 + static_cast<uint32_t>(i + 1);
            uint32_t v2 = row2 + static_cast<uint32_t>(i + 1);
            uint32_t v3 = row2 + static_cast<uint32_t>(i);

            // Upward CCW triangles (Y+ normal)
            indices.push_back(v0);
            indices.push_back(v2);
            indices.push_back(v1);

            indices.push_back(v0);
            indices.push_back(v3);
            indices.push_back(v2);
        }
    }

    return std::make_shared<StaticMesh>(verts, indices);
}

int main() {
    std::cout << "=== V9: Yogun Mesh Manifold ve Mutasyon Kontrol Testi ===" << std::endl;

    // TEST 1: Yoğun mesh (200 üçgen >= 100) üstünde 4 temaslı ve geniş alanlı manifold testi
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, 0, 0)); // Saf temas geometrisi

        auto meshPtr = createDensePlaneMesh(10, 10, 10.0f); // 10x10 grid = 200 üçgen

        Body meshBody = sim.createBody(Pose{Vector3(0, 0, 0), Quaternion::identity()},
                                       CollisionShape{StaticMeshShape(meshPtr)});
        (void)meshBody.setBodyType(Core::BodyType::Static);

        // 1x1x1 kutu zemine 1cm penetrasyonla konuluyor
        Body boxBody = sim.createBody(Pose{Vector3(0, 0.49f, 0), Quaternion::identity()},
                                      CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)boxBody.setBodyType(Core::BodyType::Dynamic);

        // 1 adım simüle et (dar faz ve manifold üretimini tetikle)
        sim.step(1.0f / 60.0f);

        const auto& manifolds = sim.getManifolds();
        CHECK_MSG(!manifolds.empty(), "Kutu ve mesh arasinda manifold uretilmeli");

        if (!manifolds.empty()) {
            const auto& m = manifolds.begin()->second;

            CHECK_MSG(m.contactCount == 4, "Yogun mesh uzerindeki kutu 4 temas noktasi uretmeli");
            CHECK_MSG(m.normal.y > 0.95f, "Manifold normali +Y yonunde olmali");

            // 4 temas noktasının X ve Z yayılımını hesapla (geniş taban desteği)
            float minX = 1e9f, maxX = -1e9f;
            float minZ = 1e9f, maxZ = -1e9f;

            for (uint32_t i = 0; i < m.contactCount; ++i) {
                const auto& cp = m.contacts[i];
                if (cp.worldPosition.x < minX) minX = cp.worldPosition.x;
                if (cp.worldPosition.x > maxX) maxX = cp.worldPosition.x;
                if (cp.worldPosition.z < minZ) minZ = cp.worldPosition.z;
                if (cp.worldPosition.z > maxZ) maxZ = cp.worldPosition.z;
            }

            float widthX = maxX - minX;
            float widthZ = maxZ - minZ;
            float area = widthX * widthZ;

            std::cout << "  [DenseMesh] contacts: " << m.contactCount
                      << " widthX: " << widthX << " widthZ: " << widthZ
                      << " area: " << area << std::endl;

            CHECK_MSG(widthX > 0.8f, "Temas alani X genisligi > 0.8m olmali");
            CHECK_MSG(widthZ > 0.8f, "Temas alani Z genisligi > 0.8m olmali");
            CHECK_MSG(area > 0.6f, "Temas destek poligonu genis alana sahip olmali (> 0.6 m2)");
        }
    }

    // TEST 2: inheritImpulses / Warm-Starting Doğrulama Testi
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));

        Body ground = sim.createBody(Pose{Vector3(0, -0.5f, 0), Quaternion::identity()},
                                     CollisionShape{BoxShape(Vector3(10, 0.5f, 10))});
        (void)ground.setBodyType(Core::BodyType::Static);

        Body box = sim.createBody(Pose{Vector3(0, 0.5f, 0), Quaternion::identity()},
                                  CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);

        // 10 adım koşturarak normal impuls birikmesini sağla
        for (int i = 0; i < 10; ++i) {
            sim.step(1.0f / 60.0f);
        }

        const auto& manifolds1 = sim.getManifolds();
        CHECK_MSG(!manifolds1.empty(), "Kutu-zemin manifoldu var olmali");

        float initialNormalImpulse = 0.0f;
        if (!manifolds1.empty()) {
            initialNormalImpulse = manifolds1.begin()->second.contacts[0].normalImpulse;
        }

        CHECK_MSG(initialNormalImpulse > 0.01f,
                  "10 adim sonra yercekimi dengeleme impulsu birikmis olmali");

        // 1 adım daha ilerle ve yeni manifoldun bu impulsu devraldığını (warm starting) doğrula
        sim.step(1.0f / 60.0f);

        const auto& manifolds2 = sim.getManifolds();
        float inheritedImpulse = 0.0f;
        if (!manifolds2.empty()) {
            inheritedImpulse = manifolds2.begin()->second.contacts[0].normalImpulse;
        }

        std::cout << "  [WarmStarting] initial impulse: " << initialNormalImpulse
                  << " -> inherited impulse: " << inheritedImpulse << std::endl;

        CHECK_MSG(inheritedImpulse > 0.01f,
                  "Sonraki adimda impuls miras alinmis olmali (inheritImpulses calismali)");
    }

    // TEST 3: ContactPoint local vs world tutarlılığı
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, 0, 0));

        Pose poseA{Vector3(1, 2, 3), Quaternion(Vector3(0, 1, 0), 0.5f)};
        Pose poseB{Vector3(1.98f, 2, 3), Quaternion::identity()};

        Body bA = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        Body bB = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)bA.setBodyType(Core::BodyType::Dynamic);
        (void)bB.setBodyType(Core::BodyType::Dynamic);

        sim.step(1.0f / 60.0f);

        const auto& manifolds = sim.getManifolds();
        CHECK_MSG(!manifolds.empty(), "Kutular arasi temas bulunmali");

        if (!manifolds.empty()) {
            const auto& m = manifolds.begin()->second;
            for (uint32_t i = 0; i < m.contactCount; ++i) {
                const auto& cp = m.contacts[i];
                Vector3 recomputedWorldA = poseA.position + (poseA.orientation * cp.localPointA);
                float err = (recomputedWorldA - cp.worldPosition).length();
                CHECK_MSG(err < 1e-3f, "ContactPoint localPointA dunya konumuyla birebir uyusmali");
            }
        }
    }

    return reportAndExit();
}
