#include "TestCommon.hpp"
#include <Baryon/collision/NarrowPhase.hpp>
#include <Baryon/collision/Ray.hpp>
#include <iostream>

using namespace Baryon;
using namespace Baryon::collision;

int main() {
    std::cout << "=== C4: NarrowPhase Refactor Unit Test ===" << std::endl;

    // 1. Raycast Testleri
    // 1.1 Sphere Raycast
    {
        SphereShape sphere(1.0f);
        Pose pose{Vector3(0, 0, 5), Quaternion::identity()};
        Ray ray{Vector3(0, 0, 0), Vector3(0, 0, 1), 10.0f};
        RaycastHit hit;
        bool hasHit = NarrowPhase::raycast(CollisionShape(sphere), pose, ray, hit);
        CHECK_MSG(hasHit, "Kure isin kesismeli");
        CHECK_MSG(std::abs(hit.distance - 4.0f) < 1e-3f, "Kureye olan mesafe 4.0m olmali");
        CHECK_MSG(hit.normal.z < -0.99f, "Kure carpisma normali -Z olmali");
    }

    // 1.2 Box Raycast
    {
        BoxShape box(Vector3(1, 1, 1)); // 2x2x2
        Pose pose{Vector3(0, 0, 5), Quaternion::identity()};
        Ray ray{Vector3(0, 0, 0), Vector3(0, 0, 1), 10.0f};
        RaycastHit hit;
        bool hasHit = NarrowPhase::raycast(CollisionShape(box), pose, ray, hit);
        CHECK_MSG(hasHit, "Kutu isin kesismeli");
        CHECK_MSG(std::abs(hit.distance - 4.0f) < 1e-3f, "Kutuya olan mesafe 4.0m olmali");
        CHECK_MSG(hit.normal.z < -0.99f, "Kutu carpisma normali -Z olmali");
    }

    // 1.3 Capsule Raycast
    {
        CapsuleShape cap(0.5f, 2.0f); // radius 0.5, height 2.0 (Y ekseninde)
        Pose pose{Vector3(0, 0, 5), Quaternion::identity()};
        Ray ray{Vector3(0, 0, 0), Vector3(0, 0, 1), 10.0f};
        RaycastHit hit;
        bool hasHit = NarrowPhase::raycast(CollisionShape(cap), pose, ray, hit);
        CHECK_MSG(hasHit, "Kapsul isin kesismeli");
        CHECK_MSG(std::abs(hit.distance - 4.5f) < 1e-3f, "Kapsul govdesine mesafe 4.5m olmali");
    }

    // 1.4 Triangle Raycast
    {
        TriangleShape tri{Vector3(-1, -1, 5), Vector3(1, -1, 5), Vector3(0, 1, 5)};
        Pose pose{Vector3(0, 0, 0), Quaternion::identity()};
        Ray ray{Vector3(0, 0, 0), Vector3(0, 0, 1), 10.0f};
        RaycastHit hit;
        bool hasHit = NarrowPhase::raycast(CollisionShape(tri), pose, ray, hit);
        CHECK_MSG(hasHit, "Ucgen isin kesismeli");
        CHECK_MSG(std::abs(hit.distance - 5.0f) < 1e-3f, "Ucgense olan mesafe 5.0m olmali");
    }

    // 2. testBoxBox Testleri
    // 2.1 Yuzey-Yuzey Temasi
    {
        BoxShape boxA(Vector3(1, 1, 1));
        BoxShape boxB(Vector3(1, 1, 1));
        Pose poseA{Vector3(0, 1.9f, 0), Quaternion::identity()};
        Pose poseB{Vector3(0, 0, 0), Quaternion::identity()};
        ContactManifold manifold{ecs::Entity(1), ecs::Entity(2)};
        bool col = NarrowPhase::testBoxBox(boxA, poseA, boxB, poseB, manifold);
        CHECK_MSG(col, "Kutular temas etmeli");
        CHECK_MSG(manifold.contactCount == 4, "Yuzey-yuzey temasi 4 temas noktasi uretmeli");
        CHECK_MSG(std::abs(manifold.penetration - 0.1f) < 1e-2f, "Penetrasyon ~0.1m olmali");
    }

    // 2.2 Ayrık Kutular (Temas Yok)
    {
        BoxShape boxA(Vector3(1, 1, 1));
        BoxShape boxB(Vector3(1, 1, 1));
        Pose poseA{Vector3(0, 5.0f, 0), Quaternion::identity()};
        Pose poseB{Vector3(0, 0, 0), Quaternion::identity()};
        ContactManifold manifold{ecs::Entity(1), ecs::Entity(2)};
        bool col = NarrowPhase::testBoxBox(boxA, poseA, boxB, poseB, manifold);
        CHECK_MSG(!col, "Ayrik kutular carpismamali");
    }

    // 3. GJK ve EPA Testleri
    {
        SphereShape spA(1.0f);
        SphereShape spB(1.0f);
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(1.5f, 0, 0), Quaternion::identity()};
        Simplex simplex;
        bool gjkHit = NarrowPhase::GJK(CollisionShape(spA), poseA, CollisionShape(spB), poseB, simplex);
        CHECK_MSG(gjkHit, "Kesisan kureler icin GJK dogru donmeli");

        CollisionInfo info = NarrowPhase::EPA(simplex, CollisionShape(spA), poseA, CollisionShape(spB), poseB);
        CHECK_MSG(info.hasCollision, "EPA carpisma bildirmeli");
        CHECK_MSG(std::abs(info.penetration - 0.5f) < 0.05f, "EPA penetrasyonu ~0.5m olmali");
    }

    return reportAndExit();
}
