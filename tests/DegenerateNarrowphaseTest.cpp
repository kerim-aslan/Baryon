#include "TestCommon.hpp"
#include <Baryon/collision/NarrowPhase.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <Baryon/Simulator.hpp>
#include <cmath>
#include <iostream>

using namespace Baryon;
using namespace Baryon::collision;

static bool isFiniteVec(const Vector3& v) {
    return !std::isnan(v.x) && !std::isnan(v.y) && !std::isnan(v.z) &&
           !std::isinf(v.x) && !std::isinf(v.y) && !std::isinf(v.z);
}

static bool isValidNormal(const Vector3& n) {
    if (!isFiniteVec(n)) return false;
    float len = n.length();
    return std::abs(len - 1.0f) < 1e-2f; // Must be normalized unit vector
}

int main() {
    std::cout << "=== V3: Degenerate ve Teget Narrowphase / EPA Testi ===" << std::endl;

    // 1. Concentric Spheres (Full penetration at exactly same position)
    {
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(0, 0, 0), Quaternion::identity()};
        CollisionShape sphereA{SphereShape{1.0f}};
        CollisionShape sphereB{SphereShape{0.5f}};

        Simplex simplex;
        bool gjkHit = NarrowPhase::GJK(sphereA, poseA, sphereB, poseB, simplex);
        CHECK_MSG(gjkHit, "Concentric spheres GJK collision detect edilmeli");

        CollisionInfo info = NarrowPhase::EPA(simplex, sphereA, poseA, sphereB, poseB);
        CHECK_MSG(info.hasCollision, "Concentric spheres EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Concentric spheres contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Concentric spheres EPA normali birim vektor olmali");
        CHECK_MSG(info.penetration > 0.0f && !std::isnan(info.penetration), "Concentric spheres penetration > 0 olmali");
    }

    // 2. Concentric Boxes (Full penetration)
    {
        Pose poseA{Vector3(1, 2, 3), Quaternion::identity()};
        Pose poseB{Vector3(1, 2, 3), Quaternion::identity()};
        CollisionShape boxA{BoxShape{Vector3(1.0f, 1.0f, 1.0f)}};
        CollisionShape boxB{BoxShape{Vector3(0.5f, 0.5f, 0.5f)}};

        Simplex simplex;
        bool gjkHit = NarrowPhase::GJK(boxA, poseA, boxB, poseB, simplex);
        CHECK_MSG(gjkHit, "Concentric boxes GJK collision detect edilmeli");

        CollisionInfo info = NarrowPhase::EPA(simplex, boxA, poseA, boxB, poseB);
        CHECK_MSG(info.hasCollision, "Concentric boxes EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Concentric boxes contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Concentric boxes EPA normali birim vektor olmali");
        CHECK_MSG(info.penetration > 0.0f && !std::isnan(info.penetration), "Concentric boxes penetration > 0 olmali");
    }

    // 3. Concentric Capsule and Box
    {
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(0, 0, 0), Quaternion::identity()};
        CollisionShape capA{CapsuleShape{0.5f, 1.0f}};
        CollisionShape boxB{BoxShape{Vector3(0.5f, 0.5f, 0.5f)}};

        Simplex simplex;
        bool gjkHit = NarrowPhase::GJK(capA, poseA, boxB, poseB, simplex);
        CHECK_MSG(gjkHit, "Concentric capsule-box GJK detect edilmeli");

        CollisionInfo info = NarrowPhase::EPA(simplex, capA, poseA, boxB, poseB);
        CHECK_MSG(info.hasCollision, "Concentric capsule-box EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Concentric capsule-box contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Concentric capsule-box normali birim vektor olmali");
    }

    // 4. Capsule-Sphere tangent contact (exact surface touch)
    {
        // Capsule radius=0.5, height=1.0 along Y (segment y: -0.5 to +0.5).
        // Sphere radius=0.5 at x = 1.0 (tangent to capsule cylinder at x=0.5).
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(0.999f, 0, 0), Quaternion::identity()}; // 1mm penetration
        CollisionShape cap{CapsuleShape{0.5f, 1.0f}};
        CollisionShape sph{SphereShape{0.5f}};

        Simplex simplex;
        bool hit = NarrowPhase::GJK(cap, poseA, sph, poseB, simplex);
        CHECK_MSG(hit, "Tangent capsule-sphere (1mm pen) GJK collision detect edilmeli");
        CollisionInfo info = NarrowPhase::EPA(simplex, cap, poseA, sph, poseB);
        CHECK_MSG(info.hasCollision, "Tangent capsule-sphere EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Tangent capsule-sphere contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Tangent capsule-sphere normali birim vektor olmali");
        CHECK_MSG(info.normal.x > 0.9f, "Tangent capsule-sphere normali +X yonunde olmali");
    }

    // 5. Tangent at Capsule Tip (cap hemisphere touch)
    {
        // Capsule top at y = 0.5 + 0.5 = 1.0.
        // Sphere radius 0.5 centered at y = 1.499f (1mm penetration at the tip).
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(0, 1.499f, 0), Quaternion::identity()};
        CollisionShape cap{CapsuleShape{0.5f, 1.0f}};
        CollisionShape sph{SphereShape{0.5f}};

        Simplex simplex;
        bool hit = NarrowPhase::GJK(cap, poseA, sph, poseB, simplex);
        CHECK_MSG(hit, "Capsule tip tangent sphere GJK detect edilmeli");
        CollisionInfo info = NarrowPhase::EPA(simplex, cap, poseA, sph, poseB);
        CHECK_MSG(info.hasCollision, "Capsule tip tangent sphere EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Capsule tip tangent contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Capsule tip tangent normal birim vektor olmali");
        CHECK_MSG(info.normal.y > 0.9f, "Capsule tip tangent normal +Y yonunde olmali");
    }

    // 6. Hull-Box Corner-to-Corner tangent contact
    {
        // Box at (0,0,0) extent (1,1,1) -> corner at (1,1,1).
        // Convex hull box at (1.998, 1.998, 1.998) extent (1,1,1) -> corner at (0.998, 0.998, 0.998)
        std::vector<Vector3> verts = {
            {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1},
            {-1,-1, 1}, {1,-1, 1}, {1,1, 1}, {-1,1, 1}
        };
        CollisionShape boxA{BoxShape{Vector3(1, 1, 1)}};
        CollisionShape hullB{ConvexHullShape{verts}};

        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(1.998f, 1.998f, 1.998f), Quaternion::identity()};

        Simplex simplex;
        bool hit = NarrowPhase::GJK(boxA, poseA, hullB, poseB, simplex);
        CHECK_MSG(hit, "Corner-to-corner corner touch GJK detect edilmeli");
        CollisionInfo info = NarrowPhase::EPA(simplex, boxA, poseA, hullB, poseB);
        CHECK_MSG(info.hasCollision, "Corner-to-corner EPA hasCollision true olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Corner-to-corner contact point finite olmali");
        CHECK_MSG(isValidNormal(info.normal), "Corner-to-corner normal birim vektor olmali");
        CHECK_MSG(info.penetration > 0.0f, "Corner-to-corner penetration > 0 olmali");
    }

    // 7. Full step in Simulator with concentric and tangent bodies: no NaNs in velocities/positions
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, 0, 0));

        Pose pose1{Vector3(0, 0, 0), Quaternion::identity()};
        Pose pose2{Vector3(0, 0, 0), Quaternion::identity()};

        Body b1 = sim.createBody(pose1, CollisionShape(BoxShape{Vector3(1, 1, 1)}));
        Body b2 = sim.createBody(pose2, CollisionShape(BoxShape{Vector3(0.5f, 0.5f, 0.5f)}));

        b1.setBodyType(Core::BodyType::Dynamic);
        b2.setBodyType(Core::BodyType::Dynamic);

        // Step simulation for 5 steps
        for (int i = 0; i < 5; ++i) {
            sim.step(1.0f / 60.0f);
        }

        Vector3 p1 = b1.getPosition().value();
        Vector3 p2 = b2.getPosition().value();
        Vector3 v1 = b1.getLinearVelocity().value();
        Vector3 v2 = b2.getLinearVelocity().value();

        CHECK_MSG(isFiniteVec(p1), "b1 position finite olmali (NaN olmamali)");
        CHECK_MSG(isFiniteVec(p2), "b2 position finite olmali (NaN olmamali)");
        CHECK_MSG(isFiniteVec(v1), "b1 velocity finite olmali (NaN olmamali)");
        CHECK_MSG(isFiniteVec(v2), "b2 velocity finite olmali (NaN olmamali)");
    }

    // 8. Simplex of size < 4 passed to EPA (robustness against early termination)
    {
        Simplex simplex;
        simplex.push_front(Vector3(1, 0, 0));
        simplex.push_front(Vector3(0, 1, 0));
        CollisionShape sA{SphereShape{1.0f}};
        CollisionShape sB{SphereShape{1.0f}};
        Pose pA{Vector3(0,0,0), Quaternion::identity()};
        Pose pB{Vector3(0.5f,0,0), Quaternion::identity()};
        CollisionInfo info = NarrowPhase::EPA(simplex, sA, pA, sB, pB);
        // Should not crash and if collision returned, normal must be valid unit vector
        if (info.hasCollision) {
            CHECK_MSG(isValidNormal(info.normal), "Simplex < 4 durumunda normal gecerli olmali");
        }
    }

    // 9. Degenerate flat / coplanar simplex in EPA
    {
        Simplex simplex;
        simplex.assign({Vector3(0,0,0), Vector3(1,0,0), Vector3(0,1,0), Vector3(1,1,0)});
        CollisionShape sA{SphereShape{1.0f}};
        CollisionShape sB{SphereShape{1.0f}};
        Pose pA{Vector3(0,0,0), Quaternion::identity()};
        Pose pB{Vector3(0.5f,0,0), Quaternion::identity()};
        CollisionInfo info = NarrowPhase::EPA(simplex, sA, pA, sB, pB);
        if (info.hasCollision) {
            CHECK_MSG(isValidNormal(info.normal), "Degenerate coplanar simplex normali gecerli birim vektor olmali");
            CHECK_MSG(isFiniteVec(info.contactPoint), "Degenerate coplanar simplex contact point finite olmali");
        }
    }

    // 10. Flat 2D Convex Hull vs Box
    {
        std::vector<Vector3> flatQuad = {
            {-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}
        };
        CollisionShape flatHull{ConvexHullShape{flatQuad}};
        CollisionShape box{BoxShape{Vector3(1, 1, 1)}};
        Pose pA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose pB{Vector3(0, 0, 0.5f), Quaternion::identity()};
        Simplex simplex;
        bool hit = NarrowPhase::GJK(flatHull, pA, box, pB, simplex);
        if (hit) {
            CollisionInfo info = NarrowPhase::EPA(simplex, flatHull, pA, box, pB);
            CHECK_MSG(isValidNormal(info.normal), "Flat hull EPA normali birim vektor olmali");
            CHECK_MSG(isFiniteVec(info.contactPoint), "Flat hull contact point finite olmali");
        }
    }

    // 11. Exact Tangent Contact (zero penetration)
    {
        // Spheres touching at exactly 1 point: dist = 2.0, r1=1, r2=1
        Pose poseA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose poseB{Vector3(2.0f, 0, 0), Quaternion::identity()};
        CollisionShape sphereA{SphereShape{1.0f}};
        CollisionShape sphereB{SphereShape{1.0f}};

        Simplex simplex;
        bool hit = NarrowPhase::GJK(sphereA, poseA, sphereB, poseB, simplex);
        if (hit) {
            CollisionInfo info = NarrowPhase::EPA(simplex, sphereA, poseA, sphereB, poseB);
            CHECK_MSG(isValidNormal(info.normal), "Exact tangent EPA normali birim vektor olmali");
            CHECK_MSG(isFiniteVec(info.contactPoint), "Exact tangent contact point finite olmali");
        }
    }

    // 12. Identical coincident shapes with zero relative offset
    {
        Pose poseA{Vector3(5.0f, 5.0f, 5.0f), Quaternion::identity()};
        Pose poseB{Vector3(5.0f, 5.0f, 5.0f), Quaternion::identity()};
        CollisionShape boxA{BoxShape{Vector3(1, 1, 1)}};
        CollisionShape boxB{BoxShape{Vector3(1, 1, 1)}};

        Simplex simplex;
        bool hit = NarrowPhase::GJK(boxA, poseA, boxB, poseB, simplex);
        CHECK_MSG(hit, "Identical coincident boxes GJK detect edilmeli");
        CollisionInfo info = NarrowPhase::EPA(simplex, boxA, poseA, boxB, poseB);
        CHECK_MSG(info.hasCollision, "Identical coincident boxes EPA hasCollision true olmali");
        CHECK_MSG(isValidNormal(info.normal), "Identical coincident boxes normali birim vektor olmali");
        CHECK_MSG(isFiniteVec(info.contactPoint), "Identical coincident boxes contact point finite olmali");
        CHECK_MSG(info.penetration > 0.0f, "Identical coincident boxes penetration > 0 olmali");
    }

    // 13. Faz 2.4: Zero and extremely small cylinder/cone dimensions
    {
        Pose pA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose pB{Vector3(0, 0.5f, 0), Quaternion::identity()};

        CollisionShape zeroCyl{CylinderShape{0.0f, 0.0f}};
        CollisionShape tinyCone{ConeShape{1e-7f, 1e-7f}};

        AABB aabbCyl = zeroCyl.computeLocalAABB();
        AABB aabbCone = tinyCone.computeLocalAABB();
        CHECK(isFiniteVec(aabbCyl.minBounds) && isFiniteVec(aabbCyl.maxBounds));
        CHECK(isFiniteVec(aabbCone.minBounds) && isFiniteVec(aabbCone.maxBounds));

        Simplex simplex;
        (void)NarrowPhase::GJK(zeroCyl, pA, tinyCone, pB, simplex);

        Ray ray(Vector3(0, 2.0f, 0), Vector3(0, -1.0f, 0), 10.0f);
        RaycastHit hitCyl, hitCone;
        (void)NarrowPhase::raycast(zeroCyl, pA, ray, hitCyl);
        (void)NarrowPhase::raycast(tinyCone, pB, ray, hitCone);
    }

    // 14. Faz 2.4: Extremely large dimensions (1e6)
    {
        Pose pA{Vector3(0, 0, 0), Quaternion::identity()};
        Pose pB{Vector3(0, 100.0f, 0), Quaternion::identity()};

        CollisionShape hugeCyl{CylinderShape{1e6f, 1e6f}};
        CollisionShape hugeCone{ConeShape{1e6f, 1e6f}};

        AABB aabbCyl = hugeCyl.computeLocalAABB();
        AABB aabbCone = hugeCone.computeLocalAABB();
        CHECK(isFiniteVec(aabbCyl.minBounds) && isFiniteVec(aabbCyl.maxBounds));
        CHECK(isFiniteVec(aabbCone.minBounds) && isFiniteVec(aabbCone.maxBounds));

        Simplex simplex;
        bool gjkHit = NarrowPhase::GJK(hugeCyl, pA, hugeCone, pB, simplex);
        if (gjkHit) {
            CollisionInfo info = NarrowPhase::EPA(simplex, hugeCyl, pA, hugeCone, pB);
            CHECK(isFiniteVec(info.normal));
        }
    }

    // 15. Faz 2.4: NaN inputs and Simulator stability
    {
        Simulator sim;
        std::vector<float> nanHeights = {NAN, 0.0f, INFINITY, -INFINITY};
        HeightfieldShape hf(2, 2, 1.0f, nanHeights, true);
        Body terrain = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{hf});
        terrain.setBodyType(Core::BodyType::Static);

        Body cyl = sim.createBody(Pose(Vector3(0, 1.0f, 0)), CollisionShape{CylinderShape{0.5f, 0.5f}});
        cyl.setBodyType(Core::BodyType::Dynamic);

        Body cone = sim.createBody(Pose(Vector3(1.0f, 1.0f, 0)), CollisionShape{ConeShape{0.5f, 1.0f}});
        cone.setBodyType(Core::BodyType::Dynamic);

        // Simulator stepping should never throw or produce NaN positions
        for (int i = 0; i < 10; ++i) {
            sim.step(1.0f / 60.0f);
        }

        Vector3 posCyl = cyl.getPosition().value();
        Vector3 posCone = cone.getPosition().value();
        CHECK(isFiniteVec(posCyl));
        CHECK(isFiniteVec(posCone));
    }

    reportAndExit();
}
