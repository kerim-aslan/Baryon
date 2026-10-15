#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

void testBoxInertia() {
    std::cout << "--- V2: Box Inertia Verification ---" << std::endl;
    Simulator sim;
    // Yarım boyutlar 0.5, 1.0, 1.5 -> tam boyutlar x=1.0, y=2.0, z=3.0
    Vector3 halfExtents(0.5f, 1.0f, 1.5f);
    Body box = sim.createBody(Pose(), CollisionShape{BoxShape(halfExtents)});
    (void)box.setBodyType(Core::BodyType::Dynamic);

    Matrix3x3 I = box.getLocalInertiaTensor().value();
    float m = 1.0f; // varsayılan kütle

    Vector3 size = halfExtents * 2.0f;
    float expectedIx = (m / 12.0f) * (size.y * size.y + size.z * size.z);
    float expectedIy = (m / 12.0f) * (size.x * size.x + size.z * size.z);
    float expectedIz = (m / 12.0f) * (size.x * size.x + size.y * size.y);

    std::cout << "Box Ix: " << I[0].x << " (Expected: " << expectedIx << ")" << std::endl;
    std::cout << "Box Iy: " << I[1].y << " (Expected: " << expectedIy << ")" << std::endl;
    std::cout << "Box Iz: " << I[2].z << " (Expected: " << expectedIz << ")" << std::endl;

    CHECK_MSG(std::abs(I[0].x - expectedIx) / expectedIx <= 0.02f, "Box Ix +-2% icinde olmali");
    CHECK_MSG(std::abs(I[1].y - expectedIy) / expectedIy <= 0.02f, "Box Iy +-2% icinde olmali");
    CHECK_MSG(std::abs(I[2].z - expectedIz) / expectedIz <= 0.02f, "Box Iz +-2% icinde olmali");
}

void testCapsuleInertia() {
    std::cout << "\n--- V2: Capsule Inertia Verification ---" << std::endl;
    Simulator sim;
    float r = 0.5f;
    float h = 2.0f;
    Body capsule = sim.createBody(Pose(), CollisionShape{CapsuleShape(r, h)});
    (void)capsule.setBodyType(Core::BodyType::Dynamic);

    Matrix3x3 I = capsule.getLocalInertiaTensor().value();
    float m = 1.0f;

    // Analitik formüller:
    // m_c = m * h / (h + 4*r/3)
    // m_h = m - m_c
    // I_y = m_c * r^2 / 2 + (2/5) * m_h * r^2
    // I_x = I_z = m_c * (r^2/4 + h^2/12) + m_h * (2*r^2/5 + h^2/4 + 3*h*r/8)
    float mc = m * h / (h + (4.0f / 3.0f) * r);
    float mh = m - mc;
    float expectedIy = mc * (r * r) * 0.5f + (2.0f / 5.0f) * mh * (r * r);
    float expectedIx = mc * ((r * r) * 0.25f + (h * h) / 12.0f) +
                       mh * ((2.0f / 5.0f) * (r * r) + (h * h) * 0.25f + (3.0f / 8.0f) * h * r);
    float expectedIz = expectedIx;

    float errIx = std::abs(I[0].x - expectedIx) / expectedIx;
    float errIy = std::abs(I[1].y - expectedIy) / expectedIy;
    float errIz = std::abs(I[2].z - expectedIz) / expectedIz;

    std::cout << "Capsule Ix: " << I[0].x << " (Expected: " << expectedIx << ", Hata: %" << (errIx * 100.0f) << ")" << std::endl;
    std::cout << "Capsule Iy: " << I[1].y << " (Expected: " << expectedIy << ", Hata: %" << (errIy * 100.0f) << ")" << std::endl;
    std::cout << "Capsule Iz: " << I[2].z << " (Expected: " << expectedIz << ", Hata: %" << (errIz * 100.0f) << ")" << std::endl;

    CHECK_MSG(errIx <= 0.02f, "Capsule Ix +-2% icinde olmali");
    CHECK_MSG(errIy <= 0.02f, "Capsule Iy +-2% icinde olmali");
    CHECK_MSG(errIz <= 0.02f, "Capsule Iz +-2% icinde olmali");
}

int main() {
    testBoxInertia();
    testCapsuleInertia();
    return reportAndExit();
}
