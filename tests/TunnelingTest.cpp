/*
 * Baryon - A custom physics engine
 * Automated Tunneling Verification Test
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Baryon/Simulator.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Baryon;

void testHighSpeedBulletVsThinWall() {
    std::cout << "[TEST 1] High-Speed Bullet (100 m/s) vs Thin Wall (0.05m)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0)); // Yerçekimsiz saf hız testi

    // 1. İnce Duvar (Kalınlık: 0.05m, x = 5.0'da)
    Pose wallPose;
    wallPose.position = Vector3(5.0f, 0.0f, 0.0f);
    wallPose.orientation = Quaternion::identity();
    collision::BoxShape wallBox(Vector3(0.025f, 2.0f, 2.0f));
    Body wall = sim.createBody(wallPose, collision::CollisionShape(wallBox));
    wall.setBodyType(Core::BodyType::Static);

    // 2. Yüksek Hızlı Mermi (Yarıçap: 0.1m, Hız: 100 m/s, x = 0'dan başlıyor)
    Pose bulletPose;
    bulletPose.position = Vector3(0.0f, 0.0f, 0.0f);
    bulletPose.orientation = Quaternion::identity();
    collision::SphereShape bulletSphere(0.1f);
    Body bullet = sim.createBody(bulletPose, collision::CollisionShape(bulletSphere));
    bullet.setBodyType(Core::BodyType::Dynamic);
    bullet.setCCD(true);
    bullet.setLinearVelocity(Vector3(100.0f, 0.0f, 0.0f));

    // 10 adım simüle et (1/60s * 10 = ~0.16s, 100 m/s ile normalde 16 metre uzağa giderdi)
    for (int step = 0; step < 10; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = *bullet.getPosition();
        // Mermi duvarın (x=5.0) arkasına ASLA geçmemelidir!
        if (pos.x > 5.05f) {
            std::cerr << "FAIL: Bullet tunneled through wall! Pos: " << pos.x << std::endl;
            assert(false && "Bullet tunneled through thin wall!");
        }
    }

    Vector3 finalPos = *bullet.getPosition();
    std::cout << "SUCCESS: Bullet stopped at wall! Final X: " << finalPos.x 
              << " (Wall is at x=5.0)" << std::endl;
    assert(finalPos.x <= 5.05f);
}

void testRotatedThinWall() {
    std::cout << "[TEST 2] High-Speed Box (60 m/s) vs 45-degree Rotated Thin Wall..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // 45 derece Y ekseninde döndürülmüş ince duvar (x=4.0)
    Pose wallPose;
    wallPose.position = Vector3(4.0f, 0.0f, 0.0f);
    wallPose.orientation = Quaternion(Vector3(0, 1, 0), 3.14159f / 4.0f);
    collision::BoxShape wallBox(Vector3(0.03f, 2.0f, 2.0f));
    Body wall = sim.createBody(wallPose, collision::CollisionShape(wallBox));
    wall.setBodyType(Core::BodyType::Static);

    // Hızlı Kutu
    Pose boxPose;
    boxPose.position = Vector3(0.0f, 0.0f, 0.0f);
    collision::BoxShape bulletBox(Vector3(0.15f, 0.15f, 0.15f));
    Body box = sim.createBody(boxPose, collision::CollisionShape(bulletBox));
    box.setBodyType(Core::BodyType::Dynamic);
    box.setCCD(true);
    box.setLinearVelocity(Vector3(60.0f, 0.0f, 0.0f));

    for (int step = 0; step < 10; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = *box.getPosition();
        Vector3 vel = *box.getLinearVelocity();
        std::cout << "  Step " << step << " | Pos: (" << pos.x << ", " << pos.y << ", " << pos.z << ") | Vel: (" << vel.x << ", " << vel.y << ", " << vel.z << ")" << std::endl;
        // Dönmüş duvarın yüzey denklemi: n = (cos 45, 0, -sin 45) = (0.707, 0, -0.707).
        // (p - wallPos).dot(n) değeri duvarın arkasına geçip geçmediğini söyler.
        Vector3 wallNormal = wallPose.orientation * Vector3(1, 0, 0); // veya yerel normal
        float planeDist = (pos - wallPose.position).dot(wallNormal);
        if (planeDist > 0.3f) {
            std::cerr << "FAIL: Box penetrated behind rotated wall! Plane dist: " << planeDist << std::endl;
            assert(false && "Box penetrated behind rotated wall!");
        }
    }

    Vector3 finalPos = *box.getPosition();
    Vector3 wallNormal = wallPose.orientation * Vector3(1, 0, 0);
    float planeDist = (finalPos - wallPose.position).dot(wallNormal);
    std::cout << "SUCCESS: Box safely slid along rotated wall without penetrating! Final Pos: (" 
              << finalPos.x << ", " << finalPos.z << "), Plane dist: " << planeDist << std::endl;
    assert(planeDist <= 0.2f);
}

void testDynamicHeadOnCollision() {
    std::cout << "[TEST 3] Two Dynamic Bodies Head-On (Relative 100 m/s)..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // A: x = -3.0, hız = +50 m/s
    Pose poseA;
    poseA.position = Vector3(-3.0f, 0.0f, 0.0f);
    collision::SphereShape sphereA(0.2f);
    Body bodyA = sim.createBody(poseA, collision::CollisionShape(sphereA));
    bodyA.setBodyType(Core::BodyType::Dynamic);
    bodyA.setCCD(true);
    bodyA.setLinearVelocity(Vector3(50.0f, 0.0f, 0.0f));

    // B: x = +3.0, hız = -50 m/s
    Pose poseB;
    poseB.position = Vector3(3.0f, 0.0f, 0.0f);
    collision::SphereShape sphereB(0.2f);
    Body bodyB = sim.createBody(poseB, collision::CollisionShape(sphereB));
    bodyB.setBodyType(Core::BodyType::Dynamic);
    bodyB.setCCD(true);
    bodyB.setLinearVelocity(Vector3(-50.0f, 0.0f, 0.0f));

    for (int step = 0; step < 10; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 posA = *bodyA.getPosition();
        Vector3 posB = *bodyB.getPosition();
        // A her zaman B'nin solunda kalmalıdır, birbirlerinin içinden geçip yer değiştirmemelidirler!
        if (posA.x > posB.x) {
            std::cerr << "FAIL: Bodies swapped positions (tunneling)! posA: " << posA.x << " posB: " << posB.x << std::endl;
            assert(false && "Dynamic bodies tunneled through each other!");
        }
    }

    std::cout << "SUCCESS: Dynamic bodies collided without tunneling! PosA: " << (*bodyA.getPosition()).x 
              << " PosB: " << (*bodyB.getPosition()).x << std::endl;
}

void testStaticMeshFloor() {
    std::cout << "[TEST 4] High-Speed Sphere (-80 m/s) vs Static Mesh Floor..." << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, 0, 0));

    // Y = 0'da iki üçgenden oluşan yatay zemin
    std::vector<Vector3> verts = {
        Vector3(-10.0f, 0.0f, -10.0f),
        Vector3( 10.0f, 0.0f, -10.0f),
        Vector3( 10.0f, 0.0f,  10.0f),
        Vector3(-10.0f, 0.0f,  10.0f)
    };
    std::vector<uint32_t> indices = {
        0, 1, 2,
        0, 2, 3
    };
    auto meshPtr = std::make_shared<collision::StaticMesh>(verts, indices);
    collision::StaticMeshShape meshShape(meshPtr);

    Pose floorPose;
    floorPose.position = Vector3(0.0f, 0.0f, 0.0f);
    Body floor = sim.createBody(floorPose, collision::CollisionShape(meshShape));
    floor.setBodyType(Core::BodyType::Static);

    // Y = 5.0'dan aşağıya -80 m/s ile fırlatılan küre
    Pose spherePose;
    spherePose.position = Vector3(0.0f, 5.0f, 0.0f);
    collision::SphereShape sphereShape(0.2f);
    Body fallingBody = sim.createBody(spherePose, collision::CollisionShape(sphereShape));
    fallingBody.setBodyType(Core::BodyType::Dynamic);
    fallingBody.setCCD(true);
    fallingBody.setLinearVelocity(Vector3(0.0f, -80.0f, 0.0f));

    for (int step = 0; step < 10; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = *fallingBody.getPosition();
        if (pos.y < -0.1f) {
            std::cerr << "FAIL: Falling body tunneled through static mesh floor! Y: " << pos.y << std::endl;
            assert(false && "Fell through static mesh floor!");
        }
    }

    Vector3 finalPos = *fallingBody.getPosition();
    std::cout << "SUCCESS: Sphere stopped on Static Mesh Floor! Final Y: " << finalPos.y << std::endl;
    assert(finalPos.y >= -0.05f);
}

int main() {
    std::cout << "=== BARYON TUNNELING TEST SUITE ===" << std::endl;
    testHighSpeedBulletVsThinWall();
    testRotatedThinWall();
    testDynamicHeadOnCollision();
    testStaticMeshFloor();
    std::cout << "=== ALL TUNNELING TESTS PASSED (0 TUNNELING) ===" << std::endl;
    return 0;
}
