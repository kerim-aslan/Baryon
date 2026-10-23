/*
 * Baryon - A custom physics engine
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

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/FixedConstraint.hpp>
#include <Baryon/collision/DistanceConstraint.hpp>
#include <Baryon/collision/RevoluteConstraint.hpp>
#include <Baryon/collision/BallSocketConstraint.hpp>
#include <Baryon/collision/JointBreakListener.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief Test amaçlı eklem kırılma dinleyicisi implementasyonu.
 */
class TestBreakListener : public JointBreakListener {
public:
    int breakCount{0};
    ecs::Entity lastEntityA{0};
    ecs::Entity lastEntityB{0};
    float lastBreakImpulse{0.0f};

    void onJointBroken(ecs::Entity entityA, ecs::Entity entityB, float breakImpulse) override {
        ++breakCount;
        lastEntityA = entityA;
        lastEntityB = entityB;
        lastBreakImpulse = breakImpulse;
    }

    void reset() {
        breakCount = 0;
        lastEntityA = ecs::Entity{0};
        lastEntityB = ecs::Entity{0};
        lastBreakImpulse = 0.0f;
    }
};

} // namespace

// Test 1: Ağır Yük Altında Sabit Eklem Kırılması (Cantilever Weld Breakage)
void testCantileverWeldBreakage() {
    std::cout << "--- 1. Test: Agir Yuk Altinda Sabit Eklem Kirilmasi (FixedConstraint) ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    TestBreakListener listener;
    sim.setJointBreakListener(&listener);

    Pose poseA(Vector3(0.0f, 10.0f, 0.0f));
    Body anchor = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    Pose poseB(Vector3(1.5f, 10.0f, 0.0f));
    Body beam = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.5f, 0.1f, 0.1f))});
    (void)beam.setBodyType(Core::BodyType::Dynamic);
    (void)beam.setMass(1.0f); // 1 kg -> ~9.81 N agirlik

    sim.createFixedConstraint(anchor, beam);
    sim.setConstraintBreakForce<FixedConstraint>(anchor, 50.0f); // 50 N kirilma esigi

    const float dt = 1.0f / 60.0f;

    // Normal yuk altinda 60 adim calistir (eklem saglam kalmali)
    for (int i = 0; i < 60; ++i) {
        sim.step(dt);
    }

    CHECK(!sim.isConstraintBroken<FixedConstraint>(anchor));
    CHECK(listener.breakCount == 0);

    std::cout << "  Normal yuk altinda eklem saglam: F_reaksiyon < 50 N dogrulandi." << std::endl;

    // Asiri yuk uygula: 25 kg kutle (yaklasik 245 N agirlik > 50 N esik)
    (void)beam.setMass(25.0f);

    bool brokenDetected = false;
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
        if (sim.isConstraintBroken<FixedConstraint>(anchor)) {
            brokenDetected = true;
            break;
        }
    }

    CHECK(brokenDetected);
    CHECK(listener.breakCount == 1);
    CHECK(listener.lastEntityA == anchor.getEntity());
    CHECK(listener.lastEntityB == beam.getEntity());
    CHECK(listener.lastBreakImpulse > 0.0f);

    std::cout << "  Asiri yukte eklem basariyla kirildi. Reaksiyon itmesi: " 
              << listener.lastBreakImpulse << " N*s" << std::endl;

    // Kirilma sonrasi serbest dusme dogrulamasi
    float yBefore = beam.getPosition().value().y;
    for (int i = 0; i < 60; ++i) {
        sim.step(dt);
    }
    float yAfter = beam.getPosition().value().y;
    CHECK(yAfter < yBefore - 1.0f);

    std::cout << "  Kirilma sonrasi kiris serbest dusmeye gecti (y: " 
              << yBefore << " -> " << yAfter << " m)." << std::endl;
}

// Test 2: Cekme Halat/Cubuk Kirilmasi (Tensile DistanceConstraint Breakage)
void testTensileDistanceBreakage() {
    std::cout << "--- 2. Test: Cekme Halat Kirilmasi (DistanceConstraint) ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    bool callbackTriggered = false;
    sim.setJointBreakCallback([&](ecs::Entity a, ecs::Entity b, float impulse) {
        (void)a; (void)b; (void)impulse;
        callbackTriggered = true;
    });

    Pose poseA(Vector3(0.0f, 15.0f, 0.0f));
    Body ceiling = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)ceiling.setBodyType(Core::BodyType::Static);

    Pose poseB(Vector3(0.0f, 12.0f, 0.0f));
    Body weight = sim.createBody(poseB, CollisionShape{SphereShape(0.3f)});
    (void)weight.setBodyType(Core::BodyType::Dynamic);
    (void)weight.setMass(2.0f); // 2 kg -> ~19.6 N

    sim.createDistanceConstraint(ceiling, weight, 3.0f);
    sim.setConstraintBreakForce<DistanceConstraint>(ceiling, 100.0f); // 100 N esik

    const float dt = 1.0f / 60.0f;

    for (int i = 0; i < 60; ++i) {
        sim.step(dt);
    }

    CHECK(!sim.isConstraintBroken<DistanceConstraint>(ceiling));
    CHECK(!callbackTriggered);

    // Kopus yukune gec: Asiri kutle ekle (40 kg -> ~392 N > 100 N)
    (void)weight.setMass(40.0f);

    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
        if (callbackTriggered) break;
    }

    CHECK(callbackTriggered);
    CHECK(sim.isConstraintBroken<DistanceConstraint>(ceiling));

    std::cout << "  Halat yuksek cekme kuvveti altinda basariyla koptu." << std::endl;
}

// Test 3: Asiri Tork Altinda Mentese Kirilmasi (RevoluteConstraint Breakage)
void testRevoluteTorqueBreakage() {
    std::cout << "--- 3. Test: Asiri Tork Altinda Mentese Kirilmasi (RevoluteConstraint) ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Yercekimsiz saf tork testi

    TestBreakListener listener;
    sim.setJointBreakListener(&listener);

    Pose poseA(Vector3(0.0f, 5.0f, 0.0f));
    Body base = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)base.setBodyType(Core::BodyType::Static);

    Pose poseB(Vector3(1.0f, 5.0f, 0.0f));
    Body lever = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.5f, 0.1f, 0.1f))});
    (void)lever.setBodyType(Core::BodyType::Dynamic);
    (void)lever.setMass(1.0f);

    // Y ekseni etrafinda donen mentese ekle
    sim.createRevoluteConstraint(base, lever, 
                                Vector3(0.0f, 0.0f, 0.0f), 
                                Vector3(-1.0f, 0.0f, 0.0f), 
                                Vector3(0.0f, 1.0f, 0.0f),
                                Vector3(0.0f, 1.0f, 0.0f));

    // Kirilma torkunu 20 N*m olarak ayarla
    sim.setConstraintBreakForce<RevoluteConstraint>(base, 1e10f, 20.0f);

    const float dt = 1.0f / 60.0f;
    sim.step(dt);
    CHECK(!sim.isConstraintBroken<RevoluteConstraint>(base));

    // Mentese eksenine dik eksende buyuk bir acisal hiz uygulayarak kilitli eksene tork bindir
    (void)lever.setAngularVelocity(Vector3(100.0f, 0.0f, 0.0f)); // X ekseni etrafinda istenmeyen sert donme

    sim.step(dt);

    CHECK(sim.isConstraintBroken<RevoluteConstraint>(base));
    CHECK(listener.breakCount == 1);

    std::cout << "  Kilitli acisal serbestlikte asiri tork olustu ve mentese kirildi." << std::endl;
}

// Test 4: Esik Altinda Tam Mukavemet ve Butunluk (Unbroken Integrity)
void testUnbrokenIntegrity() {
    std::cout << "--- 4. Test: Esik Altinda Tam Mukavemet ve Butunluk ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    TestBreakListener listener;
    sim.setJointBreakListener(&listener);

    Pose poseA(Vector3(0.0f, 10.0f, 0.0f));
    Body anchor = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    Pose poseB(Vector3(0.0f, 8.0f, 0.0f));
    Body bob = sim.createBody(poseB, CollisionShape{SphereShape(0.3f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(5.0f); // 5 kg -> ~49.05 N

    sim.createBallSocketConstraint(anchor, bob, Vector3(0.0f, 10.0f, 0.0f));
    // Kirilma esigi 1000 N -> 49 N yuk altinda asla kirilmamali
    sim.setConstraintBreakForce<BallSocketConstraint>(anchor, 1000.0f);

    const float dt = 1.0f / 60.0f;
    const int totalSteps = 300; // 5 saniye

    for (int step = 0; step < totalSteps; ++step) {
        sim.step(dt);
        CHECK(!sim.isConstraintBroken<BallSocketConstraint>(anchor));
    }

    CHECK(listener.breakCount == 0);

    auto* c = sim.getConstraint<BallSocketConstraint>(anchor);
    CHECK(c != nullptr);
    float lastReactionForce = c->getReactionForce(dt);
    std::cout << "  5 saniye sonra eklem saglam. Son olculen reaksiyon kuvveti: " 
              << lastReactionForce << " N (Beklenen: ~49 N)." << std::endl;
    CHECK(std::abs(lastReactionForce - 49.05f) < 5.0f);
}

// Test 5: Kirilma Sonrasi Sayisal Kararlilik ve Ada Ayrilmasi (Post-Breakage Stability)
void testPostBreakageStability() {
    std::cout << "--- 5. Test: Kirilma Sonrasi Sayisal Kararlilik ve Ada Ayrilmasi ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // A (Statik) -> B (Dinamik) -> C (Dinamik) zinciri
    Pose poseA(Vector3(0.0f, 10.0f, 0.0f));
    Body bA = sim.createBody(poseA, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)bA.setBodyType(Core::BodyType::Static);

    Pose poseB(Vector3(1.0f, 10.0f, 0.0f));
    Body bB = sim.createBody(poseB, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)bB.setBodyType(Core::BodyType::Dynamic);
    (void)bB.setMass(1.0f);

    Pose poseC(Vector3(2.0f, 10.0f, 0.0f));
    Body bC = sim.createBody(poseC, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)bC.setBodyType(Core::BodyType::Dynamic);
    (void)bC.setMass(1.0f);

    // bA ve bB arasi FixedConstraint (kirilabilir)
    sim.createFixedConstraint(bA, bB);
    sim.setConstraintBreakForce<FixedConstraint>(bA, 25.0f); // 25 N

    // bB ve bC arasi FixedConstraint (kirilmaz: esik cok yuksek)
    sim.createFixedConstraint(bB, bC);
    sim.setConstraintBreakForce<FixedConstraint>(bB, 1e9f);

    const float dt = 1.0f / 60.0f;

    // Normal yuk altinda test
    for (int i = 0; i < 30; ++i) {
        sim.step(dt);
    }
    CHECK(!sim.isConstraintBroken<FixedConstraint>(bA));

    // Ekstra agirlik ekleyerek bA-bB eklemini kir
    (void)bC.setMass(10.0f); // Toplam yuk ~110 N > 25 N

    for (int i = 0; i < 10; ++i) {
        sim.step(dt);
        if (sim.isConstraintBroken<FixedConstraint>(bA)) break;
    }
    CHECK(sim.isConstraintBroken<FixedConstraint>(bA));
    CHECK(!sim.isConstraintBroken<FixedConstraint>(bB)); // bB-bC saglam kalmali!

    std::cout << "  A-B eklemi kirildi, B-C eklemi saglam kaldi." << std::endl;

    // 120 adim simule et (B ve C birlikte bagli dusmeli, hicbir NaN / Inf olmamali)
    for (int i = 0; i < 120; ++i) {
        sim.step(dt);
        Vector3 posB = bB.getPosition().value();
        Vector3 posC = bC.getPosition().value();

        CHECK(!std::isnan(posB.x) && !std::isnan(posB.y) && !std::isnan(posB.z));
        CHECK(!std::isinf(posB.x) && !std::isinf(posB.y) && !std::isinf(posB.z));
        CHECK(!std::isnan(posC.x) && !std::isnan(posC.y) && !std::isnan(posC.z));
        CHECK(!std::isinf(posC.x) && !std::isinf(posC.y) && !std::isinf(posC.z));

        // B ve C arasindaki bagli mesafe korunmali (~1.0 m)
        float distBC = (posC - posB).length();
        CHECK(std::abs(distBC - 1.0f) < 0.05f);
    }

    std::cout << "  Kirilma sonrasi B-C ikili kumesi kararliligini ve baglantisini korudu." << std::endl;
}

int main() {
    std::cout << "=== Baryon Joint Breakage Testleri Baslatiliyor ===" << std::endl;

    testCantileverWeldBreakage();
    testTensileDistanceBreakage();
    testRevoluteTorqueBreakage();
    testUnbrokenIntegrity();
    testPostBreakageStability();

    return reportAndExit();
}
