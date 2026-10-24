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

/**
 * @file DistanceDriftTest.cpp
 * @brief Mesafe kısıtlarında pozisyonel kayma (drift) iyileştirme testleri (Faz 3.10).
 * @details Split-impulse tabanlı pozisyon projeksiyonu sayesinde sarkaç ve mesafe
 *          kısıtlarındaki sayısal sapmanın < 0.003 m (hedef) seviyesine indirildiğini doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief Dikey asılı sarkaçta uzun vadeli (10 saniye / 600 adım) drift testi.
 */
void testHangingPendulumZeroDrift() {
    std::cout << "[DistanceDriftTest] Dikey Asili Sarkac Drift Testi (600 adim)...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Pose anchorPose(Vector3(0.0f, 10.0f, 0.0f));
    Body anchor = sim.createBody(anchorPose, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    const float targetDist = 2.0f;
    Pose bobPose(Vector3(0.0f, 10.0f - targetDist, 0.0f));
    Body bob = sim.createBody(bobPose, CollisionShape{SphereShape(0.3f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(5.0f);

    sim.createDistanceConstraint(anchor, bob, targetDist);

    const float dt = 1.0f / 60.0f;
    float maxDrift = 0.0f;

    for (int step = 0; step < 600; ++step) {
        sim.step(dt);
        Vector3 pA = anchor.getPosition().value();
        Vector3 pB = bob.getPosition().value();
        float dist = (pB - pA).length();
        float drift = std::abs(dist - targetDist);
        if (drift > maxDrift) {
            maxDrift = drift;
        }
    }

    std::cout << "  -> En buyuk drift: " << maxDrift << " m (Kriter: < 0.003 m)\n";
    CHECK(maxDrift < 0.003f);
}

/**
 * @brief Serbest salınan dinamik sarkaç drift testi.
 */
void testSwingingPendulumDrift() {
    std::cout << "[DistanceDriftTest] Salinan Dinamik Sarkac Drift Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    Pose anchorPose(Vector3(0.0f, 10.0f, 0.0f));
    Body anchor = sim.createBody(anchorPose, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    const float targetDist = 2.0f;
    // 45 derece açıyla yerleştir
    float angle = 0.785398f; // ~45 derece
    float initX = targetDist * std::sin(angle);
    float initY = 10.0f - targetDist * std::cos(angle);

    Pose bobPose(Vector3(initX, initY, 0.0f));
    Body bob = sim.createBody(bobPose, CollisionShape{SphereShape(0.3f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(3.0f);

    sim.createDistanceConstraint(anchor, bob, targetDist);

    const float dt = 1.0f / 60.0f;
    float maxDrift = 0.0f;

    for (int step = 0; step < 300; ++step) {
        sim.step(dt);
        Vector3 pA = anchor.getPosition().value();
        Vector3 pB = bob.getPosition().value();
        float dist = (pB - pA).length();
        float drift = std::abs(dist - targetDist);
        if (drift > maxDrift) {
            maxDrift = drift;
        }
    }

    std::cout << "  -> Salinim sirasinda en buyuk drift: " << maxDrift << " m (Kriter: < 0.003 m)\n";
    CHECK(maxDrift < 0.003f);
}

/**
 * @brief 5 elemanlı mesafe zincirinde (halat) kümülatif drift testi.
 */
void testMultiLinkDistanceChain() {
    std::cout << "[DistanceDriftTest] Cok Halkali Mesafe Zinciri Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    const int linkCount = 5;
    const float segmentDist = 1.0f;

    Pose rootPose(Vector3(0.0f, 15.0f, 0.0f));
    Body prevBody = sim.createBody(rootPose, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)prevBody.setBodyType(Core::BodyType::Static);

    std::vector<Body> links;
    for (int i = 1; i <= linkCount; ++i) {
        Pose linkPose(Vector3(0.0f, 15.0f - static_cast<float>(i) * segmentDist, 0.0f));
        Body link = sim.createBody(linkPose, CollisionShape{SphereShape(0.2f)});
        (void)link.setBodyType(Core::BodyType::Dynamic);
        (void)link.setMass(2.0f);

        sim.createDistanceConstraint(prevBody, link, segmentDist);
        links.push_back(link);
        prevBody = link;
    }

    const float dt = 1.0f / 60.0f;
    for (int step = 0; step < 200; ++step) {
        sim.step(dt);
    }

    // Toplam zincir uzunluğu = linkCount * segmentDist = 5.0 m
    Vector3 rootPos = Vector3(0.0f, 15.0f, 0.0f);
    Vector3 tipPos = links.back().getPosition().value();
    float totalMeasured = (rootPos - tipPos).length();
    float totalExpected = static_cast<float>(linkCount) * segmentDist;
    float cumulativeDrift = std::abs(totalMeasured - totalExpected);

    std::cout << "  -> 5 halkali zincir toplam sapmasi: " << cumulativeDrift << " m (Kriter: < 0.01 m)\n";
    CHECK(cumulativeDrift < 0.01f);
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "Baryon Fizik Motoru - Mesafe Drift Testleri (Faz 3.10)\n";
    std::cout << "========================================\n";

    testHangingPendulumZeroDrift();
    testSwingingPendulumDrift();
    testMultiLinkDistanceChain();

    std::cout << "Tum Mesafe Drift (Faz 3.10) testleri basariyla gecti!\n";
    return 0;
}
