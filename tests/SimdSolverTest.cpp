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
#include <Baryon/collision/SimdContactSolver.hpp>
#include <Baryon/collision/ContactManifold.hpp>
#include <Baryon/Core/CoreComponents.hpp>
#include <Baryon/math/Matrix3x3.hpp>
#include <Baryon/math/Vector3.hpp>

#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;
using namespace Baryon::collision::simd;

namespace {

void testNormalImpulseEquivalence() {
    std::cout << "[SimdSolverTest] 1. Kontak Normal Itmesi Dogrulugu (SIMD vs Skaler)..." << std::endl;

    ContactPoint cpScalar;
    cpScalar.penetration = 0.05f;
    cpScalar.velocityBias = 0.8f;
    cpScalar.normalImpulse = 0.0f;

    ContactPoint cpSimd = cpScalar;

    Vector3 rA(0.2f, -0.5f, 0.1f);
    Vector3 rB(-0.1f, 0.5f, -0.3f);
    Vector3 n = Vector3(0.1f, 0.98f, -0.1f).getNormalized();

    Vector3 invMassA(0.5f, 0.5f, 0.5f);
    Vector3 invMassB(0.25f, 0.25f, 0.25f);

    Matrix3x3 invIA(
        1.2f, 0.1f, 0.0f,
        0.1f, 1.5f, 0.05f,
        0.0f, 0.05f, 0.9f
    );
    Matrix3x3 invIB(
        0.8f, 0.0f, 0.0f,
        0.0f, 0.7f, 0.0f,
        0.0f, 0.0f, 0.6f
    );

    Core::Motion motionA_scalar;
    motionA_scalar.linearVelocity = Vector3(1.0f, -2.5f, 0.5f);
    motionA_scalar.angularVelocity = Vector3(0.2f, 0.1f, -0.4f);

    Core::Motion motionB_scalar;
    motionB_scalar.linearVelocity = Vector3(-0.5f, 0.5f, -0.2f);
    motionB_scalar.angularVelocity = Vector3(-0.1f, 0.3f, 0.2f);

    Core::Motion motionA_simd = motionA_scalar;
    Core::Motion motionB_simd = motionB_scalar;

    solveContactNormalScalar(cpScalar, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA_scalar, motionB_scalar);
    solveContactNormalSIMD(cpSimd, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA_simd, motionB_simd);

    CHECK(cpScalar.normalImpulse > 0.0f);
    CHECK(std::abs(cpSimd.normalImpulse - cpScalar.normalImpulse) < 1e-5f);

    float diffLinA = (motionA_simd.linearVelocity - motionA_scalar.linearVelocity).length();
    float diffAngA = (motionA_simd.angularVelocity - motionA_scalar.angularVelocity).length();
    float diffLinB = (motionB_simd.linearVelocity - motionB_scalar.linearVelocity).length();
    float diffAngB = (motionB_simd.angularVelocity - motionB_scalar.angularVelocity).length();

    CHECK(diffLinA < 1e-5f);
    CHECK(diffAngA < 1e-5f);
    CHECK(diffLinB < 1e-5f);
    CHECK(diffAngB < 1e-5f);

    std::cout << "  -> Normal itme: scalar=" << cpScalar.normalImpulse
              << ", simd=" << cpSimd.normalImpulse
              << ", delta=" << std::abs(cpSimd.normalImpulse - cpScalar.normalImpulse)
              << " (Tolerans < 1e-5 OK)" << std::endl;
}

void testFrictionImpulseEquivalence() {
    std::cout << "[SimdSolverTest] 2. Kontak Sürtünme Itmesi Dogrulugu (Cift Teget SIMD vs Skaler)..." << std::endl;

    ContactPoint cpScalar;
    cpScalar.normalImpulse = 15.0f; // Mevcut normal itme
    cpScalar.tangentImpulse1 = 0.0f;
    cpScalar.tangentImpulse2 = 0.0f;

    ContactPoint cpSimd = cpScalar;

    Vector3 rA(0.3f, -0.5f, 0.2f);
    Vector3 rB(-0.2f, 0.5f, -0.1f);
    Vector3 n(0.0f, 1.0f, 0.0f);
    Vector3 t1(1.0f, 0.0f, 0.0f);
    Vector3 t2(0.0f, 0.0f, 1.0f);
    float friction = 0.5f; // maxFriction = 7.5

    Vector3 invMassA(0.4f, 0.4f, 0.4f);
    Vector3 invMassB(0.2f, 0.2f, 0.2f);

    Matrix3x3 invIA(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    Matrix3x3 invIB(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.5f);

    Core::Motion motionA_scalar;
    motionA_scalar.linearVelocity = Vector3(4.0f, 0.0f, -3.0f);
    motionA_scalar.angularVelocity = Vector3(0.5f, 1.0f, -0.5f);

    Core::Motion motionB_scalar;
    motionB_scalar.linearVelocity = Vector3(-1.0f, 0.0f, 1.0f);
    motionB_scalar.angularVelocity = Vector3(0.0f, -0.2f, 0.0f);

    Core::Motion motionA_simd = motionA_scalar;
    Core::Motion motionB_simd = motionB_scalar;

    solveContactFrictionScalar(cpScalar, rA, rB, t1, t2, friction, invMassA, invMassB, invIA, invIB, motionA_scalar, motionB_scalar);
    solveContactFrictionSIMD(cpSimd, rA, rB, t1, t2, friction, invMassA, invMassB, invIA, invIB, motionA_simd, motionB_simd);

    CHECK(std::abs(cpSimd.tangentImpulse1 - cpScalar.tangentImpulse1) < 1e-5f);
    CHECK(std::abs(cpSimd.tangentImpulse2 - cpScalar.tangentImpulse2) < 1e-5f);

    float diffLinA = (motionA_simd.linearVelocity - motionA_scalar.linearVelocity).length();
    float diffAngA = (motionA_simd.angularVelocity - motionA_scalar.angularVelocity).length();
    float diffLinB = (motionB_simd.linearVelocity - motionB_scalar.linearVelocity).length();
    float diffAngB = (motionB_simd.angularVelocity - motionB_scalar.angularVelocity).length();

    CHECK(diffLinA < 1e-5f);
    CHECK(diffAngA < 1e-5f);
    CHECK(diffLinB < 1e-5f);
    CHECK(diffAngB < 1e-5f);

    std::cout << "  -> Teget 1 itme: scalar=" << cpScalar.tangentImpulse1 << ", simd=" << cpSimd.tangentImpulse1
              << " | Teget 2: scalar=" << cpScalar.tangentImpulse2 << ", simd=" << cpSimd.tangentImpulse2
              << " (Tolerans < 1e-5 OK)" << std::endl;
}

void testMultiPointManifoldIterations() {
    std::cout << "[SimdSolverTest] 3. 4 Noktali Manifold ve Coklu Iterasyon (15 Iterasyon)..." << std::endl;

    ContactManifold manifoldScalar(ecs::Entity{1}, ecs::Entity{2});
    manifoldScalar.contactCount = 4;
    manifoldScalar.normal = Vector3(0.0f, 1.0f, 0.0f);
    manifoldScalar.computeTangents();

    // 4 köşe temas noktası
    float corners[4][2] = {{-0.5f, -0.5f}, {0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.5f}};
    for (int i = 0; i < 4; ++i) {
        manifoldScalar.contacts[i].localPointA = Vector3(corners[i][0], -0.5f, corners[i][1]);
        manifoldScalar.contacts[i].localPointB = Vector3(corners[i][0], 0.5f, corners[i][1]);
        manifoldScalar.contacts[i].penetration = 0.01f;
        manifoldScalar.contacts[i].velocityBias = 0.1f;
        manifoldScalar.contacts[i].normalImpulse = 0.0f;
        manifoldScalar.contacts[i].tangentImpulse1 = 0.0f;
        manifoldScalar.contacts[i].tangentImpulse2 = 0.0f;
    }

    ContactManifold manifoldSimd = manifoldScalar;

    Vector3 invMassA(1.0f, 1.0f, 1.0f);
    Vector3 invMassB(0.0f, 0.0f, 0.0f); // Statik zemin
    Matrix3x3 invIA(2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 2.0f);
    Matrix3x3 invIB(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    Core::Motion motionA_scalar;
    motionA_scalar.linearVelocity = Vector3(0.5f, -2.0f, -0.3f);
    motionA_scalar.angularVelocity = Vector3(0.1f, -0.2f, 0.3f);

    Core::Motion motionB_scalar; // Statik zemin

    Core::Motion motionA_simd = motionA_scalar;
    Core::Motion motionB_simd = motionB_scalar;

    float friction = 0.4f;

    // 15 ardışık iterasyon
    for (int it = 0; it < 15; ++it) {
        // Skaler çözüm
        for (uint32_t c = 0; c < 4; ++c) {
            Vector3 rA = manifoldScalar.contacts[c].localPointA;
            Vector3 rB = manifoldScalar.contacts[c].localPointB;
            solveContactNormalScalar(manifoldScalar.contacts[c], rA, rB, manifoldScalar.normal,
                                     invMassA, invMassB, invIA, invIB, motionA_scalar, motionB_scalar);
            solveContactFrictionScalar(manifoldScalar.contacts[c], rA, rB, manifoldScalar.tangent1, manifoldScalar.tangent2,
                                       friction, invMassA, invMassB, invIA, invIB, motionA_scalar, motionB_scalar);
        }

        // SIMD çözüm
        for (uint32_t c = 0; c < 4; ++c) {
            Vector3 rA = manifoldSimd.contacts[c].localPointA;
            Vector3 rB = manifoldSimd.contacts[c].localPointB;
            solveContactNormalSIMD(manifoldSimd.contacts[c], rA, rB, manifoldSimd.normal,
                                   invMassA, invMassB, invIA, invIB, motionA_simd, motionB_simd);
            solveContactFrictionSIMD(manifoldSimd.contacts[c], rA, rB, manifoldSimd.tangent1, manifoldSimd.tangent2,
                                     friction, invMassA, invMassB, invIA, invIB, motionA_simd, motionB_simd);
        }
    }

    for (int i = 0; i < 4; ++i) {
        float diffN = std::abs(manifoldSimd.contacts[i].normalImpulse - manifoldScalar.contacts[i].normalImpulse);
        float diffT1 = std::abs(manifoldSimd.contacts[i].tangentImpulse1 - manifoldScalar.contacts[i].tangentImpulse1);
        float diffT2 = std::abs(manifoldSimd.contacts[i].tangentImpulse2 - manifoldScalar.contacts[i].tangentImpulse2);

        CHECK(diffN < 1e-5f);
        CHECK(diffT1 < 1e-5f);
        CHECK(diffT2 < 1e-5f);
    }

    float diffLin = (motionA_simd.linearVelocity - motionA_scalar.linearVelocity).length();
    float diffAng = (motionA_simd.angularVelocity - motionA_scalar.angularVelocity).length();

    CHECK(diffLin < 1e-5f);
    CHECK(diffAng < 1e-5f);

    std::cout << "  -> 15 iterasyon sonrasi: diffLin=" << diffLin << ", diffAng=" << diffAng
              << " (Tolerans < 1e-5 OK)" << std::endl;
}

void testLockedAxesEquivalence() {
    std::cout << "[SimdSolverTest] 4. Eksen Kilitleme (DOF Lock) SIMD Dogrulugu..." << std::endl;

    ContactPoint cpScalar;
    cpScalar.normalImpulse = 0.0f;
    ContactPoint cpSimd = cpScalar;

    Vector3 rA(0.0f, -0.5f, 0.0f);
    Vector3 rB(0.0f, 0.5f, 0.0f);
    Vector3 n(0.0f, 1.0f, 0.0f);

    // LinearZ ve AngularX / AngularY kilitli (2D fizigi)
    uint8_t lockedAxes = Core::LockAxis::LinearZ | Core::LockAxis::AngularX | Core::LockAxis::AngularY;
    Vector3 invMassA = detail::computeInverseMassVector(1.0f, lockedAxes);
    Vector3 invMassB(0.0f, 0.0f, 0.0f);

    Matrix3x3 invIA(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f); // Sadece Z serbest
    Matrix3x3 invIB(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    Core::Motion motionA_scalar;
    motionA_scalar.linearVelocity = Vector3(0.0f, -3.0f, 0.0f);
    Core::Motion motionB_scalar;

    Core::Motion motionA_simd = motionA_scalar;
    Core::Motion motionB_simd = motionB_scalar;

    solveContactNormalScalar(cpScalar, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA_scalar, motionB_scalar);
    solveContactNormalSIMD(cpSimd, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA_simd, motionB_simd);

    CHECK(std::abs(cpSimd.normalImpulse - cpScalar.normalImpulse) < 1e-5f);
    CHECK(std::abs(motionA_simd.linearVelocity.z) < 1e-6f);
    CHECK(std::abs(motionA_simd.angularVelocity.x) < 1e-6f);
    CHECK(std::abs(motionA_simd.angularVelocity.y) < 1e-6f);

    std::cout << "  -> Locked DOF SIMD eslesmesi tam dogrulandi." << std::endl;
}

void testDegenerateCases() {
    std::cout << "[SimdSolverTest] 5. Dejenere Durumlar (Sifir kutle, sifir hiz, NaN olmama)..." << std::endl;

    ContactPoint cpScalar;
    cpScalar.normalImpulse = 0.0f;
    ContactPoint cpSimd = cpScalar;

    Vector3 rA(0.0f, 0.0f, 0.0f);
    Vector3 rB(0.0f, 0.0f, 0.0f);
    Vector3 n(0.0f, 1.0f, 0.0f);

    // Her iki cisim de statik / sıfır ters kütle
    Vector3 zeroInvMass(0.0f, 0.0f, 0.0f);
    Matrix3x3 zeroInvI(0, 0, 0, 0, 0, 0, 0, 0, 0);

    Core::Motion motionA;
    Core::Motion motionB;

    solveContactNormalScalar(cpScalar, rA, rB, n, zeroInvMass, zeroInvMass, zeroInvI, zeroInvI, motionA, motionB);
    solveContactNormalSIMD(cpSimd, rA, rB, n, zeroInvMass, zeroInvMass, zeroInvI, zeroInvI, motionA, motionB);

    CHECK(!std::isnan(cpSimd.normalImpulse));
    CHECK(!std::isinf(cpSimd.normalImpulse));
    CHECK(cpSimd.normalImpulse == 0.0f);
    CHECK(cpScalar.normalImpulse == 0.0f);

    std::cout << "  -> Dejenere sifir kutle durumunda NaN yok, guvenli cikis OK." << std::endl;
}

void testSimdContactBatch4() {
    std::cout << "[SimdSolverTest] 6. SoA SimdContactBatch4 Vektor Paralel Testi..." << std::endl;

    SimdContactBatch4 batch;
    batch.activeCount = 4;
    for (int i = 0; i < 4; ++i) {
        batch.normalVel[i] = static_cast<float>(i + 1) * -1.5f;
        batch.velocityBias[i] = 0.2f * static_cast<float>(i);
    }

    float deltaV[4];
    alignas(16) float vZero[4] = {0, 0, 0, 0};
#if defined(BARYON_SIMD_SSE_SUPPORTED)
    __m128 zero = _mm_load_ps(vZero);
    batch.computeBatchDeltaV(Vector3(0, 1, 0), zero, zero, zero, zero, deltaV);
#else
    (void)vZero;
    batch.computeBatchDeltaV(Vector3(0, 1, 0), {}, {}, {}, {}, deltaV);
#endif

    for (int i = 0; i < 4; ++i) {
        float expected = -batch.normalVel[i] + batch.velocityBias[i];
        CHECK(std::abs(deltaV[i] - expected) < 1e-6f);
    }

    std::cout << "  -> Batch 4-lane deltaV hesaplamalari eksiksiz dogrulandi." << std::endl;
}

} // namespace

int main() {
    std::cout << "=== Baryon Faz 5.2: SIMD Kontak Cozucu Test Paketi ===" << std::endl;

    testNormalImpulseEquivalence();
    testFrictionImpulseEquivalence();
    testMultiPointManifoldIterations();
    testLockedAxesEquivalence();
    testDegenerateCases();
    testSimdContactBatch4();

    std::cout << "=== [SimdSolverTest] TUM TESTLER BASARIYLA TAMAMLANDI! ===" << std::endl;
    return reportAndExit();
}
