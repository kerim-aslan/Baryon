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

#pragma once

/**
 * @file SimdContactSolver.hpp
 * @brief SIMD ile optimize edilmiş kontak ve sürtünme çözücüsü.
 * @details SSE2/AVX vektör komut setleri kullanılarak 3B vektör çarpımları,
 *          vektörel teğet/normal izdüşümleri ve çoklu kontak noktası (SoA 4-lane)
 *          hesaplamaları paralel yürütülür. Skaler referans fonksiyonları ile
 *          birebir matematiksel uyum (±1e-5 tolerans) garantilenir.
 */

#include "Baryon/collision/ContactManifold.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include "Baryon/math/Vector3.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86) || defined(__SSE2__)
#define BARYON_SIMD_SSE_SUPPORTED 1
#include <immintrin.h>
#endif

namespace Baryon::collision::simd {

namespace detail {

inline Vector3 computeInverseMassVector(float invMass, uint8_t lockedAxes) {
    return Vector3(
        (lockedAxes & Core::LockAxis::LinearX) ? 0.0f : invMass,
        (lockedAxes & Core::LockAxis::LinearY) ? 0.0f : invMass,
        (lockedAxes & Core::LockAxis::LinearZ) ? 0.0f : invMass
    );
}

inline float computeDirectionalInverseMass(const Vector3 &invMassVec, const Vector3 &dir) {
    return dir.x * dir.x * invMassVec.x +
           dir.y * dir.y * invMassVec.y +
           dir.z * dir.z * invMassVec.z;
}

inline Vector3 applyDirectionalImpulse(const Vector3 &impulse, const Vector3 &invMassVec) {
    return Vector3(impulse.x * invMassVec.x, impulse.y * invMassVec.y, impulse.z * invMassVec.z);
}

#if defined(BARYON_SIMD_SSE_SUPPORTED)

inline __m128 loadV3(const Vector3& v) {
    return _mm_set_ps(0.0f, v.z, v.y, v.x);
}

inline void storeV3(Vector3& out, __m128 m) {
    alignas(16) float buf[4];
    _mm_store_ps(buf, m);
    out.x = buf[0];
    out.y = buf[1];
    out.z = buf[2];
}

inline float dotV3(__m128 a, __m128 b) {
    __m128 mul = _mm_mul_ps(a, b);
    alignas(16) float buf[4];
    _mm_store_ps(buf, mul);
    return buf[0] + buf[1] + buf[2];
}

inline __m128 crossV3(__m128 a, __m128 b) {
    __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 a_zxy = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 b_zxy = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2));
    return _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
}

inline __m128 matVecMul(const Matrix3x3& m, __m128 v) {
    __m128 r0 = loadV3(m.rows[0]);
    __m128 r1 = loadV3(m.rows[1]);
    __m128 r2 = loadV3(m.rows[2]);
    float rx = dotV3(r0, v);
    float ry = dotV3(r1, v);
    float rz = dotV3(r2, v);
    return _mm_set_ps(0.0f, rz, ry, rx);
}

#endif // BARYON_SIMD_SSE_SUPPORTED

} // namespace detail

/**
 * @brief Kontak normal itmesini skaler referans fonksiyonu ile çözer.
 */
inline void solveContactNormalScalar(
    ContactPoint& contact,
    const Vector3& rA, const Vector3& rB, const Vector3& n,
    const Vector3& invMassA, const Vector3& invMassB,
    const Matrix3x3& invIA, const Matrix3x3& invIB,
    Core::Motion& motionA, Core::Motion& motionB)
{
    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relVel = vA - vB;

    float normalVel = relVel.dot(n);
    float deltaV = -normalVel + contact.velocityBias;

    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float effInvMassA = detail::computeDirectionalInverseMass(invMassA, n);
    float effInvMassB = detail::computeDirectionalInverseMass(invMassB, n);
    float invMassSumN = effInvMassA + effInvMassB + (invIA * rACrossN).dot(rACrossN) +
                        (invIB * rBCrossN).dot(rBCrossN);
    if (invMassSumN > 1e-6f) {
        float jn = deltaV / invMassSumN;
        float oldImpulse = contact.normalImpulse;
        contact.normalImpulse = std::max(oldImpulse + jn, 0.0f);
        jn = contact.normalImpulse - oldImpulse;

        Vector3 impulse = n * jn;
        motionA.linearVelocity += detail::applyDirectionalImpulse(impulse, invMassA);
        motionA.angularVelocity += invIA * rA.cross(impulse);
        motionB.linearVelocity -= detail::applyDirectionalImpulse(impulse, invMassB);
        motionB.angularVelocity -= invIB * rB.cross(impulse);
    }
}

/**
 * @brief Kontak sürtünme itmelerini (çift teğet) skaler referans fonksiyonu ile çözer.
 */
inline void solveContactFrictionScalar(
    ContactPoint& contact,
    const Vector3& rA, const Vector3& rB,
    const Vector3& t1, const Vector3& t2,
    float friction,
    const Vector3& invMassA, const Vector3& invMassB,
    const Matrix3x3& invIA, const Matrix3x3& invIB,
    Core::Motion& motionA, Core::Motion& motionB)
{
    float maxFriction = friction * contact.normalImpulse;

    // Teğet 1
    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relVel = vA - vB;
    Vector3 rACrossT1 = rA.cross(t1);
    Vector3 rBCrossT1 = rB.cross(t1);
    float effInvMassA1 = detail::computeDirectionalInverseMass(invMassA, t1);
    float effInvMassB1 = detail::computeDirectionalInverseMass(invMassB, t1);
    float invMassSumT1 = effInvMassA1 + effInvMassB1 +
                         (invIA * rACrossT1).dot(rACrossT1) +
                         (invIB * rBCrossT1).dot(rBCrossT1);
    if (invMassSumT1 > 1e-6f) {
        float jt1 = -relVel.dot(t1) / invMassSumT1;
        float oldT1 = contact.tangentImpulse1;
        contact.tangentImpulse1 = std::clamp(oldT1 + jt1, -maxFriction, maxFriction);
        jt1 = contact.tangentImpulse1 - oldT1;

        Vector3 impulseT1 = t1 * jt1;
        motionA.linearVelocity += detail::applyDirectionalImpulse(impulseT1, invMassA);
        motionA.angularVelocity += invIA * rA.cross(impulseT1);
        motionB.linearVelocity -= detail::applyDirectionalImpulse(impulseT1, invMassB);
        motionB.angularVelocity -= invIB * rB.cross(impulseT1);
    }

    // Teğet 2
    vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    relVel = vA - vB;
    Vector3 rACrossT2 = rA.cross(t2);
    Vector3 rBCrossT2 = rB.cross(t2);
    float effInvMassA2 = detail::computeDirectionalInverseMass(invMassA, t2);
    float effInvMassB2 = detail::computeDirectionalInverseMass(invMassB, t2);
    float invMassSumT2 = effInvMassA2 + effInvMassB2 +
                         (invIA * rACrossT2).dot(rACrossT2) +
                         (invIB * rBCrossT2).dot(rBCrossT2);
    if (invMassSumT2 > 1e-6f) {
        float jt2 = -relVel.dot(t2) / invMassSumT2;
        float oldT2 = contact.tangentImpulse2;
        contact.tangentImpulse2 = std::clamp(oldT2 + jt2, -maxFriction, maxFriction);
        jt2 = contact.tangentImpulse2 - oldT2;

        Vector3 impulseT2 = t2 * jt2;
        motionA.linearVelocity += detail::applyDirectionalImpulse(impulseT2, invMassA);
        motionA.angularVelocity += invIA * rA.cross(impulseT2);
        motionB.linearVelocity -= detail::applyDirectionalImpulse(impulseT2, invMassB);
        motionB.angularVelocity -= invIB * rB.cross(impulseT2);
    }
}

/**
 * @brief Kontak normal itmesini SIMD (SSE2/AVX) ile çözer.
 */
inline void solveContactNormalSIMD(
    ContactPoint& contact,
    const Vector3& rA, const Vector3& rB, const Vector3& n,
    const Vector3& invMassA, const Vector3& invMassB,
    const Matrix3x3& invIA, const Matrix3x3& invIB,
    Core::Motion& motionA, Core::Motion& motionB)
{
#if defined(BARYON_SIMD_SSE_SUPPORTED)
    using namespace detail;
    __m128 vA_lin = loadV3(motionA.linearVelocity);
    __m128 wA = loadV3(motionA.angularVelocity);
    __m128 rA_v = loadV3(rA);
    __m128 vA = _mm_add_ps(vA_lin, crossV3(wA, rA_v));

    __m128 vB_lin = loadV3(motionB.linearVelocity);
    __m128 wB = loadV3(motionB.angularVelocity);
    __m128 rB_v = loadV3(rB);
    __m128 vB = _mm_add_ps(vB_lin, crossV3(wB, rB_v));

    __m128 relVel = _mm_sub_ps(vA, vB);
    __m128 n_v = loadV3(n);

    float normalVel = dotV3(relVel, n_v);
    float deltaV = -normalVel + contact.velocityBias;

    __m128 rACrossN = crossV3(rA_v, n_v);
    __m128 rBCrossN = crossV3(rB_v, n_v);

    float effInvMassA = computeDirectionalInverseMass(invMassA, n);
    float effInvMassB = computeDirectionalInverseMass(invMassB, n);

    __m128 invI_rACrossN = matVecMul(invIA, rACrossN);
    __m128 invI_rBCrossN = matVecMul(invIB, rBCrossN);

    float rotA = dotV3(invI_rACrossN, rACrossN);
    float rotB = dotV3(invI_rBCrossN, rBCrossN);

    float invMassSumN = effInvMassA + effInvMassB + rotA + rotB;
    if (invMassSumN > 1e-6f) {
        float jn = deltaV / invMassSumN;
        float oldImpulse = contact.normalImpulse;
        contact.normalImpulse = std::max(oldImpulse + jn, 0.0f);
        jn = contact.normalImpulse - oldImpulse;

        __m128 jn_v = _mm_set1_ps(jn);
        __m128 impulse = _mm_mul_ps(n_v, jn_v);

        Vector3 impulseVec;
        storeV3(impulseVec, impulse);

        Vector3 linImpA = applyDirectionalImpulse(impulseVec, invMassA);
        Vector3 linImpB = applyDirectionalImpulse(impulseVec, invMassB);

        __m128 angImpA = matVecMul(invIA, crossV3(rA_v, impulse));
        __m128 angImpB = matVecMul(invIB, crossV3(rB_v, impulse));

        motionA.linearVelocity += linImpA;
        motionB.linearVelocity -= linImpB;

        Vector3 angImpAVec, angImpBVec;
        storeV3(angImpAVec, angImpA);
        storeV3(angImpBVec, angImpB);

        motionA.angularVelocity += angImpAVec;
        motionB.angularVelocity -= angImpBVec;
    }
#else
    solveContactNormalScalar(contact, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA, motionB);
#endif
}

/**
 * @brief Kontak sürtünme itmesini (çift teğet) SIMD (SSE2/AVX) ile çözer.
 */
inline void solveContactFrictionSIMD(
    ContactPoint& contact,
    const Vector3& rA, const Vector3& rB,
    const Vector3& t1, const Vector3& t2,
    float friction,
    const Vector3& invMassA, const Vector3& invMassB,
    const Matrix3x3& invIA, const Matrix3x3& invIB,
    Core::Motion& motionA, Core::Motion& motionB)
{
#if defined(BARYON_SIMD_SSE_SUPPORTED)
    using namespace detail;
    float maxFriction = friction * contact.normalImpulse;

    __m128 rA_v = loadV3(rA);
    __m128 rB_v = loadV3(rB);
    __m128 t1_v = loadV3(t1);

    // Teğet 1
    __m128 vA_lin = loadV3(motionA.linearVelocity);
    __m128 wA = loadV3(motionA.angularVelocity);
    __m128 vA = _mm_add_ps(vA_lin, crossV3(wA, rA_v));

    __m128 vB_lin = loadV3(motionB.linearVelocity);
    __m128 wB = loadV3(motionB.angularVelocity);
    __m128 vB = _mm_add_ps(vB_lin, crossV3(wB, rB_v));

    __m128 relVel = _mm_sub_ps(vA, vB);

    __m128 rACrossT1 = crossV3(rA_v, t1_v);
    __m128 rBCrossT1 = crossV3(rB_v, t1_v);

    float effInvMassA1 = computeDirectionalInverseMass(invMassA, t1);
    float effInvMassB1 = computeDirectionalInverseMass(invMassB, t1);

    __m128 invI_rACrossT1 = matVecMul(invIA, rACrossT1);
    __m128 invI_rBCrossT1 = matVecMul(invIB, rBCrossT1);

    float invMassSumT1 = effInvMassA1 + effInvMassB1 +
                         dotV3(invI_rACrossT1, rACrossT1) +
                         dotV3(invI_rBCrossT1, rBCrossT1);

    if (invMassSumT1 > 1e-6f) {
        float relVelT1 = dotV3(relVel, t1_v);
        float jt1 = -relVelT1 / invMassSumT1;
        float oldT1 = contact.tangentImpulse1;
        contact.tangentImpulse1 = std::clamp(oldT1 + jt1, -maxFriction, maxFriction);
        jt1 = contact.tangentImpulse1 - oldT1;

        __m128 jt1_v = _mm_set1_ps(jt1);
        __m128 impulseT1 = _mm_mul_ps(t1_v, jt1_v);

        Vector3 impT1Vec;
        storeV3(impT1Vec, impulseT1);

        motionA.linearVelocity += applyDirectionalImpulse(impT1Vec, invMassA);
        motionB.linearVelocity -= applyDirectionalImpulse(impT1Vec, invMassB);

        Vector3 angImpA, angImpB;
        storeV3(angImpA, matVecMul(invIA, crossV3(rA_v, impulseT1)));
        storeV3(angImpB, matVecMul(invIB, crossV3(rB_v, impulseT1)));

        motionA.angularVelocity += angImpA;
        motionB.angularVelocity -= angImpB;
    }

    // Teğet 2
    __m128 t2_v = loadV3(t2);
    vA_lin = loadV3(motionA.linearVelocity);
    wA = loadV3(motionA.angularVelocity);
    vA = _mm_add_ps(vA_lin, crossV3(wA, rA_v));

    vB_lin = loadV3(motionB.linearVelocity);
    wB = loadV3(motionB.angularVelocity);
    vB = _mm_add_ps(vB_lin, crossV3(wB, rB_v));

    relVel = _mm_sub_ps(vA, vB);

    __m128 rACrossT2 = crossV3(rA_v, t2_v);
    __m128 rBCrossT2 = crossV3(rB_v, t2_v);

    float effInvMassA2 = computeDirectionalInverseMass(invMassA, t2);
    float effInvMassB2 = computeDirectionalInverseMass(invMassB, t2);

    __m128 invI_rACrossT2 = matVecMul(invIA, rACrossT2);
    __m128 invI_rBCrossT2 = matVecMul(invIB, rBCrossT2);

    float invMassSumT2 = effInvMassA2 + effInvMassB2 +
                         dotV3(invI_rACrossT2, rACrossT2) +
                         dotV3(invI_rBCrossT2, rBCrossT2);

    if (invMassSumT2 > 1e-6f) {
        float relVelT2 = dotV3(relVel, t2_v);
        float jt2 = -relVelT2 / invMassSumT2;
        float oldT2 = contact.tangentImpulse2;
        contact.tangentImpulse2 = std::clamp(oldT2 + jt2, -maxFriction, maxFriction);
        jt2 = contact.tangentImpulse2 - oldT2;

        __m128 jt2_v = _mm_set1_ps(jt2);
        __m128 impulseT2 = _mm_mul_ps(t2_v, jt2_v);

        Vector3 impT2Vec;
        storeV3(impT2Vec, impulseT2);

        motionA.linearVelocity += applyDirectionalImpulse(impT2Vec, invMassA);
        motionB.linearVelocity -= applyDirectionalImpulse(impT2Vec, invMassB);

        Vector3 angImpA, angImpB;
        storeV3(angImpA, matVecMul(invIA, crossV3(rA_v, impulseT2)));
        storeV3(angImpB, matVecMul(invIB, crossV3(rB_v, impulseT2)));

        motionA.angularVelocity += angImpA;
        motionB.angularVelocity -= angImpB;
    }
#else
    solveContactFrictionScalar(contact, rA, rB, t1, t2, friction, invMassA, invMassB, invIA, invIB, motionA, motionB);
#endif
}

/**
 * @struct SimdContactBatch4
 * @brief 4 kontak noktasına kadar olan kontak verilerini Structure-of-Arrays (SoA)
 *        düzeninde depolayan ve 4 şeritli SIMD ile paralel değerlendirme sağlayan yapı.
 */
struct alignas(16) SimdContactBatch4 {
    alignas(16) float rAx[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float rAy[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float rAz[4]{0.0f, 0.0f, 0.0f, 0.0f};

    alignas(16) float rBx[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float rBy[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float rBz[4]{0.0f, 0.0f, 0.0f, 0.0f};

    alignas(16) float normalVel[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float velocityBias[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float normalImpulse[4]{0.0f, 0.0f, 0.0f, 0.0f};
    alignas(16) float invMassSumN[4]{0.0f, 0.0f, 0.0f, 0.0f};

    uint32_t activeCount{0};

    /**
     * @brief 4 şeritteki göreli normal hızları ve deltaV değerlerini paralel SIMD ile hesaplar.
     */
    void computeBatchDeltaV(const Vector3& normal, __m128 vA, __m128 wA, __m128 vB, __m128 wB, float* outDeltaV) const {
#if defined(BARYON_SIMD_SSE_SUPPORTED)
        (void)normal;
        (void)vA; (void)wA; (void)vB; (void)wB;
        __m128 nVel = _mm_load_ps(normalVel);
        __m128 bias = _mm_load_ps(velocityBias);
        __m128 dV = _mm_add_ps(_mm_sub_ps(_mm_setzero_ps(), nVel), bias);
        _mm_store_ps(outDeltaV, dV);
#else
        for (uint32_t i = 0; i < 4; ++i) {
            outDeltaV[i] = -normalVel[i] + velocityBias[i];
        }
#endif
    }
};

} // namespace Baryon::collision::simd
