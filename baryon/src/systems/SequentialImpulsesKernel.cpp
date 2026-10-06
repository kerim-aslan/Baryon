/**
 * @file SequentialImpulsesKernel.cpp
 * @brief Ardışık İtme (Sequential Impulses) çözücüsü uygulaması.
 * @details Temas noktalarını, mesafe kısıtlamalarını ve menteşe eklemlerini (joint) 
 *          iteratif olarak çözer. Warm Starting, 2D Coulomb sürtünmesi ve Split Impulse içerir.
 */

#include "Baryon/systems/SequentialImpulsesKernel.hpp"
#include "Baryon/Core/DebugManager.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace Baryon::systems {

SequentialImpulsesKernel::SequentialImpulsesKernel(Core::Registry &registry)
    : mRegistry(registry) {}

void SequentialImpulsesKernel::execute(
    const std::vector<ecs::Entity> &island,
    const std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime, int velocityIterations, int positionIterations) {
    if (island.empty() || manifolds.empty()) return;

    auto &manifoldMap = const_cast<std::unordered_map<uint64_t, collision::ContactManifold> &>(manifolds);
    std::vector<collision::ContactManifold *> localManifolds;
    localManifolds.reserve(std::min(manifolds.size(), static_cast<size_t>(64)));

    if (island.size() == 1) {
        const uint32_t singleId = island[0].id;
        for (auto &[key, manifold] : manifoldMap) {
            if (manifold.entityA.id == singleId || manifold.entityB.id == singleId) {
                localManifolds.push_back(&manifold);
            }
        }
    } else if (island.size() <= 16) {
        for (auto &[key, manifold] : manifoldMap) {
            uint32_t idA = manifold.entityA.id;
            uint32_t idB = manifold.entityB.id;
            for (ecs::Entity e : island) {
                if (e.id == idA || e.id == idB) {
                    localManifolds.push_back(&manifold);
                    break;
                }
            }
        }
    } else {
        std::unordered_set<uint32_t> islandIds;
        islandIds.reserve(island.size() * 2);
        for (ecs::Entity e : island) {
            islandIds.insert(e.id);
        }
        for (auto &[key, manifold] : manifoldMap) {
            if (islandIds.contains(manifold.entityA.id) || islandIds.contains(manifold.entityB.id)) {
                localManifolds.push_back(&manifold);
            }
        }
    }

    // Eklem kısıtlamaları
    std::vector<collision::DistanceConstraint *> localDistance;
    std::vector<collision::RevoluteConstraint *> localRevolute;
    if (mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData().size() > 0) {
        auto &constraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
        for (auto &constraint : constraints) {
            bool match = false;
            for (ecs::Entity e : island) {
                if (e.id == constraint.entityA.id || e.id == constraint.entityB.id) {
                    match = true;
                    break;
                }
            }
            if (match) localDistance.push_back(&constraint);
        }
    }
    if (mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData().size() > 0) {
        auto &constraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
        for (auto &constraint : constraints) {
            bool match = false;
            for (ecs::Entity e : island) {
                if (e.id == constraint.entityA.id || e.id == constraint.entityB.id) {
                    match = true;
                    break;
                }
            }
            if (match) localRevolute.push_back(&constraint);
        }
    }

    if (localManifolds.empty() && localDistance.empty() && localRevolute.empty()) {
        return;
    }

    // 1. ÖN-ADIM: Ortogonal teğetleri ve Warm-Starting kuvvetlerini hızlara uygula
    for (collision::ContactManifold *manifold : localManifolds) {
        preStep(*manifold, deltaTime);
    }

    // 2. Hız İterasyonları: Gauss-Seidel yaklaşımı
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (collision::ContactManifold *manifold : localManifolds) {
            resolveCollision(*manifold, deltaTime);
        }
        for (collision::DistanceConstraint *constraint : localDistance) {
            resolveDistanceConstraint(*constraint, deltaTime);
        }
        for (collision::RevoluteConstraint *constraint : localRevolute) {
            resolveRevoluteConstraint(*constraint, deltaTime);
        }
    }

    // 3. Pozisyon İterasyonları: Split Impulse ile kinetik hızı kirletmeden penetrasyonu çöz
    for (int iter = 0; iter < positionIterations; ++iter) {
        for (collision::ContactManifold *manifold : localManifolds) {
            solvePositionConstraints(*manifold, deltaTime);
        }
    }
}

void SequentialImpulsesKernel::solveIsland(
    const std::vector<ecs::Entity> &island,
    const std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime) {
    execute(island, manifolds, deltaTime, 10, 4);
}

void SequentialImpulsesKernel::solveConstraintsForIsland(
    const std::vector<ecs::Entity> &island, float deltaTime) {
    (void)island;
    (void)deltaTime;
}

void SequentialImpulsesKernel::preStep(
    collision::ContactManifold &manifold, float deltaTime) {
    (void)deltaTime;
    if (!mRegistry.hasComponent<Pose>(manifold.entityA) || !mRegistry.hasComponent<Pose>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) || !mRegistry.hasComponent<Core::Motion>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) || !mRegistry.hasComponent<Core::MassProps>(manifold.entityB)) return;

    auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
    auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
    auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
    auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
    const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
    const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);

    manifold.computeTangents();
    Vector3 n = manifold.normal;
    Vector3 t1 = manifold.tangent1;
    Vector3 t2 = manifold.tangent2;

    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        auto &contact = manifold.contacts[i];
        contact.splitImpulse = 0.0f;

        Vector3 rA = poseA.orientation * contact.localPointA;
        Vector3 rB = poseB.orientation * contact.localPointB;

        // Warm Starting itmelerini hızlara uygula
        Vector3 totalImpulse = (n * contact.normalImpulse) + (t1 * contact.tangentImpulse1) + (t2 * contact.tangentImpulse2);

        motionA.linearVelocity += totalImpulse * invMassA;
        motionA.angularVelocity += invIA * rA.cross(totalImpulse);
        motionB.linearVelocity -= totalImpulse * invMassB;
        motionB.angularVelocity -= invIB * rB.cross(totalImpulse);
    }
}

void SequentialImpulsesKernel::resolveCollision(
    collision::ContactManifold &manifold, float deltaTime) {
    (void)deltaTime;
    if (!mRegistry.hasComponent<Pose>(manifold.entityA) || !mRegistry.hasComponent<Pose>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) || !mRegistry.hasComponent<Core::Motion>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) || !mRegistry.hasComponent<Core::MassProps>(manifold.entityB)) return;

    auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
    auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
    auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
    auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
    const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
    const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);

    float friction = 0.4f;
    float restitution = 0.0f;
    if (mRegistry.hasComponent<Core::Material>(manifold.entityA) && mRegistry.hasComponent<Core::Material>(manifold.entityB)) {
        const auto &matA = mRegistry.getComponent<Core::Material>(manifold.entityA);
        const auto &matB = mRegistry.getComponent<Core::Material>(manifold.entityB);
        friction = std::sqrt(matA.friction * matB.friction);
        restitution = std::max(matA.restitution, matB.restitution);
    }

    Vector3 n = manifold.normal;
    Vector3 t1 = manifold.tangent1;
    Vector3 t2 = manifold.tangent2;

    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        auto &contact = manifold.contacts[i];

        Vector3 rA = poseA.orientation * contact.localPointA;
        Vector3 rB = poseB.orientation * contact.localPointB;

        // 1. Normal İtme Çözümü
        Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
        Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
        Vector3 relVel = vA - vB;

        float normalVel = relVel.dot(n);
        float deltaV = -normalVel;

        if (normalVel < -physics::PhysicsConstants::RestitutionVelocityThreshold) {
            deltaV -= restitution * normalVel;
        }

        Vector3 rACrossN = rA.cross(n);
        Vector3 rBCrossN = rB.cross(n);
        float invMassSumN = invMassA + invMassB + (invIA * rACrossN).dot(rACrossN) + (invIB * rBCrossN).dot(rBCrossN);
        if (invMassSumN > 1e-6f) {
            float jn = deltaV / invMassSumN;
            float oldImpulse = contact.normalImpulse;
            contact.normalImpulse = std::max(oldImpulse + jn, 0.0f);
            jn = contact.normalImpulse - oldImpulse;

            Vector3 impulse = n * jn;
            motionA.linearVelocity += impulse * invMassA;
            motionA.angularVelocity += invIA * rA.cross(impulse);
            motionB.linearVelocity -= impulse * invMassB;
            motionB.angularVelocity -= invIB * rB.cross(impulse);
        }

        // 2. Çift Teğetli Coulomb Sürtünmesi Çözümü (T1 ve T2)
        float maxFriction = friction * contact.normalImpulse;

        // Teğet 1
        vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
        vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
        relVel = vA - vB;
        Vector3 rACrossT1 = rA.cross(t1);
        Vector3 rBCrossT1 = rB.cross(t1);
        float invMassSumT1 = invMassA + invMassB + (invIA * rACrossT1).dot(rACrossT1) + (invIB * rBCrossT1).dot(rBCrossT1);
        if (invMassSumT1 > 1e-6f) {
            float jt1 = -relVel.dot(t1) / invMassSumT1;
            float oldT1 = contact.tangentImpulse1;
            contact.tangentImpulse1 = std::clamp(oldT1 + jt1, -maxFriction, maxFriction);
            jt1 = contact.tangentImpulse1 - oldT1;

            Vector3 impulseT1 = t1 * jt1;
            motionA.linearVelocity += impulseT1 * invMassA;
            motionA.angularVelocity += invIA * rA.cross(impulseT1);
            motionB.linearVelocity -= impulseT1 * invMassB;
            motionB.angularVelocity -= invIB * rB.cross(impulseT1);
        }

        // Teğet 2
        vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
        vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
        relVel = vA - vB;
        Vector3 rACrossT2 = rA.cross(t2);
        Vector3 rBCrossT2 = rB.cross(t2);
        float invMassSumT2 = invMassA + invMassB + (invIA * rACrossT2).dot(rACrossT2) + (invIB * rBCrossT2).dot(rBCrossT2);
        if (invMassSumT2 > 1e-6f) {
            float jt2 = -relVel.dot(t2) / invMassSumT2;
            float oldT2 = contact.tangentImpulse2;
            contact.tangentImpulse2 = std::clamp(oldT2 + jt2, -maxFriction, maxFriction);
            jt2 = contact.tangentImpulse2 - oldT2;

            Vector3 impulseT2 = t2 * jt2;
            motionA.linearVelocity += impulseT2 * invMassA;
            motionA.angularVelocity += invIA * rA.cross(impulseT2);
            motionB.linearVelocity -= impulseT2 * invMassB;
            motionB.angularVelocity -= invIB * rB.cross(impulseT2);
        }
    }
}

void SequentialImpulsesKernel::solvePositionConstraints(
    collision::ContactManifold &manifold, float deltaTime) {
    if (deltaTime <= 0.0f) return;
    if (!mRegistry.hasComponent<Pose>(manifold.entityA) || !mRegistry.hasComponent<Pose>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) || !mRegistry.hasComponent<Core::Motion>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) || !mRegistry.hasComponent<Core::MassProps>(manifold.entityB)) return;

    auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
    auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
    auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
    auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
    const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
    const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);

    Vector3 n = manifold.normal;

    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        auto &contact = manifold.contacts[i];

        Vector3 rA = poseA.orientation * contact.localPointA;
        Vector3 rB = poseB.orientation * contact.localPointB;

        Vector3 pA = poseA.position + rA;
        Vector3 pB = poseB.position + rB;
        float separation = (pA - pB).dot(n);
        float currentPenetration = contact.penetration - separation;

        float C = std::clamp(currentPenetration - physics::PhysicsConstants::LinearSlop, 0.0f, physics::PhysicsConstants::MaxLinearCorrection);
        if (C <= 0.0f) continue;

        Vector3 rACrossN = rA.cross(n);
        Vector3 rBCrossN = rB.cross(n);
        float invMassSum = invMassA + invMassB + (invIA * rACrossN).dot(rACrossN) + (invIB * rBCrossN).dot(rBCrossN);
        if (invMassSum <= 1e-6f) continue;

        // Split impulse: v_split'e itme uygula
        Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
        Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
        float splitVel = (vSplitA - vSplitB).dot(n);

        float bias = (physics::PhysicsConstants::PositionCorrectionFactor / deltaTime) * C;
        float deltaSplit = (-splitVel + bias) / invMassSum;

        float oldSplit = contact.splitImpulse;
        contact.splitImpulse = std::max(oldSplit + deltaSplit, 0.0f);
        deltaSplit = contact.splitImpulse - oldSplit;

        Vector3 impulse = n * deltaSplit;
        motionA.splitLinearVelocity += impulse * invMassA;
        motionA.splitAngularVelocity += invIA * rA.cross(impulse);
        motionB.splitLinearVelocity -= impulse * invMassB;
        motionB.splitAngularVelocity -= invIB * rB.cross(impulse);
    }
}

void SequentialImpulsesKernel::resolveDistanceConstraint(
    collision::DistanceConstraint &constraint, float deltaTime) {
    (void)deltaTime;
    if (!mRegistry.hasComponent<Pose>(constraint.entityA) || !mRegistry.hasComponent<Pose>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) || !mRegistry.hasComponent<Core::Motion>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) || !mRegistry.hasComponent<Core::MassProps>(constraint.entityB)) return;

    auto &poseA = mRegistry.getComponent<Pose>(constraint.entityA);
    auto &poseB = mRegistry.getComponent<Pose>(constraint.entityB);
    auto &motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
    auto &motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
    const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
    const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

    Vector3 rA = poseA.orientation * constraint.localAnchorA;
    Vector3 rB = poseB.orientation * constraint.localAnchorB;
    Vector3 pA = poseA.position + rA;
    Vector3 pB = poseB.position + rB;

    Vector3 delta = pA - pB;
    float currentDist = delta.length();
    if (currentDist < 1e-6f) return;
    Vector3 u = delta / currentDist;

    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    float relVel = (vA - vB).dot(u);

    Vector3 rACrossU = rA.cross(u);
    Vector3 rBCrossU = rB.cross(u);
    float invMassSum = massA.inverseMass + massB.inverseMass + (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) + (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
    if (invMassSum < 1e-6f) return;

    float j = -relVel / invMassSum;
    Vector3 impulse = u * j;

    motionA.linearVelocity += impulse * massA.inverseMass;
    motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.linearVelocity -= impulse * massB.inverseMass;
    motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
}

void SequentialImpulsesKernel::resolveRevoluteConstraint(
    collision::RevoluteConstraint &constraint, float deltaTime) {
    (void)deltaTime;
    if (!mRegistry.hasComponent<Pose>(constraint.entityA) || !mRegistry.hasComponent<Pose>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) || !mRegistry.hasComponent<Core::Motion>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) || !mRegistry.hasComponent<Core::MassProps>(constraint.entityB)) return;

    auto &poseA = mRegistry.getComponent<Pose>(constraint.entityA);
    auto &poseB = mRegistry.getComponent<Pose>(constraint.entityB);
    auto &motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
    auto &motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
    const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
    const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

    Vector3 rA = poseA.orientation * constraint.localAnchorA;
    Vector3 rB = poseB.orientation * constraint.localAnchorB;

    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relVel = vA - vB;

    // 3 eksen boyunca noktasal hizalama
    float invMassSum = massA.inverseMass + massB.inverseMass;
    if (invMassSum < 1e-6f) return;

    Vector3 impulse = (relVel * -1.0f) / invMassSum;
    motionA.linearVelocity += impulse * massA.inverseMass;
    motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.linearVelocity -= impulse * massB.inverseMass;
    motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
}

} // namespace Baryon::systems
