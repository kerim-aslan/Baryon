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
 * @file SequentialImpulsesKernel.cpp
 * @brief Ardışık İtme (Sequential Impulses) çözücüsü uygulaması.
 * @details Temas noktalarını, mesafe kısıtlamalarını ve menteşe eklemlerini
 * (joint) iteratif olarak çözer. Warm Starting, 2D Coulomb sürtünmesi ve Split
 * Impulse içerir. SolverBody yapısı sayesinde iç döngüde Registry erişimi yapmaz.
 */

#include "Baryon/systems/SequentialImpulsesKernel.hpp"
#include "Baryon/Core/DebugManager.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include <algorithm>
#include <cmath>

namespace Baryon::systems {

SequentialImpulsesKernel::SequentialImpulsesKernel(Core::Registry &registry)
    : mRegistry(registry) {}

uint32_t SequentialImpulsesKernel::getOrAddBody(ecs::Entity entity) {
  auto it = mEntityToBodyMap.find(entity.id);
  if (it != mEntityToBodyMap.end()) {
    return it->second;
  }

  uint32_t index = static_cast<uint32_t>(mSolverBodies.size());
  SolverBody body{};
  body.entity = entity;

  if (mRegistry.hasComponent<Pose>(entity)) {
    const auto &pose = mRegistry.getComponent<Pose>(entity);
    body.position = pose.position;
    body.orientation = pose.orientation;
  }

  if (mRegistry.hasComponent<Core::BodyState>(entity)) {
    body.type = mRegistry.getComponent<Core::BodyState>(entity).type;
  } else {
    body.type = Core::BodyType::Dynamic;
  }

  if (body.type == Core::BodyType::Dynamic) {
    if (mRegistry.hasComponent<Core::Motion>(entity)) {
      const auto &motion = mRegistry.getComponent<Core::Motion>(entity);
      body.linearVelocity = motion.linearVelocity;
      body.angularVelocity = motion.angularVelocity;
      body.splitLinearVelocity = motion.splitLinearVelocity;
      body.splitAngularVelocity = motion.splitAngularVelocity;
    }
    if (mRegistry.hasComponent<Core::MassProps>(entity)) {
      const auto &mass = mRegistry.getComponent<Core::MassProps>(entity);
      body.invMass = mass.inverseMass;
      body.invInertiaTensor = mass.inverseInertiaTensor;
    }
  } else {
    body.invMass = 0.0f;
    body.invInertiaTensor = Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  mSolverBodies.push_back(body);
  mEntityToBodyMap[entity.id] = index;
  return index;
}

void SequentialImpulsesKernel::prepareIslandData(Island &island, float deltaTime) {
  mSolverBodies.clear();
  mEntityToBodyMap.clear();
  mSolverContacts.clear();
  mSolverDistanceConstraints.clear();
  mSolverRevoluteConstraints.clear();

  for (ecs::Entity e : island.bodies) {
    getOrAddBody(e);
  }

  for (collision::ContactManifold *manifold : island.manifolds) {
    if (manifold->isTriggerPair)
      continue;

    uint32_t idxA = getOrAddBody(manifold->entityA);
    uint32_t idxB = getOrAddBody(manifold->entityB);

    float friction = 0.4f;
    if (mRegistry.hasComponent<Core::Material>(manifold->entityA) &&
        mRegistry.hasComponent<Core::Material>(manifold->entityB)) {
      const auto &matA = mRegistry.getComponent<Core::Material>(manifold->entityA);
      const auto &matB = mRegistry.getComponent<Core::Material>(manifold->entityB);
      friction = std::sqrt(matA.friction * matB.friction);
    }

    mSolverContacts.push_back(SolverContact{
        .bodyAIndex = idxA,
        .bodyBIndex = idxB,
        .manifold = manifold,
        .friction = friction
    });

    preStep(*manifold, mSolverBodies[idxA], mSolverBodies[idxB], deltaTime);
  }

  for (collision::DistanceConstraint *constraint : island.distanceConstraints) {
    uint32_t idxA = getOrAddBody(constraint->entityA);
    uint32_t idxB = getOrAddBody(constraint->entityB);
    mSolverDistanceConstraints.push_back(SolverDistanceConstraint{
        .bodyAIndex = idxA,
        .bodyBIndex = idxB,
        .constraint = constraint
    });
  }

  for (collision::RevoluteConstraint *constraint : island.revoluteConstraints) {
    uint32_t idxA = getOrAddBody(constraint->entityA);
    uint32_t idxB = getOrAddBody(constraint->entityB);
    mSolverRevoluteConstraints.push_back(SolverRevoluteConstraint{
        .bodyAIndex = idxA,
        .bodyBIndex = idxB,
        .constraint = constraint
    });
  }

  if (mSolverContacts.size() > 1) {
    std::sort(mSolverContacts.begin(), mSolverContacts.end(),
              [&](const SolverContact &a, const SolverContact &b) {
                bool aHasStatic = (mSolverBodies[a.bodyAIndex].type != Core::BodyType::Dynamic) ||
                                  (mSolverBodies[a.bodyBIndex].type != Core::BodyType::Dynamic);
                bool bHasStatic = (mSolverBodies[b.bodyAIndex].type != Core::BodyType::Dynamic) ||
                                  (mSolverBodies[b.bodyBIndex].type != Core::BodyType::Dynamic);
                if (aHasStatic != bHasStatic)
                  return aHasStatic > bHasStatic;

                float yA = (mSolverBodies[a.bodyAIndex].position.y + mSolverBodies[a.bodyBIndex].position.y) * 0.5f;
                float yB = (mSolverBodies[b.bodyAIndex].position.y + mSolverBodies[b.bodyBIndex].position.y) * 0.5f;
                return yA < yB;
              });
  }
}

void SequentialImpulsesKernel::solveVelocities(int velocityIterations, float deltaTime) {
  for (int iter = 0; iter < velocityIterations; ++iter) {
    for (const auto &contact : mSolverContacts) {
      resolveCollision(*contact.manifold, mSolverBodies[contact.bodyAIndex],
                       mSolverBodies[contact.bodyBIndex], contact.friction, deltaTime);
    }
    for (const auto &dc : mSolverDistanceConstraints) {
      resolveDistanceConstraint(*dc.constraint, mSolverBodies[dc.bodyAIndex],
                                mSolverBodies[dc.bodyBIndex], deltaTime);
    }
    for (const auto &rc : mSolverRevoluteConstraints) {
      resolveRevoluteConstraint(*rc.constraint, mSolverBodies[rc.bodyAIndex],
                                mSolverBodies[rc.bodyBIndex], deltaTime);
    }
  }
}

void SequentialImpulsesKernel::solvePositions(int positionIterations, float deltaTime) {
  for (int iter = 0; iter < positionIterations; ++iter) {
    for (const auto &contact : mSolverContacts) {
      solvePositionConstraints(*contact.manifold, mSolverBodies[contact.bodyAIndex],
                               mSolverBodies[contact.bodyBIndex], deltaTime);
    }
  }
}

void SequentialImpulsesKernel::writeBackVelocities() {
  for (const auto &sb : mSolverBodies) {
    if (sb.type == Core::BodyType::Dynamic) {
      if (mRegistry.hasComponent<Core::Motion>(sb.entity)) {
        auto &motion = mRegistry.getComponent<Core::Motion>(sb.entity);
        motion.linearVelocity = sb.linearVelocity;
        motion.angularVelocity = sb.angularVelocity;
        motion.splitLinearVelocity = sb.splitLinearVelocity;
        motion.splitAngularVelocity = sb.splitAngularVelocity;
      }
    }
  }
}

void SequentialImpulsesKernel::execute(
    Island &island,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (island.bodies.empty())
    return;

  if (island.manifolds.empty() && island.distanceConstraints.empty() &&
      island.revoluteConstraints.empty()) {
    return;
  }

  prepareIslandData(island, deltaTime);
  solveVelocities(velocityIterations, deltaTime);
  solvePositions(positionIterations, deltaTime);
  writeBackVelocities();
}

void SequentialImpulsesKernel::preStep(
    collision::ContactManifold &manifold,
    SolverBody &bodyA,
    SolverBody &bodyB,
    float deltaTime) {
  (void)deltaTime;

  const float invMassA = bodyA.invMass;
  const float invMassB = bodyB.invMass;
  const Matrix3x3 &invIA = bodyA.invInertiaTensor;
  const Matrix3x3 &invIB = bodyB.invInertiaTensor;

  float restitution = 0.0f;
  if (mRegistry.hasComponent<Core::Material>(manifold.entityA) &&
      mRegistry.hasComponent<Core::Material>(manifold.entityB)) {
    const auto &matA = mRegistry.getComponent<Core::Material>(manifold.entityA);
    const auto &matB = mRegistry.getComponent<Core::Material>(manifold.entityB);
    restitution = std::max(matA.restitution, matB.restitution);
  }

  manifold.computeTangents();
  Vector3 n = manifold.normal;
  Vector3 t1 = manifold.tangent1;
  Vector3 t2 = manifold.tangent2;

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];
    contact.splitImpulse = 0.0f;

    Vector3 rA = bodyA.orientation * contact.localPointA;
    Vector3 rB = bodyB.orientation * contact.localPointB;

    // Ön-çözüm hızları üzerinden Restitution bias hesaplama (Warm starting öncesi)
    Vector3 vA0 = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
    Vector3 vB0 = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
    float vn0 = (vA0 - vB0).dot(n);
    contact.velocityBias =
        (vn0 < -physics::PhysicsConstants::RestitutionVelocityThreshold)
            ? -restitution * vn0
            : 0.0f;

    // Warm Starting itmelerini hızlara uygula
    Vector3 totalImpulse = (n * contact.normalImpulse) +
                           (t1 * contact.tangentImpulse1) +
                           (t2 * contact.tangentImpulse2);

    bodyA.linearVelocity += totalImpulse * invMassA;
    bodyA.angularVelocity += invIA * rA.cross(totalImpulse);
    bodyB.linearVelocity -= totalImpulse * invMassB;
    bodyB.angularVelocity -= invIB * rB.cross(totalImpulse);
  }
}

void SequentialImpulsesKernel::resolveCollision(
    collision::ContactManifold &manifold,
    SolverBody &bodyA,
    SolverBody &bodyB,
    float friction,
    float deltaTime) {
  (void)deltaTime;

  const float invMassA = bodyA.invMass;
  const float invMassB = bodyB.invMass;
  const Matrix3x3 &invIA = bodyA.invInertiaTensor;
  const Matrix3x3 &invIB = bodyB.invInertiaTensor;

  Vector3 n = manifold.normal;
  Vector3 t1 = manifold.tangent1;
  Vector3 t2 = manifold.tangent2;
  float totalNormalImpulse = 0.0f;
  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];

    Vector3 rA = bodyA.orientation * contact.localPointA;
    Vector3 rB = bodyB.orientation * contact.localPointB;

    // 1. Normal İtme Çözümü
    Vector3 vA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
    Vector3 vB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
    Vector3 relVel = vA - vB;

    float normalVel = relVel.dot(n);
    float deltaV = -normalVel + contact.velocityBias;

    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSumN = invMassA + invMassB + (invIA * rACrossN).dot(rACrossN) +
                        (invIB * rBCrossN).dot(rBCrossN);
    if (invMassSumN > 1e-6f) {
      float jn = deltaV / invMassSumN;
      float oldImpulse = contact.normalImpulse;
      contact.normalImpulse = std::max(oldImpulse + jn, 0.0f);
      jn = contact.normalImpulse - oldImpulse;

      Vector3 impulse = n * jn;
      bodyA.linearVelocity += impulse * invMassA;
      bodyA.angularVelocity += invIA * rA.cross(impulse);
      bodyB.linearVelocity -= impulse * invMassB;
      bodyB.angularVelocity -= invIB * rB.cross(impulse);
    }

    // 2. Çift Teğetli Coulomb Sürtünmesi Çözümü (T1 ve T2)
    float maxFriction = friction * contact.normalImpulse;

    // Teğet 1
    vA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
    vB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
    relVel = vA - vB;
    Vector3 rACrossT1 = rA.cross(t1);
    Vector3 rBCrossT1 = rB.cross(t1);
    float invMassSumT1 = invMassA + invMassB +
                         (invIA * rACrossT1).dot(rACrossT1) +
                         (invIB * rBCrossT1).dot(rBCrossT1);
    if (invMassSumT1 > 1e-6f) {
      float jt1 = -relVel.dot(t1) / invMassSumT1;
      float oldT1 = contact.tangentImpulse1;
      contact.tangentImpulse1 =
          std::clamp(oldT1 + jt1, -maxFriction, maxFriction);
      jt1 = contact.tangentImpulse1 - oldT1;

      Vector3 impulseT1 = t1 * jt1;
      bodyA.linearVelocity += impulseT1 * invMassA;
      bodyA.angularVelocity += invIA * rA.cross(impulseT1);
      bodyB.linearVelocity -= impulseT1 * invMassB;
      bodyB.angularVelocity -= invIB * rB.cross(impulseT1);
    }

    // Teğet 2
    vA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
    vB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
    relVel = vA - vB;
    Vector3 rACrossT2 = rA.cross(t2);
    Vector3 rBCrossT2 = rB.cross(t2);
    float invMassSumT2 = invMassA + invMassB +
                         (invIA * rACrossT2).dot(rACrossT2) +
                         (invIB * rBCrossT2).dot(rBCrossT2);
    if (invMassSumT2 > 1e-6f) {
      float jt2 = -relVel.dot(t2) / invMassSumT2;
      float oldT2 = contact.tangentImpulse2;
      contact.tangentImpulse2 =
          std::clamp(oldT2 + jt2, -maxFriction, maxFriction);
      jt2 = contact.tangentImpulse2 - oldT2;

      Vector3 impulseT2 = t2 * jt2;
      bodyA.linearVelocity += impulseT2 * invMassA;
      bodyA.angularVelocity += invIA * rA.cross(impulseT2);
      bodyB.linearVelocity -= impulseT2 * invMassB;
      bodyB.angularVelocity -= invIB * rB.cross(impulseT2);
    }

    totalNormalImpulse += contact.normalImpulse;
  }

  // 3. Yuvarlanma Direnci (Manifold / Gövde çifti başına tekil uygulanır)
  Vector3 relAngVel = bodyA.angularVelocity - bodyB.angularVelocity;
  float relAngSpeed = relAngVel.length();
  if (relAngSpeed > 1e-4f && totalNormalImpulse > 0.0f) {
    Vector3 rollDir = relAngVel / relAngSpeed;
    float maxRollImpulse = physics::PhysicsConstants::RollingResistanceCoefficient * friction * totalNormalImpulse;
    float invAngMassSum = (invIA * rollDir).dot(rollDir) + (invIB * rollDir).dot(rollDir);
    if (invAngMassSum > 1e-6f) {
      float jRoll = std::min(relAngSpeed / invAngMassSum, maxRollImpulse);
      Vector3 rollImpulse = rollDir * jRoll;
      bodyA.angularVelocity -= invIA * rollImpulse;
      bodyB.angularVelocity += invIB * rollImpulse;
    }
  }
}

void SequentialImpulsesKernel::solvePositionConstraints(
    collision::ContactManifold &manifold,
    SolverBody &bodyA,
    SolverBody &bodyB,
    float deltaTime) {
  if (deltaTime <= 0.0f)
    return;

  const float invMassA = bodyA.invMass;
  const float invMassB = bodyB.invMass;
  const Matrix3x3 &invIA = bodyA.invInertiaTensor;
  const Matrix3x3 &invIB = bodyB.invInertiaTensor;

  Vector3 n = manifold.normal;

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];

    Vector3 rA = bodyA.orientation * contact.localPointA;
    Vector3 rB = bodyB.orientation * contact.localPointB;

    Vector3 pA = bodyA.position + rA;
    Vector3 pB = bodyB.position + rB;
    float separation = (pA - pB).dot(n);
    float currentPenetration = contact.penetration - separation;

    float C =
        std::clamp(currentPenetration - physics::PhysicsConstants::LinearSlop,
                   0.0f, physics::PhysicsConstants::MaxLinearCorrection);
    if (C <= 0.0f)
      continue;

    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = invMassA + invMassB + (invIA * rACrossN).dot(rACrossN) +
                       (invIB * rBCrossN).dot(rBCrossN);
    if (invMassSum <= 1e-6f)
      continue;

    // Split impulse: v_split'e itme uygula
    Vector3 vSplitA =
        bodyA.splitLinearVelocity + bodyA.splitAngularVelocity.cross(rA);
    Vector3 vSplitB =
        bodyB.splitLinearVelocity + bodyB.splitAngularVelocity.cross(rB);
    float splitVel = (vSplitA - vSplitB).dot(n);

    float bias =
        (physics::PhysicsConstants::PositionCorrectionFactor / deltaTime) * C;
    float deltaSplit = (-splitVel + bias) / invMassSum;

    float oldSplit = contact.splitImpulse;
    contact.splitImpulse = std::max(oldSplit + deltaSplit, 0.0f);
    deltaSplit = contact.splitImpulse - oldSplit;

    Vector3 impulse = n * deltaSplit;
    bodyA.splitLinearVelocity += impulse * invMassA;
    bodyA.splitAngularVelocity += invIA * rA.cross(impulse);
    bodyB.splitLinearVelocity -= impulse * invMassB;
    bodyB.splitAngularVelocity -= invIB * rB.cross(impulse);
  }
}

void SequentialImpulsesKernel::resolveDistanceConstraint(
    collision::DistanceConstraint &constraint,
    SolverBody &bodyA,
    SolverBody &bodyB,
    float deltaTime) {
  if (deltaTime <= 0.0f)
    return;

  // 1. Bağlantı noktalarının dünya koordinatlarındaki yerini bul
  Vector3 rA = bodyA.orientation * constraint.localAnchorA;
  Vector3 rB = bodyB.orientation * constraint.localAnchorB;
  Vector3 pA = bodyA.position + rA;
  Vector3 pB = bodyB.position + rB;

  // 2. İki nokta arasındaki mesafe vektörü ve yönü
  Vector3 delta = pA - pB;
  float currentDist = delta.length();
  if (currentDist < 1e-6f)
    return;
  Vector3 u = delta / currentDist; // Birim yön vektörü (B'den A'ya)

  // 3. Bağlantı noktalarındaki çizgisel hızları hesapla (v + w x r)
  Vector3 vA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
  Vector3 vB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
  float relVel = (vA - vB).dot(u);

  // 4. BAUMGARTE POZİSYON DÜZELTMESİ (Drift Engelleme)
  float positionError = currentDist - constraint.targetDistance;
  float baumgarteFactor = 0.2f; // Her karede hatanın %20'sini telafi et
  float bias = (baumgarteFactor / deltaTime) * positionError;

  // 5. Etkili kütle toplamı (Doğrusal kütle + Dönme dirençleri)
  Vector3 rACrossU = rA.cross(u);
  Vector3 rBCrossU = rB.cross(u);
  float invMassSum = bodyA.invMass + bodyB.invMass +
                     (bodyA.invInertiaTensor * rACrossU).dot(rACrossU) +
                     (bodyB.invInertiaTensor * rBCrossU).dot(rBCrossU);
  if (invMassSum < 1e-6f)
    return;

  // 6. Gerekli itme kuvvetini hesapla ve sertlik (stiffness) ile uygula
  float j = (-relVel - bias) / invMassSum;
  j *= std::clamp(constraint.stiffness, 0.0f, 1.0f);

  Vector3 impulse = u * j;

  // 7. Hızları güncelle
  bodyA.linearVelocity += impulse * bodyA.invMass;
  bodyA.angularVelocity += bodyA.invInertiaTensor * rA.cross(impulse);
  bodyB.linearVelocity -= impulse * bodyB.invMass;
  bodyB.angularVelocity -= bodyB.invInertiaTensor * rB.cross(impulse);
}

void SequentialImpulsesKernel::resolveRevoluteConstraint(
    collision::RevoluteConstraint &constraint,
    SolverBody &bodyA,
    SolverBody &bodyB,
    float deltaTime) {
  (void)deltaTime;

  Vector3 rA = bodyA.orientation * constraint.localAnchorA;
  Vector3 rB = bodyB.orientation * constraint.localAnchorB;

  // AŞAMA 1: Noktasal Bağlantı (3 Boyutta Ayrılmayı Engelle)
  Vector3 vA = bodyA.linearVelocity + bodyA.angularVelocity.cross(rA);
  Vector3 vB = bodyB.linearVelocity + bodyB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  // 3 eksen için itme hesaplama (Basitleştirilmiş skaler yaklaşım yerine yönlü itme)
  for (int axis = 0; axis < 3; ++axis) {
    Vector3 n(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f,
              axis == 2 ? 1.0f : 0.0f);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = bodyA.invMass + bodyB.invMass +
                       (bodyA.invInertiaTensor * rACrossN).dot(rACrossN) +
                       (bodyB.invInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float deltaV = -relVel.dot(n);
      float j = deltaV / invMassSum;
      Vector3 impulse = n * j;
      bodyA.linearVelocity += impulse * bodyA.invMass;
      bodyA.angularVelocity += bodyA.invInertiaTensor * rA.cross(impulse);
      bodyB.linearVelocity -= impulse * bodyB.invMass;
      bodyB.angularVelocity -= bodyB.invInertiaTensor * rB.cross(impulse);
    }
  }

  // AŞAMA 2: Açısal Eksen Kısıtlaması (Sadece Menteşe Ekseni Etrafında Dönüşe İzin Ver)
  Vector3 axisA_World =
      (bodyA.orientation * constraint.hingeAxisLocalA).getNormalized();
  Vector3 u1, u2;
  // Menteşe eksenine dik iki ortogonal vektör bul:
  if (std::abs(axisA_World.x) >= 0.57735f)
    u1 = Vector3(axisA_World.y, -axisA_World.x, 0.0f).getNormalized();
  else
    u1 = Vector3(0.0f, axisA_World.z, -axisA_World.y).getNormalized();
  u2 = axisA_World.cross(u1);

  Vector3 relAngVel = bodyA.angularVelocity - bodyB.angularVelocity;

  // U1 eksenindeki istenmeyen dönmeyi sıfırla
  float invIA_u1 = (bodyA.invInertiaTensor * u1).dot(u1);
  float invIB_u1 = (bodyB.invInertiaTensor * u1).dot(u1);
  if (invIA_u1 + invIB_u1 > 1e-6f) {
    float jAng1 = -relAngVel.dot(u1) / (invIA_u1 + invIB_u1);
    bodyA.angularVelocity += bodyA.invInertiaTensor * (u1 * jAng1);
    bodyB.angularVelocity -= bodyB.invInertiaTensor * (u1 * jAng1);
  }

  // U2 eksenindeki istenmeyen dönmeyi sıfırla
  float invIA_u2 = (bodyA.invInertiaTensor * u2).dot(u2);
  float invIB_u2 = (bodyB.invInertiaTensor * u2).dot(u2);
  if (invIA_u2 + invIB_u2 > 1e-6f) {
    float jAng2 = -relAngVel.dot(u2) / (invIA_u2 + invIB_u2);
    bodyA.angularVelocity += bodyA.invInertiaTensor * (u2 * jAng2);
    bodyB.angularVelocity -= bodyB.invInertiaTensor * (u2 * jAng2);
  }
}

} // namespace Baryon::systems
