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
 * Impulse içerir.
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

void SequentialImpulsesKernel::collectIslandManifolds(
    const std::vector<ecs::Entity> &island,
    std::unordered_map<uint64_t, collision::ContactManifold> &manifoldMap,
    std::vector<collision::ContactManifold *> &localManifolds) {
  localManifolds.reserve(std::min(manifoldMap.size(), static_cast<size_t>(64)));

  if (island.size() == 1) {
    const uint64_t singleId = island[0].id;
    for (auto &[key, manifold] : manifoldMap) {
      if (manifold.entityA.id == singleId || manifold.entityB.id == singleId) {
        localManifolds.push_back(&manifold);
      }
    }
  } else if (island.size() <= 16) {
    for (auto &[key, manifold] : manifoldMap) {
      uint64_t idA = manifold.entityA.id;
      uint64_t idB = manifold.entityB.id;
      for (ecs::Entity e : island) {
        if (e.id == idA || e.id == idB) {
          localManifolds.push_back(&manifold);
          break;
        }
      }
    }
  } else {
    std::unordered_set<uint64_t> islandIds;
    islandIds.reserve(island.size() * 2);
    for (ecs::Entity e : island) {
      islandIds.insert(e.id);
    }
    for (auto &[key, manifold] : manifoldMap) {
      if (islandIds.contains(manifold.entityA.id) ||
          islandIds.contains(manifold.entityB.id)) {
        localManifolds.push_back(&manifold);
      }
    }
  }
}

void SequentialImpulsesKernel::collectIslandConstraints(
    const std::vector<ecs::Entity> &island,
    std::vector<collision::DistanceConstraint *> &localDistance,
    std::vector<collision::RevoluteConstraint *> &localRevolute) {
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
      if (match)
        localDistance.push_back(&constraint);
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
      if (match)
        localRevolute.push_back(&constraint);
    }
  }
}

void SequentialImpulsesKernel::solveSingleIsland(
    const std::vector<collision::ContactManifold *> &localManifolds,
    const std::vector<collision::DistanceConstraint *> &localDist,
    const std::vector<collision::RevoluteConstraint *> &localRev,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (localManifolds.empty() && localDist.empty() && localRev.empty()) {
    return;
  }

  // 1. ÖN-ADIM: Ortogonal teğetleri ve Warm-Starting kuvvetlerini hızlara uygula
  for (collision::ContactManifold *manifold : localManifolds) {
    if (!manifold->isTriggerPair)
      preStep(*manifold, deltaTime);
  }

  // 2. Hız İterasyonları: Gauss-Seidel yaklaşımı
  for (int iter = 0; iter < velocityIterations; ++iter) {
    for (collision::ContactManifold *manifold : localManifolds) {
      if (!manifold->isTriggerPair)
        resolveCollision(*manifold, deltaTime);
    }
    for (collision::DistanceConstraint *constraint : localDist) {
      resolveDistanceConstraint(*constraint, deltaTime);
    }
    for (collision::RevoluteConstraint *constraint : localRev) {
      resolveRevoluteConstraint(*constraint, deltaTime);
    }
  }

  // 3. Pozisyon İterasyonları: Split Impulse
  for (int iter = 0; iter < positionIterations; ++iter) {
    for (collision::ContactManifold *manifold : localManifolds) {
      if (!manifold->isTriggerPair)
        solvePositionConstraints(*manifold, deltaTime);
    }
  }
}

void SequentialImpulsesKernel::execute(
    const std::vector<ecs::Entity> &island,
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (island.empty())
    return;

  std::vector<collision::ContactManifold *> localManifolds;
  collectIslandManifolds(island, manifolds, localManifolds);

  std::vector<collision::DistanceConstraint *> localDistance;
  std::vector<collision::RevoluteConstraint *> localRevolute;
  collectIslandConstraints(island, localDistance, localRevolute);

  solveSingleIsland(localManifolds, localDistance, localRevolute, deltaTime,
                    velocityIterations, positionIterations);
}

std::vector<std::vector<collision::ContactManifold *>>
SequentialImpulsesKernel::routeAndSortIslandManifolds(
    std::unordered_map<uint64_t, collision::ContactManifold> &manifoldMap,
    const std::unordered_map<uint64_t, size_t> &entityToIsland,
    size_t islandCount) {
  std::vector<std::vector<collision::ContactManifold *>> islandManifolds(islandCount);
  for (auto &[key, manifold] : manifoldMap) {
    auto itA = entityToIsland.find(manifold.entityA.id);
    if (itA != entityToIsland.end()) {
      islandManifolds[itA->second].push_back(&manifold);
    } else {
      auto itB = entityToIsland.find(manifold.entityB.id);
      if (itB != entityToIsland.end()) {
        islandManifolds[itB->second].push_back(&manifold);
      }
    }
  }

  for (auto &list : islandManifolds) {
    auto lowY = [&](collision::ContactManifold *m) {
      float ya = mRegistry.hasComponent<Pose>(m->entityA) ? mRegistry.getComponent<Pose>(m->entityA).position.y : 1e9f;
      float yb = mRegistry.hasComponent<Pose>(m->entityB) ? mRegistry.getComponent<Pose>(m->entityB).position.y : 1e9f;
      return std::min(ya, yb);
    };
    std::sort(list.begin(), list.end(), [&](auto *a, auto *b) {
      float ya = lowY(a), yb = lowY(b);
      return ya != yb ? ya < yb : a->entityA.id < b->entityA.id;
    });
  }
  return islandManifolds;
}

std::vector<std::vector<collision::DistanceConstraint *>>
SequentialImpulsesKernel::routeIslandDistanceConstraints(
    const std::unordered_map<uint64_t, size_t> &entityToIsland,
    size_t islandCount) {
  std::vector<std::vector<collision::DistanceConstraint *>> islandDistance(islandCount);
  if (mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData().size() > 0) {
    auto &constraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
    for (auto &constraint : constraints) {
      auto itA = entityToIsland.find(constraint.entityA.id);
      if (itA != entityToIsland.end()) {
        islandDistance[itA->second].push_back(&constraint);
      } else {
        auto itB = entityToIsland.find(constraint.entityB.id);
        if (itB != entityToIsland.end()) {
          islandDistance[itB->second].push_back(&constraint);
        }
      }
    }
  }
  return islandDistance;
}

std::vector<std::vector<collision::RevoluteConstraint *>>
SequentialImpulsesKernel::routeIslandRevoluteConstraints(
    const std::unordered_map<uint64_t, size_t> &entityToIsland,
    size_t islandCount) {
  std::vector<std::vector<collision::RevoluteConstraint *>> islandRevolute(islandCount);
  if (mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData().size() > 0) {
    auto &constraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
    for (auto &constraint : constraints) {
      auto itA = entityToIsland.find(constraint.entityA.id);
      if (itA != entityToIsland.end()) {
        islandRevolute[itA->second].push_back(&constraint);
      } else {
        auto itB = entityToIsland.find(constraint.entityB.id);
        if (itB != entityToIsland.end()) {
          islandRevolute[itB->second].push_back(&constraint);
        }
      }
    }
  }
  return islandRevolute;
}

void SequentialImpulsesKernel::executeIslands(
    const std::vector<std::vector<ecs::Entity>> &islands,
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (islands.empty())
    return;

  // 1. Dinamik varlıkları ada indekslerine eşle (O(N))
  std::unordered_map<uint64_t, size_t> entityToIsland;
  size_t totalEntities = 0;
  for (const auto &island : islands) {
    totalEntities += island.size();
  }
  entityToIsland.reserve(totalEntities * 2);

  for (size_t i = 0; i < islands.size(); ++i) {
    for (ecs::Entity e : islands[i]) {
      entityToIsland[e.id] = i;
    }
  }

  // 2. Manifoldları ilgili adalara yönlendir ve sırala (O(M))
  auto islandManifolds = routeAndSortIslandManifolds(manifolds, entityToIsland, islands.size());

  // 3. Mesafe kısıtlamalarını ilgili adalara yönlendir
  auto islandDistance = routeIslandDistanceConstraints(entityToIsland, islands.size());

  // 4. Menteşe kısıtlamalarını ilgili adalara yönlendir
  auto islandRevolute = routeIslandRevoluteConstraints(entityToIsland, islands.size());

  // 5. Her adayı kendi temasları ve kısıtlamaları ile çöz
  for (size_t i = 0; i < islands.size(); ++i) {
    solveSingleIsland(islandManifolds[i], islandDistance[i], islandRevolute[i],
                      deltaTime, velocityIterations, positionIterations);
  }
}

void SequentialImpulsesKernel::solveIsland(
    const std::vector<ecs::Entity> &island,
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime) {
  execute(island, manifolds, deltaTime, 10, 4);
}

void SequentialImpulsesKernel::solveConstraintsForIsland(
    const std::vector<ecs::Entity> &island, float deltaTime) {
  (void)island;
  (void)deltaTime;
}

void SequentialImpulsesKernel::preStep(collision::ContactManifold &manifold,
                                       float deltaTime) {
  (void)deltaTime;
  if (!mRegistry.hasComponent<Pose>(manifold.entityA) ||
      !mRegistry.hasComponent<Pose>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::Motion>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::MassProps>(manifold.entityB))
    return;

  auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
  auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
  auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
  auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
  const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

  Core::BodyType typeA = Core::BodyType::Dynamic;
  Core::BodyType typeB = Core::BodyType::Dynamic;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA))
    typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB))
    typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic)
                              ? massA.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic)
                              ? massB.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

  manifold.computeTangents();
  Vector3 n = manifold.normal;
  Vector3 t1 = manifold.tangent1;
  Vector3 t2 = manifold.tangent2;

  float restitution = 0.0f;
  if (mRegistry.hasComponent<Core::Material>(manifold.entityA) &&
      mRegistry.hasComponent<Core::Material>(manifold.entityB)) {
    const auto &matA = mRegistry.getComponent<Core::Material>(manifold.entityA);
    const auto &matB = mRegistry.getComponent<Core::Material>(manifold.entityB);
    restitution = std::max(matA.restitution, matB.restitution);
  }

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];
    contact.splitImpulse = 0.0f;

    Vector3 rA = poseA.orientation * contact.localPointA;
    Vector3 rB = poseB.orientation * contact.localPointB;

    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relVel = vA - vB;
    float normalVel = relVel.dot(n);

    contact.velocityBias = 0.0f;
    if (normalVel < -physics::PhysicsConstants::RestitutionVelocityThreshold) {
      contact.velocityBias = -restitution * normalVel;
    }
  }

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];
    Vector3 rA = poseA.orientation * contact.localPointA;
    Vector3 rB = poseB.orientation * contact.localPointB;

    // Warm Starting itmelerini hızlara uygula
    Vector3 totalImpulse = (n * contact.normalImpulse) +
                           (t1 * contact.tangentImpulse1) +
                           (t2 * contact.tangentImpulse2);

    motionA.linearVelocity += totalImpulse * invMassA;
    motionA.angularVelocity += invIA * rA.cross(totalImpulse);
    motionB.linearVelocity -= totalImpulse * invMassB;
    motionB.angularVelocity -= invIB * rB.cross(totalImpulse);
  }
}

void SequentialImpulsesKernel::solveContactNormal(
    collision::ContactPoint &contact,
    const Vector3 &rA, const Vector3 &rB, const Vector3 &n,
    float invMassA, float invMassB,
    const Matrix3x3 &invIA, const Matrix3x3 &invIB,
    Core::Motion &motionA, Core::Motion &motionB) {
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
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
    motionA.linearVelocity += impulse * invMassA;
    motionA.angularVelocity += invIA * rA.cross(impulse);
    motionB.linearVelocity -= impulse * invMassB;
    motionB.angularVelocity -= invIB * rB.cross(impulse);
  }
}

void SequentialImpulsesKernel::solveContactFriction(
    collision::ContactPoint &contact,
    const Vector3 &rA, const Vector3 &rB,
    const Vector3 &t1, const Vector3 &t2,
    float friction,
    float invMassA, float invMassB,
    const Matrix3x3 &invIA, const Matrix3x3 &invIB,
    Core::Motion &motionA, Core::Motion &motionB) {
  float maxFriction = friction * contact.normalImpulse;

  // Teğet 1
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;
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
    motionA.linearVelocity += impulseT2 * invMassA;
    motionA.angularVelocity += invIA * rA.cross(impulseT2);
    motionB.linearVelocity -= impulseT2 * invMassB;
    motionB.angularVelocity -= invIB * rB.cross(impulseT2);
  }
}

void SequentialImpulsesKernel::applyRollingResistance(
    float friction, float totalNormalImpulse,
    const Matrix3x3 &invIA, const Matrix3x3 &invIB,
    Core::Motion &motionA, Core::Motion &motionB) {
  Vector3 relAngVel = motionA.angularVelocity - motionB.angularVelocity;
  float relAngSpeed = relAngVel.length();
  if (relAngSpeed > physics::PhysicsConstants::RollingFrictionAngularVelocityThreshold && totalNormalImpulse > 0.0f) {
    Vector3 rollDir = relAngVel / relAngSpeed;
    float maxRollImpulse = physics::PhysicsConstants::RollingFrictionCoefficient * friction * totalNormalImpulse;
    float invAngMassSum = (invIA * rollDir).dot(rollDir) + (invIB * rollDir).dot(rollDir);
    if (invAngMassSum > 1e-6f) {
      float jRoll = std::min(relAngSpeed / invAngMassSum, maxRollImpulse);
      Vector3 rollImpulse = rollDir * jRoll;
      motionA.angularVelocity -= invIA * rollImpulse;
      motionB.angularVelocity += invIB * rollImpulse;
    }
  }
}

void SequentialImpulsesKernel::resolveCollision(
    collision::ContactManifold &manifold, float deltaTime) {
  (void)deltaTime;
  if (!mRegistry.hasComponent<Pose>(manifold.entityA) ||
      !mRegistry.hasComponent<Pose>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::Motion>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::MassProps>(manifold.entityB))
    return;

  auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
  auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
  auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
  auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
  const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

  Core::BodyType typeA = Core::BodyType::Dynamic;
  Core::BodyType typeB = Core::BodyType::Dynamic;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA))
    typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB))
    typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic)
                              ? massA.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic)
                              ? massB.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

  float friction = 0.4f;
  if (mRegistry.hasComponent<Core::Material>(manifold.entityA) &&
      mRegistry.hasComponent<Core::Material>(manifold.entityB)) {
    const auto &matA = mRegistry.getComponent<Core::Material>(manifold.entityA);
    const auto &matB = mRegistry.getComponent<Core::Material>(manifold.entityB);
    friction = std::sqrt(matA.friction * matB.friction);
  }

  Vector3 n = manifold.normal;
  Vector3 t1 = manifold.tangent1;
  Vector3 t2 = manifold.tangent2;
  float totalNormalImpulse = 0.0f;

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];
    Vector3 rA = poseA.orientation * contact.localPointA;
    Vector3 rB = poseB.orientation * contact.localPointB;

    solveContactNormal(contact, rA, rB, n, invMassA, invMassB, invIA, invIB, motionA, motionB);
    solveContactFriction(contact, rA, rB, t1, t2, friction, invMassA, invMassB, invIA, invIB, motionA, motionB);
    totalNormalImpulse += contact.normalImpulse;
  }

  applyRollingResistance(friction, totalNormalImpulse, invIA, invIB, motionA, motionB);
}

void SequentialImpulsesKernel::solvePositionConstraints(
    collision::ContactManifold &manifold, float deltaTime) {
  if (deltaTime <= 0.0f)
    return;
  if (!mRegistry.hasComponent<Pose>(manifold.entityA) ||
      !mRegistry.hasComponent<Pose>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::Motion>(manifold.entityB))
    return;
  if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) ||
      !mRegistry.hasComponent<Core::MassProps>(manifold.entityB))
    return;

  auto &poseA = mRegistry.getComponent<Pose>(manifold.entityA);
  auto &poseB = mRegistry.getComponent<Pose>(manifold.entityB);
  auto &motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
  auto &motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
  const auto &massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

  Core::BodyType typeA = Core::BodyType::Dynamic;
  Core::BodyType typeB = Core::BodyType::Dynamic;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA))
    typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB))
    typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic)
                              ? massA.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic)
                              ? massB.inverseInertiaTensor
                              : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

  Vector3 n = manifold.normal;

  for (uint32_t i = 0; i < manifold.contactCount; ++i) {
    auto &contact = manifold.contacts[i];

    Vector3 rA = poseA.orientation * contact.localPointA;
    Vector3 rB = poseB.orientation * contact.localPointB;

    Vector3 pA = poseA.position + rA;
    Vector3 pB = poseB.position + rB;
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
        motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
    Vector3 vSplitB =
        motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
    float splitVel = (vSplitA - vSplitB).dot(n);

    float bias =
        (physics::PhysicsConstants::PositionCorrectionFactor / deltaTime) * C;
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
  if (deltaTime <= 0.0f)
    return;
  if (!mRegistry.hasComponent<Pose>(constraint.entityA) ||
      !mRegistry.hasComponent<Pose>(constraint.entityB))
    return;
  if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) ||
      !mRegistry.hasComponent<Core::Motion>(constraint.entityB))
    return;
  if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) ||
      !mRegistry.hasComponent<Core::MassProps>(constraint.entityB))
    return;

  auto &poseA = mRegistry.getComponent<Pose>(constraint.entityA);
  auto &poseB = mRegistry.getComponent<Pose>(constraint.entityB);
  auto &motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
  auto &motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
  const auto &massA =
      mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB =
      mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  // 1. Bağlantı noktalarının dünya koordinatlarındaki yerini bul
  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // 2. İki nokta arasındaki mesafe vektörü ve yönü
  Vector3 delta = pA - pB;
  float currentDist = delta.length();
  if (currentDist < 1e-6f)
    return;
  Vector3 u = delta / currentDist; // Birim yön vektörü (B'den A'ya)

  // 3. Bağlantı noktalarındaki çizgisel hızları hesapla (v + w x r)
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  float relVel = (vA - vB).dot(u);

  // 4. BAUMGARTE POZİSYON DÜZELTMESİ (Drift Engelleme)
  float positionError = currentDist - constraint.targetDistance;
  float baumgarteFactor = 0.2f; // Her karede hatanın %20'sini telafi et
  float bias = (baumgarteFactor / deltaTime) * positionError;

  // 5. Etkili kütle toplamı (Doğrusal kütle + Dönme dirençleri)
  Vector3 rACrossU = rA.cross(u);
  Vector3 rBCrossU = rB.cross(u);
  float invMassSum = massA.inverseMass + massB.inverseMass +
                     (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) +
                     (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
  if (invMassSum < 1e-6f)
    return;

  // 6. Gerekli itme kuvvetini hesapla ve sertlik (stiffness) ile uygula
  float j = (-relVel - bias) / invMassSum;
  j *= std::clamp(constraint.stiffness, 0.0f, 1.0f);

  Vector3 impulse = u * j;

  // 7. Hızları güncelle
  motionA.linearVelocity += impulse * massA.inverseMass;
  motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
  motionB.linearVelocity -= impulse * massB.inverseMass;
  motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
}

void SequentialImpulsesKernel::resolveRevoluteConstraint(
    collision::RevoluteConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  if (!mRegistry.hasComponent<Pose>(constraint.entityA) ||
      !mRegistry.hasComponent<Pose>(constraint.entityB))
    return;
  if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) ||
      !mRegistry.hasComponent<Core::Motion>(constraint.entityB))
    return;
  if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) ||
      !mRegistry.hasComponent<Core::MassProps>(constraint.entityB))
    return;

  auto &poseA = mRegistry.getComponent<Pose>(constraint.entityA);
  auto &poseB = mRegistry.getComponent<Pose>(constraint.entityB);
  auto &motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
  auto &motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
  const auto &massA =
      mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB =
      mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;

  // AŞAMA 1: Noktasal Bağlantı (3 Boyutta Ayrılmayı Engelle)
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  // 3 eksen için itme hesaplama (Basitleştirilmiş skaler yaklaşım yerine yönlü
  // itme)
  for (int axis = 0; axis < 3; ++axis) {
    Vector3 n(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f,
              axis == 2 ? 1.0f : 0.0f);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float deltaV = -relVel.dot(n);
      float j = deltaV / invMassSum;
      Vector3 impulse = n * j;
      motionA.linearVelocity += impulse * massA.inverseMass;
      motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
      motionB.linearVelocity -= impulse * massB.inverseMass;
      motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
    }
  }

  // AŞAMA 2: Açısal Eksen Kısıtlaması (Sadece Menteşe Ekseni Etrafında Dönüşe
  // İzin Ver)
  Vector3 axisA_World =
      (poseA.orientation * constraint.hingeAxisLocalA).getNormalized();
  Vector3 u1, u2;
  // Menteşe eksenine dik iki ortogonal vektör bul:
  if (std::abs(axisA_World.x) >= 0.57735f)
    u1 = Vector3(axisA_World.y, -axisA_World.x, 0.0f).getNormalized();
  else
    u1 = Vector3(0.0f, axisA_World.z, -axisA_World.y).getNormalized();
  u2 = axisA_World.cross(u1);

  Vector3 relAngVel = motionA.angularVelocity - motionB.angularVelocity;

  // U1 eksenindeki istenmeyen dönmeyi sıfırla
  float invIA_u1 = (massA.inverseInertiaTensor * u1).dot(u1);
  float invIB_u1 = (massB.inverseInertiaTensor * u1).dot(u1);
  if (invIA_u1 + invIB_u1 > 1e-6f) {
    float jAng1 = -relAngVel.dot(u1) / (invIA_u1 + invIB_u1);
    motionA.angularVelocity += massA.inverseInertiaTensor * (u1 * jAng1);
    motionB.angularVelocity -= massB.inverseInertiaTensor * (u1 * jAng1);
  }

  // U2 eksenindeki istenmeyen dönmeyi sıfırla
  float invIA_u2 = (massA.inverseInertiaTensor * u2).dot(u2);
  float invIB_u2 = (massB.inverseInertiaTensor * u2).dot(u2);
  if (invIA_u2 + invIB_u2 > 1e-6f) {
    float jAng2 = -relAngVel.dot(u2) / (invIA_u2 + invIB_u2);
    motionA.angularVelocity += massA.inverseInertiaTensor * (u2 * jAng2);
    motionB.angularVelocity -= massB.inverseInertiaTensor * (u2 * jAng2);
  }
}

} // namespace Baryon::systems
