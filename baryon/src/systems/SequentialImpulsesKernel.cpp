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
#include "Baryon/collision/BallSocketConstraint.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/FixedConstraint.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include "Baryon/collision/PrismaticConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace Baryon::systems {

namespace {

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

} // namespace

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

void SequentialImpulsesKernel::prepareConstraint(
    collision::DistanceConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::DistanceConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::DistanceConstraint &constraint, float deltaTime) {
  resolveDistanceConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::DistanceConstraint &constraint, float deltaTime) {
  solveDistanceConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::RevoluteConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.accumulatedImpulsePoint = Vector3(0.0f, 0.0f, 0.0f);
  constraint.accumulatedImpulseHinge = Vector3(0.0f, 0.0f, 0.0f);
  constraint.accumulatedMotorImpulse = 0.0f;
  constraint.accumulatedLimitImpulse = 0.0f;
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::RevoluteConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::RevoluteConstraint &constraint, float deltaTime) {
  resolveRevoluteConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::RevoluteConstraint &constraint, float deltaTime) {
  solveRevoluteConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::FixedConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::FixedConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::FixedConstraint &constraint, float deltaTime) {
  resolveFixedConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::FixedConstraint &constraint, float deltaTime) {
  solveFixedConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::BallSocketConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.accumulatedImpulseLinear = Vector3(0.0f, 0.0f, 0.0f);
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::BallSocketConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::BallSocketConstraint &constraint, float deltaTime) {
  resolveBallSocketConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::BallSocketConstraint &constraint, float deltaTime) {
  solveBallSocketConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::PrismaticConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.accumulatedImpulseLinear = Vector3(0.0f, 0.0f, 0.0f);
  constraint.accumulatedImpulseAngular = Vector3(0.0f, 0.0f, 0.0f);
  constraint.motorImpulse = 0.0f;
  constraint.limitImpulse = 0.0f;
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::PrismaticConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::PrismaticConstraint &constraint, float deltaTime) {
  resolvePrismaticConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::PrismaticConstraint &constraint, float deltaTime) {
  solvePrismaticConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::SpringConstraint &constraint, float deltaTime) {
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
  if (deltaTime <= 0.0f)
    return;
  float k = std::max(0.0f, constraint.stiffness);
  float c = std::max(0.0f, constraint.damping);
  float denom = deltaTime * (c + deltaTime * k);
  if (denom > 1e-7f) {
    constraint.gamma = 1.0f / denom;
    constraint.biasFactor = (deltaTime * k) / (c + deltaTime * k);
  } else {
    constraint.gamma = 0.0f;
    constraint.biasFactor = 0.0f;
  }
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::SpringConstraint &constraint) {
  if (std::abs(constraint.accumulatedImpulse) < 1e-6f)
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;
  Vector3 delta = pB - pA;
  float currentDist = delta.length();
  if (currentDist < 1e-6f)
    return;
  Vector3 u = delta / currentDist;

  Vector3 impulse = u * constraint.accumulatedImpulse;
  motionB.linearVelocity += impulse * massB.inverseMass;
  motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
  motionA.linearVelocity -= impulse * massA.inverseMass;
  motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::SpringConstraint &constraint, float deltaTime) {
  resolveSpringConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::SpringConstraint &constraint, float deltaTime) {
  solveSpringConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::prepareConstraint(
    collision::ConeTwistConstraint &constraint, float deltaTime) {
  (void)deltaTime;
  constraint.accumulatedImpulseLinear = Vector3(0.0f, 0.0f, 0.0f);
  constraint.accumulatedSwingImpulse = 0.0f;
  constraint.accumulatedTwistImpulse = 0.0f;
  constraint.appliedImpulseLinear = 0.0f;
  constraint.appliedImpulseAngular = 0.0f;
}

void SequentialImpulsesKernel::warmStartConstraint(
    collision::ConeTwistConstraint &constraint) {
  (void)constraint;
}

void SequentialImpulsesKernel::solveConstraintVelocity(
    collision::ConeTwistConstraint &constraint, float deltaTime) {
  resolveConeTwistConstraint(constraint, deltaTime);
}

void SequentialImpulsesKernel::solveConstraintPosition(
    collision::ConeTwistConstraint &constraint, float deltaTime) {
  solveConeTwistConstraintPosition(constraint, deltaTime);
}

void SequentialImpulsesKernel::collectIslandConstraints(
    const std::vector<ecs::Entity> &island,
    std::vector<ConstraintRef> &localConstraints) {
  if (island.empty())
    return;

  auto collectForTypes = [&]<typename... Cs>(std::type_identity<std::tuple<Cs...>>) {
    ([&]() {
      auto &pool = mRegistry.getComponentPool<Cs>();
      if (pool.getAllData().size() > 0) {
        for (auto &constraint : pool.getAllData()) {
          if (!constraint.isEnabled || constraint.isBroken)
            continue;
          bool match = false;
          for (ecs::Entity e : island) {
            if (e.id == constraint.entityA.id || e.id == constraint.entityB.id) {
              match = true;
              break;
            }
          }
          if (match) {
            localConstraints.push_back(&constraint);
          }
        }
      }
    }(), ...);
  };
  collectForTypes(std::type_identity<Core::ConstraintTypes>{});
}

void SequentialImpulsesKernel::collectIslandConstraints(
    const std::vector<ecs::Entity> &island,
    std::vector<collision::DistanceConstraint *> &localDistance,
    std::vector<collision::RevoluteConstraint *> &localRevolute) {
  std::vector<ConstraintRef> localConstraints;
  collectIslandConstraints(island, localConstraints);
  for (auto &cr : localConstraints) {
    if (std::holds_alternative<collision::DistanceConstraint *>(cr)) {
      localDistance.push_back(std::get<collision::DistanceConstraint *>(cr));
    } else if (std::holds_alternative<collision::RevoluteConstraint *>(cr)) {
      localRevolute.push_back(std::get<collision::RevoluteConstraint *>(cr));
    }
  }
}

void SequentialImpulsesKernel::solveSingleIsland(
    const std::vector<collision::ContactManifold *> &localManifolds,
    const std::vector<ConstraintRef> &localConstraints,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (localManifolds.empty() && localConstraints.empty()) {
    return;
  }

  // 1. ÖN-ADIM: Ortogonal teğetleri ve Warm-Starting kuvvetlerini hızlara uygula
  for (collision::ContactManifold *manifold : localManifolds) {
    if (!manifold->isTriggerPair)
      preStep(*manifold, deltaTime);
  }
  for (const auto &cv : localConstraints) {
    std::visit([&](auto *c) {
      if (c && c->isEnabled && !c->isBroken) {
        prepareConstraint(*c, deltaTime);
        warmStartConstraint(*c);
      }
    }, cv);
  }

  // 2. Hız İterasyonları: Gauss-Seidel yaklaşımı
  for (int iter = 0; iter < velocityIterations; ++iter) {
    for (collision::ContactManifold *manifold : localManifolds) {
      if (!manifold->isTriggerPair)
        resolveCollision(*manifold, deltaTime);
    }
    for (const auto &cv : localConstraints) {
      std::visit([&](auto *c) {
        if (c && c->isEnabled && !c->isBroken) {
          solveConstraintVelocity(*c, deltaTime);
        }
      }, cv);
    }
  }

  // 2b. Eklem Kırılma Eşiği Kontrolü (Joint Breakage Check)
  for (const auto &cv : localConstraints) {
    std::visit([&](auto *c) {
      if (c && c->isEnabled && !c->isBroken) {
        bool broken = false;
        if (c->appliedImpulseLinear > c->breakImpulse) {
          broken = true;
        } else if (deltaTime > 0.0f) {
          float reactionForce = c->appliedImpulseLinear / deltaTime;
          float reactionTorque = c->appliedImpulseAngular / deltaTime;
          if (reactionForce > c->breakForce || reactionTorque > c->breakTorque) {
            broken = true;
          }
        }
        if (broken) {
          c->isBroken = true;
          c->isEnabled = false;
          if (mRegistry.hasComponent<Core::BodyState>(c->entityA)) {
            auto &stA = mRegistry.getComponent<Core::BodyState>(c->entityA);
            stA.isSleeping = false;
            stA.sleepTimer = 0.0f;
          }
          if (mRegistry.hasComponent<Core::BodyState>(c->entityB)) {
            auto &stB = mRegistry.getComponent<Core::BodyState>(c->entityB);
            stB.isSleeping = false;
            stB.sleepTimer = 0.0f;
          }
          if (mJointBreakListener) {
            mJointBreakListener->onJointBroken(c->entityA, c->entityB, c->appliedImpulseLinear);
          }
          if (mJointBreakCallback) {
            mJointBreakCallback(c->entityA, c->entityB, c->appliedImpulseLinear);
          }
        }
      }
    }, cv);
  }

  // 3. Pozisyon İterasyonları: Split Impulse
  for (int iter = 0; iter < positionIterations; ++iter) {
    for (collision::ContactManifold *manifold : localManifolds) {
      if (!manifold->isTriggerPair)
        solvePositionConstraints(*manifold, deltaTime);
    }
    for (const auto &cv : localConstraints) {
      std::visit([&](auto *c) {
        if (c && c->isEnabled && !c->isBroken) {
          solveConstraintPosition(*c, deltaTime);
        }
      }, cv);
    }
  }

  // Kilitli eksenlerdeki hızları sıfırla (sayısal artık birikimini engellemek için)
  for (collision::ContactManifold *manifold : localManifolds) {
    if (mRegistry.hasComponent<Core::BodyState>(manifold->entityA) &&
        mRegistry.hasComponent<Core::Motion>(manifold->entityA)) {
      const auto &stA = mRegistry.getComponent<Core::BodyState>(manifold->entityA);
      auto &motA = mRegistry.getComponent<Core::Motion>(manifold->entityA);
      if (stA.lockedAxes & Core::LockAxis::LinearX) { motA.linearVelocity.x = 0.0f; motA.splitLinearVelocity.x = 0.0f; }
      if (stA.lockedAxes & Core::LockAxis::LinearY) { motA.linearVelocity.y = 0.0f; motA.splitLinearVelocity.y = 0.0f; }
      if (stA.lockedAxes & Core::LockAxis::LinearZ) { motA.linearVelocity.z = 0.0f; motA.splitLinearVelocity.z = 0.0f; }
      if (stA.lockedAxes & Core::LockAxis::AngularX) { motA.angularVelocity.x = 0.0f; motA.splitAngularVelocity.x = 0.0f; }
      if (stA.lockedAxes & Core::LockAxis::AngularY) { motA.angularVelocity.y = 0.0f; motA.splitAngularVelocity.y = 0.0f; }
      if (stA.lockedAxes & Core::LockAxis::AngularZ) { motA.angularVelocity.z = 0.0f; motA.splitAngularVelocity.z = 0.0f; }
    }
    if (mRegistry.hasComponent<Core::BodyState>(manifold->entityB) &&
        mRegistry.hasComponent<Core::Motion>(manifold->entityB)) {
      const auto &stB = mRegistry.getComponent<Core::BodyState>(manifold->entityB);
      auto &motB = mRegistry.getComponent<Core::Motion>(manifold->entityB);
      if (stB.lockedAxes & Core::LockAxis::LinearX) { motB.linearVelocity.x = 0.0f; motB.splitLinearVelocity.x = 0.0f; }
      if (stB.lockedAxes & Core::LockAxis::LinearY) { motB.linearVelocity.y = 0.0f; motB.splitLinearVelocity.y = 0.0f; }
      if (stB.lockedAxes & Core::LockAxis::LinearZ) { motB.linearVelocity.z = 0.0f; motB.splitLinearVelocity.z = 0.0f; }
      if (stB.lockedAxes & Core::LockAxis::AngularX) { motB.angularVelocity.x = 0.0f; motB.splitAngularVelocity.x = 0.0f; }
      if (stB.lockedAxes & Core::LockAxis::AngularY) { motB.angularVelocity.y = 0.0f; motB.splitAngularVelocity.y = 0.0f; }
      if (stB.lockedAxes & Core::LockAxis::AngularZ) { motB.angularVelocity.z = 0.0f; motB.splitAngularVelocity.z = 0.0f; }
    }
  }
}

void SequentialImpulsesKernel::solveSingleIsland(
    const std::vector<collision::ContactManifold *> &localManifolds,
    const std::vector<collision::DistanceConstraint *> &localDist,
    const std::vector<collision::RevoluteConstraint *> &localRev,
    float deltaTime, int velocityIterations, int positionIterations) {
  std::vector<ConstraintRef> localConstraints;
  localConstraints.reserve(localDist.size() + localRev.size());
  for (auto *d : localDist)
    localConstraints.push_back(d);
  for (auto *r : localRev)
    localConstraints.push_back(r);

  solveSingleIsland(localManifolds, localConstraints, deltaTime,
                    velocityIterations, positionIterations);
}

void SequentialImpulsesKernel::execute(
    const std::vector<ecs::Entity> &island,
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime, int velocityIterations, int positionIterations) {
  if (island.empty())
    return;

  std::vector<collision::ContactManifold *> localManifolds;
  collectIslandManifolds(island, manifolds, localManifolds);

  std::vector<ConstraintRef> localConstraints;
  collectIslandConstraints(island, localConstraints);

  solveSingleIsland(localManifolds, localConstraints, deltaTime,
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

std::vector<std::vector<SequentialImpulsesKernel::ConstraintRef>>
SequentialImpulsesKernel::routeIslandConstraints(
    const std::unordered_map<uint64_t, size_t> &entityToIsland,
    size_t islandCount) {
  std::vector<std::vector<ConstraintRef>> islandConstraints(islandCount);

  auto routeForTypes = [&]<typename... Cs>(std::type_identity<std::tuple<Cs...>>) {
    ([&]() {
      auto &pool = mRegistry.getComponentPool<Cs>();
      if (pool.getAllData().size() > 0) {
        for (auto &constraint : pool.getAllData()) {
          if (!constraint.isEnabled || constraint.isBroken)
            continue;
          auto itA = entityToIsland.find(constraint.entityA.id);
          if (itA != entityToIsland.end()) {
            islandConstraints[itA->second].push_back(&constraint);
          } else {
            auto itB = entityToIsland.find(constraint.entityB.id);
            if (itB != entityToIsland.end()) {
              islandConstraints[itB->second].push_back(&constraint);
            }
          }
        }
      }
    }(), ...);
  };
  routeForTypes(std::type_identity<Core::ConstraintTypes>{});

  return islandConstraints;
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

  // 3. Kısıtlamaları ilgili adalara yönlendir (O(C))
  auto islandConstraints = routeIslandConstraints(entityToIsland, islands.size());

  // 4. Her adayı kendi temasları ve kısıtlamaları ile çöz
  for (size_t i = 0; i < islands.size(); ++i) {
    solveSingleIsland(islandManifolds[i], islandConstraints[i],
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
  uint8_t lockedAxesA = 0;
  uint8_t lockedAxesB = 0;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
    const auto &stA = mRegistry.getComponent<Core::BodyState>(manifold.entityA);
    typeA = stA.type;
    lockedAxesA = stA.lockedAxes;
  }
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
    const auto &stB = mRegistry.getComponent<Core::BodyState>(manifold.entityB);
    typeB = stB.type;
    lockedAxesB = stB.lockedAxes;
  }

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Vector3 invMassVecA = computeInverseMassVector(invMassA, lockedAxesA);
  const Vector3 invMassVecB = computeInverseMassVector(invMassB, lockedAxesB);
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

    motionA.linearVelocity += applyDirectionalImpulse(totalImpulse, invMassVecA);
    motionA.angularVelocity += invIA * rA.cross(totalImpulse);
    motionB.linearVelocity -= applyDirectionalImpulse(totalImpulse, invMassVecB);
    motionB.angularVelocity -= invIB * rB.cross(totalImpulse);
  }
}

void SequentialImpulsesKernel::solveContactNormal(
    collision::ContactPoint &contact,
    const Vector3 &rA, const Vector3 &rB, const Vector3 &n,
    const Vector3 &invMassA, const Vector3 &invMassB,
    const Matrix3x3 &invIA, const Matrix3x3 &invIB,
    Core::Motion &motionA, Core::Motion &motionB) {
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  float normalVel = relVel.dot(n);
  float deltaV = -normalVel + contact.velocityBias;

  Vector3 rACrossN = rA.cross(n);
  Vector3 rBCrossN = rB.cross(n);
  float effInvMassA = computeDirectionalInverseMass(invMassA, n);
  float effInvMassB = computeDirectionalInverseMass(invMassB, n);
  float invMassSumN = effInvMassA + effInvMassB + (invIA * rACrossN).dot(rACrossN) +
                      (invIB * rBCrossN).dot(rBCrossN);
  if (invMassSumN > 1e-6f) {
    float jn = deltaV / invMassSumN;
    float oldImpulse = contact.normalImpulse;
    contact.normalImpulse = std::max(oldImpulse + jn, 0.0f);
    jn = contact.normalImpulse - oldImpulse;

    Vector3 impulse = n * jn;
    motionA.linearVelocity += applyDirectionalImpulse(impulse, invMassA);
    motionA.angularVelocity += invIA * rA.cross(impulse);
    motionB.linearVelocity -= applyDirectionalImpulse(impulse, invMassB);
    motionB.angularVelocity -= invIB * rB.cross(impulse);
  }
}

void SequentialImpulsesKernel::solveContactFriction(
    collision::ContactPoint &contact,
    const Vector3 &rA, const Vector3 &rB,
    const Vector3 &t1, const Vector3 &t2,
    float friction,
    const Vector3 &invMassA, const Vector3 &invMassB,
    const Matrix3x3 &invIA, const Matrix3x3 &invIB,
    Core::Motion &motionA, Core::Motion &motionB) {
  float maxFriction = friction * contact.normalImpulse;

  // Teğet 1
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;
  Vector3 rACrossT1 = rA.cross(t1);
  Vector3 rBCrossT1 = rB.cross(t1);
  float effInvMassA1 = computeDirectionalInverseMass(invMassA, t1);
  float effInvMassB1 = computeDirectionalInverseMass(invMassB, t1);
  float invMassSumT1 = effInvMassA1 + effInvMassB1 +
                       (invIA * rACrossT1).dot(rACrossT1) +
                       (invIB * rBCrossT1).dot(rBCrossT1);
  if (invMassSumT1 > 1e-6f) {
    float jt1 = -relVel.dot(t1) / invMassSumT1;
    float oldT1 = contact.tangentImpulse1;
    contact.tangentImpulse1 =
        std::clamp(oldT1 + jt1, -maxFriction, maxFriction);
    jt1 = contact.tangentImpulse1 - oldT1;

    Vector3 impulseT1 = t1 * jt1;
    motionA.linearVelocity += applyDirectionalImpulse(impulseT1, invMassA);
    motionA.angularVelocity += invIA * rA.cross(impulseT1);
    motionB.linearVelocity -= applyDirectionalImpulse(impulseT1, invMassB);
    motionB.angularVelocity -= invIB * rB.cross(impulseT1);
  }

  // Teğet 2
  vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  relVel = vA - vB;
  Vector3 rACrossT2 = rA.cross(t2);
  Vector3 rBCrossT2 = rB.cross(t2);
  float effInvMassA2 = computeDirectionalInverseMass(invMassA, t2);
  float effInvMassB2 = computeDirectionalInverseMass(invMassB, t2);
  float invMassSumT2 = effInvMassA2 + effInvMassB2 +
                       (invIA * rACrossT2).dot(rACrossT2) +
                       (invIB * rBCrossT2).dot(rBCrossT2);
  if (invMassSumT2 > 1e-6f) {
    float jt2 = -relVel.dot(t2) / invMassSumT2;
    float oldT2 = contact.tangentImpulse2;
    contact.tangentImpulse2 =
        std::clamp(oldT2 + jt2, -maxFriction, maxFriction);
    jt2 = contact.tangentImpulse2 - oldT2;

    Vector3 impulseT2 = t2 * jt2;
    motionA.linearVelocity += applyDirectionalImpulse(impulseT2, invMassA);
    motionA.angularVelocity += invIA * rA.cross(impulseT2);
    motionB.linearVelocity -= applyDirectionalImpulse(impulseT2, invMassB);
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
  uint8_t lockedAxesA = 0;
  uint8_t lockedAxesB = 0;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
    const auto &stA = mRegistry.getComponent<Core::BodyState>(manifold.entityA);
    typeA = stA.type;
    lockedAxesA = stA.lockedAxes;
  }
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
    const auto &stB = mRegistry.getComponent<Core::BodyState>(manifold.entityB);
    typeB = stB.type;
    lockedAxesB = stB.lockedAxes;
  }

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Vector3 invMassVecA = computeInverseMassVector(invMassA, lockedAxesA);
  const Vector3 invMassVecB = computeInverseMassVector(invMassB, lockedAxesB);
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

    solveContactNormal(contact, rA, rB, n, invMassVecA, invMassVecB, invIA, invIB, motionA, motionB);
    solveContactFriction(contact, rA, rB, t1, t2, friction, invMassVecA, invMassVecB, invIA, invIB, motionA, motionB);
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
  uint8_t lockedAxesA = 0;
  uint8_t lockedAxesB = 0;
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
    const auto &stA = mRegistry.getComponent<Core::BodyState>(manifold.entityA);
    typeA = stA.type;
    lockedAxesA = stA.lockedAxes;
  }
  if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
    const auto &stB = mRegistry.getComponent<Core::BodyState>(manifold.entityB);
    typeB = stB.type;
    lockedAxesB = stB.lockedAxes;
  }

  const float invMassA =
      (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB =
      (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Vector3 invMassVecA = computeInverseMassVector(invMassA, lockedAxesA);
  const Vector3 invMassVecB = computeInverseMassVector(invMassB, lockedAxesB);
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
    float effInvMassA = computeDirectionalInverseMass(invMassVecA, n);
    float effInvMassB = computeDirectionalInverseMass(invMassVecB, n);
    float invMassSum = effInvMassA + effInvMassB + (invIA * rACrossN).dot(rACrossN) +
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
    motionA.splitLinearVelocity += applyDirectionalImpulse(impulse, invMassVecA);
    motionA.splitAngularVelocity += invIA * rA.cross(impulse);
    motionB.splitLinearVelocity -= applyDirectionalImpulse(impulse, invMassVecB);
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
  constraint.appliedImpulseLinear += std::abs(j);

  Vector3 impulse = u * j;

  // 7. Hızları güncelle
  motionA.linearVelocity += impulse * massA.inverseMass;
  motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
  motionB.linearVelocity -= impulse * massB.inverseMass;
  motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
}

void SequentialImpulsesKernel::solveDistanceConstraintPosition(
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Core::BodyType typeA = Core::BodyType::Dynamic;
  Core::BodyType typeB = Core::BodyType::Dynamic;
  uint8_t lockedAxesA = 0;
  uint8_t lockedAxesB = 0;
  if (mRegistry.hasComponent<Core::BodyState>(constraint.entityA)) {
    const auto &stA = mRegistry.getComponent<Core::BodyState>(constraint.entityA);
    typeA = stA.type;
    lockedAxesA = stA.lockedAxes;
  }
  if (mRegistry.hasComponent<Core::BodyState>(constraint.entityB)) {
    const auto &stB = mRegistry.getComponent<Core::BodyState>(constraint.entityB);
    typeB = stB.type;
    lockedAxesB = stB.lockedAxes;
  }

  const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
  const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
  const Vector3 invMassVecA = computeInverseMassVector(invMassA, lockedAxesA);
  const Vector3 invMassVecB = computeInverseMassVector(invMassB, lockedAxesB);
  const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);
  const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0,0,0,0,0,0,0,0,0);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // Mevcut split hızlarıyla öngörülen pozisyonlar
  Vector3 effA = pA + (motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA)) * deltaTime;
  Vector3 effB = pB + (motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB)) * deltaTime;

  Vector3 delta = effA - effB;
  float currentDist = delta.length();
  if (currentDist < 1e-6f)
    return;
  Vector3 u = delta / currentDist;

  float positionError = currentDist - constraint.targetDistance;
  if (std::abs(positionError) < 1e-5f)
    return;

  const float splitFactor = 0.8f;
  float bias = (splitFactor / deltaTime) * positionError;

  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  float relSplitVel = (vSplitA - vSplitB).dot(u);

  Vector3 rACrossU = rA.cross(u);
  Vector3 rBCrossU = rB.cross(u);
  float effInvMassA = computeDirectionalInverseMass(invMassVecA, u);
  float effInvMassB = computeDirectionalInverseMass(invMassVecB, u);
  float invMassSum = effInvMassA + effInvMassB +
                     (invIA * rACrossU).dot(rACrossU) +
                     (invIB * rBCrossU).dot(rBCrossU);
  if (invMassSum < 1e-6f)
    return;

  float deltaSplit = -(relSplitVel + bias) / invMassSum;
  deltaSplit *= std::clamp(constraint.stiffness, 0.0f, 1.0f);

  Vector3 impulse = u * deltaSplit;
  motionA.splitLinearVelocity += applyDirectionalImpulse(impulse, invMassVecA);
  motionA.splitAngularVelocity += invIA * rA.cross(impulse);
  motionB.splitLinearVelocity -= applyDirectionalImpulse(impulse, invMassVecB);
  motionB.splitAngularVelocity -= invIB * rB.cross(impulse);
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
      constraint.appliedImpulseLinear += std::abs(j);
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
    constraint.appliedImpulseAngular += std::abs(jAng1);
    motionA.angularVelocity += massA.inverseInertiaTensor * (u1 * jAng1);
    motionB.angularVelocity -= massB.inverseInertiaTensor * (u1 * jAng1);
  }

  // U2 eksenindeki istenmeyen dönmeyi sıfırla
  float invIA_u2 = (massA.inverseInertiaTensor * u2).dot(u2);
  float invIB_u2 = (massB.inverseInertiaTensor * u2).dot(u2);
  if (invIA_u2 + invIB_u2 > 1e-6f) {
    float jAng2 = -relAngVel.dot(u2) / (invIA_u2 + invIB_u2);
    constraint.appliedImpulseAngular += std::abs(jAng2);
    motionA.angularVelocity += massA.inverseInertiaTensor * (u2 * jAng2);
    motionB.angularVelocity -= massB.inverseInertiaTensor * (u2 * jAng2);
  }

  // AŞAMA 3: Açısal Motor (Menteşe Ekseni Boyunca)
  float invIA_axis = (massA.inverseInertiaTensor * axisA_World).dot(axisA_World);
  float invIB_axis = (massB.inverseInertiaTensor * axisA_World).dot(axisA_World);
  float invAngMassAxis = invIA_axis + invIB_axis;

  if (constraint.enableMotor && constraint.maxMotorTorque > 0.0f && invAngMassAxis > 1e-6f && deltaTime > 0.0f) {
    float curRelSpeed = (motionB.angularVelocity - motionA.angularVelocity).dot(axisA_World);
    float deltaW = constraint.motorSpeed - curRelSpeed;
    float jMotor = deltaW / invAngMassAxis;
    float maxImpulse = constraint.maxMotorTorque * deltaTime;
    float oldImpulse = constraint.accumulatedMotorImpulse;
    constraint.accumulatedMotorImpulse = std::clamp(oldImpulse + jMotor, -maxImpulse, maxImpulse);
    float dImpulse = constraint.accumulatedMotorImpulse - oldImpulse;
    constraint.appliedImpulseAngular += std::abs(dImpulse);

    Vector3 angImpulse = axisA_World * dImpulse;
    motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
    motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
  }

  // AŞAMA 4: Açısal Limitler (Menteşe Ekseni Boyunca Alt/Üst Sınır)
  if (constraint.enableLimit && invAngMassAxis > 1e-6f) {
    Vector3 a_A = constraint.hingeAxisLocalA.getNormalized();
    Vector3 u_localA;
    if (std::abs(a_A.x) >= 0.57735f)
      u_localA = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
    else
      u_localA = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();
    Vector3 v_localA = a_A.cross(u_localA);
    Vector3 u_localB = constraint.initialRelativeOrientation.getInverse() * u_localA;

    Vector3 u_worldA = poseA.orientation * u_localA;
    Vector3 v_worldA = poseA.orientation * v_localA;
    Vector3 u_worldB = poseB.orientation * u_localB;
    float currentAngle = std::atan2(u_worldB.dot(v_worldA), u_worldB.dot(u_worldA));

    float curRelSpeed = (motionB.angularVelocity - motionA.angularVelocity).dot(axisA_World);
    if (currentAngle <= constraint.lowerAngleLimit) {
      float deltaW = -curRelSpeed;
      float jLimit = deltaW / invAngMassAxis;
      float oldImpulse = constraint.accumulatedLimitImpulse;
      constraint.accumulatedLimitImpulse = std::max(0.0f, oldImpulse + jLimit);
      float dImpulse = constraint.accumulatedLimitImpulse - oldImpulse;
      constraint.appliedImpulseAngular += std::abs(dImpulse);

      Vector3 angImpulse = axisA_World * dImpulse;
      motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
      motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
    } else if (currentAngle >= constraint.upperAngleLimit) {
      float deltaW = -curRelSpeed;
      float jLimit = deltaW / invAngMassAxis;
      float oldImpulse = constraint.accumulatedLimitImpulse;
      constraint.accumulatedLimitImpulse = std::min(0.0f, oldImpulse + jLimit);
      float dImpulse = constraint.accumulatedLimitImpulse - oldImpulse;
      constraint.appliedImpulseAngular += std::abs(dImpulse);

      Vector3 angImpulse = axisA_World * dImpulse;
      motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
      motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
    } else {
      constraint.accumulatedLimitImpulse = 0.0f;
    }
  }
}

void SequentialImpulsesKernel::solveRevoluteConstraintPosition(
    collision::RevoluteConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // 1. Doğrusal Konum Düzeltmesi (Split Impulse)
  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  Vector3 effA = pA + vSplitA * deltaTime;
  Vector3 effB = pB + vSplitB * deltaTime;
  Vector3 dEff = effB - effA;
  Vector3 relSplitVel = vSplitB - vSplitA;

  const float splitFactorLinear = 0.8f;
  for (int axis = 0; axis < 3; ++axis) {
    Vector3 n(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    float error = dEff.dot(n);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float bias = error * (splitFactorLinear / deltaTime);
      float deltaV = -(relSplitVel.dot(n) + bias);
      float j = deltaV / invMassSum;
      Vector3 impulse = n * j;

      motionB.splitLinearVelocity += impulse * massB.inverseMass;
      motionB.splitAngularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
      motionA.splitLinearVelocity -= impulse * massA.inverseMass;
      motionA.splitAngularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);

      vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
      vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
      relSplitVel = vSplitB - vSplitA;
      effA = pA + vSplitA * deltaTime;
      effB = pB + vSplitB * deltaTime;
      dEff = effB - effA;
    }
  }

  // 2. Menteşe Dışı Açısal Düzeltme (u1 ve u2 eksenleri)
  Vector3 axisA_World = (poseA.orientation * constraint.hingeAxisLocalA).getNormalized();
  Vector3 u1, u2;
  if (std::abs(axisA_World.x) >= 0.57735f)
    u1 = Vector3(axisA_World.y, -axisA_World.x, 0.0f).getNormalized();
  else
    u1 = Vector3(0.0f, axisA_World.z, -axisA_World.y).getNormalized();
  u2 = axisA_World.cross(u1);

  Vector3 relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;
  for (const Vector3 &uAxis : {u1, u2}) {
    float invIA = (massA.inverseInertiaTensor * uAxis).dot(uAxis);
    float invIB = (massB.inverseInertiaTensor * uAxis).dot(uAxis);
    float invAngSum = invIA + invIB;
    if (invAngSum > 1e-6f) {
      float deltaW = -relSplitAngVel.dot(uAxis);
      float jAng = deltaW / invAngSum;
      Vector3 angImpulse = uAxis * jAng;
      motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
      motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
      relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;
    }
  }

  // 3. Açısal Sınırlar (Limit) Split Impulse
  if (constraint.enableLimit) {
    Vector3 a_A = constraint.hingeAxisLocalA.getNormalized();
    Vector3 u_localA;
    if (std::abs(a_A.x) >= 0.57735f)
      u_localA = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
    else
      u_localA = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();
    Vector3 v_localA = a_A.cross(u_localA);
    Vector3 u_localB = constraint.initialRelativeOrientation.getInverse() * u_localA;

    Vector3 u_worldA = poseA.orientation * u_localA;
    Vector3 v_worldA = poseA.orientation * v_localA;
    Vector3 u_worldB = poseB.orientation * u_localB;
    float currentAngle = std::atan2(u_worldB.dot(v_worldA), u_worldB.dot(u_worldA));

    float invIA_axis = (massA.inverseInertiaTensor * axisA_World).dot(axisA_World);
    float invIB_axis = (massB.inverseInertiaTensor * axisA_World).dot(axisA_World);
    float invAngMassAxis = invIA_axis + invIB_axis;

    if (invAngMassAxis > 1e-6f) {
      if (currentAngle < constraint.lowerAngleLimit) {
        float violation = currentAngle - constraint.lowerAngleLimit;
        float bias = violation * (0.8f / deltaTime);
        float deltaW = -(relSplitAngVel.dot(axisA_World) + bias);
        float j = deltaW / invAngMassAxis;
        if (j > 0.0f) {
          Vector3 angImpulse = axisA_World * j;
          motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
          motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
        }
      } else if (currentAngle > constraint.upperAngleLimit) {
        float violation = currentAngle - constraint.upperAngleLimit;
        float bias = violation * (0.8f / deltaTime);
        float deltaW = -(relSplitAngVel.dot(axisA_World) + bias);
        float j = deltaW / invAngMassAxis;
        if (j < 0.0f) {
          Vector3 angImpulse = axisA_World * j;
          motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
          motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
        }
      }
    }
  }
}

void SequentialImpulsesKernel::resolveFixedConstraint(
    collision::FixedConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // 1. ÇİZGİSEL KISIT: Bağlantı noktalarının ayrılmasını engelle (3 DOF)
  Vector3 linearError = pA - pB;
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  const float baumgarteLinear = 0.4f;
  Vector3 linearBias = linearError * (baumgarteLinear / deltaTime);

  for (int axis = 0; axis < 3; ++axis) {
    Vector3 n(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float deltaV = -(relVel.dot(n) + linearBias.dot(n));
      float j = deltaV / invMassSum;
      constraint.appliedImpulseLinear += std::abs(j);
      Vector3 impulse = n * j;
      motionA.linearVelocity += impulse * massA.inverseMass;
      motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
      motionB.linearVelocity -= impulse * massB.inverseMass;
      motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);

      vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
      vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
      relVel = vA - vB;
    }
  }

  // 2. AÇISAL KISIT: Bağıl rotasyonun korunması (3 DOF)
  Quaternion targetB = poseA.orientation * constraint.initialRelativeOrientation;
  Quaternion qDiff = poseB.orientation * targetB.getInverse();
  if (qDiff.w < 0.0f) {
    qDiff.x = -qDiff.x;
    qDiff.y = -qDiff.y;
    qDiff.z = -qDiff.z;
    qDiff.w = -qDiff.w;
  }
  Vector3 angularError = Vector3(qDiff.x, qDiff.y, qDiff.z) * 2.0f;
  const float baumgarteAngular = 0.4f;
  Vector3 angularBias = angularError * (baumgarteAngular / deltaTime);

  Vector3 relAngVel = motionA.angularVelocity - motionB.angularVelocity;

  for (int axis = 0; axis < 3; ++axis) {
    Vector3 a(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    float invAngMass = (massA.inverseInertiaTensor * a).dot(a) +
                       (massB.inverseInertiaTensor * a).dot(a);
    if (invAngMass > 1e-6f) {
      float deltaW = -(relAngVel.dot(a) - angularBias.dot(a));
      float jAng = deltaW / invAngMass;
      constraint.appliedImpulseAngular += std::abs(jAng);
      Vector3 angImpulse = a * jAng;
      motionA.angularVelocity += massA.inverseInertiaTensor * angImpulse;
      motionB.angularVelocity -= massB.inverseInertiaTensor * angImpulse;

      relAngVel = motionA.angularVelocity - motionB.angularVelocity;
    }
  }
}

void SequentialImpulsesKernel::solveFixedConstraintPosition(
    collision::FixedConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // 1. Çizgisel split impulse (mevcut split hızıyla öngörülen konuma göre dinamik hata)
  Vector3 effA = pA + (motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA)) * deltaTime;
  Vector3 effB = pB + (motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB)) * deltaTime;
  Vector3 linearError = effA - effB;

  const float splitFactorLinear = 0.8f;
  Vector3 linearBias = linearError * (splitFactorLinear / deltaTime);

  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  Vector3 relSplitVel = vSplitA - vSplitB;

  for (int axis = 0; axis < 3; ++axis) {
    Vector3 n(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float deltaV = -(relSplitVel.dot(n) + linearBias.dot(n));
      float j = deltaV / invMassSum;
      Vector3 impulse = n * j;
      motionA.splitLinearVelocity += impulse * massA.inverseMass;
      motionA.splitAngularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
      motionB.splitLinearVelocity -= impulse * massB.inverseMass;
      motionB.splitAngularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);

      vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
      vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
      relSplitVel = vSplitA - vSplitB;
    }
  }

  // 2. Açısal split impulse
  Quaternion targetB = poseA.orientation * constraint.initialRelativeOrientation;
  Quaternion qDiff = poseB.orientation * targetB.getInverse();
  if (qDiff.w < 0.0f) {
    qDiff.x = -qDiff.x;
    qDiff.y = -qDiff.y;
    qDiff.z = -qDiff.z;
    qDiff.w = -qDiff.w;
  }
  Vector3 angularError = Vector3(qDiff.x, qDiff.y, qDiff.z) * 2.0f;
  const float splitFactorAngular = 0.25f;
  Vector3 angularBias = angularError * (splitFactorAngular / deltaTime);

  Vector3 relSplitAngVel = motionA.splitAngularVelocity - motionB.splitAngularVelocity;

  for (int axis = 0; axis < 3; ++axis) {
    Vector3 a(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    float invAngMass = (massA.inverseInertiaTensor * a).dot(a) +
                       (massB.inverseInertiaTensor * a).dot(a);
    if (invAngMass > 1e-6f) {
      float deltaW = -(relSplitAngVel.dot(a) - angularBias.dot(a));
      float jAng = deltaW / invAngMass;
      Vector3 angImpulse = a * jAng;
      motionA.splitAngularVelocity += massA.inverseInertiaTensor * angImpulse;
      motionB.splitAngularVelocity -= massB.inverseInertiaTensor * angImpulse;

      relSplitAngVel = motionA.splitAngularVelocity - motionB.splitAngularVelocity;
    }
  }
}

void SequentialImpulsesKernel::resolveBallSocketConstraint(
    collision::BallSocketConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;

  // Çizgisel Hız Kısıtı: Bağlantı noktalarının bağıl hızını sıfırla (v_rel = 0)
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  // 3x3 Blok Efektif Kütle Matrisi: K = (mA^-1 + mB^-1)*I - [rA]x IA^-1 [rA]x - [rB]x IB^-1 [rB]x
  float wSum = massA.inverseMass + massB.inverseMass;
  Matrix3x3 K;
  for (std::size_t col = 0; col < 3; ++col) {
    Vector3 e(col == 0 ? 1.0f : 0.0f, col == 1 ? 1.0f : 0.0f, col == 2 ? 1.0f : 0.0f);
    Vector3 colA = (massA.inverseMass > 0.0f) ? -rA.cross(massA.inverseInertiaTensor * rA.cross(e)) : Vector3(0, 0, 0);
    Vector3 colB = (massB.inverseMass > 0.0f) ? -rB.cross(massB.inverseInertiaTensor * rB.cross(e)) : Vector3(0, 0, 0);
    Vector3 kCol = e * wSum + colA + colB;
    K.rows[0][col] = kCol.x;
    K.rows[1][col] = kCol.y;
    K.rows[2][col] = kCol.z;
  }

  float det = K.getDeterminant();
  if (std::abs(det) > 1e-7f) {
    Matrix3x3 invK = K.getInverse();
    Vector3 impulse = invK * (-relVel);
    constraint.accumulatedImpulseLinear += impulse;
    constraint.appliedImpulseLinear += impulse.length();

    motionA.linearVelocity += impulse * massA.inverseMass;
    motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.linearVelocity -= impulse * massB.inverseMass;
    motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
  }
}

void SequentialImpulsesKernel::solveBallSocketConstraintPosition(
    collision::BallSocketConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // Split impulse ile dinamik öngörülen pozisyon hatası
  Vector3 effA = pA + (motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA)) * deltaTime;
  Vector3 effB = pB + (motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB)) * deltaTime;
  Vector3 linearError = effA - effB;

  const float splitFactor = 0.8f;
  Vector3 linearBias = linearError * (splitFactor / deltaTime);

  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  Vector3 relSplitVel = vSplitA - vSplitB;

  float wSum = massA.inverseMass + massB.inverseMass;
  Matrix3x3 K;
  for (std::size_t col = 0; col < 3; ++col) {
    Vector3 e(col == 0 ? 1.0f : 0.0f, col == 1 ? 1.0f : 0.0f, col == 2 ? 1.0f : 0.0f);
    Vector3 colA = (massA.inverseMass > 0.0f) ? -rA.cross(massA.inverseInertiaTensor * rA.cross(e)) : Vector3(0, 0, 0);
    Vector3 colB = (massB.inverseMass > 0.0f) ? -rB.cross(massB.inverseInertiaTensor * rB.cross(e)) : Vector3(0, 0, 0);
    Vector3 kCol = e * wSum + colA + colB;
    K.rows[0][col] = kCol.x;
    K.rows[1][col] = kCol.y;
    K.rows[2][col] = kCol.z;
  }

  float det = K.getDeterminant();
  if (std::abs(det) > 1e-7f) {
    Matrix3x3 invK = K.getInverse();
    Vector3 impulse = invK * (-(relSplitVel + linearBias));

    motionA.splitLinearVelocity += impulse * massA.inverseMass;
    motionA.splitAngularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.splitLinearVelocity -= impulse * massB.inverseMass;
    motionB.splitAngularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
  }
}

void SequentialImpulsesKernel::resolvePrismaticConstraint(
    collision::PrismaticConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 u = (poseA.orientation * constraint.localAxisA).getNormalized();

  Vector3 v, w;
  if (std::abs(u.x) < 0.57735f) {
    v = u.cross(Vector3(1.0f, 0.0f, 0.0f)).getNormalized();
  } else {
    v = u.cross(Vector3(0.0f, 1.0f, 0.0f)).getNormalized();
  }
  w = u.cross(v).getNormalized();

  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;
  Vector3 dVec = pB - pA;
  float s = dVec.dot(u);

  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vB - vA;

  // 1. Eksen dışı kilitli 2 çizgisel serbestlik derecesi (v ve w eksenleri)
  for (const Vector3 &n : {v, w}) {
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float deltaV = -relVel.dot(n);
      float j = deltaV / invMassSum;
      constraint.appliedImpulseLinear += std::abs(j);
      Vector3 impulse = n * j;

      motionB.linearVelocity += impulse * massB.inverseMass;
      motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
      motionA.linearVelocity -= impulse * massA.inverseMass;
      motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);

      vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
      vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
      relVel = vB - vA;
    }
  }

  // 2. Kilitli 3 açısal serbestlik derecesi (bağıl rotasyon = 0)
  Vector3 relAngVel = motionB.angularVelocity - motionA.angularVelocity;
  for (int axis = 0; axis < 3; ++axis) {
    Vector3 a(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    float invAngMass = (massA.inverseInertiaTensor * a).dot(a) +
                       (massB.inverseInertiaTensor * a).dot(a);
    if (invAngMass > 1e-6f) {
      float deltaW = -relAngVel.dot(a);
      float jAng = deltaW / invAngMass;
      constraint.appliedImpulseAngular += std::abs(jAng);
      Vector3 angImpulse = a * jAng;
      motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
      motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;

      relAngVel = motionB.angularVelocity - motionA.angularVelocity;
    }
  }

  // 3. Doğrusal Motor (u ekseni boyunca)
  if (constraint.enableMotor && constraint.maxMotorForce > 0.0f) {
    Vector3 rACrossU = rA.cross(u);
    Vector3 rBCrossU = rB.cross(u);
    float invMassU = massA.inverseMass + massB.inverseMass +
                     (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) +
                     (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
    if (invMassU > 1e-6f) {
      float deltaV = constraint.motorSpeed - relVel.dot(u);
      float j = deltaV / invMassU;
      float maxImpulse = constraint.maxMotorForce * deltaTime;
      float oldImpulse = constraint.motorImpulse;
      constraint.motorImpulse = std::clamp(oldImpulse + j, -maxImpulse, maxImpulse);
      float dImpulse = constraint.motorImpulse - oldImpulse;
      constraint.appliedImpulseLinear += std::abs(dImpulse);

      Vector3 impulse = u * dImpulse;
      motionB.linearVelocity += impulse * massB.inverseMass;
      motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
      motionA.linearVelocity -= impulse * massA.inverseMass;
      motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);

      vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
      vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
      relVel = vB - vA;
    }
  }

  // 4. Alt ve Üst Sınırlar (u ekseni boyunca)
  if (constraint.enableLimits) {
    Vector3 rACrossU = rA.cross(u);
    Vector3 rBCrossU = rB.cross(u);
    float invMassU = massA.inverseMass + massB.inverseMass +
                     (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) +
                     (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
    if (invMassU > 1e-6f) {
      if (s <= constraint.lowerLimit) {
        float deltaV = -relVel.dot(u);
        float j = deltaV / invMassU;
        float oldImpulse = constraint.limitImpulse;
        constraint.limitImpulse = std::max(0.0f, oldImpulse + j);
        float dImpulse = constraint.limitImpulse - oldImpulse;
        constraint.appliedImpulseLinear += std::abs(dImpulse);

        Vector3 impulse = u * dImpulse;
        motionB.linearVelocity += impulse * massB.inverseMass;
        motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
        motionA.linearVelocity -= impulse * massA.inverseMass;
        motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
      } else if (s >= constraint.upperLimit) {
        float deltaV = -relVel.dot(u);
        float j = deltaV / invMassU;
        float oldImpulse = constraint.limitImpulse;
        constraint.limitImpulse = std::min(0.0f, oldImpulse + j);
        float dImpulse = constraint.limitImpulse - oldImpulse;
        constraint.appliedImpulseLinear += std::abs(dImpulse);

        Vector3 impulse = u * dImpulse;
        motionB.linearVelocity += impulse * massB.inverseMass;
        motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
        motionA.linearVelocity -= impulse * massA.inverseMass;
        motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
      } else {
        constraint.limitImpulse = 0.0f;
      }
    }
  }
}

void SequentialImpulsesKernel::solvePrismaticConstraintPosition(
    collision::PrismaticConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 u = (poseA.orientation * constraint.localAxisA).getNormalized();

  Vector3 v, w;
  if (std::abs(u.x) < 0.57735f) {
    v = u.cross(Vector3(1.0f, 0.0f, 0.0f)).getNormalized();
  } else {
    v = u.cross(Vector3(0.0f, 1.0f, 0.0f)).getNormalized();
  }
  w = u.cross(v).getNormalized();

  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  Vector3 effA = pA + vSplitA * deltaTime;
  Vector3 effB = pB + vSplitB * deltaTime;
  Vector3 dEff = effB - effA;
  Vector3 relSplitVel = vSplitB - vSplitA;

  // 1. Eksen dışı kilitli 2 çizgisel serbestlik derecesi (v ve w eksenleri Split Impulse)
  const float splitFactorLinear = 0.8f;
  for (const Vector3 &n : {v, w}) {
    float error = dEff.dot(n);
    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = massA.inverseMass + massB.inverseMass +
                       (massA.inverseInertiaTensor * rACrossN).dot(rACrossN) +
                       (massB.inverseInertiaTensor * rBCrossN).dot(rBCrossN);
    if (invMassSum > 1e-6f) {
      float bias = error * (splitFactorLinear / deltaTime);
      float deltaV = -(relSplitVel.dot(n) + bias);
      float j = deltaV / invMassSum;
      Vector3 impulse = n * j;

      motionB.splitLinearVelocity += impulse * massB.inverseMass;
      motionB.splitAngularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
      motionA.splitLinearVelocity -= impulse * massA.inverseMass;
      motionA.splitAngularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);

      vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
      vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
      relSplitVel = vSplitB - vSplitA;
      effA = pA + vSplitA * deltaTime;
      effB = pB + vSplitB * deltaTime;
      dEff = effB - effA;
    }
  }

  // 2. Kilitli 3 açısal serbestlik derecesi Split Impulse
  Quaternion targetB = poseA.orientation * constraint.initialRelativeOrientation;
  Quaternion qDiff = poseB.orientation * targetB.getInverse();
  if (qDiff.w < 0.0f) {
    qDiff.x = -qDiff.x;
    qDiff.y = -qDiff.y;
    qDiff.z = -qDiff.z;
    qDiff.w = -qDiff.w;
  }
  Vector3 angularError = Vector3(qDiff.x, qDiff.y, qDiff.z) * 2.0f;
  const float splitFactorAngular = 0.4f;
  Vector3 angularBias = angularError * (splitFactorAngular / deltaTime);
  Vector3 relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;

  for (int axis = 0; axis < 3; ++axis) {
    Vector3 a(axis == 0 ? 1.0f : 0.0f, axis == 1 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f);
    float invAngMass = (massA.inverseInertiaTensor * a).dot(a) +
                       (massB.inverseInertiaTensor * a).dot(a);
    if (invAngMass > 1e-6f) {
      float deltaW = -(relSplitAngVel.dot(a) + angularBias.dot(a));
      float jAng = deltaW / invAngMass;
      Vector3 angImpulse = a * jAng;
      motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
      motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;

      relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;
    }
  }

  // 3. Sınırlar (Limit) Split Impulse
  if (constraint.enableLimits) {
    float sEff = dEff.dot(u);
    Vector3 rACrossU = rA.cross(u);
    Vector3 rBCrossU = rB.cross(u);
    float invMassU = massA.inverseMass + massB.inverseMass +
                     (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) +
                     (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
    if (invMassU > 1e-6f) {
      if (sEff < constraint.lowerLimit) {
        float violation = sEff - constraint.lowerLimit;
        float bias = violation * (0.8f / deltaTime);
        float deltaV = -(relSplitVel.dot(u) + bias);
        float j = deltaV / invMassU;
        if (j > 0.0f) {
          Vector3 impulse = u * j;
          motionB.splitLinearVelocity += impulse * massB.inverseMass;
          motionB.splitAngularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
          motionA.splitLinearVelocity -= impulse * massA.inverseMass;
          motionA.splitAngularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
        }
      } else if (sEff > constraint.upperLimit) {
        float violation = sEff - constraint.upperLimit;
        float bias = violation * (0.8f / deltaTime);
        float deltaV = -(relSplitVel.dot(u) + bias);
        float j = deltaV / invMassU;
        if (j < 0.0f) {
          Vector3 impulse = u * j;
          motionB.splitLinearVelocity += impulse * massB.inverseMass;
          motionB.splitAngularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
          motionA.splitLinearVelocity -= impulse * massA.inverseMass;
          motionA.splitAngularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
        }
      }
    }
  }
}

void SequentialImpulsesKernel::resolveSpringConstraint(
    collision::SpringConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;
  Vector3 delta = pB - pA;
  float currentDist = delta.length();
  if (currentDist < 1e-6f)
    return;
  Vector3 u = delta / currentDist;

  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vB - vA;
  float relSpeed = relVel.dot(u);

  float positionError = currentDist - constraint.restLength;
  float bias = (constraint.biasFactor / deltaTime) * positionError;

  Vector3 rACrossU = rA.cross(u);
  Vector3 rBCrossU = rB.cross(u);
  float invMassSum = massA.inverseMass + massB.inverseMass +
                     (massA.inverseInertiaTensor * rACrossU).dot(rACrossU) +
                     (massB.inverseInertiaTensor * rBCrossU).dot(rBCrossU);
  float effectiveMass = invMassSum + constraint.gamma;
  if (effectiveMass < 1e-6f)
    return;

  float deltaLambda = -(relSpeed + bias + constraint.gamma * constraint.accumulatedImpulse) / effectiveMass;
  constraint.accumulatedImpulse += deltaLambda;
  constraint.appliedImpulseLinear += std::abs(deltaLambda);

  Vector3 impulse = u * deltaLambda;
  motionB.linearVelocity += impulse * massB.inverseMass;
  motionB.angularVelocity += massB.inverseInertiaTensor * rB.cross(impulse);
  motionA.linearVelocity -= impulse * massA.inverseMass;
  motionA.angularVelocity -= massA.inverseInertiaTensor * rA.cross(impulse);
}

void SequentialImpulsesKernel::solveSpringConstraintPosition(
    collision::SpringConstraint &constraint, float deltaTime) {
  (void)constraint;
  (void)deltaTime;
}

void SequentialImpulsesKernel::resolveConeTwistConstraint(
    collision::ConeTwistConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;

  // 1. Noktasal Bağlantı (3 Boyutta Ayrılmayı Kilitle - 3 DOF Coupled Block Solver)
  Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
  Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
  Vector3 relVel = vA - vB;

  float wSum = massA.inverseMass + massB.inverseMass;
  Matrix3x3 K;
  for (std::size_t col = 0; col < 3; ++col) {
    Vector3 e(col == 0 ? 1.0f : 0.0f, col == 1 ? 1.0f : 0.0f, col == 2 ? 1.0f : 0.0f);
    Vector3 colA = (massA.inverseMass > 0.0f) ? -rA.cross(massA.inverseInertiaTensor * rA.cross(e)) : Vector3(0, 0, 0);
    Vector3 colB = (massB.inverseMass > 0.0f) ? -rB.cross(massB.inverseInertiaTensor * rB.cross(e)) : Vector3(0, 0, 0);
    Vector3 kCol = e * wSum + colA + colB;
    K.rows[0][col] = kCol.x;
    K.rows[1][col] = kCol.y;
    K.rows[2][col] = kCol.z;
  }

  float det = K.getDeterminant();
  if (std::abs(det) > 1e-7f) {
    Matrix3x3 invK = K.getInverse();
    Vector3 posError = (poseA.position + rA) - (poseB.position + rB);
    const float baumgarteLinear = 0.35f;
    Vector3 linearBias = posError * (baumgarteLinear / deltaTime);
    Vector3 impulse = invK * (-(relVel + linearBias));
    constraint.appliedImpulseLinear += impulse.length();

    motionA.linearVelocity += impulse * massA.inverseMass;
    motionA.angularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.linearVelocity -= impulse * massB.inverseMass;
    motionB.angularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
  }

  Vector3 uA = (poseA.orientation * constraint.twistAxisLocalA).getNormalized();
  Vector3 uB = (poseB.orientation * constraint.twistAxisLocalB).getNormalized();

  // 2. Koni Salınım Sınırı (Swing / Cone Limit)
  if (constraint.enableSwingLimit) {
    float cosSwing = std::clamp(uA.dot(uB), -1.0f, 1.0f);
    float swingAngle = std::acos(cosSwing);
    if (swingAngle > constraint.swingLimitAngle) {
      Vector3 swingAxis = uA.cross(uB);
      if (swingAxis.length() > 1e-5f) {
        swingAxis = swingAxis.getNormalized();
        float curSwingVel = (motionB.angularVelocity - motionA.angularVelocity).dot(swingAxis);
        float violation = swingAngle - constraint.swingLimitAngle;
        float bias = violation * (0.2f / deltaTime);
        float deltaW = -(curSwingVel + bias);
        if (deltaW < 0.0f) {
          float invAngMass = (massA.inverseInertiaTensor * swingAxis).dot(swingAxis) +
                             (massB.inverseInertiaTensor * swingAxis).dot(swingAxis);
          if (invAngMass > 1e-6f) {
            float j = deltaW / invAngMass;
            float oldImpulse = constraint.accumulatedSwingImpulse;
            constraint.accumulatedSwingImpulse = std::min(0.0f, oldImpulse + j);
            float dImpulse = constraint.accumulatedSwingImpulse - oldImpulse;
            constraint.appliedImpulseAngular += std::abs(dImpulse);

            Vector3 angImpulse = swingAxis * dImpulse;
            motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
            motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
          }
        }
      }
    } else {
      constraint.accumulatedSwingImpulse = 0.0f;
    }
  }

  // 3. Eksenel Burulma Sınırı (Twist Limit)
  if (constraint.enableTwistLimit) {
    Vector3 uAvg = (uA + uB);
    if (uAvg.length() > 1e-5f) {
      uAvg = uAvg.getNormalized();

      Vector3 a_A = constraint.twistAxisLocalA.getNormalized();
      Vector3 p_A1;
      if (std::abs(a_A.x) >= 0.57735f)
        p_A1 = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
      else
        p_A1 = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();

      Vector3 p_B1 = constraint.initialRelativeOrientation.getInverse() * p_A1;
      Vector3 p_worldA = poseA.orientation * p_A1;
      Vector3 p_worldB = poseB.orientation * p_B1;

      Vector3 pA_proj = (p_worldA - uAvg * p_worldA.dot(uAvg));
      Vector3 pB_proj = (p_worldB - uAvg * p_worldB.dot(uAvg));
      if (pA_proj.length() > 1e-5f && pB_proj.length() > 1e-5f) {
        pA_proj = pA_proj.getNormalized();
        pB_proj = pB_proj.getNormalized();

        float sinTwist = (pA_proj.cross(pB_proj)).dot(uAvg);
        float cosTwist = std::clamp(pA_proj.dot(pB_proj), -1.0f, 1.0f);
        float twistAngle = std::atan2(sinTwist, cosTwist);

        float curTwistVel = (motionB.angularVelocity - motionA.angularVelocity).dot(uAvg);
        float invAngMassTwist = (massA.inverseInertiaTensor * uAvg).dot(uAvg) +
                                (massB.inverseInertiaTensor * uAvg).dot(uAvg);
        if (invAngMassTwist > 1e-6f) {
          if (twistAngle <= constraint.lowerTwistLimit) {
            float violation = twistAngle - constraint.lowerTwistLimit;
            float bias = violation * (0.2f / deltaTime);
            float deltaW = -(curTwistVel + bias);
            float j = deltaW / invAngMassTwist;
            float oldImpulse = constraint.accumulatedTwistImpulse;
            constraint.accumulatedTwistImpulse = std::max(0.0f, oldImpulse + j);
            float dImpulse = constraint.accumulatedTwistImpulse - oldImpulse;
            constraint.appliedImpulseAngular += std::abs(dImpulse);

            Vector3 angImpulse = uAvg * dImpulse;
            motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
            motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
          } else if (twistAngle >= constraint.upperTwistLimit) {
            float violation = twistAngle - constraint.upperTwistLimit;
            float bias = violation * (0.2f / deltaTime);
            float deltaW = -(curTwistVel + bias);
            float j = deltaW / invAngMassTwist;
            float oldImpulse = constraint.accumulatedTwistImpulse;
            constraint.accumulatedTwistImpulse = std::min(0.0f, oldImpulse + j);
            float dImpulse = constraint.accumulatedTwistImpulse - oldImpulse;
            constraint.appliedImpulseAngular += std::abs(dImpulse);

            Vector3 angImpulse = uAvg * dImpulse;
            motionB.angularVelocity += massB.inverseInertiaTensor * angImpulse;
            motionA.angularVelocity -= massA.inverseInertiaTensor * angImpulse;
          } else {
            constraint.accumulatedTwistImpulse = 0.0f;
          }
        }
      }
    }
  }
}

void SequentialImpulsesKernel::solveConeTwistConstraintPosition(
    collision::ConeTwistConstraint &constraint, float deltaTime) {
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
  const auto &massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
  const auto &massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

  Vector3 rA = poseA.orientation * constraint.localAnchorA;
  Vector3 rB = poseB.orientation * constraint.localAnchorB;
  Vector3 pA = poseA.position + rA;
  Vector3 pB = poseB.position + rB;

  // 1. Doğrusal Konum Düzeltmesi (Split Impulse - 3 DOF Coupled Block Solver)
  Vector3 effA = pA + (motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA)) * deltaTime;
  Vector3 effB = pB + (motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB)) * deltaTime;
  Vector3 linearError = effA - effB;

  const float splitFactorLinear = 0.8f;
  Vector3 linearBias = linearError * (splitFactorLinear / deltaTime);

  Vector3 vSplitA = motionA.splitLinearVelocity + motionA.splitAngularVelocity.cross(rA);
  Vector3 vSplitB = motionB.splitLinearVelocity + motionB.splitAngularVelocity.cross(rB);
  Vector3 relSplitVel = vSplitA - vSplitB;

  float wSum = massA.inverseMass + massB.inverseMass;
  Matrix3x3 K;
  for (std::size_t col = 0; col < 3; ++col) {
    Vector3 e(col == 0 ? 1.0f : 0.0f, col == 1 ? 1.0f : 0.0f, col == 2 ? 1.0f : 0.0f);
    Vector3 colA = (massA.inverseMass > 0.0f) ? -rA.cross(massA.inverseInertiaTensor * rA.cross(e)) : Vector3(0, 0, 0);
    Vector3 colB = (massB.inverseMass > 0.0f) ? -rB.cross(massB.inverseInertiaTensor * rB.cross(e)) : Vector3(0, 0, 0);
    Vector3 kCol = e * wSum + colA + colB;
    K.rows[0][col] = kCol.x;
    K.rows[1][col] = kCol.y;
    K.rows[2][col] = kCol.z;
  }

  float det = K.getDeterminant();
  if (std::abs(det) > 1e-7f) {
    Matrix3x3 invK = K.getInverse();
    Vector3 impulse = invK * (-(relSplitVel + linearBias));

    motionA.splitLinearVelocity += impulse * massA.inverseMass;
    motionA.splitAngularVelocity += massA.inverseInertiaTensor * rA.cross(impulse);
    motionB.splitLinearVelocity -= impulse * massB.inverseMass;
    motionB.splitAngularVelocity -= massB.inverseInertiaTensor * rB.cross(impulse);
  }


  Vector3 uA = (poseA.orientation * constraint.twistAxisLocalA).getNormalized();
  Vector3 uB = (poseB.orientation * constraint.twistAxisLocalB).getNormalized();
  Vector3 relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;

  // 2. Koni Salınım Sınırı (Swing Split Impulse)
  if (constraint.enableSwingLimit) {
    float cosSwing = std::clamp(uA.dot(uB), -1.0f, 1.0f);
    float swingAngle = std::acos(cosSwing);
    if (swingAngle > constraint.swingLimitAngle) {
      Vector3 swingAxis = uA.cross(uB);
      if (swingAxis.length() > 1e-5f) {
        swingAxis = swingAxis.getNormalized();
        float violation = swingAngle - constraint.swingLimitAngle;
        float bias = violation * (0.8f / deltaTime);
        float deltaW = -(relSplitAngVel.dot(swingAxis) + bias);
        float invAngMass = (massA.inverseInertiaTensor * swingAxis).dot(swingAxis) +
                           (massB.inverseInertiaTensor * swingAxis).dot(swingAxis);
        if (invAngMass > 1e-6f) {
          float j = deltaW / invAngMass;
          if (j < 0.0f) {
            Vector3 angImpulse = swingAxis * j;
            motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
            motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
            relSplitAngVel = motionB.splitAngularVelocity - motionA.splitAngularVelocity;
          }
        }
      }
    }
  }

  // 3. Eksenel Burulma Sınırı (Twist Split Impulse)
  if (constraint.enableTwistLimit) {
    Vector3 uAvg = (uA + uB);
    if (uAvg.length() > 1e-5f) {
      uAvg = uAvg.getNormalized();

      Vector3 a_A = constraint.twistAxisLocalA.getNormalized();
      Vector3 p_A1;
      if (std::abs(a_A.x) >= 0.57735f)
        p_A1 = Vector3(a_A.y, -a_A.x, 0.0f).getNormalized();
      else
        p_A1 = Vector3(0.0f, a_A.z, -a_A.y).getNormalized();

      Vector3 p_B1 = constraint.initialRelativeOrientation.getInverse() * p_A1;
      Vector3 p_worldA = poseA.orientation * p_A1;
      Vector3 p_worldB = poseB.orientation * p_B1;

      Vector3 pA_proj = (p_worldA - uAvg * p_worldA.dot(uAvg));
      Vector3 pB_proj = (p_worldB - uAvg * p_worldB.dot(uAvg));
      if (pA_proj.length() > 1e-5f && pB_proj.length() > 1e-5f) {
        pA_proj = pA_proj.getNormalized();
        pB_proj = pB_proj.getNormalized();

        float sinTwist = (pA_proj.cross(pB_proj)).dot(uAvg);
        float cosTwist = std::clamp(pA_proj.dot(pB_proj), -1.0f, 1.0f);
        float twistAngle = std::atan2(sinTwist, cosTwist);

        float invAngMassTwist = (massA.inverseInertiaTensor * uAvg).dot(uAvg) +
                                (massB.inverseInertiaTensor * uAvg).dot(uAvg);
        if (invAngMassTwist > 1e-6f) {
          if (twistAngle < constraint.lowerTwistLimit) {
            float violation = twistAngle - constraint.lowerTwistLimit;
            float bias = violation * (0.8f / deltaTime);
            float deltaW = -(relSplitAngVel.dot(uAvg) + bias);
            float j = deltaW / invAngMassTwist;
            if (j > 0.0f) {
              Vector3 angImpulse = uAvg * j;
              motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
              motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
            }
          } else if (twistAngle > constraint.upperTwistLimit) {
            float violation = twistAngle - constraint.upperTwistLimit;
            float bias = violation * (0.8f / deltaTime);
            float deltaW = -(relSplitAngVel.dot(uAvg) + bias);
            float j = deltaW / invAngMassTwist;
            if (j < 0.0f) {
              Vector3 angImpulse = uAvg * j;
              motionB.splitAngularVelocity += massB.inverseInertiaTensor * angImpulse;
              motionA.splitAngularVelocity -= massA.inverseInertiaTensor * angImpulse;
            }
          }
        }
      }
    }
  }
}

} // namespace Baryon::systems

