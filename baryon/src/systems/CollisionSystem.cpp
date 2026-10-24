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
 * @file CollisionSystem.cpp
 * @brief Çarpışma algılama ve temas (manifold) yönetimi sistemi.
 * @details Geniş fazdan (Broad Phase) gelen aday çiftleri dar fazda (GJK/EPA)
 * test eder, temas noktalarını (Contact Manifold) üretir ve bu noktaları
 * kareler boyunca koruyarak (Persistence) simülasyonun kararlı kalmasını
 * sağlar.
 */

#include "Baryon/systems/CollisionSystem.hpp"
#include "Baryon/collision/NarrowPhase.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include "Baryon/collision/CollisionListener.hpp"
#include <iostream>
#include <algorithm>

namespace Baryon::systems {

namespace {
inline uint64_t makeEntityPairKey(ecs::Entity a, ecs::Entity b) {
  uint32_t idx1 = ecs::EntityManager::getIndex(a);
  uint32_t idx2 = ecs::EntityManager::getIndex(b);
  if (idx1 > idx2)
    std::swap(idx1, idx2);
  return (static_cast<uint64_t>(idx1) << 32) | static_cast<uint64_t>(idx2);
}
} // namespace



CollisionSystem::CollisionSystem(Core::Registry &registry,
                                 SpatialPartitioning &sp)
    : mRegistry(registry), mSpatialPartitioning(sp) {}

void CollisionSystem::wakeIfSleeping(ecs::Entity e) {
  if (!mRegistry.hasComponent<Core::BodyState>(e))
    return;
  auto &st = mRegistry.getComponent<Core::BodyState>(e);
  if (!st.isSleeping)
    return;
  st.isSleeping = false;
  st.sleepTimer = 0.0f;
}

void CollisionSystem::processMeshCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    bool isMeshB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifoldMap) {
  auto entMesh = isMeshB ? entityB : entityA;
  auto entConv = isMeshB ? entityA : entityB;
  const auto &transMesh = isMeshB ? transB : transA;
  const auto &transConv = isMeshB ? transA : transB;
  const auto &collMesh = isMeshB ? collB : collA;
  const auto &collConv = isMeshB ? collA : collB;

  auto meshShape =
      std::get<collision::StaticMeshShape>(collMesh.shape.getVariant());
  if (!meshShape.mesh)
    return;

  // Nesnenin boyutuna ve hızına göre dinamik sorgu AABB'si hesapla
  collision::AABB localConvAABB = collConv.shape.computeLocalAABB();
  float convRadius =
      (localConvAABB.maxBounds - localConvAABB.minBounds).length() * 0.5f;
  float queryExtent = std::max(physics::PhysicsConstants::DefaultMeshQueryExtent, convRadius + 1.0f);
  Vector3 localPos = transMesh.orientation.getConjugate() *
                     (transConv.position - transMesh.position);
  collision::AABB queryAABB(
      localPos - Vector3(queryExtent, queryExtent, queryExtent),
      localPos + Vector3(queryExtent, queryExtent, queryExtent));

  meshShape.mesh->queryTriangles(
      queryAABB, [&](const collision::TriangleShape &tri) {
        collision::CollisionShape triShapeWrapper(tri);
        collision::Simplex simplex;

        if (collision::NarrowPhase::GJK(collConv.shape, transConv,
                                        triShapeWrapper, transMesh,
                                        simplex)) {
          collision::CollisionInfo info = collision::NarrowPhase::EPA(
              simplex, collConv.shape, transConv, triShapeWrapper,
              transMesh);

          if (info.hasCollision) {
            bool awakeConv = mRegistry.hasComponent<Core::BodyState>(entConv) && !mRegistry.getComponent<Core::BodyState>(entConv).isSleeping;
            bool awakeMesh = mRegistry.hasComponent<Core::BodyState>(entMesh) && !mRegistry.getComponent<Core::BodyState>(entMesh).isSleeping;
            if ((awakeConv && !awakeMesh) || (awakeMesh && !awakeConv) ||
                info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
              wakeIfSleeping(entConv);
              wakeIfSleeping(entMesh);
            }

            // İki nesne için benzersiz anahtar üret
            uint64_t key = makeEntityPairKey(entConv, entMesh);

            auto [it, inserted] =
                mManifoldMap.try_emplace(key, entConv, entMesh);
            it->second.isTriggerPair = collConv.isTrigger || collMesh.isTrigger;
            Vector3 currentNormal = -info.normal; // Normal sozlesmesi: B'den A'ya (mesh'ten convex'e)

            if (!inserted && it->second.contactCount > 0 &&
                it->second.normal.dot(currentNormal) < physics::PhysicsConstants::ManifoldNormalAlignmentThreshold) {
              it->second.contactCount = 0;
            }

            if (info.penetration > it->second.penetration ||
                it->second.contactCount == 0) {
              it->second.normal = currentNormal;
              it->second.penetration = info.penetration;
              it->second.computeTangents();
            }

            bool isMeshBox = std::holds_alternative<collision::BoxShape>(collConv.shape.getVariant());
            bool isMeshHull = std::holds_alternative<collision::ConvexHullShape>(collConv.shape.getVariant());
            bool isMeshCap = std::holds_alternative<collision::CapsuleShape>(collConv.shape.getVariant());

            if (isMeshBox) {
              const auto &box = std::get<collision::BoxShape>(collConv.shape.getVariant());
              Vector3 h = box.halfExtents;
              std::array<Vector3, 8> localVerts = {
                  Vector3(-h.x, -h.y, -h.z), Vector3(h.x, -h.y, -h.z),
                  Vector3(h.x,  h.y, -h.z), Vector3(-h.x,  h.y, -h.z),
                  Vector3(-h.x, -h.y,  h.z), Vector3(h.x, -h.y,  h.z),
                  Vector3(h.x,  h.y,  h.z), Vector3(-h.x,  h.y,  h.z)
              };
              Vector3 searchDir = -currentNormal;
              float maxProj = -physics::PhysicsConstants::Infinity;
              for (const auto &v : localVerts) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                float p = wP.dot(searchDir);
                if (p > maxProj) maxProj = p;
              }
              for (const auto &v : localVerts) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                if (wP.dot(searchDir) >= maxProj - 0.005f && it->second.contactCount < 4) {
                  collision::ContactPoint faceCp;
                  faceCp.worldPosition = wP;
                  faceCp.localPointA = v;
                  faceCp.localPointB = transMesh.orientation.getConjugate() * (faceCp.worldPosition - transMesh.position);
                  faceCp.penetration = info.penetration;
                  faceCp.normalImpulse = 0.0f;
                  faceCp.tangentImpulse1 = 0.0f;
                  faceCp.tangentImpulse2 = 0.0f;
                  faceCp.splitImpulse = 0.0f;
                  it->second.addContact(faceCp);
                }
              }
            } else if (isMeshHull) {
              const auto &hull = std::get<collision::ConvexHullShape>(collConv.shape.getVariant());
              Vector3 searchDir = -currentNormal;
              float maxProj = -physics::PhysicsConstants::Infinity;
              for (const auto &v : hull.vertices) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                float p = wP.dot(searchDir);
                if (p > maxProj) maxProj = p;
              }
              for (const auto &v : hull.vertices) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                if (wP.dot(searchDir) >= maxProj - 0.005f && it->second.contactCount < 4) {
                  collision::ContactPoint faceCp;
                  faceCp.worldPosition = wP;
                  faceCp.localPointA = v;
                  faceCp.localPointB = transMesh.orientation.getConjugate() * (faceCp.worldPosition - transMesh.position);
                  faceCp.penetration = info.penetration;
                  faceCp.normalImpulse = 0.0f;
                  faceCp.tangentImpulse1 = 0.0f;
                  faceCp.tangentImpulse2 = 0.0f;
                  faceCp.splitImpulse = 0.0f;
                  it->second.addContact(faceCp);
                }
              }
            } else if (isMeshCap) {
              const auto &cap = std::get<collision::CapsuleShape>(collConv.shape.getVariant());
              Vector3 axisConv = transConv.orientation * Vector3(0, 1, 0);
              if (std::abs(axisConv.dot(currentNormal)) < 0.25f) {
                Vector3 localDir = transConv.orientation.getConjugate() * (-currentNormal);
                Vector3 radialDir(localDir.x, 0.0f, localDir.z);
                if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
                float halfH = cap.height * 0.5f;
                std::array<Vector3, 2> capPts = {
                    Vector3(0, halfH, 0) + radialDir * cap.radius,
                    Vector3(0, -halfH, 0) + radialDir * cap.radius
                };
                Vector3 searchDir = -currentNormal;
                Vector3 p0 = transConv.position + (transConv.orientation * capPts[0]);
                Vector3 p1 = transConv.position + (transConv.orientation * capPts[1]);
                if (std::abs(p0.dot(searchDir) - p1.dot(searchDir)) <= 0.005f) {
                  for (const auto &v : capPts) {
                    collision::ContactPoint faceCp;
                    faceCp.worldPosition = transConv.position + (transConv.orientation * v);
                    faceCp.localPointA = v;
                    faceCp.localPointB = transMesh.orientation.getConjugate() * (faceCp.worldPosition - transMesh.position);
                    faceCp.penetration = info.penetration;
                    faceCp.normalImpulse = 0.0f;
                    faceCp.tangentImpulse1 = 0.0f;
                    faceCp.tangentImpulse2 = 0.0f;
                    faceCp.splitImpulse = 0.0f;
                    it->second.addContact(faceCp);
                  }
                }
              }
            }

            if (it->second.contactCount == 0) {
              collision::ContactPoint cp;
              cp.worldPosition = info.contactPoint;
              cp.localPointA = transConv.orientation.getConjugate() *
                               (cp.worldPosition - transConv.position);
              cp.localPointB = transMesh.orientation.getConjugate() *
                               (cp.worldPosition - transMesh.position);
              cp.penetration = info.penetration;
              cp.normalImpulse = 0.0f;
              cp.tangentImpulse1 = 0.0f;
              cp.tangentImpulse2 = 0.0f;
              cp.splitImpulse = 0.0f;

              if (it->second.contactCount < 4)
                it->second.addContact(cp);
              else
                it->second.contacts[3] = cp;
            }

            auto oldIt = oldManifoldMap.find(key);
            if (oldIt != oldManifoldMap.end()) {
              it->second.inheritImpulses(oldIt->second, physics::PhysicsConstants::ContactPointMatchRadiusSq);
            }
          }
        }
      });
}

void CollisionSystem::processBoxBoxCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifoldMap) {
  uint64_t key = makeEntityPairKey(entityA, entityB);
  bool isTriggerPair = collA.isTrigger || collB.isTrigger;
  collision::ContactManifold satManifold(entityA, entityB);
  const auto &bA = std::get<collision::BoxShape>(collA.shape.getVariant());
  const auto &bB = std::get<collision::BoxShape>(collB.shape.getVariant());

  if (collision::NarrowPhase::testBoxBox(bA, transA, bB, transB, satManifold)) {
    if (satManifold.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
      wakeIfSleeping(entityA);
      wakeIfSleeping(entityB);
    }

    auto oldIt = oldManifoldMap.find(key);
    if (oldIt != oldManifoldMap.end()) {
      satManifold.inheritImpulses(oldIt->second, physics::PhysicsConstants::ContactPointMatchRadiusSq);
    }

    satManifold.isTriggerPair = isTriggerPair;
    satManifold.computeTangents();
    mManifoldMap.insert_or_assign(key, satManifold);
  } else {
    auto oldIt = oldManifoldMap.find(key);
    if (oldIt != oldManifoldMap.end() && oldIt->second.contactCount > 0) {
      float minSep = physics::PhysicsConstants::Infinity;
      for (uint32_t cIdx = 0; cIdx < oldIt->second.contactCount; ++cIdx) {
        const auto &oldCp = oldIt->second.contacts[cIdx];
        Vector3 rA = transA.orientation * oldCp.localPointA;
        Vector3 rB = transB.orientation * oldCp.localPointB;
        float sep = ((transA.position + rA) - (transB.position + rB)).dot(oldIt->second.normal);
        minSep = std::min(minSep, sep);
      }
      if (minSep < physics::PhysicsConstants::LinearSlop) {
        satManifold = oldIt->second;
        satManifold.penetration = std::max(0.0f, -minSep);
        for (uint32_t cIdx = 0; cIdx < satManifold.contactCount; ++cIdx) {
          satManifold.contacts[cIdx].penetration = satManifold.penetration;
          satManifold.contacts[cIdx].worldPosition = transB.position + transB.orientation * satManifold.contacts[cIdx].localPointB;
        }
        mManifoldMap.insert_or_assign(key, satManifold);
      }
    }
  }
}

void CollisionSystem::processHeightfieldCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    bool isHfB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifoldMap) {
  auto entHf = isHfB ? entityB : entityA;
  auto entConv = isHfB ? entityA : entityB;
  const auto &transHf = isHfB ? transB : transA;
  const auto &transConv = isHfB ? transA : transB;
  const auto &collHf = isHfB ? collB : collA;
  const auto &collConv = isHfB ? collA : collB;

  const auto &hfShape = std::get<collision::HeightfieldShape>(collHf.shape.getVariant());

  collision::AABB localConvAABB = collConv.shape.computeLocalAABB();
  float convRadius = (localConvAABB.maxBounds - localConvAABB.minBounds).length() * 0.5f;
  float queryExtent = std::max(physics::PhysicsConstants::DefaultMeshQueryExtent, convRadius + 1.0f);
  Vector3 localPos = transHf.orientation.getConjugate() * (transConv.position - transHf.position);
  collision::AABB queryAABB(
      localPos - Vector3(queryExtent, queryExtent, queryExtent),
      localPos + Vector3(queryExtent, queryExtent, queryExtent));

  hfShape.queryTriangles(
      queryAABB, [&](const collision::TriangleShape &tri) {
        collision::CollisionShape triShapeWrapper(tri);
        collision::Simplex simplex;

        if (collision::NarrowPhase::GJK(collConv.shape, transConv,
                                        triShapeWrapper, transHf,
                                        simplex)) {
          collision::CollisionInfo info = collision::NarrowPhase::EPA(
              simplex, collConv.shape, transConv, triShapeWrapper,
              transHf);

          if (info.hasCollision && info.penetration > 1e-4f) {
            bool awakeConv = mRegistry.hasComponent<Core::BodyState>(entConv) && !mRegistry.getComponent<Core::BodyState>(entConv).isSleeping;
            bool awakeHf = mRegistry.hasComponent<Core::BodyState>(entHf) && !mRegistry.getComponent<Core::BodyState>(entHf).isSleeping;
            if ((awakeConv && !awakeHf) || (awakeHf && !awakeConv) ||
                info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
              wakeIfSleeping(entConv);
              wakeIfSleeping(entHf);
            }

            uint64_t key = makeEntityPairKey(entConv, entHf);

            auto [it, inserted] = mManifoldMap.try_emplace(key, entConv, entHf);
            it->second.isTriggerPair = collConv.isTrigger || collHf.isTrigger;
            Vector3 currentNormal = -info.normal; // Normal sozlesmesi: B'den A'ya (Hf'den conv'e)

            // Ghost-collision filtresi: Uçgen yüzey normali ile yumuşatma
            Vector3 triEdge1 = tri.v1 - tri.v0;
            Vector3 triEdge2 = tri.v2 - tri.v0;
            Vector3 triFaceNorm = transHf.orientation * (triEdge1.cross(triEdge2)).getNormalized();
            if (currentNormal.dot(triFaceNorm) > 0.65f) {
              currentNormal = triFaceNorm;
            }

            if (inserted || info.penetration > it->second.penetration + 1e-4f || it->second.contactCount == 0) {
              it->second.contactCount = 0;
              it->second.normal = currentNormal;
              it->second.penetration = info.penetration;
              it->second.computeTangents();
            } else if (it->second.normal.dot(currentNormal) < physics::PhysicsConstants::ManifoldNormalAlignmentThreshold) {
              return;
            }

            bool isMeshBox = std::holds_alternative<collision::BoxShape>(collConv.shape.getVariant());
            bool isMeshCap = std::holds_alternative<collision::CapsuleShape>(collConv.shape.getVariant());

            if (isMeshBox) {
              const auto &box = std::get<collision::BoxShape>(collConv.shape.getVariant());
              Vector3 h = box.halfExtents;
              std::array<Vector3, 8> localVerts = {
                  Vector3(-h.x, -h.y, -h.z), Vector3(h.x, -h.y, -h.z),
                  Vector3(h.x,  h.y, -h.z), Vector3(-h.x,  h.y, -h.z),
                  Vector3(-h.x, -h.y,  h.z), Vector3(h.x,  h.y,  h.z),
                  Vector3(h.x,  h.y,  h.z), Vector3(-h.x,  h.y,  h.z)
              };
              Vector3 searchDir = -currentNormal;
              float maxProj = -physics::PhysicsConstants::Infinity;
              for (const auto &v : localVerts) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                float p = wP.dot(searchDir);
                if (p > maxProj) maxProj = p;
              }
              for (const auto &v : localVerts) {
                Vector3 wP = transConv.position + (transConv.orientation * v);
                if (wP.dot(searchDir) >= maxProj - 0.005f && it->second.contactCount < 4) {
                  collision::ContactPoint faceCp;
                  faceCp.worldPosition = wP;
                  faceCp.localPointA = v;
                  faceCp.localPointB = transHf.orientation.getConjugate() * (faceCp.worldPosition - transHf.position);
                  faceCp.penetration = info.penetration;
                  faceCp.normalImpulse = 0.0f;
                  faceCp.tangentImpulse1 = 0.0f;
                  faceCp.tangentImpulse2 = 0.0f;
                  faceCp.splitImpulse = 0.0f;
                  it->second.addContact(faceCp);
                }
              }
            } else if (isMeshCap) {
              const auto &cap = std::get<collision::CapsuleShape>(collConv.shape.getVariant());
              Vector3 axisConv = transConv.orientation * Vector3(0, 1, 0);
              if (std::abs(axisConv.dot(currentNormal)) < 0.25f) {
                Vector3 localDir = transConv.orientation.getConjugate() * (-currentNormal);
                Vector3 radialDir(localDir.x, 0.0f, localDir.z);
                if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
                float halfH = cap.height * 0.5f;
                std::array<Vector3, 2> capPts = {
                    Vector3(0, halfH, 0) + radialDir * cap.radius,
                    Vector3(0, -halfH, 0) + radialDir * cap.radius
                };
                Vector3 searchDir = -currentNormal;
                Vector3 p0 = transConv.position + (transConv.orientation * capPts[0]);
                Vector3 p1 = transConv.position + (transConv.orientation * capPts[1]);
                if (std::abs(p0.dot(searchDir) - p1.dot(searchDir)) <= 0.005f) {
                  for (const auto &v : capPts) {
                    collision::ContactPoint faceCp;
                    faceCp.worldPosition = transConv.position + (transConv.orientation * v);
                    faceCp.localPointA = v;
                    faceCp.localPointB = transHf.orientation.getConjugate() * (faceCp.worldPosition - transHf.position);
                    faceCp.penetration = info.penetration;
                    faceCp.normalImpulse = 0.0f;
                    faceCp.tangentImpulse1 = 0.0f;
                    faceCp.tangentImpulse2 = 0.0f;
                    faceCp.splitImpulse = 0.0f;
                    it->second.addContact(faceCp);
                  }
                }
              }
            }

            if (it->second.contactCount == 0) {
              collision::ContactPoint cp;
              cp.worldPosition = info.contactPoint;
              cp.localPointA = transConv.orientation.getConjugate() * (cp.worldPosition - transConv.position);
              cp.localPointB = transHf.orientation.getConjugate() * (cp.worldPosition - transHf.position);
              cp.penetration = info.penetration;
              cp.normalImpulse = 0.0f;
              cp.tangentImpulse1 = 0.0f;
              cp.tangentImpulse2 = 0.0f;
              cp.splitImpulse = 0.0f;

              if (it->second.contactCount < 4)
                it->second.addContact(cp);
              else
                it->second.contacts[3] = cp;
            }

            auto oldIt = oldManifoldMap.find(key);
            if (oldIt != oldManifoldMap.end()) {
              it->second.inheritImpulses(oldIt->second, physics::PhysicsConstants::ContactPointMatchRadiusSq);
            }
          }
        }
      });
}

void CollisionSystem::processConvexCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifoldMap) {
  bool isBoxA = std::holds_alternative<collision::BoxShape>(collA.shape.getVariant());
  bool isBoxB = std::holds_alternative<collision::BoxShape>(collB.shape.getVariant());
  bool isSphereA = std::holds_alternative<collision::SphereShape>(collA.shape.getVariant());
  bool isSphereB = std::holds_alternative<collision::SphereShape>(collB.shape.getVariant());

  bool isTriggerPair = collA.isTrigger || collB.isTrigger;
  uint64_t key = makeEntityPairKey(entityA, entityB);

  collision::CollisionInfo info;
  bool hasCollided = false;

  if (isSphereA && isSphereB) {
    const auto &spA = std::get<collision::SphereShape>(collA.shape.getVariant());
    const auto &spB = std::get<collision::SphereShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereSphere(spA, transA, spB, transB, info);
  } else if (isSphereA && isBoxB) {
    const auto &spA = std::get<collision::SphereShape>(collA.shape.getVariant());
    const auto &bB = std::get<collision::BoxShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereBox(spA, transA, bB, transB, info);
  } else if (isBoxA && isSphereB) {
    const auto &bA = std::get<collision::BoxShape>(collA.shape.getVariant());
    const auto &spB = std::get<collision::SphereShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereBox(spB, transB, bA, transA, info);
    if (hasCollided) {
      info.normal = -info.normal; // Normal must point B to A
    }
  } else {
    collision::Simplex simplex;
    if (collision::NarrowPhase::GJK(collA.shape, transA, collB.shape, transB, simplex)) {
      info = collision::NarrowPhase::EPA(simplex, collA.shape, transA, collB.shape, transB);
      hasCollided = info.hasCollision;
      if (hasCollided) {
        info.normal = -info.normal; // Normal sözleşmesi: manifold normali B'den A'ya bakar. EPA A->B döndürür.
      }
    } else {
      auto oldIt = oldManifoldMap.find(key);
      if (oldIt != oldManifoldMap.end() && oldIt->second.contactCount > 0) {
        const auto &oldCp = oldIt->second.contacts[0];
        Vector3 rA = transA.orientation * oldCp.localPointA;
        Vector3 rB = transB.orientation * oldCp.localPointB;
        float separation = ((transA.position + rA) - (transB.position + rB)).dot(oldIt->second.normal);
        if (separation < physics::PhysicsConstants::LinearSlop) {
          hasCollided = true;
          info.hasCollision = true;
          info.normal = oldIt->second.normal;
          info.penetration = std::max(0.0f, -separation);
          info.contactPoint = transB.position + rB;
        }
      }
    }
  }

  if (hasCollided) {
    bool awakeA = mRegistry.hasComponent<Core::BodyState>(entityA) && !mRegistry.getComponent<Core::BodyState>(entityA).isSleeping;
    bool awakeB = mRegistry.hasComponent<Core::BodyState>(entityB) && !mRegistry.getComponent<Core::BodyState>(entityB).isSleeping;
    if ((awakeA && !awakeB) || (awakeB && !awakeA) ||
        info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
      wakeIfSleeping(entityA);
      wakeIfSleeping(entityB);
    }

    auto [it, inserted] = mManifoldMap.try_emplace(key, entityA, entityB);
    it->second.isTriggerPair = isTriggerPair;
    it->second.normal = info.normal;
    it->second.penetration = info.penetration;
    it->second.computeTangents();

    collision::ContactPoint cp;
    cp.worldPosition = info.contactPoint;
    cp.localPointA = transA.orientation.getConjugate() *
                     (cp.worldPosition - transA.position);
    cp.localPointB = transB.orientation.getConjugate() *
                     (cp.worldPosition - transB.position);
    cp.penetration = info.penetration;
    cp.normalImpulse = 0.0f;
    cp.tangentImpulse1 = 0.0f;
    cp.tangentImpulse2 = 0.0f;
    cp.splitImpulse = 0.0f;

    it->second.contactCount = 0;

    bool isHullA = std::holds_alternative<collision::ConvexHullShape>(collA.shape.getVariant());
    bool isHullB = std::holds_alternative<collision::ConvexHullShape>(collB.shape.getVariant());
    bool isCapsuleA = std::holds_alternative<collision::CapsuleShape>(collA.shape.getVariant());
    bool isCapsuleB = std::holds_alternative<collision::CapsuleShape>(collB.shape.getVariant());
    bool isCylinderA = std::holds_alternative<collision::CylinderShape>(collA.shape.getVariant());
    bool isCylinderB = std::holds_alternative<collision::CylinderShape>(collB.shape.getVariant());
    bool isConeA = std::holds_alternative<collision::ConeShape>(collA.shape.getVariant());
    bool isConeB = std::holds_alternative<collision::ConeShape>(collB.shape.getVariant());

    if (isHullA) {
      const auto &hullA = std::get<collision::ConvexHullShape>(collA.shape.getVariant());
      Vector3 searchDir = -it->second.normal;
      float maxProj = -physics::PhysicsConstants::Infinity;
      for (const auto &v : hullA.vertices) {
        Vector3 wP = transA.position + (transA.orientation * v);
        float p = wP.dot(searchDir);
        if (p > maxProj) maxProj = p;
      }
      for (const auto &v : hullA.vertices) {
        Vector3 wP = transA.position + (transA.orientation * v);
        if (wP.dot(searchDir) >= maxProj - 0.005f && it->second.contactCount < 4) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = wP;
          faceCp.localPointA = v;
          faceCp.localPointB = transB.orientation.getConjugate() *
                              (faceCp.worldPosition - transB.position);
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    } else if (isHullB) {
      const auto &hullB = std::get<collision::ConvexHullShape>(collB.shape.getVariant());
      Vector3 searchDir = it->second.normal;
      float maxProj = -physics::PhysicsConstants::Infinity;
      for (const auto &v : hullB.vertices) {
        Vector3 wP = transB.position + (transB.orientation * v);
        float p = wP.dot(searchDir);
        if (p > maxProj) maxProj = p;
      }
      for (const auto &v : hullB.vertices) {
        Vector3 wP = transB.position + (transB.orientation * v);
        if (wP.dot(searchDir) >= maxProj - 0.005f && it->second.contactCount < 4) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = wP;
          faceCp.localPointA = transA.orientation.getConjugate() *
                              (faceCp.worldPosition - transA.position);
          faceCp.localPointB = v;
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    } else if (isCapsuleA) {
      const auto &capA = std::get<collision::CapsuleShape>(collA.shape.getVariant());
      Vector3 axisA = transA.orientation * Vector3(0, 1, 0);
      if (std::abs(axisA.dot(it->second.normal)) < 0.25f) {
        Vector3 localDir = transA.orientation.getConjugate() * (-it->second.normal);
        Vector3 radialDir(localDir.x, 0.0f, localDir.z);
        if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
        float halfH = capA.height * 0.5f;
        std::array<Vector3, 2> capPts = {
            Vector3(0, halfH, 0) + radialDir * capA.radius,
            Vector3(0, -halfH, 0) + radialDir * capA.radius
        };
        Vector3 searchDir = -it->second.normal;
        Vector3 p0 = transA.position + (transA.orientation * capPts[0]);
        Vector3 p1 = transA.position + (transA.orientation * capPts[1]);
        if (std::abs(p0.dot(searchDir) - p1.dot(searchDir)) <= 0.005f) {
          for (const auto &v : capPts) {
            collision::ContactPoint faceCp;
            faceCp.worldPosition = transA.position + (transA.orientation * v);
            faceCp.localPointA = v;
            faceCp.localPointB = transB.orientation.getConjugate() *
                                (faceCp.worldPosition - transB.position);
            faceCp.penetration = info.penetration;
            faceCp.normalImpulse = 0.0f;
            faceCp.tangentImpulse1 = 0.0f;
            faceCp.tangentImpulse2 = 0.0f;
            faceCp.splitImpulse = 0.0f;
            it->second.addContact(faceCp);
          }
        }
      }
    } else if (isCapsuleB) {
      const auto &capB = std::get<collision::CapsuleShape>(collB.shape.getVariant());
      Vector3 axisB = transB.orientation * Vector3(0, 1, 0);
      if (std::abs(axisB.dot(it->second.normal)) < 0.25f) {
        Vector3 localDir = transB.orientation.getConjugate() * (it->second.normal);
        Vector3 radialDir(localDir.x, 0.0f, localDir.z);
        if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
        float halfH = capB.height * 0.5f;
        std::array<Vector3, 2> capPts = {
            Vector3(0, halfH, 0) + radialDir * capB.radius,
            Vector3(0, -halfH, 0) + radialDir * capB.radius
        };
        Vector3 searchDir = it->second.normal;
        Vector3 p0 = transB.position + (transB.orientation * capPts[0]);
        Vector3 p1 = transB.position + (transB.orientation * capPts[1]);
        if (std::abs(p0.dot(searchDir) - p1.dot(searchDir)) <= 0.005f) {
          for (const auto &v : capPts) {
            collision::ContactPoint faceCp;
            faceCp.worldPosition = transB.position + (transB.orientation * v);
            faceCp.localPointA = transA.orientation.getConjugate() *
                                (faceCp.worldPosition - transA.position);
            faceCp.localPointB = v;
            faceCp.penetration = info.penetration;
            faceCp.normalImpulse = 0.0f;
            faceCp.tangentImpulse1 = 0.0f;
            faceCp.tangentImpulse2 = 0.0f;
            faceCp.splitImpulse = 0.0f;
            it->second.addContact(faceCp);
          }
        }
      }
    } else if (isCylinderA) {
      const auto &cylA = std::get<collision::CylinderShape>(collA.shape.getVariant());
      Vector3 axisA = transA.orientation * Vector3(0, 1, 0);
      float axisDot = axisA.dot(it->second.normal);
      if (std::abs(axisDot) < 0.25f) {
        Vector3 localDir = transA.orientation.getConjugate() * (-it->second.normal);
        Vector3 radialDir(localDir.x, 0.0f, localDir.z);
        if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
        Vector3 v = radialDir * cylA.radius;
        collision::ContactPoint faceCp;
        faceCp.worldPosition = transA.position + (transA.orientation * v);
        faceCp.localPointA = v;
        faceCp.localPointB = transB.orientation.getConjugate() *
                            (faceCp.worldPosition - transB.position);
        faceCp.penetration = info.penetration;
        faceCp.normalImpulse = 0.0f;
        faceCp.tangentImpulse1 = 0.0f;
        faceCp.tangentImpulse2 = 0.0f;
        faceCp.splitImpulse = 0.0f;
        it->second.addContact(faceCp);
      } else if (std::abs(axisDot) > 0.85f) {
        float capSign = (axisA.dot(-it->second.normal) > 0.0f) ? 1.0f : -1.0f;
        float capY = capSign * cylA.halfHeight;
        std::array<Vector3, 4> capPts = {
            Vector3(cylA.radius, capY, 0.0f),
            Vector3(-cylA.radius, capY, 0.0f),
            Vector3(0.0f, capY, cylA.radius),
            Vector3(0.0f, capY, -cylA.radius)
        };
        for (const auto &v : capPts) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = transA.position + (transA.orientation * v);
          faceCp.localPointA = v;
          faceCp.localPointB = transB.orientation.getConjugate() *
                              (faceCp.worldPosition - transB.position);
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    } else if (isCylinderB) {
      const auto &cylB = std::get<collision::CylinderShape>(collB.shape.getVariant());
      Vector3 axisB = transB.orientation * Vector3(0, 1, 0);
      float axisDot = axisB.dot(it->second.normal);
      if (std::abs(axisDot) < 0.25f) {
        Vector3 localDir = transB.orientation.getConjugate() * (it->second.normal);
        Vector3 radialDir(localDir.x, 0.0f, localDir.z);
        if (radialDir.lengthSquare() > 1e-6f) radialDir.normalize(); else radialDir = Vector3(1, 0, 0);
        Vector3 v = radialDir * cylB.radius;
        collision::ContactPoint faceCp;
        faceCp.worldPosition = transB.position + (transB.orientation * v);
        faceCp.localPointA = transA.orientation.getConjugate() *
                            (faceCp.worldPosition - transA.position);
        faceCp.localPointB = v;
        faceCp.penetration = info.penetration;
        faceCp.normalImpulse = 0.0f;
        faceCp.tangentImpulse1 = 0.0f;
        faceCp.tangentImpulse2 = 0.0f;
        faceCp.splitImpulse = 0.0f;
        it->second.addContact(faceCp);
      } else if (std::abs(axisDot) > 0.85f) {
        float capSign = (axisB.dot(it->second.normal) > 0.0f) ? 1.0f : -1.0f;
        float capY = capSign * cylB.halfHeight;
        std::array<Vector3, 4> capPts = {
            Vector3(cylB.radius, capY, 0.0f),
            Vector3(-cylB.radius, capY, 0.0f),
            Vector3(0.0f, capY, cylB.radius),
            Vector3(0.0f, capY, -cylB.radius)
        };
        for (const auto &v : capPts) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = transB.position + (transB.orientation * v);
          faceCp.localPointA = transA.orientation.getConjugate() *
                              (faceCp.worldPosition - transA.position);
          faceCp.localPointB = v;
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    } else if (isConeA) {
      const auto &coneA = std::get<collision::ConeShape>(collA.shape.getVariant());
      Vector3 baseNormalA = transA.orientation * Vector3(0, -1, 0);
      if (baseNormalA.dot(-it->second.normal) > 0.85f) {
        float baseY = coneA.getBaseY();
        std::array<Vector3, 4> basePts = {
            Vector3(coneA.radius, baseY, 0.0f),
            Vector3(-coneA.radius, baseY, 0.0f),
            Vector3(0.0f, baseY, coneA.radius),
            Vector3(0.0f, baseY, -coneA.radius)
        };
        for (const auto &v : basePts) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = transA.position + (transA.orientation * v);
          faceCp.localPointA = v;
          faceCp.localPointB = transB.orientation.getConjugate() *
                              (faceCp.worldPosition - transB.position);
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    } else if (isConeB) {
      const auto &coneB = std::get<collision::ConeShape>(collB.shape.getVariant());
      Vector3 baseNormalB = transB.orientation * Vector3(0, -1, 0);
      if (baseNormalB.dot(it->second.normal) > 0.85f) {
        float baseY = coneB.getBaseY();
        std::array<Vector3, 4> basePts = {
            Vector3(coneB.radius, baseY, 0.0f),
            Vector3(-coneB.radius, baseY, 0.0f),
            Vector3(0.0f, baseY, coneB.radius),
            Vector3(0.0f, baseY, -coneB.radius)
        };
        for (const auto &v : basePts) {
          collision::ContactPoint faceCp;
          faceCp.worldPosition = transB.position + (transB.orientation * v);
          faceCp.localPointA = transA.orientation.getConjugate() *
                              (faceCp.worldPosition - transA.position);
          faceCp.localPointB = v;
          faceCp.penetration = info.penetration;
          faceCp.normalImpulse = 0.0f;
          faceCp.tangentImpulse1 = 0.0f;
          faceCp.tangentImpulse2 = 0.0f;
          faceCp.splitImpulse = 0.0f;
          it->second.addContact(faceCp);
        }
      }
    }

    if (it->second.contactCount == 0) {
      it->second.addContact(cp);
    }

    auto oldIt = oldManifoldMap.find(key);
    if (oldIt != oldManifoldMap.end()) {
      it->second.inheritImpulses(oldIt->second, physics::PhysicsConstants::ContactPointMatchRadiusSq);
    }
  }
}

namespace {
inline collision::AABB transformAabb(const collision::AABB& local, const Pose& pose) {
    Vector3 minB = local.minBounds;
    Vector3 maxB = local.maxBounds;
    Vector3 corners[8] = {
        Vector3(minB.x, minB.y, minB.z), Vector3(maxB.x, minB.y, minB.z),
        Vector3(minB.x, maxB.y, minB.z), Vector3(maxB.x, maxB.y, minB.z),
        Vector3(minB.x, minB.y, maxB.z), Vector3(maxB.x, minB.y, maxB.z),
        Vector3(minB.x, maxB.y, maxB.z), Vector3(maxB.x, maxB.y, maxB.z)
    };
    Vector3 outMin(1e10f, 1e10f, 1e10f);
    Vector3 outMax(-1e10f, -1e10f, -1e10f);
    for (const auto& c : corners) {
        Vector3 wc = pose.position + (pose.orientation * c);
        outMin.x = std::min(outMin.x, wc.x);
        outMin.y = std::min(outMin.y, wc.y);
        outMin.z = std::min(outMin.z, wc.z);
        outMax.x = std::max(outMax.x, wc.x);
        outMax.y = std::max(outMax.y, wc.y);
        outMax.z = std::max(outMax.z, wc.z);
    }
    return collision::AABB(outMin, outMax);
}
} // namespace

void CollisionSystem::processCompoundCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifoldMap) {
  
  bool isCompA = std::holds_alternative<collision::CompoundShape>(collA.shape.getVariant());
  bool isCompB = std::holds_alternative<collision::CompoundShape>(collB.shape.getVariant());

  struct SubShape {
    const collision::CollisionShape* shape;
    Pose worldPose;
  };

  std::vector<SubShape> subShapesA;
  if (isCompA) {
    const auto &compA = std::get<collision::CompoundShape>(collA.shape.getVariant());
    subShapesA.reserve(compA.children.size());
    for (const auto &child : compA.children) {
      Pose wp;
      wp.position = transA.position + (transA.orientation * child.localPose.position);
      wp.orientation = transA.orientation * child.localPose.orientation;
      subShapesA.push_back({&child.getShape(), wp});
    }
  } else {
    subShapesA.push_back({&collA.shape, transA});
  }

  std::vector<SubShape> subShapesB;
  if (isCompB) {
    const auto &compB = std::get<collision::CompoundShape>(collB.shape.getVariant());
    subShapesB.reserve(compB.children.size());
    for (const auto &child : compB.children) {
      Pose wp;
      wp.position = transB.position + (transB.orientation * child.localPose.position);
      wp.orientation = transB.orientation * child.localPose.orientation;
      subShapesB.push_back({&child.getShape(), wp});
    }
  } else {
    subShapesB.push_back({&collB.shape, transB});
  }

  if (subShapesA.empty() || subShapesB.empty())
    return;

  uint64_t key = makeEntityPairKey(entityA, entityB);
  bool isTriggerPair = collA.isTrigger || collB.isTrigger;

  std::vector<collision::ContactPoint> accumulatedContacts;
  Vector3 avgNormal(0, 0, 0);
  float maxPenetration = 0.0f;
  bool hasAnyCollision = false;

  for (const auto &subA : subShapesA) {
    collision::AABB aabbA = transformAabb(subA.shape->computeLocalAABB(), subA.worldPose);

    for (const auto &subB : subShapesB) {
      collision::AABB aabbB = transformAabb(subB.shape->computeLocalAABB(), subB.worldPose);

      if (!aabbA.testCollision(aabbB))
        continue;

      bool isSubBoxA = std::holds_alternative<collision::BoxShape>(subA.shape->getVariant());
      bool isSubBoxB = std::holds_alternative<collision::BoxShape>(subB.shape->getVariant());
      bool isSubSphereA = std::holds_alternative<collision::SphereShape>(subA.shape->getVariant());
      bool isSubSphereB = std::holds_alternative<collision::SphereShape>(subB.shape->getVariant());

      if (isSubBoxA && isSubBoxB) {
        const auto &bA = std::get<collision::BoxShape>(subA.shape->getVariant());
        const auto &bB = std::get<collision::BoxShape>(subB.shape->getVariant());
        collision::ContactManifold satManifold(entityA, entityB);

        if (collision::NarrowPhase::testBoxBox(bA, subA.worldPose, bB, subB.worldPose, satManifold)) {
          hasAnyCollision = true;
          if (satManifold.penetration > maxPenetration) {
            maxPenetration = satManifold.penetration;
            avgNormal = satManifold.normal;
          }
          for (uint32_t cIdx = 0; cIdx < satManifold.contactCount; ++cIdx) {
            const auto &cp = satManifold.contacts[cIdx];
            collision::ContactPoint transformedCp;
            transformedCp.worldPosition = cp.worldPosition;
            transformedCp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
            transformedCp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
            transformedCp.penetration = cp.penetration;
            accumulatedContacts.push_back(transformedCp);
          }
        }
      } else if (isSubSphereA && isSubSphereB) {
        const auto &spA = std::get<collision::SphereShape>(subA.shape->getVariant());
        const auto &spB = std::get<collision::SphereShape>(subB.shape->getVariant());
        collision::CollisionInfo info;
        if (collision::NarrowPhase::testSphereSphere(spA, subA.worldPose, spB, subB.worldPose, info)) {
          hasAnyCollision = true;
          if (info.penetration > maxPenetration) {
            maxPenetration = info.penetration;
            avgNormal = info.normal;
          }
          collision::ContactPoint cp;
          cp.worldPosition = info.contactPoint;
          cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
          cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
          cp.penetration = info.penetration;
          accumulatedContacts.push_back(cp);
        }
      } else if (isSubSphereA && isSubBoxB) {
        const auto &spA = std::get<collision::SphereShape>(subA.shape->getVariant());
        const auto &bB = std::get<collision::BoxShape>(subB.shape->getVariant());
        collision::CollisionInfo info;
        if (collision::NarrowPhase::testSphereBox(spA, subA.worldPose, bB, subB.worldPose, info)) {
          hasAnyCollision = true;
          if (info.penetration > maxPenetration) {
            maxPenetration = info.penetration;
            avgNormal = info.normal;
          }
          collision::ContactPoint cp;
          cp.worldPosition = info.contactPoint;
          cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
          cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
          cp.penetration = info.penetration;
          accumulatedContacts.push_back(cp);
        }
      } else if (isSubBoxA && isSubSphereB) {
        const auto &bA = std::get<collision::BoxShape>(subA.shape->getVariant());
        const auto &spB = std::get<collision::SphereShape>(subB.shape->getVariant());
        collision::CollisionInfo info;
        if (collision::NarrowPhase::testSphereBox(spB, subB.worldPose, bA, subA.worldPose, info)) {
          hasAnyCollision = true;
          Vector3 normalBtoA = -info.normal;
          if (info.penetration > maxPenetration) {
            maxPenetration = info.penetration;
            avgNormal = normalBtoA;
          }
          collision::ContactPoint cp;
          cp.worldPosition = info.contactPoint;
          cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
          cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
          cp.penetration = info.penetration;
          accumulatedContacts.push_back(cp);
        }
      } else {
        collision::Simplex simplex;
        if (collision::NarrowPhase::GJK(*subA.shape, subA.worldPose, *subB.shape, subB.worldPose, simplex)) {
          collision::CollisionInfo info = collision::NarrowPhase::EPA(simplex, *subA.shape, subA.worldPose, *subB.shape, subB.worldPose);
          if (info.hasCollision) {
            hasAnyCollision = true;
            Vector3 normalBtoA = -info.normal;
            if (info.penetration > maxPenetration) {
              maxPenetration = info.penetration;
              avgNormal = normalBtoA;
            }
            collision::ContactPoint cp;
            cp.worldPosition = info.contactPoint;
            cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
            cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
            cp.penetration = info.penetration;
            accumulatedContacts.push_back(cp);
          }
        }
      }
    }
  }

  if (hasAnyCollision && !accumulatedContacts.empty()) {
    bool awakeA = mRegistry.hasComponent<Core::BodyState>(entityA) && !mRegistry.getComponent<Core::BodyState>(entityA).isSleeping;
    bool awakeB = mRegistry.hasComponent<Core::BodyState>(entityB) && !mRegistry.getComponent<Core::BodyState>(entityB).isSleeping;
    if ((awakeA && !awakeB) || (awakeB && !awakeA) || maxPenetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
      wakeIfSleeping(entityA);
      wakeIfSleeping(entityB);
    }

    auto [it, inserted] = mManifoldMap.try_emplace(key, entityA, entityB);
    it->second.isTriggerPair = isTriggerPair;
    it->second.normal = avgNormal;
    it->second.penetration = maxPenetration;
    it->second.computeTangents();
    it->second.contactCount = 0;

    if (accumulatedContacts.size() > 4) {
      std::sort(accumulatedContacts.begin(), accumulatedContacts.end(), [](const auto &a, const auto &b) {
        return a.penetration > b.penetration;
      });
    }

    uint32_t limit = std::min(static_cast<uint32_t>(accumulatedContacts.size()), 4u);
    for (uint32_t c = 0; c < limit; ++c) {
      it->second.addContact(accumulatedContacts[c]);
    }

    auto oldIt = oldManifoldMap.find(key);
    if (oldIt != oldManifoldMap.end()) {
      it->second.inheritImpulses(oldIt->second, physics::PhysicsConstants::ContactPointMatchRadiusSq);
    }
  }
}

/**
 * @brief Dar faz çarpışma testlerini gerçekleştirir ve temas verilerini
 * hazırlar.
 * @details
 * 1. Olası Çiftler: Geniş fazdan gelen adayları işler.
 * 2. GJK/EPA: Nesnelerin kesişip kesişmediğini ve iç içe geçme detaylarını
 * bulur.
 * 3. Kalıcı Manifold (Persistent Manifold): Önceki kareden kalan temas
 * noktalarını yeni noktalarla karşılaştırarak geçerli olanları korur.
 * 4. Sıcak Başlatma (Warm Starting): Önceki karede uygulanan kuvvetleri
 * hatırlar. Bu sayede nesne yığınları (stacking) titreme yapmadan üst üste
 * durabilir.
 * 5. Uyandırma: Çarpışma yaşayan uyuyan (sleeping) nesneleri tekrar simülasyona
 * dahil eder.
 */
void CollisionSystem::step(float deltaTime) {
  const auto &pairs = mSpatialPartitioning.getPotentialCollisions();

  // Warm starting için önceki kareden kalan temas verilerini yedekle
  std::unordered_map<uint64_t, collision::ContactManifold> oldManifoldMap =
      std::move(mManifoldMap);
  mManifoldMap.clear();
  mManifoldMap.reserve(pairs.size());

  for (const auto &[entityA, entityB] : pairs) {
    if (!mRegistry.hasComponent<Pose>(entityA) ||
        !mRegistry.hasComponent<Pose>(entityB))
      continue;
    if (!mRegistry.hasComponent<ecs::ColliderData>(entityA) ||
        !mRegistry.hasComponent<ecs::ColliderData>(entityB))
      continue;

    // Statik-Statik çiftlerini doğrudan ele
    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(entityA))
      typeA = mRegistry.getComponent<Core::BodyState>(entityA).type;
    if (mRegistry.hasComponent<Core::BodyState>(entityB))
      typeB = mRegistry.getComponent<Core::BodyState>(entityB).type;
    if (typeA == Core::BodyType::Static && typeB == Core::BodyType::Static)
      continue;

    const auto &transA = mRegistry.getComponent<Pose>(entityA);
    const auto &transB = mRegistry.getComponent<Pose>(entityB);
    const auto &collA = mRegistry.getComponent<ecs::ColliderData>(entityA);
    const auto &collB = mRegistry.getComponent<ecs::ColliderData>(entityB);

    // Çarpışma Filtreleme (Bitmask): İki nesne birbirinin maskesinde yoksa çarpışmayı yoksay!
    if ((collA.layer & collB.mask) == 0 || (collB.layer & collA.mask) == 0)
      continue;

    bool isMeshA = std::holds_alternative<collision::StaticMeshShape>(
        collA.shape.getVariant());
    bool isMeshB = std::holds_alternative<collision::StaticMeshShape>(
        collB.shape.getVariant());
    bool isHfA = std::holds_alternative<collision::HeightfieldShape>(
        collA.shape.getVariant());
    bool isHfB = std::holds_alternative<collision::HeightfieldShape>(
        collB.shape.getVariant());

    if ((isMeshA || isHfA) && (isMeshB || isHfB))
      continue;

    bool isCompA = std::holds_alternative<collision::CompoundShape>(collA.shape.getVariant());
    bool isCompB = std::holds_alternative<collision::CompoundShape>(collB.shape.getVariant());

    if (isCompA || isCompB) {
      processCompoundCollision(entityA, entityB, transA, transB, collA, collB, oldManifoldMap);
    } else if (isMeshA || isMeshB) {
      processMeshCollision(entityA, entityB, transA, transB, collA, collB, isMeshB, oldManifoldMap);
    } else if (isHfA || isHfB) {
      processHeightfieldCollision(entityA, entityB, transA, transB, collA, collB, isHfB, oldManifoldMap);
    } else {
      bool isBoxA = std::holds_alternative<collision::BoxShape>(collA.shape.getVariant());
      bool isBoxB = std::holds_alternative<collision::BoxShape>(collB.shape.getVariant());

      if (isBoxA && isBoxB) {
        processBoxBoxCollision(entityA, entityB, transA, transB, collA, collB, oldManifoldMap);
      } else {
        processConvexCollision(entityA, entityB, transA, transB, collA, collB, oldManifoldMap);
      }
    }

    // Spekülatif Kontak Kontrolü:
    // Eğer ayrık dar faz çarpışma üretmediyse, hızlı yaklaşan cisimler için öngörülü kontak üret
    uint64_t key = makeEntityPairKey(entityA, entityB);
    if (!mManifoldMap.contains(key)) {
      tryGenerateSpeculativeContact(entityA, entityB, transA, transB, collA, collB, deltaTime);
    }
  }

  // Çarpışma ve Tetikleyici Olaylarını Dinleyiciye Bildir (Callbacks)
  if (mListener) {
    for (const auto &[key, manifold] : mManifoldMap) {
      if (!oldManifoldMap.contains(key)) {
        if (manifold.isTriggerPair) {
          mListener->onTriggerEnter(manifold.entityA, manifold.entityB);
        } else {
          mListener->onCollisionEnter(manifold.entityA, manifold.entityB, manifold);
        }
      }
    }
    for (const auto &[key, oldManifold] : oldManifoldMap) {
      if (!mManifoldMap.contains(key)) {
        if (oldManifold.isTriggerPair) {
          mListener->onTriggerExit(oldManifold.entityA, oldManifold.entityB);
        } else {
          mListener->onCollisionExit(oldManifold.entityA, oldManifold.entityB);
        }
      }
    }
  }
}

/**
 * @brief Sürekli Çarpışma Algılama (Continuous Collision Detection - CCD).
 * @details Çok hızlı hareket eden nesnelerin ince engellerin içinden geçmesini
 *          (tünelleme) önlemek için "Muhafazakar İlerleme" yöntemini kullanır.
 *
 * Süreç:
 * 1. Hız Kontrolü: Yalnızca tünelleme riski taşıyan hızlı nesnelerde çalışır.
 * 2. Yol Taraması (Sweep): Nesnenin gideceği yol boyunca olası engelleri bulur.
 * 3. Etki Anı (TOI): Engel varsa, nesneyi engele çarpacağı tam ana kadar
 * ilerletir.
 * 4. Tepki: Nesne çarpışma yüzeyinde durdurulur ve hızı yüzey boyunca kayacak
 * şekilde kırpılır.
 */
void CollisionSystem::applyCCD(ecs::Entity entity, float deltaTime) {
  if (!mRegistry.hasComponent<Core::BodyState>(entity) ||
      !mRegistry.hasComponent<ecs::ColliderData>(entity) ||
      !mRegistry.hasComponent<Core::Motion>(entity))
    return;

  auto &state = mRegistry.getComponent<Core::BodyState>(entity);
  if (!state.useCCD)
    return;

  auto &transform = mRegistry.getComponent<Pose>(entity);
  auto &motion = mRegistry.getComponent<Core::Motion>(entity);

  auto &colliderA = mRegistry.getComponent<ecs::ColliderData>(entity);
  collision::AABB localAABB = colliderA.shape.computeLocalAABB();
  Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
  Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

  // Nesnenin rotasyonuna göre dünya uzayındaki boyutlarını (OBB -> AABB) tam
  // hesapla
  Vector3 xAxis = transform.orientation * Vector3(1.0f, 0.0f, 0.0f);
  Vector3 yAxis = transform.orientation * Vector3(0.0f, 1.0f, 0.0f);
  Vector3 zAxis = transform.orientation * Vector3(0.0f, 0.0f, 1.0f);

  Vector3 rotatedExtents(
      std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y +
          std::abs(zAxis.x) * extents.z,
      std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y +
          std::abs(zAxis.y) * extents.z,
      std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y +
          std::abs(zAxis.z) * extents.z);

  Vector3 worldCenter = transform.position + (transform.orientation * center);
  collision::AABB currentAABB(worldCenter - rotatedExtents,
                              worldCenter + rotatedExtents);
  float radiusA = rotatedExtents.length();

  // Hız ve hareket mesafesi kontrolü
  float speed = motion.linearVelocity.length();
  float motionDist = speed * deltaTime;
  if (speed < 1e-4f)
    return;

  // Nesne yarıçapının %25'i veya ince engelleri yakalamak için 0.05m
  float sizeBasedMin = radiusA * 0.25f;
  float minMotionForCcd = (state.ccdMotionThreshold > 0.0f)
                              ? state.ccdMotionThreshold
                              : std::min(sizeBasedMin, 0.05f);

  if (motionDist < minMotionForCcd)
    return;

  float timeOfImpact = deltaTime;
  bool hasCollision = false;
  Vector3 collisionNormal(0.0f, 0.0f, 0.0f);
  ecs::Entity hitEntity{0};

  // Hareket yolunu kapsayan genişletilmiş sınır kutusu (Rotasyonlu Swept AABB)
  Vector3 deltaPos = motion.linearVelocity * deltaTime;
  collision::AABB sweepAABB = currentAABB;
  if (deltaPos.x > 0.0f)
    sweepAABB.maxBounds.x += deltaPos.x;
  else
    sweepAABB.minBounds.x += deltaPos.x;
  if (deltaPos.y > 0.0f)
    sweepAABB.maxBounds.y += deltaPos.y;
  else
    sweepAABB.minBounds.y += deltaPos.y;
  if (deltaPos.z > 0.0f)
    sweepAABB.maxBounds.z += deltaPos.z;
  else
    sweepAABB.minBounds.z += deltaPos.z;

  mSpatialPartitioning.getTree().query(sweepAABB, [&](ecs::Entity obstacle) {
    if (obstacle == entity)
      return;
    if (!mRegistry.hasComponent<Pose>(obstacle) ||
        !mRegistry.hasComponent<ecs::ColliderData>(obstacle))
      return;

    const auto &obsState = mRegistry.getComponent<Core::BodyState>(obstacle);
    if (state.type == Core::BodyType::Kinematic &&
        obsState.type == Core::BodyType::Kinematic)
      return;

    const auto &obsTrans = mRegistry.getComponent<Pose>(obstacle);
    const auto &obsColl = mRegistry.getComponent<ecs::ColliderData>(obstacle);

    if (colliderA.isTrigger || obsColl.isTrigger)
      return;
    if ((colliderA.layer & obsColl.mask) == 0 ||
        (obsColl.layer & colliderA.mask) == 0)
      return;

    bool isMesh = std::holds_alternative<collision::StaticMeshShape>(
        obsColl.shape.getVariant());
    bool isHf = std::holds_alternative<collision::HeightfieldShape>(
        obsColl.shape.getVariant());

    auto checkShapeTOI = [&](const collision::CollisionShape &shapeB) {
      float currentTime = 0.0f;
      Pose currentTransform = transform;
      Vector3 pA, pB;

      for (int iter = 0; iter < 16; ++iter) {
        currentTransform.position =
            transform.position + (motion.linearVelocity * currentTime);

        Pose obsCurrentTrans = obsTrans;
        Vector3 obsVel(0.0f, 0.0f, 0.0f);
        if (mRegistry.hasComponent<Core::Motion>(obstacle)) {
          obsVel =
              mRegistry.getComponent<Core::Motion>(obstacle).linearVelocity;
          obsCurrentTrans.position += obsVel * currentTime;
        }

        float dist = collision::NarrowPhase::GJK_distance(
            colliderA.shape, currentTransform, shapeB, obsCurrentTrans, pA, pB);

        Vector3 relVel = motion.linearVelocity - obsVel;
        float relSpeed = relVel.length();
        if (relSpeed < 0.001f)
          break;

        constexpr float targetSlop = 0.5f * physics::PhysicsConstants::LinearSlop;
        if (dist <= targetSlop) {
          float touchTime = currentTime + (dist / relSpeed) + (0.017f / relSpeed);
          touchTime = std::min(deltaTime, touchTime);

          if (touchTime < timeOfImpact) {
            timeOfImpact = touchTime;
            hasCollision = true;
            hitEntity = obstacle;

            Vector3 diff = pA - pB;
            float diffLen = diff.length();
            if (diffLen > 1e-4f) {
              collisionNormal = diff / diffLen;
              if (relVel.dot(collisionNormal) > 0.0f) {
                collisionNormal = collisionNormal * -1.0f;
              }
            } else {
              collisionNormal = relVel.getNormalized() * -1.0f;
            }
          }
          break;
        }

        Vector3 diff = pA - pB;
        float diffLen = diff.length();
        Vector3 normal = (diffLen > 1e-6f) ? (diff / diffLen) : (relVel * (-1.0f / relSpeed));

        // Yaklasma hizi: ayrilma eksenindeki hiz bileseni
        float approachSpeed = -relVel.dot(normal);
        if (approachSpeed <= 1e-4f) {
          // Cisimler birbirinden uzaklasiyor (ayriliyor), carpisma olamaz
          break;
        }

        currentTime += dist / relSpeed;
        if (currentTime >= timeOfImpact)
          break;
      }
    };

    if (isMesh) {
      const auto &meshShape =
          std::get<collision::StaticMeshShape>(obsColl.shape.getVariant());
      if (meshShape.mesh) {
        Vector3 localStart = obsTrans.orientation.getConjugate() *
                             (transform.position - obsTrans.position);
        Vector3 localEnd = obsTrans.orientation.getConjugate() *
                           (transform.position + deltaPos - obsTrans.position);
        Vector3 minP(std::min(localStart.x, localEnd.x) - radiusA,
                     std::min(localStart.y, localEnd.y) - radiusA,
                     std::min(localStart.z, localEnd.z) - radiusA);
        Vector3 maxP(std::max(localStart.x, localEnd.x) + radiusA,
                     std::max(localStart.y, localEnd.y) + radiusA,
                     std::max(localStart.z, localEnd.z) + radiusA);
        collision::AABB meshQueryAABB(minP, maxP);

        meshShape.mesh->queryTriangles(
            meshQueryAABB, [&](const collision::TriangleShape &tri) {
              collision::CollisionShape triShapeWrapper(tri);
              checkShapeTOI(triShapeWrapper);
            });
      }
    } else if (isHf) {
      const auto &hfShape =
          std::get<collision::HeightfieldShape>(obsColl.shape.getVariant());
      Vector3 localStart = obsTrans.orientation.getConjugate() *
                           (transform.position - obsTrans.position);
      Vector3 localEnd = obsTrans.orientation.getConjugate() *
                         (transform.position + deltaPos - obsTrans.position);
      Vector3 minP(std::min(localStart.x, localEnd.x) - radiusA,
                   std::min(localStart.y, localEnd.y) - radiusA,
                   std::min(localStart.z, localEnd.z) - radiusA);
      Vector3 maxP(std::max(localStart.x, localEnd.x) + radiusA,
                   std::max(localStart.y, localEnd.y) + radiusA,
                   std::max(localStart.z, localEnd.z) + radiusA);
      collision::AABB hfQueryAABB(minP, maxP);

      hfShape.queryTriangles(
          hfQueryAABB, [&](const collision::TriangleShape &tri) {
            collision::CollisionShape triShapeWrapper(tri);
            checkShapeTOI(triShapeWrapper);
          });
    } else {
      checkShapeTOI(obsColl.shape);
    }
  });

  if (hasCollision && timeOfImpact < deltaTime) {
    float safeTime = std::max(0.0f, timeOfImpact);
    float remainingTime = deltaTime - safeTime;

    // Erken yercekimi entegrasyonunu carpisma anina geri al
    motion.linearVelocity -= mAccelerationField * remainingTime;

    // 1. Pozisyonu temas anina otele (yuzeye temas konumu)
    transform.position += motion.linearVelocity * safeTime;

    state.ccdIntegrated = true;
  }
}

bool CollisionSystem::tryGenerateSpeculativeContact(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    float deltaTime) {
  if (collA.isTrigger || collB.isTrigger)
    return false;

  bool specA = mRegistry.hasComponent<Core::BodyState>(entityA) &&
               mRegistry.getComponent<Core::BodyState>(entityA).useSpeculativeContacts;
  bool specB = mRegistry.hasComponent<Core::BodyState>(entityB) &&
               mRegistry.getComponent<Core::BodyState>(entityB).useSpeculativeContacts;
  if (!specA && !specB && !mEnableSpeculativeContacts)
    return false;

  Core::BodyType typeA = Core::BodyType::Dynamic;
  Core::BodyType typeB = Core::BodyType::Dynamic;
  if (mRegistry.hasComponent<Core::BodyState>(entityA))
    typeA = mRegistry.getComponent<Core::BodyState>(entityA).type;
  if (mRegistry.hasComponent<Core::BodyState>(entityB))
    typeB = mRegistry.getComponent<Core::BodyState>(entityB).type;
  if (typeA != Core::BodyType::Dynamic && typeB != Core::BodyType::Dynamic)
    return false;

  Vector3 velA(0.0f, 0.0f, 0.0f);
  Vector3 velB(0.0f, 0.0f, 0.0f);
  if (mRegistry.hasComponent<Core::Motion>(entityA))
    velA = mRegistry.getComponent<Core::Motion>(entityA).linearVelocity;
  if (mRegistry.hasComponent<Core::Motion>(entityB))
    velB = mRegistry.getComponent<Core::Motion>(entityB).linearVelocity;

  Vector3 relVel = velA - velB;
  if (relVel.lengthSquare() < 1e-4f)
    return false;

  float closestDist = 1e9f;
  Vector3 bestPA(0.0f, 0.0f, 0.0f);
  Vector3 bestPB(0.0f, 0.0f, 0.0f);
  bool found = false;

  bool isMeshA = std::holds_alternative<collision::StaticMeshShape>(collA.shape.getVariant());
  bool isMeshB = std::holds_alternative<collision::StaticMeshShape>(collB.shape.getVariant());

  if (!isMeshA && !isMeshB) {
    Vector3 pA, pB;
    float dist = collision::NarrowPhase::GJK_distance(collA.shape, transA, collB.shape, transB, pA, pB);
    closestDist = dist;
    bestPA = pA;
    bestPB = pB;
    found = true;
  } else if (isMeshB) {
    const auto &meshShape = std::get<collision::StaticMeshShape>(collB.shape.getVariant());
    if (meshShape.mesh) {
      collision::AABB localAABB = collA.shape.computeLocalAABB();
      float radiusA = (localAABB.maxBounds - localAABB.minBounds).length() * 0.5f;
      Vector3 deltaPos = velA * deltaTime;
      Vector3 localStart = transB.orientation.getConjugate() * (transA.position - transB.position);
      Vector3 localEnd = transB.orientation.getConjugate() * (transA.position + deltaPos - transB.position);
      Vector3 minP(std::min(localStart.x, localEnd.x) - radiusA - 1.0f,
                   std::min(localStart.y, localEnd.y) - radiusA - 1.0f,
                   std::min(localStart.z, localEnd.z) - radiusA - 1.0f);
      Vector3 maxP(std::max(localStart.x, localEnd.x) + radiusA + 1.0f,
                   std::max(localStart.y, localEnd.y) + radiusA + 1.0f,
                   std::max(localStart.z, localEnd.z) + radiusA + 1.0f);
      meshShape.mesh->queryTriangles(collision::AABB(minP, maxP), [&](const collision::TriangleShape &tri) {
        collision::CollisionShape triShape(tri);
        Vector3 pA, pB;
        float dist = collision::NarrowPhase::GJK_distance(collA.shape, transA, triShape, transB, pA, pB);
        if (dist < closestDist) {
          closestDist = dist;
          bestPA = pA;
          bestPB = pB;
          found = true;
        }
      });
    }
  } else if (isMeshA) {
    const auto &meshShape = std::get<collision::StaticMeshShape>(collA.shape.getVariant());
    if (meshShape.mesh) {
      collision::AABB localAABB = collB.shape.computeLocalAABB();
      float radiusB = (localAABB.maxBounds - localAABB.minBounds).length() * 0.5f;
      Vector3 deltaPos = velB * deltaTime;
      Vector3 localStart = transA.orientation.getConjugate() * (transB.position - transA.position);
      Vector3 localEnd = transA.orientation.getConjugate() * (transB.position + deltaPos - transA.position);
      Vector3 minP(std::min(localStart.x, localEnd.x) - radiusB - 1.0f,
                   std::min(localStart.y, localEnd.y) - radiusB - 1.0f,
                   std::min(localStart.z, localEnd.z) - radiusB - 1.0f);
      Vector3 maxP(std::max(localStart.x, localEnd.x) + radiusB + 1.0f,
                   std::max(localStart.y, localEnd.y) + radiusB + 1.0f,
                   std::max(localStart.z, localEnd.z) + radiusB + 1.0f);
      meshShape.mesh->queryTriangles(collision::AABB(minP, maxP), [&](const collision::TriangleShape &tri) {
        collision::CollisionShape triShape(tri);
        Vector3 pA, pB;
        float dist = collision::NarrowPhase::GJK_distance(triShape, transA, collB.shape, transB, pA, pB);
        if (dist < closestDist) {
          closestDist = dist;
          bestPA = pA;
          bestPB = pB;
          found = true;
        }
      });
    }
  }

  if (!found || closestDist <= 1e-4f || closestDist > mMaxSpeculativeDistance)
    return false;

  Vector3 normal = bestPA - bestPB;
  float normLen = normal.length();
  if (normLen > 1e-6f) {
    normal = normal * (1.0f / normLen);
  } else {
    normal = (transA.position - transB.position).getNormalized();
    if (normal.lengthSquare() < 1e-4f)
      normal = Vector3(0.0f, 1.0f, 0.0f);
  }

  // Normal sözleşmesi: B'den A'ya bakar.
  // Yaklaşma hızı: -relVel.dot(normal). Yaklaşırken relVel.dot(normal) < 0 olmalıdır.
  float normalVel = relVel.dot(normal);
  if (normalVel >= -1e-4f)
    return false;

  float approachSpeed = -normalVel;
  float approachDist = approachSpeed * deltaTime;
  if (approachDist < closestDist - 1e-4f)
    return false;

  // Spekülatif temas noktası üret (negatif penetrasyon)
  uint64_t key = makeEntityPairKey(entityA, entityB);
  collision::ContactManifold specManifold(entityA, entityB);
  specManifold.normal = normal;
  specManifold.penetration = -closestDist;
  specManifold.isTriggerPair = false;
  specManifold.computeTangents();

  collision::ContactPoint cp;
  cp.worldPosition = (bestPA + bestPB) * 0.5f;
  cp.localPointA = transA.orientation.getConjugate() * (bestPA - transA.position);
  cp.localPointB = transB.orientation.getConjugate() * (bestPB - transB.position);
  cp.penetration = -closestDist;
  cp.normalImpulse = 0.0f;
  cp.tangentImpulse1 = 0.0f;
  cp.tangentImpulse2 = 0.0f;
  cp.tangentImpulse = 0.0f;
  cp.splitImpulse = 0.0f;
  cp.velocityBias = (deltaTime > 1e-6f) ? (-closestDist / deltaTime) : 0.0f;

  specManifold.addContact(cp);
  wakeIfSleeping(entityA);
  wakeIfSleeping(entityB);
  mManifoldMap.insert_or_assign(key, specManifold);
  return true;
}

} // namespace Baryon::systems
