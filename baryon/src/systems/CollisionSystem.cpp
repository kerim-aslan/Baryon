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
#include <algorithm>

namespace Baryon::systems {

CollisionSystem::CollisionSystem(Core::Registry &registry,
                                 SpatialPartitioning &sp)
    : mRegistry(registry), mSpatialPartitioning(sp) {}

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
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifolds) {
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
  float queryExtent = std::max(physics::PhysicsConstants::MinMeshQueryExtent, convRadius + 1.0f);
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
            if (info.penetration >
                physics::PhysicsConstants::LinearSlop * 2.0f) {
              wakeIfSleeping(entConv);
              wakeIfSleeping(entMesh);
            }

            uint64_t key = collision::ManifoldCache::makeKey(entConv, entMesh);

            auto [it, inserted] =
                mCache.getCurrent().try_emplace(key, entConv, entMesh);
            it->second.isTriggerPair = collConv.isTrigger || collMesh.isTrigger;
            // Manifold her zaman (entConv, entMesh) sırasıyla oluşturulur.
            // EPA A->B normali döner; çözücü B->A beklediği için ters çevrilir.
            Vector3 currentNormal = info.normal * -1.0f;

            if (!inserted && it->second.contactCount > 0 &&
                it->second.normal.dot(currentNormal) < physics::PhysicsConstants::NormalSimilarityThreshold) {
              it->second.contactCount = 0;
            }

            if (info.penetration > it->second.penetration ||
                it->second.contactCount == 0) {
              it->second.normal = currentNormal;
              it->second.penetration = info.penetration;
              it->second.computeTangents();
            }

            collision::ContactPoint cp = collision::ContactPoint::create(
                info.contactPoint, info.penetration, transConv, transMesh);

            it->second.addContact(cp);

            auto oldIt = oldManifolds.find(key);
            if (oldIt != oldManifolds.end()) {
              it->second.inheritImpulses(oldIt->second);
            }
          }
        }
      });
}

void CollisionSystem::processBoxBoxCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifolds) {
  bool isTriggerPair = collA.isTrigger || collB.isTrigger;
  uint64_t key = collision::ManifoldCache::makeKey(entityA, entityB);

  collision::ContactManifold satManifold(entityA, entityB);
  const auto &bA =
      std::get<collision::BoxShape>(collA.shape.getVariant());
  const auto &bB =
      std::get<collision::BoxShape>(collB.shape.getVariant());
  if (collision::NarrowPhase::testBoxBox(bA, transA, bB, transB,
                                         satManifold)) {
    if (satManifold.penetration >
        physics::PhysicsConstants::LinearSlop * 2.0f) {
      wakeIfSleeping(entityA);
      wakeIfSleeping(entityB);
    }

    // Warm-starting itmelerini eşleştir
    auto oldIt = oldManifolds.find(key);
    if (oldIt != oldManifolds.end()) {
      satManifold.inheritImpulses(oldIt->second);
    }

    satManifold.isTriggerPair = isTriggerPair;
    satManifold.computeTangents();
    mCache.getCurrent().insert_or_assign(key, satManifold);
  }
}

void CollisionSystem::processGeneralCollision(
    ecs::Entity entityA, ecs::Entity entityB,
    const Pose &transA, const Pose &transB,
    const ecs::ColliderData &collA, const ecs::ColliderData &collB,
    const std::unordered_map<uint64_t, collision::ContactManifold> &oldManifolds) {
  bool isBoxA =
      std::holds_alternative<collision::BoxShape>(collA.shape.getVariant());
  bool isBoxB =
      std::holds_alternative<collision::BoxShape>(collB.shape.getVariant());
  bool isSphereA = std::holds_alternative<collision::SphereShape>(
      collA.shape.getVariant());
  bool isSphereB = std::holds_alternative<collision::SphereShape>(
      collB.shape.getVariant());

  bool isTriggerPair = collA.isTrigger || collB.isTrigger;
  uint64_t key = collision::ManifoldCache::makeKey(entityA, entityB);

  collision::CollisionInfo info;
  bool hasCollided = false;

  if (isSphereA && isSphereB) {
    const auto &spA =
        std::get<collision::SphereShape>(collA.shape.getVariant());
    const auto &spB =
        std::get<collision::SphereShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereSphere(
        spA, transA, spB, transB, info);
  } else if (isSphereA && isBoxB) {
    const auto &spA =
        std::get<collision::SphereShape>(collA.shape.getVariant());
    const auto &bB =
        std::get<collision::BoxShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereBox(spA, transA, bB,
                                                        transB, info);
  } else if (isBoxA && isSphereB) {
    const auto &bA =
        std::get<collision::BoxShape>(collA.shape.getVariant());
    const auto &spB =
        std::get<collision::SphereShape>(collB.shape.getVariant());
    hasCollided = collision::NarrowPhase::testSphereBox(spB, transB, bA,
                                                        transA, info);
    if (hasCollided) {
      info.normal = -info.normal; // Normal must point B to A
    }
  } else {
    collision::Simplex simplex;
    if (collision::NarrowPhase::GJK(collA.shape, transA, collB.shape,
                                    transB, simplex)) {
      info = collision::NarrowPhase::EPA(simplex, collA.shape, transA,
                                         collB.shape, transB);
      hasCollided = info.hasCollision;
      if (hasCollided) {
        info.normal = -info.normal; // EPA: A->B, çözücü: B->A
      }
    }
  }

  if (hasCollided) {
    if (info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
      wakeIfSleeping(entityA);
      wakeIfSleeping(entityB);
    }

    auto [it, inserted] = mCache.getCurrent().try_emplace(key, entityA, entityB);
    it->second.isTriggerPair = isTriggerPair;
    it->second.normal = info.normal;
    it->second.penetration = info.penetration;
    it->second.computeTangents();

    collision::ContactPoint cp = collision::ContactPoint::create(
        info.contactPoint, info.penetration, transA, transB);

    it->second.contactCount = 0;
    it->second.addContact(cp);

    auto oldIt = oldManifolds.find(key);
    if (oldIt != oldManifolds.end()) {
      it->second.inheritImpulses(oldIt->second);
    }
  }
}

void CollisionSystem::step() {
  const auto &pairs = mSpatialPartitioning.getPotentialCollisions();

  mCache.beginFrame(pairs.size());
  const auto &oldManifoldMap = mCache.getPrevious();

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

    // B5: Çift-Dağıtım (Double Dispatch) - std::visit ile iki şekil varyantının türüne göre otomatik yönlendirme
    std::visit(
        [&](const auto &shapeA, const auto &shapeB) {
          using TypeA = std::decay_t<decltype(shapeA)>;
          using TypeB = std::decay_t<decltype(shapeB)>;

          constexpr bool isMeshA = std::is_same_v<TypeA, collision::StaticMeshShape>;
          constexpr bool isMeshB = std::is_same_v<TypeB, collision::StaticMeshShape>;

          if constexpr (isMeshA && isMeshB) {
            // İki statik mesh birbirine çarpamaz
            return;
          } else if constexpr (isMeshA || isMeshB) {
            processMeshCollision(entityA, entityB, transA, transB, collA, collB, isMeshB, oldManifoldMap);
          } else if constexpr (std::is_same_v<TypeA, collision::BoxShape> && std::is_same_v<TypeB, collision::BoxShape>) {
            processBoxBoxCollision(entityA, entityB, transA, transB, collA, collB, oldManifoldMap);
          } else {
            processGeneralCollision(entityA, entityB, transA, transB, collA, collB, oldManifoldMap);
          }
        },
        collA.shape.getVariant(), collB.shape.getVariant());
  }

  // Çarpışma ve Tetikleyici Olaylarını Dinleyiciye Bildir (Callbacks)
  collision::ContactEvents::dispatch(mListener, mCache.getCurrent(), mCache.getPrevious());
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
CollisionSystem::CCDHit CollisionSystem::findEarliestTOI(
    ecs::Entity entity,
    const Pose &transform,
    const Core::Motion &motion,
    const ecs::ColliderData &colliderA,
    float radiusA,
    float deltaTime) {
  CCDHit hit;
  hit.timeOfImpact = deltaTime;

  const auto &state = mRegistry.getComponent<Core::BodyState>(entity);
  collision::AABB localAABB = colliderA.shape.computeLocalAABB();
  Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
  Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

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
  collision::AABB currentAABB(worldCenter - rotatedExtents, worldCenter + rotatedExtents);

  Vector3 deltaPos = motion.linearVelocity * deltaTime;
  collision::AABB sweepAABB = currentAABB;
  if (deltaPos.x > 0.0f) sweepAABB.maxBounds.x += deltaPos.x;
  else sweepAABB.minBounds.x += deltaPos.x;
  if (deltaPos.y > 0.0f) sweepAABB.maxBounds.y += deltaPos.y;
  else sweepAABB.minBounds.y += deltaPos.y;
  if (deltaPos.z > 0.0f) sweepAABB.maxBounds.z += deltaPos.z;
  else sweepAABB.minBounds.z += deltaPos.z;

  mSpatialPartitioning.getTree().query(sweepAABB, [&](ecs::Entity obstacle) {
    if (obstacle == entity) return;
    if (!mRegistry.hasComponent<Pose>(obstacle) ||
        !mRegistry.hasComponent<ecs::ColliderData>(obstacle))
      return;

    const auto &obsState = mRegistry.getComponent<Core::BodyState>(obstacle);
    if (state.type == Core::BodyType::Kinematic &&
        obsState.type == Core::BodyType::Kinematic)
      return;

    const auto &obsTrans = mRegistry.getComponent<Pose>(obstacle);
    const auto &obsColl = mRegistry.getComponent<ecs::ColliderData>(obstacle);

    if (colliderA.isTrigger || obsColl.isTrigger) return;
    if ((colliderA.layer & obsColl.mask) == 0 || (obsColl.layer & colliderA.mask) == 0) return;

    bool isMesh = std::holds_alternative<collision::StaticMeshShape>(obsColl.shape.getVariant());

    auto checkShapeTOI = [&](const collision::CollisionShape &shapeB) {
      float currentTime = 0.0f;
      Pose currentTransform = transform;
      Vector3 pA, pB;

      for (int iter = 0; iter < 16; ++iter) {
        currentTransform.position = transform.position + (motion.linearVelocity * currentTime);

        Pose obsCurrentTrans = obsTrans;
        Vector3 obsVel(0.0f, 0.0f, 0.0f);
        if (mRegistry.hasComponent<Core::Motion>(obstacle)) {
          obsVel = mRegistry.getComponent<Core::Motion>(obstacle).linearVelocity;
          obsCurrentTrans.position += obsVel * currentTime;
        }

        float dist = collision::NarrowPhase::GJK_distance(
            colliderA.shape, currentTransform, shapeB, obsCurrentTrans, pA, pB);

        if (dist < 0.02f) {
          if (currentTime < hit.timeOfImpact) {
            hit.timeOfImpact = currentTime;
            hit.hasCollision = true;
            hit.hitEntity = obstacle;
            hit.collisionNormal = (pA - pB).getNormalized();
            if (motion.linearVelocity.dot(hit.collisionNormal) > 0.0f) {
              hit.collisionNormal = hit.collisionNormal * -1.0f;
            }
            if (hit.collisionNormal.lengthSquare() < 0.001f) {
              hit.collisionNormal = motion.linearVelocity.getNormalized() * -1.0f;
            }
          }
          break;
        }

        Vector3 relVel = motion.linearVelocity - obsVel;
        float maxApproachSpeed = relVel.length();
        if (maxApproachSpeed < 0.001f) break;

        currentTime += dist / maxApproachSpeed;
        if (currentTime >= hit.timeOfImpact) break;
      }
    };

    if (isMesh) {
      const auto &meshShape = std::get<collision::StaticMeshShape>(obsColl.shape.getVariant());
      if (meshShape.mesh) {
        Vector3 localStart = obsTrans.orientation.getConjugate() * (transform.position - obsTrans.position);
        Vector3 localEnd = obsTrans.orientation.getConjugate() * (transform.position + deltaPos - obsTrans.position);
        Vector3 minP(std::min(localStart.x, localEnd.x) - radiusA,
                     std::min(localStart.y, localEnd.y) - radiusA,
                     std::min(localStart.z, localEnd.z) - radiusA);
        Vector3 maxP(std::max(localStart.x, localEnd.x) + radiusA,
                     std::max(localStart.y, localEnd.y) + radiusA,
                     std::max(localStart.z, localEnd.z) + radiusA);
        collision::AABB meshQueryAABB(minP, maxP);

        meshShape.mesh->queryTriangles(meshQueryAABB, [&](const collision::TriangleShape &tri) {
          collision::CollisionShape triShapeWrapper(tri);
          checkShapeTOI(triShapeWrapper);
        });
      }
    } else {
      checkShapeTOI(obsColl.shape);
    }
  });

  return hit;
}

void CollisionSystem::resolveCCDImpact(
    ecs::Entity entity,
    Pose &transform,
    Core::Motion &motion,
    const CCDHit &hit,
    float speed,
    float deltaTime) {
  float timeMargin = (speed > 1e-4f) ? (0.005f / speed) : 0.0f;
  float safeTime = std::max(0.0f, hit.timeOfImpact - timeMargin);

  transform.position += motion.linearVelocity * safeTime;
  float remainingTime = std::max(0.0f, deltaTime - safeTime);

  float restitution = 0.0f;
  if (mRegistry.hasComponent<Core::Material>(entity) &&
      mRegistry.hasComponent<Core::Material>(hit.hitEntity)) {
    const auto &matA = mRegistry.getComponent<Core::Material>(entity);
    const auto &matB = mRegistry.getComponent<Core::Material>(hit.hitEntity);
    restitution = std::max(matA.restitution, matB.restitution);
  }

  bool isDynamicHit = false;
  if (mRegistry.hasComponent<Core::BodyState>(hit.hitEntity)) {
    isDynamicHit = (mRegistry.getComponent<Core::BodyState>(hit.hitEntity).type == Core::BodyType::Dynamic);
  }

  if (isDynamicHit && mRegistry.hasComponent<Core::Motion>(hit.hitEntity) &&
      mRegistry.hasComponent<Core::MassProps>(hit.hitEntity) &&
      mRegistry.hasComponent<Core::MassProps>(entity)) {
    auto &obsMotion = mRegistry.getComponent<Core::Motion>(hit.hitEntity);
    const auto &myMass = mRegistry.getComponent<Core::MassProps>(entity);
    const auto &obsMass = mRegistry.getComponent<Core::MassProps>(hit.hitEntity);

    Vector3 relVel = motion.linearVelocity - obsMotion.linearVelocity;
    float normalVel = relVel.dot(hit.collisionNormal);
    if (normalVel < 0.0f) {
      float invMassSum = myMass.inverseMass + obsMass.inverseMass;
      if (invMassSum > 1e-6f) {
        float impulseMag = -(1.0f + restitution) * normalVel / invMassSum;
        Vector3 impulse = hit.collisionNormal * impulseMag;
        motion.linearVelocity += impulse * myMass.inverseMass;
        obsMotion.linearVelocity -= impulse * obsMass.inverseMass;
      }
    }
  } else {
    float normalSpeed = motion.linearVelocity.dot(hit.collisionNormal);
    if (normalSpeed < 0.0f) {
      motion.linearVelocity -= hit.collisionNormal * (1.0f + restitution) * normalSpeed;
    }
    if (remainingTime > 0.0f && deltaTime > 0.0f) {
      motion.linearVelocity = motion.linearVelocity * (remainingTime / deltaTime);
    } else {
      motion.linearVelocity = Vector3(0.0f, 0.0f, 0.0f);
    }
  }
}

void CollisionSystem::applyCCD(ecs::Entity entity, float deltaTime) {
  if (!mRegistry.hasComponent<Core::BodyState>(entity) ||
      !mRegistry.hasComponent<ecs::ColliderData>(entity) ||
      !mRegistry.hasComponent<Core::Motion>(entity) ||
      !mRegistry.hasComponent<Pose>(entity))
    return;

  auto &state = mRegistry.getComponent<Core::BodyState>(entity);
  if (!state.useCCD)
    return;

  auto &transform = mRegistry.getComponent<Pose>(entity);
  auto &motion = mRegistry.getComponent<Core::Motion>(entity);
  const auto &colliderA = mRegistry.getComponent<ecs::ColliderData>(entity);

  collision::AABB localAABB = colliderA.shape.computeLocalAABB();
  Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

  Vector3 xAxis = transform.orientation * Vector3(1.0f, 0.0f, 0.0f);
  Vector3 yAxis = transform.orientation * Vector3(0.0f, 1.0f, 0.0f);
  Vector3 zAxis = transform.orientation * Vector3(0.0f, 0.0f, 1.0f);

  Vector3 rotatedExtents(
      std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y + std::abs(zAxis.x) * extents.z,
      std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y + std::abs(zAxis.y) * extents.z,
      std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y + std::abs(zAxis.z) * extents.z);

  float radiusA = rotatedExtents.length();
  float speed = motion.linearVelocity.length();
  float motionDist = speed * deltaTime;
  if (speed < 1e-4f)
    return;

  float sizeBasedMin = radiusA * 0.25f;
  float minMotionForCcd = (state.ccdMotionThreshold > 0.0f)
                              ? state.ccdMotionThreshold
                              : std::min(sizeBasedMin, 0.05f);

  if (motionDist < minMotionForCcd)
    return;

  CCDHit hit = findEarliestTOI(entity, transform, motion, colliderA, radiusA, deltaTime);

  if (hit.hasCollision && hit.timeOfImpact < deltaTime) {
    resolveCCDImpact(entity, transform, motion, hit, speed, deltaTime);
  }
}

} // namespace Baryon::systems
