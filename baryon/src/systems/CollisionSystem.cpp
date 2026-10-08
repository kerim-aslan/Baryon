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
void CollisionSystem::step() {
  const auto &pairs = mSpatialPartitioning.getPotentialCollisions();

  // Warm starting için önceki kareden kalan temas verilerini yedekle
  std::unordered_map<uint64_t, collision::ContactManifold> oldManifoldMap =
      std::move(mManifoldMap);
  mManifoldMap.clear();
  mManifoldMap.reserve(pairs.size());

  auto wakeIfSleeping = [&](ecs::Entity e) {
    if (!mRegistry.hasComponent<Core::BodyState>(e))
      return;
    auto &st = mRegistry.getComponent<Core::BodyState>(e);
    if (!st.isSleeping)
      return;
    st.isSleeping = false;
    st.sleepTimer = 0.0f;
  };

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

    if (isMeshA && isMeshB)
      continue;

    if (isMeshA || isMeshB) {
      // Karmaşık yüzey (Static Mesh) ile basit geometrilerin çarpışma testi
      auto entMesh = isMeshB ? entityB : entityA;
      auto entConv = isMeshB ? entityA : entityB;
      const auto &transMesh = isMeshB ? transB : transA;
      const auto &transConv = isMeshB ? transA : transB;
      const auto &collMesh = isMeshB ? collB : collA;
      const auto &collConv = isMeshB ? collA : collB;

      auto meshShape =
          std::get<collision::StaticMeshShape>(collMesh.shape.getVariant());
      if (!meshShape.mesh)
        continue;

      // Nesnenin boyutuna ve hızına göre dinamik sorgu AABB'si hesapla
      collision::AABB localConvAABB = collConv.shape.computeLocalAABB();
      float convRadius =
          (localConvAABB.maxBounds - localConvAABB.minBounds).length() * 0.5f;
      float queryExtent = std::max(3.0f, convRadius + 1.0f);
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

                // İki nesne için benzersiz anahtar üret
                uint32_t id1 = entConv.id;
                uint32_t id2 = entMesh.id;
                if (id1 > id2)
                  std::swap(id1, id2);
                uint64_t key = (static_cast<uint64_t>(id1) << 32) |
                               static_cast<uint64_t>(id2);

                auto [it, inserted] =
                    mManifoldMap.try_emplace(key, entConv, entMesh);
                it->second.isTriggerPair = collConv.isTrigger || collMesh.isTrigger;
                Vector3 currentNormal =
                    isMeshB ? info.normal : info.normal * -1.0f;

                if (!inserted && it->second.contactCount > 0 &&
                    it->second.normal.dot(currentNormal) < 0.95f) {
                  it->second.contactCount = 0;
                }

                if (info.penetration > it->second.penetration ||
                    it->second.contactCount == 0) {
                  it->second.normal = currentNormal;
                  it->second.penetration = info.penetration;
                  it->second.computeTangents();
                }

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

                auto oldIt = oldManifoldMap.find(key);
                if (oldIt != oldManifoldMap.end()) {
                  int bestMatch = -1;
                  float closestDistSq = 0.04f;
                  for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
                    float distSq =
                        (oldIt->second.contacts[j].localPointA - cp.localPointA)
                            .lengthSquare();
                    if (distSq < closestDistSq) {
                      closestDistSq = distSq;
                      bestMatch = j;
                    }
                  }
                  if (bestMatch != -1) {
                    cp.normalImpulse =
                        oldIt->second.contacts[bestMatch].normalImpulse;
                    cp.tangentImpulse1 =
                        oldIt->second.contacts[bestMatch].tangentImpulse1;
                    cp.tangentImpulse2 =
                        oldIt->second.contacts[bestMatch].tangentImpulse2;
                  }
                }

                if (it->second.contactCount < 4)
                  it->second.addContact(cp);
                else
                  it->second.contacts[3] = cp;
              }
            }
          });
    } else {
      // İki konveks nesne arası kesin çarpışma testi
      bool isBoxA =
          std::holds_alternative<collision::BoxShape>(collA.shape.getVariant());
      bool isBoxB =
          std::holds_alternative<collision::BoxShape>(collB.shape.getVariant());
      bool isSphereA = std::holds_alternative<collision::SphereShape>(
          collA.shape.getVariant());
      bool isSphereB = std::holds_alternative<collision::SphereShape>(
          collB.shape.getVariant());

      const auto &collA = mRegistry.getComponent<ecs::ColliderData>(entityA);
      const auto &collB = mRegistry.getComponent<ecs::ColliderData>(entityB);

      bool isTriggerPair = collA.isTrigger || collB.isTrigger;
      uint32_t id1 = entityA.id;
      uint32_t id2 = entityB.id;
      if (id1 > id2)
        std::swap(id1, id2);
      uint64_t key =
          (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);

      if (isBoxA && isBoxB) {
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
          auto oldIt = oldManifoldMap.find(key);
          if (oldIt != oldManifoldMap.end()) {
            for (uint32_t cIdx = 0; cIdx < satManifold.contactCount; ++cIdx) {
              auto &newCp = satManifold.contacts[cIdx];
              float closestDistSq = 0.04f;
              int bestMatch = -1;
              for (uint32_t oIdx = 0; oIdx < oldIt->second.contactCount;
                   ++oIdx) {
                float distSq = (oldIt->second.contacts[oIdx].localPointA -
                                newCp.localPointA)
                                   .lengthSquare();
                if (distSq < closestDistSq) {
                  closestDistSq = distSq;
                  bestMatch = oIdx;
                }
              }
              if (bestMatch != -1) {
                newCp.normalImpulse =
                    oldIt->second.contacts[bestMatch].normalImpulse;
                newCp.tangentImpulse1 =
                    oldIt->second.contacts[bestMatch].tangentImpulse1;
                newCp.tangentImpulse2 =
                    oldIt->second.contacts[bestMatch].tangentImpulse2;
              }
            }
          }

          satManifold.isTriggerPair = isTriggerPair;
          satManifold.computeTangents();
          mManifoldMap.insert_or_assign(key, satManifold);
        }
      } else {
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
          }
        }

        if (hasCollided) {
          if (info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
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

          auto oldIt = oldManifoldMap.find(key);
          if (oldIt != oldManifoldMap.end()) {
            int bestMatch = -1;
            float closestDistSq = 0.04f;
            for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
              float distSq =
                  (oldIt->second.contacts[j].localPointA - cp.localPointA)
                      .lengthSquare();
              if (distSq < closestDistSq) {
                closestDistSq = distSq;
                bestMatch = j;
              }
            }
            if (bestMatch != -1) {
              cp.normalImpulse =
                  oldIt->second.contacts[bestMatch].normalImpulse;
              cp.tangentImpulse1 =
                  oldIt->second.contacts[bestMatch].tangentImpulse1;
              cp.tangentImpulse2 =
                  oldIt->second.contacts[bestMatch].tangentImpulse2;
            }
          }

          it->second.contactCount = 0;
          it->second.addContact(cp);
        }
      }
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

        if (dist < 0.02f) {
          if (currentTime < timeOfImpact) {
            timeOfImpact = currentTime;
            hasCollision = true;
            hitEntity = obstacle;
            collisionNormal = (pA - pB).getNormalized();
            if (motion.linearVelocity.dot(collisionNormal) > 0.0f) {
              collisionNormal = collisionNormal * -1.0f;
            }
            if (collisionNormal.lengthSquare() < 0.001f) {
              collisionNormal = motion.linearVelocity.getNormalized() * -1.0f;
            }
          }
          break;
        }

        Vector3 relVel = motion.linearVelocity - obsVel;
        float maxApproachSpeed = relVel.length();
        if (maxApproachSpeed < 0.001f)
          break;

        currentTime += dist / maxApproachSpeed;
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
    } else {
      checkShapeTOI(obsColl.shape);
    }
  });

  if (hasCollision && timeOfImpact < deltaTime) {
    float timeMargin = (speed > 1e-4f) ? (0.005f / speed) : 0.0f;
    float safeTime = std::max(0.0f, timeOfImpact - timeMargin);

    // 1. Pozisyonu tam çarpışma anına (güvenli sınıra) hemen ötele
    transform.position += motion.linearVelocity * safeTime;

    // 2. Seçenek A (Sliding & Stop): Normal yönündeki hızı sıfırla
    float normalSpeed = motion.linearVelocity.dot(collisionNormal);
    if (normalSpeed < 0.0f) {
      motion.linearVelocity -= collisionNormal * normalSpeed;
    }

    // 3. Adımın kalan süresi için hızı ölçekle
    float remainingTime = deltaTime - safeTime;
    if (remainingTime > 0.0f && deltaTime > 0.0f) {
      motion.linearVelocity =
          motion.linearVelocity * (remainingTime / deltaTime);
    } else {
      motion.linearVelocity = Vector3(0.0f, 0.0f, 0.0f);
    }
  }
}

} // namespace Baryon::systems
