/**
 * @file CollisionSystem.cpp
 * @brief Çarpışma algılama ve temas (manifold) yönetimi sistemi.
 * @details Geniş fazdan (Broad Phase) gelen aday çiftleri dar fazda (GJK/EPA) test eder, 
 *          temas noktalarını (Contact Manifold) üretir ve bu noktaları kareler boyunca 
 *          koruyarak (Persistence) simülasyonun kararlı kalmasını sağlar.
 */

#include "Baryon/systems/CollisionSystem.hpp"
#include "Baryon/collision/NarrowPhase.hpp" 
#include "Baryon/collision/PhysicsConstants.hpp"
#include <algorithm> 

namespace Baryon::systems {

CollisionSystem::CollisionSystem(Core::Registry& registry, SpatialPartitioning& sp)
    : mRegistry(registry), mSpatialPartitioning(sp) {}

/**
 * @brief Dar faz çarpışma testlerini gerçekleştirir ve temas verilerini hazırlar.
 * @details 
 * 1. Olası Çiftler: Geniş fazdan gelen adayları işler.
 * 2. GJK/EPA: Nesnelerin kesişip kesişmediğini ve iç içe geçme detaylarını bulur.
 * 3. Kalıcı Manifold (Persistent Manifold): Önceki kareden kalan temas noktalarını 
 *    yeni noktalarla karşılaştırarak geçerli olanları korur.
 * 4. Sıcak Başlatma (Warm Starting): Önceki karede uygulanan kuvvetleri hatırlar. 
 *    Bu sayede nesne yığınları (stacking) titreme yapmadan üst üste durabilir.
 * 5. Uyandırma: Çarpışma yaşayan uyuyan (sleeping) nesneleri tekrar simülasyona dahil eder.
 */
void CollisionSystem::step() {
    const auto& pairs = mSpatialPartitioning.getPotentialCollisions(); 

    // Warm starting için önceki kareden kalan temas verilerini yedekle
    std::unordered_map<uint64_t, collision::ContactManifold> oldManifoldMap = std::move(mManifoldMap);
    mManifoldMap.clear();
    mManifoldMap.reserve(pairs.size());

    auto wakeIfSleeping = [&](ecs::Entity e) {
        if (!mRegistry.hasComponent<Core::BodyState>(e)) return;
        auto& st = mRegistry.getComponent<Core::BodyState>(e);
        if (!st.isSleeping) return;
        st.isSleeping = false;
        st.sleepTimer = 0.0f;
    };

    for (const auto& [entityA, entityB] : pairs) {
        if (!mRegistry.hasComponent<Pose>(entityA) || !mRegistry.hasComponent<Pose>(entityB)) continue;
        if (!mRegistry.hasComponent<ecs::ColliderData>(entityA) || !mRegistry.hasComponent<ecs::ColliderData>(entityB)) continue;

        // Statik-Statik çiftlerini doğrudan ele
        Core::BodyType typeA = Core::BodyType::Dynamic;
        Core::BodyType typeB = Core::BodyType::Dynamic;
        if (mRegistry.hasComponent<Core::BodyState>(entityA)) typeA = mRegistry.getComponent<Core::BodyState>(entityA).type;
        if (mRegistry.hasComponent<Core::BodyState>(entityB)) typeB = mRegistry.getComponent<Core::BodyState>(entityB).type;
        if (typeA == Core::BodyType::Static && typeB == Core::BodyType::Static) continue;

        const auto& transA = mRegistry.getComponent<Pose>(entityA);
        const auto& transB = mRegistry.getComponent<Pose>(entityB);
        const auto& collA = mRegistry.getComponent<ecs::ColliderData>(entityA);
        const auto& collB = mRegistry.getComponent<ecs::ColliderData>(entityB);

        bool isMeshA = std::holds_alternative<collision::StaticMeshShape>(collA.shape.getVariant());
        bool isMeshB = std::holds_alternative<collision::StaticMeshShape>(collB.shape.getVariant());

        if (isMeshA && isMeshB) continue; 

        if (isMeshA || isMeshB) {
            // Karmaşık yüzey (Static Mesh) ile basit geometrilerin çarpışma testi
            auto entMesh = isMeshB ? entityB : entityA;
            auto entConv = isMeshB ? entityA : entityB;
            const auto& transMesh = isMeshB ? transB : transA;
            const auto& transConv = isMeshB ? transA : transB;
            const auto& collMesh = isMeshB ? collB : collA;
            const auto& collConv = isMeshB ? collA : collB;

            auto meshShape = std::get<collision::StaticMeshShape>(collMesh.shape.getVariant());
            if (!meshShape.mesh) continue;

            // Sadece nesnenin yakınındaki üçgenleri sorgula (Optimizasyon)
            Vector3 localPos = transMesh.orientation.getConjugate() * (transConv.position - transMesh.position);
            collision::AABB queryAABB(localPos - Vector3(3,3,3), localPos + Vector3(3,3,3));

            meshShape.mesh->queryTriangles(queryAABB, [&](const collision::TriangleShape& tri) {
                collision::CollisionShape triShapeWrapper(tri);
                collision::Simplex simplex;
                
                if (collision::NarrowPhase::GJK(collConv.shape, transConv, triShapeWrapper, transMesh, simplex)) {
                    collision::CollisionInfo info = collision::NarrowPhase::EPA(simplex, collConv.shape, transConv, triShapeWrapper, transMesh);
                    
                    if (info.hasCollision) {
                        if (info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
                            wakeIfSleeping(entConv);
                            wakeIfSleeping(entMesh);
                        }
                        
                        // İki nesne için benzersiz anahtar üret
                        uint32_t id1 = entConv.id; uint32_t id2 = entMesh.id;
                        if (id1 > id2) std::swap(id1, id2);
                        uint64_t key = (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);

                        auto [it, inserted] = mManifoldMap.try_emplace(key, entConv, entMesh);
                        Vector3 currentNormal = isMeshB ? info.normal : info.normal * -1.0f;
                        
                        if (!inserted && it->second.contactCount > 0 && it->second.normal.dot(currentNormal) < 0.95f) {
                            it->second.contactCount = 0;
                        }

                        if (info.penetration > it->second.penetration || it->second.contactCount == 0) {
                            it->second.normal = currentNormal;
                            it->second.penetration = info.penetration;
                            it->second.computeTangents();
                        }
                        
                        collision::ContactPoint cp;
                        cp.worldPosition = info.contactPoint;
                        cp.localPointA = transConv.orientation.getConjugate() * (cp.worldPosition - transConv.position);
                        cp.localPointB = transMesh.orientation.getConjugate() * (cp.worldPosition - transMesh.position);
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
                                float distSq = (oldIt->second.contacts[j].localPointA - cp.localPointA).lengthSquare();
                                if (distSq < closestDistSq) {
                                    closestDistSq = distSq;
                                    bestMatch = j;
                                }
                            }
                            if (bestMatch != -1) {
                                cp.normalImpulse = oldIt->second.contacts[bestMatch].normalImpulse;
                                cp.tangentImpulse1 = oldIt->second.contacts[bestMatch].tangentImpulse1;
                                cp.tangentImpulse2 = oldIt->second.contacts[bestMatch].tangentImpulse2;
                            }
                        }

                        if (it->second.contactCount < 4) it->second.addContact(cp);
                        else it->second.contacts[3] = cp; 
                    }
                }
            });
        } else {
            // İki konveks nesne arası kesin çarpışma testi
            bool isBoxA = std::holds_alternative<collision::BoxShape>(collA.shape.getVariant());
            bool isBoxB = std::holds_alternative<collision::BoxShape>(collB.shape.getVariant());
            bool isSphereA = std::holds_alternative<collision::SphereShape>(collA.shape.getVariant());
            bool isSphereB = std::holds_alternative<collision::SphereShape>(collB.shape.getVariant());

            uint32_t id1 = entityA.id; uint32_t id2 = entityB.id;
            if (id1 > id2) std::swap(id1, id2);
            uint64_t key = (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);

            if (isBoxA && isBoxB) {
                collision::ContactManifold satManifold(entityA, entityB);
                const auto& bA = std::get<collision::BoxShape>(collA.shape.getVariant());
                const auto& bB = std::get<collision::BoxShape>(collB.shape.getVariant());
                if (collision::NarrowPhase::testBoxBox(bA, transA, bB, transB, satManifold)) {
                    if (satManifold.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
                        wakeIfSleeping(entityA);
                        wakeIfSleeping(entityB);
                    }

                    // Warm-starting itmelerini eşleştir
                    auto oldIt = oldManifoldMap.find(key);
                    if (oldIt != oldManifoldMap.end()) {
                        for (uint32_t cIdx = 0; cIdx < satManifold.contactCount; ++cIdx) {
                            auto& newCp = satManifold.contacts[cIdx];
                            float closestDistSq = 0.04f;
                            int bestMatch = -1;
                            for (uint32_t oIdx = 0; oIdx < oldIt->second.contactCount; ++oIdx) {
                                float distSq = (oldIt->second.contacts[oIdx].localPointA - newCp.localPointA).lengthSquare();
                                if (distSq < closestDistSq) {
                                    closestDistSq = distSq;
                                    bestMatch = oIdx;
                                }
                            }
                            if (bestMatch != -1) {
                                newCp.normalImpulse = oldIt->second.contacts[bestMatch].normalImpulse;
                                newCp.tangentImpulse1 = oldIt->second.contacts[bestMatch].tangentImpulse1;
                                newCp.tangentImpulse2 = oldIt->second.contacts[bestMatch].tangentImpulse2;
                            }
                        }
                    }

                    satManifold.computeTangents();
                    mManifoldMap.insert_or_assign(key, satManifold);
                }
            } else {
                collision::CollisionInfo info;
                bool hasCollided = false;

                if (isSphereA && isSphereB) {
                    const auto& spA = std::get<collision::SphereShape>(collA.shape.getVariant());
                    const auto& spB = std::get<collision::SphereShape>(collB.shape.getVariant());
                    hasCollided = collision::NarrowPhase::testSphereSphere(spA, transA, spB, transB, info);
                } else if (isSphereA && isBoxB) {
                    const auto& spA = std::get<collision::SphereShape>(collA.shape.getVariant());
                    const auto& bB = std::get<collision::BoxShape>(collB.shape.getVariant());
                    hasCollided = collision::NarrowPhase::testSphereBox(spA, transA, bB, transB, info);
                } else if (isBoxA && isSphereB) {
                    const auto& bA = std::get<collision::BoxShape>(collA.shape.getVariant());
                    const auto& spB = std::get<collision::SphereShape>(collB.shape.getVariant());
                    hasCollided = collision::NarrowPhase::testSphereBox(spB, transB, bA, transA, info);
                    if (hasCollided) {
                        info.normal = -info.normal; // Normal must point B to A
                    }
                } else {
                    collision::Simplex simplex;
                    if (collision::NarrowPhase::GJK(collA.shape, transA, collB.shape, transB, simplex)) {
                        info = collision::NarrowPhase::EPA(simplex, collA.shape, transA, collB.shape, transB);
                        hasCollided = info.hasCollision;
                    }
                }

                if (hasCollided) {
                    if (info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
                        wakeIfSleeping(entityA);
                        wakeIfSleeping(entityB);
                    }

                    auto [it, inserted] = mManifoldMap.try_emplace(key, entityA, entityB);
                    it->second.normal = info.normal;
                    it->second.penetration = info.penetration;
                    it->second.computeTangents();

                    collision::ContactPoint cp;
                    cp.worldPosition = info.contactPoint;
                    cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
                    cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
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
                            float distSq = (oldIt->second.contacts[j].localPointA - cp.localPointA).lengthSquare();
                            if (distSq < closestDistSq) { closestDistSq = distSq; bestMatch = j; }
                        }
                        if (bestMatch != -1) {
                            cp.normalImpulse = oldIt->second.contacts[bestMatch].normalImpulse;
                            cp.tangentImpulse1 = oldIt->second.contacts[bestMatch].tangentImpulse1;
                            cp.tangentImpulse2 = oldIt->second.contacts[bestMatch].tangentImpulse2;
                        }
                    }

                    it->second.contactCount = 0;
                    it->second.addContact(cp);
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
 * 3. Etki Anı (TOI): Engel varsa, nesneyi engele çarpacağı tam ana kadar ilerletir.
 * 4. Tepki: Nesne çarpışma yüzeyinde durdurulur ve hızı yüzey boyunca kayacak şekilde kırpılır.
 */
void CollisionSystem::applyCCD(ecs::Entity entity, float deltaTime) {
    if (!mRegistry.hasComponent<Core::BodyState>(entity) || !mRegistry.hasComponent<ecs::ColliderData>(entity) || !mRegistry.hasComponent<Core::Motion>(entity)) return;
    
    auto& state = mRegistry.getComponent<Core::BodyState>(entity);
    if (!state.useCCD) return;

    auto& transform = mRegistry.getComponent<Pose>(entity);
    auto& motion = mRegistry.getComponent<Core::Motion>(entity);

    auto& colliderA = mRegistry.getComponent<ecs::ColliderData>(entity);
    collision::AABB aabbA = colliderA.shape.computeLocalAABB();
    float radiusA = (aabbA.maxBounds - aabbA.minBounds).length() * 0.5f;

    // Nesne kendi boyutuna oranla çok hızlı hareket ediyorsa CCD'yi başlat
    constexpr float kMotionVsSize = 0.35f;
    float speed = motion.linearVelocity.length();
    float motionDist = speed * deltaTime;
    float sizeBasedMin = radiusA * kMotionVsSize;
    float minMotionForCcd = std::max(state.ccdMotionThreshold, sizeBasedMin);
    
    if (motionDist < minMotionForCcd + 0.01f) return;

    float timeOfImpact = deltaTime;
    bool hasCollision = false;
    Vector3 collisionNormal;
    ecs::Entity hitEntity{0};

    // Hareket yolunu kapsayan genişletilmiş sınır kutusu (Sweep AABB)
    Vector3 sweepEnd = transform.position + (motion.linearVelocity * deltaTime);
    (void)sweepEnd;
    collision::AABB sweepAABB = aabbA;
    sweepAABB.minBounds += Vector3(std::min(0.0f, motion.linearVelocity.x * deltaTime), std::min(0.0f, motion.linearVelocity.y * deltaTime), std::min(0.0f, motion.linearVelocity.z * deltaTime));
    sweepAABB.maxBounds += Vector3(std::max(0.0f, motion.linearVelocity.x * deltaTime), std::max(0.0f, motion.linearVelocity.y * deltaTime), std::max(0.0f, motion.linearVelocity.z * deltaTime));
    sweepAABB.minBounds += transform.position;
    sweepAABB.maxBounds += transform.position;

    mSpatialPartitioning.getTree().query(sweepAABB, [&](ecs::Entity obstacle) {
        if (obstacle == entity) return;
        if (!mRegistry.hasComponent<Pose>(obstacle) || !mRegistry.hasComponent<ecs::ColliderData>(obstacle)) return;
        
        const auto& obsState = mRegistry.getComponent<Core::BodyState>(obstacle);
        if (state.type == Core::BodyType::Kinematic && obsState.type == Core::BodyType::Kinematic) return;

        const auto& obsTrans = mRegistry.getComponent<Pose>(obstacle);
        const auto& obsColl = mRegistry.getComponent<ecs::ColliderData>(obstacle);

        float currentTime = 0.0f;
        Pose currentTransform = transform;
        Vector3 pA, pB;

        // Çarpışma anını (TOI) bulmak için iteratif yaklaşım
        for (int iter = 0; iter < 16; ++iter) {
            currentTransform.position = transform.position + (motion.linearVelocity * currentTime);
            float dist = collision::NarrowPhase::GJK_distance(colliderA.shape, currentTransform, obsColl.shape, obsTrans, pA, pB);
            
            if (dist < 0.05f) { 
                if (currentTime < timeOfImpact) {
                    timeOfImpact = currentTime;
                    hasCollision = true;
                    hitEntity = obstacle;
                    collisionNormal = (pA - pB).getNormalized();
                    // Normalin hıza ters yönde olduğundan emin ol
                    if (motion.linearVelocity.dot(collisionNormal) > 0.0f) {
                        collisionNormal = collisionNormal * -1.0f;
                    }
                    if (collisionNormal.lengthSquare() < 0.001f) collisionNormal = motion.linearVelocity.getNormalized() * -1.0f;
                }
                break;
            }

            Vector3 relVel = motion.linearVelocity;
            if (mRegistry.hasComponent<Core::Motion>(obstacle)) relVel -= mRegistry.getComponent<Core::Motion>(obstacle).linearVelocity;
            
            float maxApproachSpeed = relVel.length();
            if (maxApproachSpeed < 0.001f) break;

            currentTime += dist / maxApproachSpeed;
            if (currentTime >= timeOfImpact) break;
        }
    });

    if (hasCollision && timeOfImpact < deltaTime) {
        // Fonksiyonun başında zaten hesaplanmış olan 'speed' değişkenini kullanıyoruz.
        // Hiçbir ekstra karekök (sqrt) veya vektör uzunluğu hesaplaması yok!
        float timeMargin = (speed > 1e-4f) ? (0.01f / speed) : 0.0f;
        
        // EKSİK OLAN SATIR BURASI:
        float safeTime = std::max(0.0f, timeOfImpact - timeMargin);
        
        motion.linearVelocity = motion.linearVelocity * (safeTime / deltaTime);
    }
}

} // namespace Baryon::systems
