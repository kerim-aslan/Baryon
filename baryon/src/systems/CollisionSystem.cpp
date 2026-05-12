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
                        
                        // Normal yönü ani değişirse (köşe çarpışmaları) manifold'u sıfırla
                        if (!inserted && it->second.contactCount > 0 && it->second.normal.dot(currentNormal) < 0.95f) {
                            it->second.contactCount = 0;
                        }

                        if (info.penetration > it->second.penetration || it->second.contactCount == 0) {
                            it->second.normal = currentNormal;
                            it->second.penetration = info.penetration;
                        }
                        
                        collision::ContactPoint cp;
                        cp.worldPosition = info.contactPoint;
                        cp.localPointA = transConv.orientation.getConjugate() * (cp.worldPosition - transConv.position);
                        cp.localPointB = transMesh.orientation.getConjugate() * (cp.worldPosition - transMesh.position);
                        cp.penetration = info.penetration;
                        cp.normalImpulse = 0.0f;
                        cp.tangentImpulse = 0.0f;
                        
                        // Sıcak Başlatma: Eski kuvvetleri yeni noktaya aktar
                        auto oldIt = oldManifoldMap.find(key);
                        if (oldIt != oldManifoldMap.end()) {
                            int bestMatch = -1;
                            float closestDistSq = 0.02f; 
                            for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
                                float distSq = (oldIt->second.contacts[j].localPointA - cp.localPointA).lengthSquare();
                                if (distSq < closestDistSq) {
                                    closestDistSq = distSq;
                                    bestMatch = j;
                                }
                            }
                            if (bestMatch != -1) {
                                cp.normalImpulse = oldIt->second.contacts[bestMatch].normalImpulse;
                                cp.tangentImpulse = oldIt->second.contacts[bestMatch].tangentImpulse;
                            }

                            // Kalıcı Manifold: Hala geçerli olan eski noktaları listede tut
                            if (inserted) {
                                for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
                                    if (static_cast<int>(j) == bestMatch) continue; 
                                    auto& oldCp = oldIt->second.contacts[j];
                                    Vector3 globalA = transConv.position + (transConv.orientation * oldCp.localPointA);
                                    Vector3 globalB = transMesh.position + (transMesh.orientation * oldCp.localPointB);
                                    float distance = (globalA - globalB).dot(it->second.normal);
                                    float tangentialDist = ((globalA - globalB) - (it->second.normal * distance)).lengthSquare();
                                    
                                    if (distance < 0.05f && tangentialDist < 0.01f) {
                                        oldCp.penetration = std::max(0.0f, distance);
                                        if (it->second.contactCount < 3) it->second.addContact(oldCp);
                                    }
                                }
                            }
                        }

                        if (it->second.contactCount < 4) it->second.addContact(cp);
                        else it->second.contacts[3] = cp; 
                    }
                }
            });
        } else {
            // İki konveks nesne (Kutu, Küre vb.) arası kesin çarpışma testi
            collision::Simplex simplex;
            if (collision::NarrowPhase::GJK(collA.shape, transA, collB.shape, transB, simplex)) {
                collision::CollisionInfo info = collision::NarrowPhase::EPA(simplex, collA.shape, transA, collB.shape, transB);
                
                if (info.hasCollision) {
                    if (info.penetration > physics::PhysicsConstants::LinearSlop * 2.0f) {
                        wakeIfSleeping(entityA);
                        wakeIfSleeping(entityB);
                    }
                    
                    uint32_t id1 = entityA.id; uint32_t id2 = entityB.id;
                    if (id1 > id2) std::swap(id1, id2);
                    uint64_t key = (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);

                    auto [it, inserted] = mManifoldMap.try_emplace(key, entityA, entityB);

                    if (!inserted && it->second.contactCount > 0 && it->second.normal.dot(info.normal) < 0.95f) {
                        it->second.contactCount = 0;
                    }

                    if (info.penetration > it->second.penetration || it->second.contactCount == 0) {
                        it->second.normal = info.normal;
                        it->second.penetration = info.penetration;
                    }
                    
                    collision::ContactPoint cp;
                    cp.worldPosition = info.contactPoint;
                    cp.localPointA = transA.orientation.getConjugate() * (cp.worldPosition - transA.position);
                    cp.localPointB = transB.orientation.getConjugate() * (cp.worldPosition - transB.position);
                    cp.penetration = info.penetration;
                    cp.normalImpulse = 0.0f;
                    cp.tangentImpulse = 0.0f;
                    
                    auto oldIt = oldManifoldMap.find(key);
                    if (oldIt != oldManifoldMap.end()) {
                        int bestMatch = -1;
                        float closestDistSq = 0.02f;
                        for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
                            float distSq = (oldIt->second.contacts[j].localPointA - cp.localPointA).lengthSquare();
                            if (distSq < closestDistSq) { closestDistSq = distSq; bestMatch = j; }
                        }
                        if (bestMatch != -1) {
                            cp.normalImpulse = oldIt->second.contacts[bestMatch].normalImpulse;
                            cp.tangentImpulse = oldIt->second.contacts[bestMatch].tangentImpulse;
                        }

                        if (inserted) {
                            for (uint32_t j = 0; j < oldIt->second.contactCount; ++j) {
                                if (static_cast<int>(j) == bestMatch) continue; 
                                auto& oldCp = oldIt->second.contacts[j];
                                Vector3 globalA = transA.position + (transA.orientation * oldCp.localPointA);
                                Vector3 globalB = transB.position + (transB.orientation * oldCp.localPointB);
                                float distance = (globalA - globalB).dot(it->second.normal);
                                float tangentialDist = ((globalA - globalB) - (it->second.normal * distance)).lengthSquare();
                                
                                if (distance < 0.05f && tangentialDist < 0.01f) {
                                    oldCp.penetration = std::max(0.0f, distance);
                                    if (it->second.contactCount < 3) it->second.addContact(oldCp);
                                }
                            }
                        }
                    }

                    if (it->second.contactCount < 4) it->second.addContact(cp);
                    else it->second.contacts[3] = cp;
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
        // Nesneyi çarpışma anında durdur ve hızı yüzeye teğet olacak şekilde ayarla
        transform.position += motion.linearVelocity * std::max(0.0f, timeOfImpact - 0.001f);
        float velAlongNormal = motion.linearVelocity.dot(collisionNormal);
        if (velAlongNormal < 0.0f) motion.linearVelocity -= collisionNormal * velAlongNormal;
    }
}

} // namespace Baryon::systems