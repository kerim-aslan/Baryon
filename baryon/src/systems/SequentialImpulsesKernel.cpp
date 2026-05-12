/**
 * @file SequentialImpulsesKernel.cpp
 * @brief Ardışık İtme (Sequential Impulses) çözücüsü uygulaması.
 * @details Temas noktalarını, mesafe kısıtlamalarını ve menteşe eklemlerini (joint) 
 *          iteratif olarak çözen çekirdek algoritmadır. Her kısıtlama için itme (impulse) 
 *          değerleri hesaplanarak nesnelerin hızları ve konumları düzeltilir. 
 *          Simülasyon kararlılığı için hız çözümü sonrası "Pozisyon Düzeltme" (NGS) uygulanır.
 */

#include "Baryon/systems/SequentialImpulsesKernel.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include <algorithm> 
#include <cmath>
#include <unordered_set>

namespace Baryon::systems {

SequentialImpulsesKernel::SequentialImpulsesKernel(Core::Registry& registry)
    : mRegistry(registry) {}

/**
 * @brief Bir nesne grubu (Island) içindeki tüm kısıtlamaları ve çarpışmaları çözer.
 * @details 
 * 1. Filtreleme: Mevcut adaya ait temas verilerini ve eklem tanımlarını toplar.
 * 2. Hız Çözücü (Velocity Solver): Belirlenen iterasyon kadar hız değişimlerini (Impulse) hesaplar.
 *    Bu aşamada Çarpışma, Mesafe ve Menteşe kısıtlamaları ardışık olarak işlenir.
 * 3. Pozisyon Çözücü (Position Solver): Hızlardan bağımsız olarak, iç içe geçmiş nesneleri 
 *    birbirinden ayırarak "titreme" ve "yumuşaklık" sorunlarını giderir.
 * 
 * @param island İşlem görecek varlık listesi.
 * @param manifolds Sisteme kayıtlı tüm temas verileri.
 * @param deltaTime Zaman adımı.
 * @param velocityIterations Hız hassasiyeti.
 * @param positionIterations Konum düzeltme hassasiyeti.
 */
void SequentialImpulsesKernel::execute(const std::vector<ecs::Entity>& island,
                                        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
                                        float deltaTime, int velocityIterations, int positionIterations) {
    // 1. Verimli arama için ada içindeki varlık kimliklerini set yapısına al
    std::unordered_set<uint32_t> islandIds;
    islandIds.reserve(island.size() * 2 + 8);
    for (ecs::Entity e : island) {
        islandIds.insert(e.id);
    }

    auto& manifoldMap = const_cast<std::unordered_map<uint64_t, collision::ContactManifold>&>(manifolds);

    // 2. Sadece bu adadaki nesneleri ilgilendiren temasları seç
    std::vector<collision::ContactManifold*> localManifolds;
    localManifolds.reserve(std::min(manifolds.size(), static_cast<size_t>(256)));
    for (auto& [key, manifold] : manifoldMap) {
        (void)key;
        if (islandIds.contains(manifold.entityA.id) || islandIds.contains(manifold.entityB.id)) {
            localManifolds.push_back(&manifold);
        }
    }

    // 3. Adadaki aktif eklem kısıtlamalarını (Mesafe ve Menteşe) filtrele
    std::vector<collision::DistanceConstraint*> localDistance;
    std::vector<collision::RevoluteConstraint*> localRevolute;
    localDistance.reserve(16);
    localRevolute.reserve(16);

    if (mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData().size() > 0) {
        auto& constraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
        for (auto& constraint : constraints) {
            if (islandIds.contains(constraint.entityA.id) || islandIds.contains(constraint.entityB.id)) {
                localDistance.push_back(&constraint);
            }
        }
    }
    if (mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData().size() > 0) {
        auto& constraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
        for (auto& constraint : constraints) {
            if (islandIds.contains(constraint.entityA.id) || islandIds.contains(constraint.entityB.id)) {
                localRevolute.push_back(&constraint);
            }
        }
    }

    // 4. Hız İterasyonları: Kuvvetleri ardışık olarak uygula (Gauss-Seidel Yaklaşımı)
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (collision::ContactManifold* manifold : localManifolds) {
            resolveCollision(*manifold, deltaTime);
        }
        for (collision::DistanceConstraint* constraint : localDistance) {
            resolveDistanceConstraint(*constraint, deltaTime);
        }
        for (collision::RevoluteConstraint* constraint : localRevolute) {
            resolveRevoluteConstraint(*constraint, deltaTime);
        }
    }

    // 5. Pozisyon İterasyonları: İç içe geçmeleri fiziksel olarak ayır
    for (int iter = 0; iter < positionIterations; ++iter) {
        for (collision::ContactManifold* manifold : localManifolds) {
            solvePositionConstraints(*manifold);
        }
    }
}

/**
 * @brief Yardımcı fonksiyon: Adadaki çarpışma çözümlerini yürütür.
 */
void SequentialImpulsesKernel::solveIsland(const std::vector<ecs::Entity>& island, 
                                           const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds, 
                                           float deltaTime) {
    std::unordered_set<uint32_t> islandIds;
    islandIds.reserve(island.size() * 2 + 8);
    for (ecs::Entity e : island) {
        islandIds.insert(e.id);
    }
    for (const auto& [key, manifold] : manifolds) {
        (void)key;
        if (!islandIds.contains(manifold.entityA.id) && !islandIds.contains(manifold.entityB.id)) continue;
        auto& mutableManifold = const_cast<collision::ContactManifold&>(manifold);
        resolveCollision(mutableManifold, deltaTime);
    }
}

/**
 * @brief Yardımcı fonksiyon: Adadaki eklem kısıtlamalarını yürütür.
 */
void SequentialImpulsesKernel::solveConstraintsForIsland(const std::vector<ecs::Entity>& island, 
                                                          float deltaTime) {
    std::unordered_set<uint32_t> islandIds;
    islandIds.reserve(island.size() * 2 + 8);
    for (ecs::Entity e : island) {
        islandIds.insert(e.id);
    }

    if (mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData().size() > 0) {
        auto& constraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
        for (auto& constraint : constraints) {
            if (islandIds.contains(constraint.entityA.id) || islandIds.contains(constraint.entityB.id)) {
                resolveDistanceConstraint(constraint, deltaTime);
            }
        }
    }

    if (mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData().size() > 0) {
        auto& constraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
        for (auto& constraint : constraints) {
            if (islandIds.contains(constraint.entityA.id) || islandIds.contains(constraint.entityB.id)) {
                resolveRevoluteConstraint(constraint, deltaTime);
            }
        }
    }
}

/**
 * @brief İki nesne arasındaki çarpışma tepkisini (itme ve sürtünme) hesaplar.
 * @details 
 * 1. Bağıl Hız: Temas noktasındaki hız farkı bulunur.
 * 2. Efektif Kütle: Nesnelerin kütlesi ve dönme direncinin normal yönündeki toplam etkisi hesaplanır.
 * 3. İtme Kuvveti (Impulse): Hız farkını sıfırlayacak itme değeri bulunur. 
 *    Esneklik (Restitution) eklenerek sekme hareketi sağlanır.
 * 4. Sürtünme: Coulomb sürtünme modeli kullanılarak normal itmeye bağlı teğet kuvveti uygulanır.
 * 
 * @param manifold Temas noktası verileri.
 * @param deltaTime Zaman adımı.
 */
void SequentialImpulsesKernel::resolveCollision(collision::ContactManifold& manifold, float deltaTime) {
    if (!mRegistry.hasComponent<Pose>(manifold.entityA) || !mRegistry.hasComponent<Pose>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(manifold.entityA) || !mRegistry.hasComponent<Core::Motion>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) || !mRegistry.hasComponent<Core::MassProps>(manifold.entityB)) return;

    auto& poseA = mRegistry.getComponent<Pose>(manifold.entityA);
    auto& poseB = mRegistry.getComponent<Pose>(manifold.entityB);
    auto& motionA = mRegistry.getComponent<Core::Motion>(manifold.entityA);
    auto& motionB = mRegistry.getComponent<Core::Motion>(manifold.entityB);
    const auto& massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
    const auto& massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
        typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
    }
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
        typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;
    }

    // Sabit (Static) nesnelerin ters kütlesi 0 alınır
    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    
    // Malzeme özelliklerini birleştir (Sürtünme: Geometrik Ortalama, Esneklik: Maksimum)
    float friction = 0.4f;
    float restitution = 0.0f;
    if (mRegistry.hasComponent<Core::Material>(manifold.entityA) && mRegistry.hasComponent<Core::Material>(manifold.entityB)) {
        const auto& matA = mRegistry.getComponent<Core::Material>(manifold.entityA);
        const auto& matB = mRegistry.getComponent<Core::Material>(manifold.entityB);
        friction = std::sqrt(matA.friction * matB.friction);
        restitution = std::max(matA.restitution, matB.restitution);
    }

    Vector3 n = manifold.normal;
    if (n.lengthSquare() > 1e-12f) {
        n.normalize();
    } else {
        return;
    }

    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        auto& contact = manifold.contacts[i];
        
        // Kütle merkezinden temas noktasına giden kollar (Lever Arms)
        Vector3 rA = poseA.orientation * contact.localPointA;
        Vector3 rB = poseB.orientation * contact.localPointB;
        
        // Temas noktalarındaki anlık hızları hesapla
        Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
        Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
        Vector3 relativeVelocity = vA - vB; 
        
        float velocityAlongNormal = relativeVelocity.dot(n);
        
        // Sıçramayı kontrol et (Sadece yeterince hızlı çarpmalarda aktif olur)
        float effectiveRestitution = restitution;
        if (velocityAlongNormal > -physics::PhysicsConstants::RestitutionVelocityThreshold) effectiveRestitution = 0.0f;
        
        // Normal doğrultusundaki toplam dönme ve öteleme direnci (Jacobian mass)
        Vector3 rACrossN = rA.cross(n);
        Vector3 rBCrossN = rB.cross(n);
        float invMassSum = invMassA + invMassB + 
                           (invIA * rACrossN).dot(rACrossN) + 
                           (invIB * rBCrossN).dot(rBCrossN);
                           
        if (invMassSum <= 1e-6f) continue;
        
        // İtme miktarını hesapla
        float j = (-(1.0f + effectiveRestitution) * std::min(velocityAlongNormal, 0.0f)) / invMassSum;
        
        // Birikmiş İtme (Accumulated Impulse): İtmelerin toplamı her zaman pozitif olmalıdır
        float oldNormalImpulse = contact.normalImpulse;
        contact.normalImpulse = std::max(oldNormalImpulse + j, 0.0f);
        j = contact.normalImpulse - oldNormalImpulse;
        
        Vector3 impulse = n * j;
        
        // Hızları anlık olarak güncelle
        motionA.linearVelocity += impulse * invMassA;
        motionA.angularVelocity += invIA * rA.cross(impulse);
        motionB.linearVelocity -= impulse * invMassB;
        motionB.angularVelocity -= invIB * rB.cross(impulse);
        
        // --- SÜRTÜNME ÇÖZÜMÜ ---
        vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
        vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
        relativeVelocity = vA - vB;
        
        // Normal bileşeni çıkararak teğet (tangent) yönünü bul
        Vector3 tangent = relativeVelocity - (n * relativeVelocity.dot(n));
        if (tangent.lengthSquare() > 0.0001f) {
            tangent.normalize();
            
            // Teğet yönündeki efektif direnç
            float jt = -relativeVelocity.dot(tangent);
            Vector3 rACrossT = rA.cross(tangent);
            Vector3 rBCrossT = rB.cross(tangent);
            float invMassSumT = invMassA + invMassB + 
                                (invIA * rACrossT).dot(rACrossT) + 
                                (invIB * rBCrossT).dot(rBCrossT);
            if (invMassSumT <= 1e-6f) continue;
            jt /= invMassSumT;
            
            // Coulomb Kanunu: Sürtünme kuvveti normal kuvvetin (mu) katını aşamaz
            float maxFriction = friction * contact.normalImpulse;
            float oldTangentImpulse = contact.tangentImpulse;
            contact.tangentImpulse = std::clamp(oldTangentImpulse + jt, -maxFriction, maxFriction);
            jt = contact.tangentImpulse - oldTangentImpulse;
            
            // Sürtünme itmesini uygula
            Vector3 frictionImpulse = tangent * jt;
            motionA.linearVelocity += frictionImpulse * invMassA;
            motionA.angularVelocity += invIA * rA.cross(frictionImpulse);
            motionB.linearVelocity -= frictionImpulse * invMassB;
            motionB.angularVelocity -= invIB * rB.cross(frictionImpulse);
        }
    }
}

/**
 * @brief Nesnelerin birbirinin içine girmesini engelleyen geometrik düzeltici.
 * @details Hızlardan bağımsız olarak, sadece konum ve rotasyonları doğrudan 
 *          değiştirir. Bu sayede simülasyondaki enerji kaybı telafi edilir ve 
 *          nesneler yüzeylerde kararlı durur.
 */
void SequentialImpulsesKernel::solvePositionConstraints(collision::ContactManifold& manifold) {
    if (!mRegistry.hasComponent<Pose>(manifold.entityA) || !mRegistry.hasComponent<Pose>(manifold.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(manifold.entityA) || !mRegistry.hasComponent<Core::MassProps>(manifold.entityB)) return;

    auto& poseA = mRegistry.getComponent<Pose>(manifold.entityA);
    auto& poseB = mRegistry.getComponent<Pose>(manifold.entityB);
    const auto& massA = mRegistry.getComponent<Core::MassProps>(manifold.entityA);
    const auto& massB = mRegistry.getComponent<Core::MassProps>(manifold.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityA)) {
        typeA = mRegistry.getComponent<Core::BodyState>(manifold.entityA).type;
    }
    if (mRegistry.hasComponent<Core::BodyState>(manifold.entityB)) {
        typeB = mRegistry.getComponent<Core::BodyState>(manifold.entityB).type;
    }

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const float invSum = invMassA + invMassB;
    if (invSum <= 1e-8f) return;

    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

    Vector3 n = manifold.normal;
    if (n.lengthSquare() > 1e-12f) {
        n.normalize();
    } else {
        return;
    }

    for (uint32_t i = 0; i < manifold.contactCount; ++i) {
        auto& contact = manifold.contacts[i];

        Vector3 rA = poseA.orientation * contact.localPointA;
        Vector3 rB = poseB.orientation * contact.localPointB;

        Vector3 pA = poseA.position + rA;
        Vector3 pB = poseB.position + rB;
        
        Vector3 separationVec = pA - pB;
        float separationAlongNormal = separationVec.dot(n);
        
        // Güncel iç içe geçme miktarını hesapla
        float currentPenetration = contact.penetration - separationAlongNormal;

        // Linear Slop: Çok küçük iç içe geçmeleri görmezden gelerek titremeyi önle
        float C = std::clamp(currentPenetration - physics::PhysicsConstants::LinearSlop, 0.0f, physics::PhysicsConstants::MaxLinearCorrection);
        
        if (C <= 0.0f) continue;

        Vector3 rACrossN = rA.cross(n);
        Vector3 rBCrossN = rB.cross(n);
        float invMassSum = invMassA + invMassB + 
                           (invIA * rACrossN).dot(rACrossN) + 
                           (invIB * rBCrossN).dot(rBCrossN);
                           
        if (invMassSum <= 1e-6f) continue;

        // Baumgarte Stabilizasyonu: Pozisyon hatasını her karede belirli bir oranda (%) düzelt
        float lambda = (C * physics::PhysicsConstants::PositionCorrectionFactor) / invMassSum;
        Vector3 impulse = n * lambda;

        // Nesneleri birbirlerinden uzağa ötele
        poseA.position += impulse * invMassA;
        poseB.position -= impulse * invMassB;

        // Nesneleri rotasyonel olarak düzelt
        if (typeA == Core::BodyType::Dynamic) {
            Vector3 angularMoveA = invIA * rA.cross(impulse);
            Quaternion spinA(angularMoveA.x, angularMoveA.y, angularMoveA.z, 0.0f);
            Quaternion qDotA = spinA * poseA.orientation;
            poseA.orientation.x += qDotA.x * 0.5f;
            poseA.orientation.y += qDotA.y * 0.5f;
            poseA.orientation.z += qDotA.z * 0.5f;
            poseA.orientation.w += qDotA.w * 0.5f;
            poseA.orientation.normalize();
        }

        if (typeB == Core::BodyType::Dynamic) {
            Vector3 angularMoveB = invIB * rB.cross(impulse * -1.0f);
            Quaternion spinB(angularMoveB.x, angularMoveB.y, angularMoveB.z, 0.0f);
            Quaternion qDotB = spinB * poseB.orientation;
            poseB.orientation.x += qDotB.x * 0.5f;
            poseB.orientation.y += qDotB.y * 0.5f;
            poseB.orientation.z += qDotB.z * 0.5f;
            poseB.orientation.w += qDotB.w * 0.5f;
            poseB.orientation.normalize();
        }
    }
}

/**
 * @brief İki nesneyi belirli bir mesafede tutan kısıtlamayı çözer.
 */
void SequentialImpulsesKernel::resolveDistanceConstraint(collision::DistanceConstraint& constraint, float deltaTime) {
    if (!mRegistry.hasComponent<Pose>(constraint.entityA) || !mRegistry.hasComponent<Pose>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) || !mRegistry.hasComponent<Core::Motion>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) || !mRegistry.hasComponent<Core::MassProps>(constraint.entityB)) return;

    auto& poseA = mRegistry.getComponent<Pose>(constraint.entityA);
    auto& poseB = mRegistry.getComponent<Pose>(constraint.entityB);
    auto& motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
    auto& motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
    const auto& massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
    const auto& massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(constraint.entityA)) {
        typeA = mRegistry.getComponent<Core::BodyState>(constraint.entityA).type;
    }
    if (mRegistry.hasComponent<Core::BodyState>(constraint.entityB)) {
        typeB = mRegistry.getComponent<Core::BodyState>(constraint.entityB).type;
    }

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

    Vector3 rA = poseA.orientation * constraint.localAnchorA;
    Vector3 rB = poseB.orientation * constraint.localAnchorB;

    Vector3 pA = poseA.position + rA;
    Vector3 pB = poseB.position + rB;

    Vector3 diff = pB - pA;
    float dist = diff.length();
    if (dist < 1e-4f) return; 
    Vector3 n = diff * (1.0f / dist); 

    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relativeVelocity = vB - vA; 

    float jv = relativeVelocity.dot(n); 

    Vector3 rACrossN = rA.cross(n);
    Vector3 rBCrossN = rB.cross(n);
    float invMassSum = invMassA + invMassB + 
                       (invIA * rACrossN).dot(rACrossN) + 
                       (invIB * rBCrossN).dot(rBCrossN);
                       
    if (invMassSum <= 1e-6f) return;

    // Hata payını (bias) kullanarak mesafeyi koru
    float beta = 0.2f; 
    float bias = (beta / deltaTime) * (dist - constraint.targetDistance);

    float lambda = (-jv - bias) / invMassSum;
    Vector3 impulse = n * lambda;

    motionA.linearVelocity -= impulse * invMassA;
    motionA.angularVelocity -= invIA * rA.cross(impulse);
    
    motionB.linearVelocity += impulse * invMassB;
    motionB.angularVelocity += invIB * rB.cross(impulse);
}

/**
 * @brief Menteşe (Hinge) kısıtlamasını çözer.
 * @details Nesnelerin birbirine belirli noktalardan bağlı kalmasını ve 
 *          sadece tek bir eksen etrafında dönmesini sağlar.
 */
void SequentialImpulsesKernel::resolveRevoluteConstraint(collision::RevoluteConstraint& constraint, float deltaTime) {
    if (!mRegistry.hasComponent<Pose>(constraint.entityA) || !mRegistry.hasComponent<Pose>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::Motion>(constraint.entityA) || !mRegistry.hasComponent<Core::Motion>(constraint.entityB)) return;
    if (!mRegistry.hasComponent<Core::MassProps>(constraint.entityA) || !mRegistry.hasComponent<Core::MassProps>(constraint.entityB)) return;

    auto& poseA = mRegistry.getComponent<Pose>(constraint.entityA);
    auto& poseB = mRegistry.getComponent<Pose>(constraint.entityB);
    auto& motionA = mRegistry.getComponent<Core::Motion>(constraint.entityA);
    auto& motionB = mRegistry.getComponent<Core::Motion>(constraint.entityB);
    const auto& massA = mRegistry.getComponent<Core::MassProps>(constraint.entityA);
    const auto& massB = mRegistry.getComponent<Core::MassProps>(constraint.entityB);

    Core::BodyType typeA = Core::BodyType::Dynamic;
    Core::BodyType typeB = Core::BodyType::Dynamic;
    if (mRegistry.hasComponent<Core::BodyState>(constraint.entityA)) {
        typeA = mRegistry.getComponent<Core::BodyState>(constraint.entityA).type;
    }
    if (mRegistry.hasComponent<Core::BodyState>(constraint.entityB)) {
        typeB = mRegistry.getComponent<Core::BodyState>(constraint.entityB).type;
    }

    const float invMassA = (typeA == Core::BodyType::Dynamic) ? massA.inverseMass : 0.0f;
    const float invMassB = (typeB == Core::BodyType::Dynamic) ? massB.inverseMass : 0.0f;
    const Matrix3x3 invIA = (typeA == Core::BodyType::Dynamic) ? massA.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
    const Matrix3x3 invIB = (typeB == Core::BodyType::Dynamic) ? massB.inverseInertiaTensor : Matrix3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);

    Vector3 rA = poseA.orientation * constraint.localAnchorA;
    Vector3 rB = poseB.orientation * constraint.localAnchorB;

    Vector3 pA = poseA.position + rA;
    Vector3 pB = poseB.position + rB;

    Vector3 vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
    Vector3 vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
    Vector3 relativeVelocity = vB - vA; 

    // 1. Doğrusal Çözüm (Point-to-Point): Bağlantı noktalarını üst üste tut
    Vector3 diff = pB - pA;
    float beta = 0.2f;
    Vector3 bias = diff * (beta / deltaTime);

    Vector3 axes[3] = { Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1) };
    for (int i = 0; i < 3; ++i) {
        Vector3 n = axes[i];
        float jv = relativeVelocity.dot(n);
        float bias_n = bias.dot(n);
        
        Vector3 rACrossN = rA.cross(n);
        Vector3 rBCrossN = rB.cross(n);
        float invMassSum = invMassA + invMassB + 
                           (invIA * rACrossN).dot(rACrossN) + 
                           (invIB * rBCrossN).dot(rBCrossN);
                           
        if (invMassSum > 1e-6f) {
            float lambda = (-jv - bias_n) / invMassSum;
            Vector3 impulse = n * lambda;
            
            motionA.linearVelocity -= impulse * invMassA;
            motionA.angularVelocity -= invIA * rA.cross(impulse);
            motionB.linearVelocity += impulse * invMassB;
            motionB.angularVelocity += invIB * rB.cross(impulse);
            
            vA = motionA.linearVelocity + motionA.angularVelocity.cross(rA);
            vB = motionB.linearVelocity + motionB.angularVelocity.cross(rB);
            relativeVelocity = vB - vA;
        }
    }

    // 2. Açısal Çözüm: Sadece menteşe eksenine dik olan dönüşleri engelle
    Vector3 axisA = poseA.orientation * constraint.hingeAxisLocalA;
    Vector3 t1, t2;
    if (std::abs(axisA.x) > 0.9f) {
        t1 = Vector3(0, 1, 0).cross(axisA).getNormalized();
    } else {
        t1 = Vector3(1, 0, 0).cross(axisA).getNormalized();
    }
    t2 = axisA.cross(t1).getNormalized();
    
    Vector3 axesRot[2] = { t1, t2 };
    for (int i = 0; i < 2; ++i) {
        Vector3 n = axesRot[i];
        float jv = (motionB.angularVelocity - motionA.angularVelocity).dot(n);
        
        Vector3 axisB = poseB.orientation * constraint.hingeAxisLocalB;
        float error = n.dot(axisA.cross(axisB));
        float bias_n = (beta / deltaTime) * error;
        
        float invMassSum = (invIA * n).dot(n) + (invIB * n).dot(n);
        if (invMassSum > 1e-6f) {
            float lambda = (-jv - bias_n) / invMassSum;
            Vector3 impulse = n * lambda;
            
            motionA.angularVelocity -= invIA * impulse;
            motionB.angularVelocity += invIB * impulse;
        }
    }
}

} // namespace Baryon::systems