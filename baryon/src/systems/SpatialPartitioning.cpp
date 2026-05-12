/**
 * @file SpatialPartitioning.cpp
 * @brief Uzaysal Bölümleme (Geniş Faz) sistemi uygulaması.
 * @details Dinamik AABB Ağacı kullanarak nesneleri uzayda organize eder ve birbirine 
 *          temas etme ihtimali olan aday çiftleri belirleyerek performans sağlar.
 */

#include "Baryon/systems/SpatialPartitioning.hpp"
#include <ranges>

namespace Baryon::systems {

SpatialPartitioning::SpatialPartitioning(Core::Registry& registry, memory::MemoryManager& mm)
    : mRegistry(registry), mTree(mm) {
    // Çarpışma adayları için başlangıç kapasitesi ayır
    mPotentialCollisions.reserve(1000);
}

/**
 * @brief Yeni bir varlığı AABB ağacına ekler.
 * @details Dönüşlü AABB (OBB -> AABB) hesaplama süreci:
 *          1. Nesnenin yerel sınır kutusu (local AABB) üzerinden merkez ve genişlik bulunur.
 *          2. Nesnenin rotasyon matrisinden dünya eksenleri (X, Y, Z) elde edilir.
 *          3. Projeksiyon yöntemiyle, nesnenin her eksendeki maksimum uzanımı hesaplanır.
 *          4. Elde edilen bu genişlikler, nesnenin dünya pozisyonuna eklenerek nihai AABB oluşturulur.
 *
 * @param entity Sisteme eklenecek varlık.
 */
void SpatialPartitioning::addEntityToTree(ecs::Entity entity) {
    if (!mRegistry.hasComponent<Pose>(entity) || !mRegistry.hasComponent<ecs::ColliderData>(entity)) return;

    const auto& transform = mRegistry.getComponent<Pose>(entity);
    auto& collider = mRegistry.getComponent<ecs::ColliderData>(entity);

    collision::AABB localAABB = collider.shape.computeLocalAABB();
    Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
    Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

    // Rotasyon matrisinin eksen vektörleri
    Vector3 xAxis = transform.orientation * Vector3(1, 0, 0);
    Vector3 yAxis = transform.orientation * Vector3(0, 1, 0);
    Vector3 zAxis = transform.orientation * Vector3(0, 0, 1);

    // OBB'nin dünya eksenleri üzerindeki izdüşüm genişliklerini hesapla
    Vector3 rotatedExtents(
        std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y + std::abs(zAxis.x) * extents.z,
        std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y + std::abs(zAxis.y) * extents.z,
        std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y + std::abs(zAxis.z) * extents.z
    );

    Vector3 worldCenter = transform.position + (transform.orientation * center);
    collision::AABB worldAABB(worldCenter - rotatedExtents, worldCenter + rotatedExtents);

    collider.treeNodeIndex = mTree.insertLeaf(entity, worldAABB);
}

/**
 * @brief Hareket eden bir varlığın ağaçtaki konumunu ve sınırlarını günceller.
 * @details Nesnenin yeni konumuna göre AABB yeniden hesaplanır. "Fat AABB" (şişirilmiş kutu) 
 *          sayesinde, nesne eski sınırlarının dışına çıkmadığı sürece ağaç yapısı değişmez.
 *
 * @param entity Güncellenecek varlık.
 */
void SpatialPartitioning::updateEntityInTree(ecs::Entity entity) {
    if (!mRegistry.hasComponent<Pose>(entity) || !mRegistry.hasComponent<ecs::ColliderData>(entity)) return;

    const auto& transform = mRegistry.getComponent<Pose>(entity);
    auto& collider = mRegistry.getComponent<ecs::ColliderData>(entity);

    if (collider.treeNodeIndex == collision::NULL_NODE) return;

    collision::AABB localAABB = collider.shape.computeLocalAABB();
    Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
    Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

    Vector3 xAxis = transform.orientation * Vector3(1, 0, 0);
    Vector3 yAxis = transform.orientation * Vector3(0, 1, 0);
    Vector3 zAxis = transform.orientation * Vector3(0, 0, 1);

    Vector3 rotatedExtents(
        std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y + std::abs(zAxis.x) * extents.z,
        std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y + std::abs(zAxis.y) * extents.z,
        std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y + std::abs(zAxis.z) * extents.z
    );

    Vector3 worldCenter = transform.position + (transform.orientation * center);
    collision::AABB worldAABB(worldCenter - rotatedExtents, worldCenter + rotatedExtents);

    collider.treeNodeIndex = mTree.updateLeaf(collider.treeNodeIndex, worldAABB); 
}

/**
 * @brief Bir varlığı ve ona bağlı sınır kutusunu ağaçtan kaldırır.
 * @param entity Çıkarılacak varlık.
 */
void SpatialPartitioning::removeEntityFromTree(ecs::Entity entity) {
    if (!mRegistry.hasComponent<ecs::ColliderData>(entity)) return;
    auto& collider = mRegistry.getComponent<ecs::ColliderData>(entity);
    
    if (collider.treeNodeIndex != collision::NULL_NODE) {
        mTree.removeLeaf(collider.treeNodeIndex);
        collider.treeNodeIndex = collision::NULL_NODE;
    }
}

/**
 * @brief Ağacı tamamen sıfırlar ve mevcut tüm varlıklarla yeniden inşa eder.
 * @details Genellikle sistem geri yükleme (snapshot restore) işlemlerinden sonra 
 *          ağaç bütünlüğünü sağlamak için çağrılır.
 */
void SpatialPartitioning::rebuildTree() {
    mTree.clear();
    
    auto& collidersPool = mRegistry.getComponentPool<ecs::ColliderData>();
    auto& entities = collidersPool.getAllEntities();
    auto& colliders = collidersPool.getAllData();

    for (auto&& [entity, collider] : std::views::zip(entities, colliders)) {
        if (!mRegistry.hasComponent<Pose>(entity)) continue;

        const auto& transform = mRegistry.getComponent<Pose>(entity);
        collision::AABB localAABB = collider.shape.computeLocalAABB();
        Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
        Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

        Vector3 xAxis = transform.orientation * Vector3(1, 0, 0);
        Vector3 yAxis = transform.orientation * Vector3(0, 1, 0);
        Vector3 zAxis = transform.orientation * Vector3(0, 0, 1);

        Vector3 rotatedExtents(
            std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y + std::abs(zAxis.x) * extents.z,
            std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y + std::abs(zAxis.y) * extents.z,
            std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y + std::abs(zAxis.z) * extents.z
        );

        Vector3 worldCenter = transform.position + (transform.orientation * center);
        collision::AABB worldAABB(worldCenter - rotatedExtents, worldCenter + rotatedExtents);

        collider.treeNodeIndex = mTree.insertLeaf(entity, worldAABB);
    }
}

/**
 * @brief Her fizik adımında AABB'leri günceller ve olası çarpışmaları bulur.
 * @details Süreç iki aşamadan oluşur:
 *
 *          1. GÜNCELLEME: Hareket eden tüm nesnelerin dünya AABB'leri yenilenir 
 *             ve ağaç hiyerarşisi (BVH) güncel tutulur.
 *
 *          2. SORGULAMA: Her nesne için ağaçta kesişim testi yapılır. Gereksiz 
 *             işlemleri ve mükerrer kayıtları önlemek için sadece ID_A < ID_B 
 *             koşulunu sağlayan çiftler listeye eklenir.
 */
void SpatialPartitioning::step() {
    mPotentialCollisions.clear();

    auto& collidersPool = mRegistry.getComponentPool<ecs::ColliderData>();
    auto& entities = collidersPool.getAllEntities();
    auto& colliders = collidersPool.getAllData();

    // Aşama 1: Tüm AABB'leri güncelle
    for (auto&& [entity, collider] : std::views::zip(entities, colliders)) {
        if (collider.treeNodeIndex == collision::NULL_NODE) continue;
        if (!mRegistry.hasComponent<Pose>(entity)) continue;

        const auto& transform = mRegistry.getComponent<Pose>(entity);
        
        collision::AABB localAABB = collider.shape.computeLocalAABB();
        Vector3 center = (localAABB.minBounds + localAABB.maxBounds) * 0.5f;
        Vector3 extents = (localAABB.maxBounds - localAABB.minBounds) * 0.5f;

        Vector3 xAxis = transform.orientation * Vector3(1, 0, 0);
        Vector3 yAxis = transform.orientation * Vector3(0, 1, 0);
        Vector3 zAxis = transform.orientation * Vector3(0, 0, 1);

        Vector3 rotatedExtents(
            std::abs(xAxis.x) * extents.x + std::abs(yAxis.x) * extents.y + std::abs(zAxis.x) * extents.z,
            std::abs(xAxis.y) * extents.x + std::abs(yAxis.y) * extents.y + std::abs(zAxis.y) * extents.z,
            std::abs(xAxis.z) * extents.x + std::abs(yAxis.z) * extents.y + std::abs(zAxis.z) * extents.z
        );

        Vector3 worldCenter = transform.position + (transform.orientation * center);
        collision::AABB worldAABB(worldCenter - rotatedExtents, worldCenter + rotatedExtents);

        collider.treeNodeIndex = mTree.updateLeaf(collider.treeNodeIndex, worldAABB);
    }

    // Aşama 2: Kesişen sınır kutuları üzerinden potansiyel adayları belirle
    for (auto&& [entityA, colliderA] : std::views::zip(entities, colliders)) {
        if (colliderA.treeNodeIndex == collision::NULL_NODE) continue;

        const auto& aabbA = mTree.getNodeAABB(colliderA.treeNodeIndex);

        mTree.query(aabbA, [&](ecs::Entity entityB) {
            // Sadece bir yönde (A < B) kayıt yaparak çift tekrarını ve kendi kendine çarpışmayı önle
            if (entityA.id < entityB.id) {
                mPotentialCollisions.emplace_back(entityA, entityB);
            }
        });
    }
}

} // namespace Baryon::systems