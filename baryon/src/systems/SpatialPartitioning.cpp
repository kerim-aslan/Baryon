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
 * @file SpatialPartitioning.cpp
 * @brief Uzaysal Bölümleme (Geniş Faz) sistemi uygulaması.
 * @details Dinamik AABB Ağacı kullanarak nesneleri uzayda organize eder ve birbirine 
 *          temas etme ihtimali olan aday çiftleri belirleyerek performans sağlar.
 */

#include "Baryon/systems/SpatialPartitioning.hpp"
#include "Baryon/Core/CoreComponents.hpp"
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

    // Hareket eden nesneler için hız süpürmesi (Swept AABB): Hızlı nesnelerin broad-phase'de atlanmasını engeller
    if (mRegistry.hasComponent<Core::Motion>(entity)) {
        const auto& motion = mRegistry.getComponent<Core::Motion>(entity);
        Vector3 disp = motion.linearVelocity * 0.02f;
        if (disp.x > 0.0f) worldAABB.maxBounds.x += disp.x; else worldAABB.minBounds.x += disp.x;
        if (disp.y > 0.0f) worldAABB.maxBounds.y += disp.y; else worldAABB.minBounds.y += disp.y;
        if (disp.z > 0.0f) worldAABB.maxBounds.z += disp.z; else worldAABB.minBounds.z += disp.z;
    }

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

        if (mRegistry.hasComponent<Core::Motion>(entity)) {
            const auto& motion = mRegistry.getComponent<Core::Motion>(entity);
            Vector3 disp = motion.linearVelocity * 0.02f;
            if (disp.x > 0.0f) worldAABB.maxBounds.x += disp.x; else worldAABB.minBounds.x += disp.x;
            if (disp.y > 0.0f) worldAABB.maxBounds.y += disp.y; else worldAABB.minBounds.y += disp.y;
            if (disp.z > 0.0f) worldAABB.maxBounds.z += disp.z; else worldAABB.minBounds.z += disp.z;
        }

        collider.treeNodeIndex = mTree.updateLeaf(collider.treeNodeIndex, worldAABB);
    }

    // Aşama 2: Kesişen sınır kutuları üzerinden potansiyel adayları belirle
    // OPTİMİZASYON: Yalnızca hareket eden / uyanık dinamik nesneler sorgu başlatır!
    // Statik zemin veya uyuyan bloklar gereksiz yere yüzlerce ağaç araması yapmaz.
    for (auto&& [entityA, colliderA] : std::views::zip(entities, colliders)) {
        if (colliderA.treeNodeIndex == collision::NULL_NODE) continue;

        bool isA_DynamicAndAwake = true;
        if (mRegistry.hasComponent<Core::BodyState>(entityA)) {
            const auto& stA = mRegistry.getComponent<Core::BodyState>(entityA);
            if (stA.type == Core::BodyType::Static || stA.isSleeping) {
                isA_DynamicAndAwake = false;
            }
        }
        if (!isA_DynamicAndAwake) continue;

        const auto& aabbA = mTree.getNodeAABB(colliderA.treeNodeIndex);

        mTree.query(aabbA, [&](ecs::Entity entityB) {
            if (entityA.id == entityB.id) return;

            if (!mRegistry.hasComponent<ecs::ColliderData>(entityB)) return;
            const auto& colliderB = mRegistry.getComponent<ecs::ColliderData>(entityB);

            // Çarpışma Filtreleme (Bitmask): İki nesne birbirinin maskesinde yoksa çifti ele!
            if ((colliderA.layer & colliderB.mask) == 0 || (colliderB.layer & colliderA.mask) == 0) {
                return;
            }

            bool isB_Static = false;
            bool isB_Sleeping = false;
            if (mRegistry.hasComponent<Core::BodyState>(entityB)) {
                const auto& stB = mRegistry.getComponent<Core::BodyState>(entityB);
                isB_Static = (stB.type == Core::BodyType::Static);
                isB_Sleeping = stB.isSleeping;
            }

            if (isB_Static || isB_Sleeping) {
                mPotentialCollisions.emplace_back(entityA, entityB);
            } else {
                if (entityA.id < entityB.id) {
                    mPotentialCollisions.emplace_back(entityA, entityB);
                }
            }
        });
    }
}

} // namespace Baryon::systems