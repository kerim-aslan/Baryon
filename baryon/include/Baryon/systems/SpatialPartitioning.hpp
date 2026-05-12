#pragma once

#include "Baryon/Core/Registry.hpp"
#include "Baryon/collision/DynamicAABBTree.hpp"
#include <vector>
#include <utility>

/**
 * @file SpatialPartitioning.hpp
 * @brief Uzaysal Bölümleme ve Geniş Faz (Broad Phase) Çarpışma Sistemi.
 * @details Sahnedeki nesneleri Dinamik AABB Ağacı kullanarak uzaysal olarak gruplandırır. 
 *          Sistem, birbirine çarpma ihtimali olan "aday" çiftleri belirleyerek, 
 *          gereksiz dar faz hesaplamalarının önüne geçer ve performansı artırır.
 */
namespace Baryon::systems {

/**
 * @class SpatialPartitioning
 * @brief AABB ağaç yapısı tabanlı geniş faz çarpışma algılama ve yönetim sistemi.
 */
class SpatialPartitioning {
private:
    Core::Registry& mRegistry;                                     ///< ECS kayıt defteri referansı.
    collision::DynamicAABBTree mTree;                               ///< Uzaydaki nesneleri tutan dinamik AABB ağacı.
    std::vector<std::pair<ecs::Entity, ecs::Entity>> mPotentialCollisions; ///< Gerçek çarpışma testi yapılacak aday çiftlerin listesi.

public:
    /**
     * @brief Uzaysal bölümleme sistemini başlatır.
     * @param registry ECS kayıt defteri.
     * @param mm Bellek yöneticisi.
     */
    SpatialPartitioning(Core::Registry& registry, memory::MemoryManager& mm);

    /**
     * @brief Belirtilen varlığı fiziksel sınırlarıyla birlikte AABB ağacına ekler.
     * @details Varlığın mevcut konumu ve rotasyonuna göre dünya uzayındaki sınır kutusu (AABB) hesaplanır.
     * @param entity Ağaca eklenecek varlık.
     */
    void addEntityToTree(ecs::Entity entity);
    
    /**
     * @brief Hareket eden bir varlığın ağaçtaki sınır kutusunu (AABB) günceller.
     * @param entity Konumu veya dönüşü değişen varlık.
     */
    void updateEntityInTree(ecs::Entity entity);
    
    /**
     * @brief Bir varlığı ve ona ait sınır kutusunu ağaçtan tamamen kaldırır.
     * @param entity Sistemden çıkarılacak varlık.
     */
    void removeEntityFromTree(ecs::Entity entity);

    /**
     * @brief Mevcut ağacı tamamen boşaltıp, kayıtlı varlıklarla yeniden inşa eder.
     * @details Genellikle sahne geri yükleme (snapshot restore) işlemlerinden sonra kullanılır.
     */
    void rebuildTree();

    /**
     * @brief Her fizik adımında AABB'leri günceller ve olası çarpışma çiftlerini bulur.
     * @details 
     * 1. Hareket eden tüm nesnelerin sınır kutularını ağaç üzerinde günceller.
     * 2. Ağaç sorgusu yaparak birbirine yakın (sınır kutuları kesişen) nesne çiftlerini tespit eder.
     * 3. Mükerrer çift oluşumunu engellemek için kimlik (ID) bazlı sıralama filtresi uygular.
     */
    void step();

    /** 
     * @brief Dar faz (Narrow Phase) sistemine gönderilecek olan potansiyel çarpışma adaylarını döndürür. 
     */
    [[nodiscard]] const auto& getPotentialCollisions() const { 
        return mPotentialCollisions; 
    }
    
    /** 
     * @brief Işın fırlatma (raycast) ve alan sorguları için AABB ağacına erişim sağlar. 
     */
    [[nodiscard]] const collision::DynamicAABBTree& getTree() const { return mTree; }
};

} // namespace Baryon::systems