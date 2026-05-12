#pragma once

#include "EntityManager.hpp"
#include "../collision/CollisionShape.hpp"
#include "../memory/MemoryManager.hpp"
#include <vector>
#include <cassert>

/**
 * @file ColliderComponents.hpp
 * @brief Çarpışma şekillerini (Collider) tutan veri yapıları.
 * @details Cisimlerin oyun dünyasındaki fiziksel sınırlarını ve tetikleyici (trigger) 
 *          özelliklerini yöneten, yüksek performanslı veri deposudur.
 */
namespace Baryon::ecs {

/**
 * @struct ColliderData
 * @brief Bir varlığın çarpışma özelliklerini tutan veri paketi.
 * @details Cismin hangi geometrik şekle sahip olduğunu ve fiziksel tepki verip 
 *          vermeyeceğini belirler.
 */
struct ColliderData {
    collision::CollisionShape shape; ///< Çarpışma şekli (küre, kutu, kapsül vb.).

    /**
     * @brief Fizik motorunun uzaysal arama ağacındaki konumu.
     * @details Motorun iç kullanımı içindir. -1 değeri henüz sahneye eklenmediğini gösterir.
     */
    int32_t treeNodeIndex{-1};

    /**
     * @brief Cisim bir tetikleyici (hayalet) mi?
     * @details true yapılırsa, cisim diğer cisimlere çarpıp onları itmez veya zıplatmaz. 
     *          Sadece içinden geçildiğinde sisteme haber verir (Örn: Lazer kapılar, bitiş çizgileri).
     */
    bool isTrigger{false}; 

    /**
     * @brief Varsayılan olarak 1x1x1 boyutlarında standart bir kutu çarpıştırıcı oluşturur.
     */
    ColliderData() : shape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))) {}

    /**
     * @brief Belirtilen özel bir şekil ile çarpıştırıcı oluşturur.
     * @param s Kullanılmak istenen çarpışma şekli.
     */
    explicit ColliderData(const collision::CollisionShape& s) : shape(s) {}
};

/**
 * @class ColliderComponents
 * @brief Sahnedeki tüm çarpışma bileşenlerini yüksek performansla yöneten veri deposu.
 * @details Bellek dostu dizilimi sayesinde binlerce cismin çarpışma testlerine 
 *          çok hızlı bir şekilde girmesini sağlar.
 */
class ColliderComponents {
private:
    std::pmr::vector<uint32_t> mSparseMap;        ///< Varlık kimliğinden gerçek veri sırasına yönlendirme haritası.
    std::pmr::vector<Entity> mEntities;           ///< Veri sırasından varlık kimliğine dönüşüm listesi.
    std::pmr::vector<ColliderData> mColliderData; ///< Çarpışma verilerinin bellekte ardışık tutulduğu ana dizi.

    static constexpr uint32_t MAX_ENTITIES = 10000;
    static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;

public:
    /** 
     * @brief Veri deposunu başlatır. 
     * @param memoryManager Bellek tahsisleri için kullanılacak yönetici. 
     */
    explicit ColliderComponents(memory::MemoryManager& memoryManager)
        : mSparseMap(MAX_ENTITIES, INVALID_INDEX, memoryManager.getPoolResource()),
          mEntities(memoryManager.getPoolResource()),
          mColliderData(memoryManager.getPoolResource()) {}

    /** 
     * @brief Bir varlığa (entity) yeni bir çarpışma bileşeni ekler.
     * @param entity Bileşenin ekleneceği varlık.
     * @param data Eklenecek çarpışma verisi.
     */
    void addComponent(Entity entity, const ColliderData& data) {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        uint32_t denseIndex = static_cast<uint32_t>(mColliderData.size());
        mColliderData.push_back(data);
        mEntities.push_back(entity);
        mSparseMap[entityIndex] = denseIndex;
    }

    /** 
     * @brief Varlığın çarpışma bileşenini sistemden siler.
     * @param entity Çarpışma özelliği kaldırılacak varlık.
     */
    void removeComponent(Entity entity) {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        uint32_t indexOfRemoved = mSparseMap[entityIndex];
        uint32_t lastDenseIndex = static_cast<uint32_t>(mColliderData.size() - 1);
        
        // Hızlı silme işlemi: Silinen verinin yerine dizideki en son veriyi taşı
        if (indexOfRemoved != lastDenseIndex) {
            Entity lastEntity = mEntities[lastDenseIndex];
            mColliderData[indexOfRemoved] = mColliderData[lastDenseIndex];
            mEntities[indexOfRemoved] = lastEntity;
            mSparseMap[EntityManager::getIndex(lastEntity)] = indexOfRemoved;
        }
        mColliderData.pop_back();
        mEntities.pop_back();
        mSparseMap[entityIndex] = INVALID_INDEX;
    }

    /** 
     * @brief Varlığın bir çarpışma bileşenine sahip olup olmadığını kontrol eder. 
     * @return Çarpışma özelliği varsa true.
     */
    [[nodiscard]] bool hasComponent(Entity entity) const {
        return mSparseMap[EntityManager::getIndex(entity)] != INVALID_INDEX;
    }

    /** 
     * @brief Varlığın çarpışma verisine doğrudan erişim sağlar. 
     * @return Çarpışma verisi referansı.
     */
    [[nodiscard]] auto&& getData(this auto&& self, Entity entity) {
        uint32_t denseIndex = self.mSparseMap[EntityManager::getIndex(entity)];
        return std::forward<decltype(self)>(self).mColliderData[denseIndex];
    }

    /** 
     * @brief Sistemdeki tüm çarpışma verilerinin tutulduğu diziye toplu erişim sağlar. 
     */
    [[nodiscard]] std::pmr::vector<ColliderData>& getAllData() { return mColliderData; }
    
    /** 
     * @brief Sistemdeki, çarpışma özelliği olan tüm varlıkların kimlik listesine erişim sağlar. 
     */
    [[nodiscard]] std::pmr::vector<Entity>& getAllEntities() { return mEntities; }
};

} // namespace Baryon::ecs