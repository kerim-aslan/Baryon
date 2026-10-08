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

#pragma once

#include "../collision/CollisionShape.hpp"
#include "../memory/MemoryManager.hpp"
#include "EntityManager.hpp"
#include <cassert>
#include <vector>


/**
 * @file ColliderComponents.hpp
 * @brief Çarpışma şekillerini (Collider) tutan veri yapıları.
 * @details Cisimlerin oyun dünyasındaki fiziksel sınırlarını ve tetikleyici
 * (trigger) özelliklerini yöneten, yüksek performanslı veri deposudur.
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
   * @details Motorun iç kullanımı içindir. -1 değeri henüz sahneye
   * eklenmediğini gösterir.
   */
  int32_t treeNodeIndex{-1};

  /**
   * @brief Cisim bir tetikleyici (hayalet) mi?
   * @details true yapılırsa, cisim diğer cisimlere çarpıp onları itmez veya
   * zıplatmaz. Sadece içinden geçildiğinde sisteme haber verir (Örn: Lazer
   * kapılar, bitiş çizgileri).
   */
  bool isTrigger{false};

  uint32_t layer{0x0001}; ///< Bu nesnenin ait olduğu çarpışma katmanı (varsayılan: 1).
  uint32_t mask{0xFFFF};  ///< Bu nesnenin çarpışabileceği katmanlar maskesi (varsayılan: hepsi).

  /**
   * @brief Varsayılan olarak 1x1x1 boyutlarında standart bir kutu çarpıştırıcı
   * oluşturur.
   */
  ColliderData() : shape(collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))) {}

  /**
   * @brief Belirtilen özel bir şekil ile çarpıştırıcı oluşturur.
   * @param s Kullanılmak istenen çarpışma şekli.
   */
  explicit ColliderData(const collision::CollisionShape &s) : shape(s) {}
};

/**
 * @class ColliderComponents
 * @brief Sahnedeki tüm çarpışma bileşenlerini yüksek performansla yöneten veri
 * deposu.
 * @details Bellek dostu dizilimi sayesinde binlerce cismin çarpışma testlerine
 *          çok hızlı bir şekilde girmesini sağlar.
 */
class ColliderComponents {
private:
  std::pmr::vector<uint32_t> mSparseMap; ///< Varlık kimliğinden gerçek veri
                                         ///< sırasına yönlendirme haritası.
  std::pmr::vector<Entity>
      mEntities; ///< Veri sırasından varlık kimliğine dönüşüm listesi.
  std::pmr::vector<ColliderData>
      mColliderData; ///< Çarpışma verilerinin bellekte ardışık tutulduğu ana
                     ///< dizi.

  static constexpr uint32_t INITIAL_CAPACITY = 1024;
  static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;

public:
  /**
   * @brief Veri deposunu başlatır.
   * @param memoryManager Bellek tahsisleri için kullanılacak yönetici.
   */
  explicit ColliderComponents(memory::MemoryManager &memoryManager)
      : mSparseMap(INITIAL_CAPACITY, INVALID_INDEX,
                   memoryManager.getPoolResource()),
        mEntities(memoryManager.getPoolResource()),
        mColliderData(memoryManager.getPoolResource()) {}

  /**
   * @brief Bir varlığa (entity) yeni bir çarpışma bileşeni ekler.
   * @param entity Bileşenin ekleneceği varlık.
   * @param data Eklenecek çarpışma verisi.
   */
  void addComponent(Entity entity, const ColliderData &data) {
    uint32_t entityIndex = EntityManager::getIndex(entity);
    if (entityIndex >= mSparseMap.size()) {
      mSparseMap.resize(
          std::max(entityIndex + 1, static_cast<uint32_t>(mSparseMap.size() * 2)),
          INVALID_INDEX);
    }
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
    assert(entityIndex < mSparseMap.size());
    uint32_t indexOfRemoved = mSparseMap[entityIndex];
    assert(indexOfRemoved != INVALID_INDEX);
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
    uint32_t entityIndex = EntityManager::getIndex(entity);
    return entityIndex < mSparseMap.size() &&
           mSparseMap[entityIndex] != INVALID_INDEX;
  }

  /**
   * @brief Varlığın çarpışma verisine doğrudan erişim sağlar.
   * @return Çarpışma verisi referansı.
   */
  [[nodiscard]] ColliderData &getData(Entity entity) {
    uint32_t denseIndex = mSparseMap[EntityManager::getIndex(entity)];
    return mColliderData[denseIndex];
  }

  [[nodiscard]] const ColliderData &getData(Entity entity) const {
    uint32_t denseIndex = mSparseMap[EntityManager::getIndex(entity)];
    return mColliderData[denseIndex];
  }

  /**
   * @brief Sistemdeki tüm çarpışma verilerinin tutulduğu diziye toplu erişim
   * sağlar.
   */
  [[nodiscard]] std::pmr::vector<ColliderData> &getAllData() {
    return mColliderData;
  }

  /**
   * @brief Sistemdeki, çarpışma özelliği olan tüm varlıkların kimlik listesine
   * erişim sağlar.
   */
  [[nodiscard]] std::pmr::vector<Entity> &getAllEntities() { return mEntities; }
};

} // namespace Baryon::ecs