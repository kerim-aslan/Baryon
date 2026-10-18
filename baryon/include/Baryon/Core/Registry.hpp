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

#include "Baryon/Core/ComponentTypes.hpp"
#include "Baryon/Core/Snapshot.hpp"
#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/memory/MemoryManager.hpp"
#include <tuple>
#include <type_traits>

/**
 * @file Registry.hpp
 * @brief Merkezi ECS (Varlık Bileşen Sistemi) Kayıt Defteri.
 * @details Motor içindeki tüm varlıkları (entity) ve onlara ait bileşenleri
 * (component) yöneten ana sistemdir. Tüm veriler burada güvenli ve performanslı
 * bir şekilde saklanır. Tek bir ComponentTypes listesi üzerinden tip güvenli ve
 * otomatik genişletilebilir şekilde çalışır.
 */
namespace Baryon::Core {

template <typename Tuple>
struct MapToSparseSet;

template <typename... Ts>
struct MapToSparseSet<std::tuple<Ts...>> {
  using type = std::tuple<ecs::SparseSet<Ts>...>;
};

/**
 * @class BasicRegistry
 * @brief Belirtilen bileşen listesiyle ECS veritabanını yöneten şablon sınıf.
 * @details Varlıkların yaşam döngüsünü ve bileşenlerini kontrol eder. Ayrıca
 * sahnenin durumunu kaydetme (snapshot) ve geri yükleme özelliklerini barındırır.
 * Bileşen ekleme, çıkarma, temizleme ve snapshot işlemleri tip listesi üzerinden
 * otomatik yürütülür.
 */
template <typename ComponentsTuple = ComponentTypes>
class BasicRegistry {
public:
  using ComponentsList = ComponentsTuple;
  using SnapshotType = BasicSnapshot<ComponentsTuple>;

  template <typename T>
  static constexpr bool isRegistered = TupleContains<T, ComponentsTuple>::value;

private:
  ecs::EntityManager mEntityManager; ///< Varlıkların (entity) kimliklerini ve yaşam döngüsünü yönetir.
  typename MapToSparseSet<ComponentsTuple>::type mComponents; ///< Tip listesinden otomatik oluşturulan bileşen depoları.

  template <typename... Ts>
  static auto initComponentPools(memory::MemoryManager &mm, std::type_identity<std::tuple<Ts...>>) {
    return std::tuple<ecs::SparseSet<Ts>...>(ecs::SparseSet<Ts>(mm)...);
  }

public:
  /**
   * @brief Kayıt defterini ve bileşen havuzlarını bellek yöneticisiyle başlatır.
   */
  explicit BasicRegistry(memory::MemoryManager &mm)
      : mEntityManager(mm),
        mComponents(initComponentPools(mm, std::type_identity<ComponentsTuple>{})) {}

  /// @name Varlık (Entity) Yönetimi
  /// @{

  [[nodiscard]] std::expected<ecs::Entity, PhysicsError> createEntity() {
    return mEntityManager.createEntity();
  }

  void setMaxCapacity(uint32_t maxCapacity) {
    mEntityManager.setMaxCapacity(maxCapacity);
  }

  [[nodiscard]] uint32_t getMaxCapacity() const {
    return mEntityManager.getMaxCapacity();
  }


  /**
   * @brief Bir varlığı ve ona bağlı olan tüm bileşenleri sistemden tamamen siler.
   * @details ComponentTypes listesindeki tüm bileşen havuzları fold expression ile otomatik taranır.
   * @param entity Yok edilecek varlık.
   */
  void destroyEntity(ecs::Entity entity) {
    if (mEntityManager.isAlive(entity)) {
      auto removeAll = [&]<typename... Ts>(std::type_identity<std::tuple<Ts...>>) {
        (removeIfExists<Ts>(entity), ...);
      };
      removeAll(std::type_identity<ComponentsTuple>{});
      mEntityManager.destroyEntity(entity);
    }
  }

  /**
   * @brief Belirtilen varlığın sistemde hala var olup olmadığını kontrol eder.
   * @param entity Kontrol edilecek varlık.
   * @return Varlık yaşıyorsa true, aksi halde false.
   */
  [[nodiscard]] bool isAlive(ecs::Entity entity) const {
    return mEntityManager.isAlive(entity);
  }

  /// @}

  /// @name Durum Kaydı (Snapshot) Sistemi
  /// @{

  /**
   * @brief Sahnenin o anki tam durumunu kopyalar ve kaydeder.
   * @details Tüm bileşen havuzları tip listesi üzerinden otomatik olarak kaydedilir.
   * @return Kaydedilmiş sahne durumu (Snapshot).
   */
  [[nodiscard]] SnapshotType save() const {
    SnapshotType snap;
    snap.entityManagerState = mEntityManager.save();
    auto saveAll = [&]<typename... Ts>(std::type_identity<std::tuple<Ts...>>) {
      snap.componentStates = std::make_tuple(
          std::get<ecs::SparseSet<Ts>>(mComponents).save()...
      );
    };
    saveAll(std::type_identity<ComponentsTuple>{});
    return snap;
  }

  /**
   * @brief Daha önceden kaydedilmiş bir durumu (Snapshot) ECS'ye geri yükler.
   * @param snap Yüklenecek sahne durumu.
   */
  void load(const SnapshotType &snap) {
    mEntityManager.load(snap.entityManagerState);
    auto loadAll = [&]<typename... Ts>(std::type_identity<std::tuple<Ts...>>) {
      (std::get<ecs::SparseSet<Ts>>(mComponents).load(
          std::get<typename ecs::SparseSet<Ts>::State>(snap.componentStates)), ...);
    };
    loadAll(std::type_identity<ComponentsTuple>{});
  }

  /// @}

  /// @name Bileşen (Component) Erişim Fonksiyonları
  /// @{

  /**
   * @brief Bir varlığa yeni bir bileşen ekler.
   * @tparam T Eklenecek bileşenin türü.
   * @param entity Bileşenin ekleneceği varlık.
   * @param component Eklenecek bileşen verisi.
   */
  template <typename T>
  void addComponent(ecs::Entity entity, const T &component) {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    std::get<ecs::SparseSet<T>>(mComponents).addComponent(entity, component);
  }

  /**
   * @brief Bir varlıktan belirtilen bileşeni siler.
   * @tparam T Silinecek bileşenin türü.
   * @param entity İşlem yapılacak varlık.
   */
  template <typename T> void removeComponent(ecs::Entity entity) {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    std::get<ecs::SparseSet<T>>(mComponents).removeComponent(entity);
  }

  /**
   * @brief Varlığın belirtilen bileşene sahip olup olmadığını kontrol eder.
   * @tparam T Kontrol edilecek bileşen türü.
   * @param entity Kontrol edilecek varlık.
   * @return Bileşene sahipse true, aksi halde false.
   */
  template <typename T>
  [[nodiscard]] bool hasComponent(ecs::Entity entity) const {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    return std::get<ecs::SparseSet<T>>(mComponents).hasComponent(entity);
  }

  /**
   * @brief Varlığın sahip olduğu bileşenin verisine doğrudan erişim sağlar.
   * @tparam T Erişilecek bileşenin türü.
   * @param entity Erişilecek varlık.
   * @return İlgili bileşene ait veri referansı.
   */
  template <typename T> [[nodiscard]] T &getComponent(ecs::Entity entity) {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    return std::get<ecs::SparseSet<T>>(mComponents).getData(entity);
  }

  template <typename T>
  [[nodiscard]] const T &getComponent(ecs::Entity entity) const {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    return std::get<ecs::SparseSet<T>>(mComponents).getData(entity);
  }

  /**
   * @brief Belirli bir bileşen türüne ait tüm verilerin tutulduğu havuza (veri dizisine) erişir.
   * @tparam T İstek yapılan bileşen türü.
   * @return Bileşen veri havuzuna referans.
   */
  template <typename T> [[nodiscard]] auto &getComponentPool() {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    return std::get<ecs::SparseSet<T>>(mComponents);
  }

  template <typename T> [[nodiscard]] const auto &getComponentPool() const {
    static_assert(isRegistered<T>, "Component type is not in registered ComponentTypes list!");
    return std::get<ecs::SparseSet<T>>(mComponents);
  }

  /// @}

private:
  /**
   * @brief Varlık silinirken bileşen varsa kaldıran yardımcı fonksiyon.
   */
  template <typename T> void removeIfExists(ecs::Entity entity) {
    if (hasComponent<T>(entity)) {
      removeComponent<T>(entity);
    }
  }
};

using Registry = BasicRegistry<ComponentTypes>;

} // namespace Baryon::Core