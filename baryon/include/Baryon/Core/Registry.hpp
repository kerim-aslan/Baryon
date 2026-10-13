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

#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/Core/Snapshot.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/memory/MemoryManager.hpp"
#include <tuple>


/**
 * @file Registry.hpp
 * @brief Merkezi ECS (Varlık Bileşen Sistemi) Kayıt Defteri.
 * @details Motor içindeki tüm varlıkları (entity) ve onlara ait bileşenleri
 * (component) yöneten ana sistemdir. Tüm veriler burada güvenli ve performanslı
 * bir şekilde saklanır.
 */
namespace Baryon::Core {

/**
 * @class Registry
 * @brief Tüm ECS veritabanını yöneten ana sınıf.
 * @details Varlıkların yaşam döngüsünü ve bileşenlerini kontrol eder. Ayrıca
 * sahnenin durumunu kaydetme (snapshot) ve geri yükleme özelliklerini
 * barındırır.
 */
class Registry {
private:
  ecs::EntityManager mEntityManager; ///< Varlıkların (entity) kimliklerini ve
                                     ///< yaşam döngüsünü yönetir.

  template<typename Tuple>
  struct ComponentPoolsFromTypeList;

  template<typename... Comps>
  struct ComponentPoolsFromTypeList<std::tuple<Comps...>> {
      using type = std::tuple<ecs::SparseSet<Comps>...>;
  };

  using ComponentPoolsTuple = typename ComponentPoolsFromTypeList<ComponentTypeList>::type;
  ComponentPoolsTuple mComponents;

  template<typename... Comps>
  static auto makeComponentPools(memory::MemoryManager &mm, std::type_identity<std::tuple<Comps...>>) {
      return std::make_tuple(ecs::SparseSet<Comps>(mm)...);
  }

public:
  /**
   * @brief Kayıt defterini ve bileşen havuzlarını bellek yöneticisiyle
   * başlatır.
   */
  explicit Registry(memory::MemoryManager &mm)
      : mEntityManager(mm),
        mComponents(makeComponentPools(mm, std::type_identity<ComponentTypeList>{})) {}

  /// @name Varlık (Entity) Yönetimi
  /// @{

  /**
   * @brief Sahneye yeni, boş bir varlık (entity) ekler.
   * @return Oluşturulan yeni varlığın kimliği.
   */
  [[nodiscard]] ecs::Entity createEntity() {
    return mEntityManager.createEntity();
  }

  /**
   * @brief Bir varlığı ve ona bağlı olan tüm bileşenleri sistemden tamamen
   * siler (B1: Tek merkezli bileşen listesinden fold expression ile temizlenir).
   * @param entity Yok edilecek varlık.
   */
  void destroyEntity(ecs::Entity entity) {
    if (mEntityManager.isAlive(entity)) {
      [&]<typename... Comps>(std::type_identity<std::tuple<Comps...>>) {
        (removeIfExists<Comps>(entity), ...);
      }(std::type_identity<ComponentTypeList>{});

      mEntityManager.destroyEntity(entity);
    }
  }

  /**
   * @brief Belirtilen varlığın sistemde hala var olup olmadığını (silinip
   * silinmediğini) kontrol eder.
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
   * @brief Sahnenin o anki tam durumunu kopyalar ve kaydeder (B1: Variadic fold expression).
   * @details Geri sarma (rewind), tekrar oynatma veya oyun kaydetme
   * mekanizmaları için kullanılır.
   * @return Kaydedilmiş sahne durumu (Snapshot).
   */
  [[nodiscard]] Snapshot save() const {
    Snapshot snap;
    snap.entityManagerState = mEntityManager.save();
    [&]<typename... Comps>(std::type_identity<std::tuple<Comps...>>) {
      ((std::get<typename ecs::SparseSet<Comps>::State>(snap.componentStates) =
          std::get<ecs::SparseSet<Comps>>(mComponents).save()), ...);
    }(std::type_identity<ComponentTypeList>{});
    return snap;
  }

  /**
   * @brief Daha önceden kaydedilmiş bir durumu (Snapshot) ECS'ye geri yükler (B1: Variadic fold expression).
   * @param snap Yüklenecek sahne durumu.
   */
  void load(const Snapshot &snap) {
    mEntityManager.load(snap.entityManagerState);
    [&]<typename... Comps>(std::type_identity<std::tuple<Comps...>>) {
      (std::get<ecs::SparseSet<Comps>>(mComponents).load(
          std::get<typename ecs::SparseSet<Comps>::State>(snap.componentStates)), ...);
    }(std::type_identity<ComponentTypeList>{});
  }

  /// @}

  /// @name Bileşen (Component) Erişim Fonksiyonları
  /// @{

  /**
   * @brief Bir varlığa yeni bir bileşen ekler.
   * @tparam T Eklenecek bileşenin türü (örn. Pose, Motion).
   * @param entity Bileşenin ekleneceği varlık.
   * @param component Eklenecek bileşen verisi.
   */
  template <typename T>
  void addComponent(ecs::Entity entity, const T &component) {
    std::get<ecs::SparseSet<T>>(mComponents).addComponent(entity, component);
  }

  /**
   * @brief Bir varlıktan belirtilen bileşeni siler.
   * @tparam T Silinecek bileşenin türü.
   * @param entity İşlem yapılacak varlık.
   */
  template <typename T> void removeComponent(ecs::Entity entity) {
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
    return std::get<ecs::SparseSet<T>>(mComponents).hasComponent(entity);
  }

  /**
   * @brief Varlığın sahip olduğu bileşenin verisine doğrudan erişim sağlar.
   * @tparam T Erişilecek bileşenin türü.
   * @param entity Erişilecek varlık.
   * @return İlgili bileşene ait veri referansı.
   */
  template <typename T> [[nodiscard]] T &getComponent(ecs::Entity entity) {
    return std::get<ecs::SparseSet<T>>(mComponents).getData(entity);
  }

  template <typename T>
  [[nodiscard]] const T &getComponent(ecs::Entity entity) const {
    return std::get<ecs::SparseSet<T>>(mComponents).getData(entity);
  }

  /**
   * @brief Belirli bir bileşen türüne ait tüm verilerin tutulduğu havuza (veri
   * dizisine) erişir.
   * @details Genellikle sistemlerin (örneğin fizik sisteminin) tüm ilgili
   * varlıkları tek bir döngüde hızlıca işlemesi için kullanılır.
   * @tparam T İstek yapılan bileşen türü.
   * @return Bileşen veri havuzuna referans.
   */
  template <typename T> [[nodiscard]] auto &getComponentPool() {
    return std::get<ecs::SparseSet<T>>(mComponents);
  }

  template <typename T> [[nodiscard]] const auto &getComponentPool() const {
    return std::get<ecs::SparseSet<T>>(mComponents);
  }

  /// @}

private:
  /**
   * @brief Varlık silinirken içeride kalan parçaları temizleyen yardımcı
   * fonksiyon.
   * @details Varlıkta bu bileşen varsa siler, yoksa bir şey yapmaz.
   */
  template <typename T> void removeIfExists(ecs::Entity entity) {
    if (hasComponent<T>(entity)) {
      removeComponent<T>(entity);
    }
  }
};

} // namespace Baryon::Core