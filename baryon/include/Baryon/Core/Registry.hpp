#pragma once

#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/memory/MemoryManager.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/Core/Snapshot.hpp"
#include <tuple>

/**
 * @file Registry.hpp
 * @brief Merkezi ECS (Varlık Bileşen Sistemi) Kayıt Defteri.
 * @details Motor içindeki tüm varlıkları (entity) ve onlara ait bileşenleri (component) 
 *          yöneten ana sistemdir. Tüm veriler burada güvenli ve performanslı bir şekilde saklanır.
 */
namespace Baryon::Core {

/**
 * @class Registry
 * @brief Tüm ECS veritabanını yöneten ana sınıf.
 * @details Varlıkların yaşam döngüsünü ve bileşenlerini kontrol eder. Ayrıca sahnenin 
 *          durumunu kaydetme (snapshot) ve geri yükleme özelliklerini barındırır.
 */
class Registry {
private:
    ecs::EntityManager mEntityManager; ///< Varlıkların (entity) kimliklerini ve yaşam döngüsünü yönetir.
    
    /**
     * @brief Motorun desteklediği tüm bileşenlerin tutulduğu veri depoları.
     */
    std::tuple<
        ecs::SparseSet<Pose>,
        ecs::SparseSet<Motion>,
        ecs::SparseSet<MassProps>,
        ecs::SparseSet<Material>,
        ecs::SparseSet<BodyState>,
        ecs::SparseSet<Linkage>,
        ecs::SparseSet<ecs::ColliderData>,
        ecs::SparseSet<collision::DistanceConstraint>,
        ecs::SparseSet<collision::RevoluteConstraint>
    > mComponents;

public:
    /**
     * @brief Kayıt defterini ve bileşen havuzlarını bellek yöneticisiyle başlatır.
     */
    explicit Registry(memory::MemoryManager& mm)
        : mEntityManager(mm),
          mComponents(
              ecs::SparseSet<Pose>(mm),
              ecs::SparseSet<Motion>(mm),
              ecs::SparseSet<MassProps>(mm),
              ecs::SparseSet<Material>(mm),
              ecs::SparseSet<BodyState>(mm),
              ecs::SparseSet<Linkage>(mm),
              ecs::SparseSet<ecs::ColliderData>(mm),
              ecs::SparseSet<collision::DistanceConstraint>(mm),
              ecs::SparseSet<collision::RevoluteConstraint>(mm)
          ) {}

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
     * @brief Bir varlığı ve ona bağlı olan tüm bileşenleri sistemden tamamen siler.
     * @param entity Yok edilecek varlık.
     */
    void destroyEntity(ecs::Entity entity) {
        if (mEntityManager.isAlive(entity)) {
            removeIfExists<Pose>(entity);
            removeIfExists<Motion>(entity);
            removeIfExists<MassProps>(entity);
            removeIfExists<Material>(entity);
            removeIfExists<BodyState>(entity);
            removeIfExists<Linkage>(entity);
            removeIfExists<ecs::ColliderData>(entity);
            removeIfExists<collision::DistanceConstraint>(entity);
            removeIfExists<collision::RevoluteConstraint>(entity);
            
            mEntityManager.destroyEntity(entity);
        }
    }

    /**
     * @brief Belirtilen varlığın sistemde hala var olup olmadığını (silinip silinmediğini) kontrol eder.
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
     * @details Geri sarma (rewind), tekrar oynatma veya oyun kaydetme mekanizmaları için kullanılır.
     * @return Kaydedilmiş sahne durumu (Snapshot).
     */
    [[nodiscard]] Snapshot save() const {
        Snapshot snap;
        snap.entityManagerState = mEntityManager.save();
        snap.poses = std::get<ecs::SparseSet<Pose>>(mComponents).save();
        snap.motions = std::get<ecs::SparseSet<Motion>>(mComponents).save();
        snap.massProps = std::get<ecs::SparseSet<MassProps>>(mComponents).save();
        snap.materials = std::get<ecs::SparseSet<Material>>(mComponents).save();
        snap.bodyStates = std::get<ecs::SparseSet<BodyState>>(mComponents).save();
        snap.linkages = std::get<ecs::SparseSet<Linkage>>(mComponents).save();
        snap.colliders = std::get<ecs::SparseSet<ecs::ColliderData>>(mComponents).save();
        snap.distanceConstraints = std::get<ecs::SparseSet<collision::DistanceConstraint>>(mComponents).save();
        snap.revoluteConstraints = std::get<ecs::SparseSet<collision::RevoluteConstraint>>(mComponents).save();
        return snap;
    }

    /**
     * @brief Daha önceden kaydedilmiş bir durumu (Snapshot) ECS'ye geri yükler.
     * @param snap Yüklenecek sahne durumu.
     */
    void load(const Snapshot& snap) {
        mEntityManager.load(snap.entityManagerState);
        std::get<ecs::SparseSet<Pose>>(mComponents).load(snap.poses);
        std::get<ecs::SparseSet<Motion>>(mComponents).load(snap.motions);
        std::get<ecs::SparseSet<MassProps>>(mComponents).load(snap.massProps);
        std::get<ecs::SparseSet<Material>>(mComponents).load(snap.materials);
        std::get<ecs::SparseSet<BodyState>>(mComponents).load(snap.bodyStates);
        std::get<ecs::SparseSet<Linkage>>(mComponents).load(snap.linkages);
        std::get<ecs::SparseSet<ecs::ColliderData>>(mComponents).load(snap.colliders);
        std::get<ecs::SparseSet<collision::DistanceConstraint>>(mComponents).load(snap.distanceConstraints);
        std::get<ecs::SparseSet<collision::RevoluteConstraint>>(mComponents).load(snap.revoluteConstraints);
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
    void addComponent(ecs::Entity entity, const T& component) {
        std::get<ecs::SparseSet<T>>(mComponents).addComponent(entity, component);
    }

    /**
     * @brief Bir varlıktan belirtilen bileşeni siler.
     * @tparam T Silinecek bileşenin türü.
     * @param entity İşlem yapılacak varlık.
     */
    template <typename T>
    void removeComponent(ecs::Entity entity) {
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
    template <typename T>
    [[nodiscard]] auto&& getComponent(this auto&& self, ecs::Entity entity) {
        return std::get<ecs::SparseSet<T>>(std::forward<decltype(self)>(self).mComponents).getData(entity);
    }

    /**
     * @brief Belirli bir bileşen türüne ait tüm verilerin tutulduğu havuza (veri dizisine) erişir.
     * @details Genellikle sistemlerin (örneğin fizik sisteminin) tüm ilgili varlıkları 
     *          tek bir döngüde hızlıca işlemesi için kullanılır.
     * @tparam T İstek yapılan bileşen türü.
     * @return Bileşen veri havuzuna referans.
     */
    template <typename T>
    [[nodiscard]] auto& getComponentPool() {
        return std::get<ecs::SparseSet<T>>(mComponents);
    }

    /// @}

private:
    /**
     * @brief Varlık silinirken içeride kalan parçaları temizleyen yardımcı fonksiyon.
     * @details Varlıkta bu bileşen varsa siler, yoksa bir şey yapmaz.
     */
    template <typename T>
    void removeIfExists(ecs::Entity entity) {
        if (hasComponent<T>(entity)) {
            removeComponent<T>(entity);
        }
    }
};

} // namespace Baryon::Core