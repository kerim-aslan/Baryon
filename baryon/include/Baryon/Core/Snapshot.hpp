#pragma once

#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"

/**
 * @file Snapshot.hpp
 * @brief Fizik motorunun o anki tam durumunu kaydeden Anlık Görüntü (Snapshot) yapısı.
 * @details Sahnedeki her şeyin bir kopyasını tek bir pakette toplayarak oyunu zaman 
 *          içinde geri sarma (rewind), tekrar izleme (replay), ağ üzerinden çok oyunculu 
 *          senkronizasyon yapma veya hamle geri alma (undo) gibi sistemler kurmanızı sağlar.
 */
namespace Baryon::Core {

/**
 * @struct Snapshot
 * @brief Fizik dünyasındaki tüm varlıkların ve onlara ait bileşenlerin o anki kopyasını tutan veri yapısı.
 * @details Registry sınıfındaki save() fonksiyonu ile doldurulur ve load() fonksiyonu 
 *          ile sahneyi tam olarak bu kayıtlı anın durumuna geri döndürmek için kullanılır.
 */
struct Snapshot {
    ecs::EntityManager::State entityManagerState; ///< Varlıkların durumu (hangi varlıklar aktif, hangileri silindi).
    
    ecs::SparseSet<Pose>::State poses;                                      ///< Tüm cisimlerin konum ve dönüş (rotasyon) verileri.
    ecs::SparseSet<Motion>::State motions;                                  ///< Tüm cisimlerin anlık hız ve uygulanan kuvvet verileri.
    ecs::SparseSet<MassProps>::State massProps;                             ///< Kütle ve fiziksel dönüş direnci (eylemsizlik) verileri.
    ecs::SparseSet<Material>::State materials;                              ///< Sürtünme ve sekme gibi yüzey materyal özellikleri.
    ecs::SparseSet<BodyState>::State bodyStates;                            ///< Cisimlerin aktiflik, uyku ve genel fiziksel durumları.
    ecs::SparseSet<Linkage>::State linkages;                                ///< Ebeveyn-çocuk (parent-child) hiyerarşi bağlantıları.
    ecs::SparseSet<ecs::ColliderData>::State colliders;                     ///< Çarpışma şekilleri ve sınır kutularının (collider) verileri.
    ecs::SparseSet<collision::DistanceConstraint>::State distanceConstraints; ///< Yay, ip veya halat gibi mesafe koruyucu eklemlerin verileri.
    ecs::SparseSet<collision::RevoluteConstraint>::State revoluteConstraints; ///< Menteşe, tekerlek aksı gibi dönüş kısıtlayıcı eklemlerin verileri.
};

} // namespace Baryon::Core