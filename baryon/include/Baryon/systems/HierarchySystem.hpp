#pragma once

#include "Baryon/Core/Registry.hpp"

/**
 * @file HierarchySystem.hpp
 * @brief Nesneler arası ebeveyn-çocuk (parent-child) ilişkilerini yöneten sistem.
 * @details Birbirine bağlı nesnelerin (Linkage) dünya koordinatlarını, ebeveynlerinin 
 *          konum ve rotasyonuna göre otomatik olarak günceller.
 */
namespace Baryon::systems {

/**
 * @class HierarchySystem
 * @brief Hiyerarşik dönüşüm ve koordinat güncelleme sistemi.
 */
class HierarchySystem {
private:
    Core::Registry& mRegistry; ///< ECS kayıt defteri referansı.

public:
    /** 
     * @brief Sistemi başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit HierarchySystem(Core::Registry& registry);

    /**
     * @brief Tüm hiyerarşik bağlantıları ve dünya konumlarını günceller.
     * @details Nesneleri ebeveynden çocuğa doğru mantıksal bir sıra ile tarar. 
     *          Her bir çocuk nesnenin dünya üzerindeki yeni konumunu ve bakış yönünü, 
     *          bağlı olduğu ebeveynin hareketine göre yeniden hesaplar.
     */
    void step();
};

} // namespace Baryon::systems