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