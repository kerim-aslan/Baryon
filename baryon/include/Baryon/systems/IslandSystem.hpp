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
#include "Baryon/collision/ContactManifold.hpp"
#include <vector>
#include <unordered_map>

/**
 * @file IslandSystem.hpp
 * @brief Fiziksel Ada (Island) tespiti ve uyku (sleep) yönetim sistemi.
 * @details Birbirine temas eden nesneleri bağımsız gruplara (adalar) ayırarak 
 *          simülasyonu optimize eder. Hareketsiz kalan adaları "uyku" moduna 
 *          alarak gereksiz işlemci (CPU) kullanımını engeller.
 */
namespace Baryon::systems {

/**
 * @class IslandSystem
 * @brief Nesne gruplandırma, çözücü dağıtımı ve enerji tasarrufu sistemi.
 */
class IslandSystem {
private:
    Core::Registry& mRegistry; ///< ECS kayıt defteri referansı.

public:
    /** 
     * @brief Sistemi başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit IslandSystem(Core::Registry& registry);

    /**
     * @brief Nesneleri gruplara ayırır ve uyku durumlarını kontrol eder.
     * @details 
     * 1. Temas halindeki tüm nesneleri analiz ederek bir bağlantı haritası çıkarır.
     * 2. Birbirine değen nesneleri "ada" adı verilen bağımsız gruplara atar.
     * 3. Her adayı kendi içinde fizik çözücüsüne (solver) gönderir.
     * 4. Enerji Tasarrufu: Bir adadaki tüm nesneler durma noktasına gelirse, 
     *    ada tamamen uyutulur ve tekrar hareket edene kadar hesaplanmaz.
     * 
     * @param manifolds Mevcut aktif temas verileri.
     * @param deltaTime Geçen zaman adımı.
     * @return Tespit edilen ve işlenen adaların (nesne gruplarının) listesi.
     */
    std::vector<std::vector<ecs::Entity>> step(
        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime);
};

} // namespace Baryon::systems