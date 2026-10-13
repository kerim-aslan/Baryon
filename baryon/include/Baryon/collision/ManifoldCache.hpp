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

#include "ContactManifold.hpp"
#include <unordered_map>
#include <cstdint>
#include <utility>

namespace Baryon::collision {

/**
 * @class ManifoldCache
 * @brief Kareler boyunca aktif temas manifoldlarını saklayan ve warm-starting eşleştirmesini yöneten önbellek.
 */
class ManifoldCache {
private:
    std::unordered_map<uint64_t, ContactManifold> mCurrent;
    std::unordered_map<uint64_t, ContactManifold> mPrevious;

public:
    /**
     * @brief İki varlık (Entity) için simetrik ve benzersiz 64-bit bir anahtar üretir.
     */
    static constexpr uint64_t makeKey(ecs::Entity entityA, ecs::Entity entityB) {
        uint32_t id1 = entityA.id;
        uint32_t id2 = entityB.id;
        if (id1 > id2) {
            std::swap(id1, id2);
        }
        return (static_cast<uint64_t>(id1) << 32) | static_cast<uint64_t>(id2);
    }

    /**
     * @brief Yeni kareyi başlatır: mevcut manifoldları önceki kare tamponuna taşır ve mevcut tamponu temizler.
     * @param reserveSize Tahmini yeni manifold sayısı.
     */
    void beginFrame(size_t reserveSize = 0) {
        mPrevious = std::move(mCurrent);
        mCurrent.clear();
        if (reserveSize > 0) {
            mCurrent.reserve(reserveSize);
        }
    }

    /**
     * @brief Önceki kareden kalan manifoldlar içinde verilen anahtara sahip olanı arar.
     * @param key 64-bit varlık çifti anahtarı.
     * @return Bulunursa ContactManifold göstericisi, bulunamazsa nullptr.
     */
    [[nodiscard]] const ContactManifold* findPrevious(uint64_t key) const {
        auto it = mPrevious.find(key);
        if (it != mPrevious.end()) {
            return &it->second;
        }
        return nullptr;
    }

    /**
     * @brief Mevcut manifold haritasına erişim sağlar.
     */
    [[nodiscard]] const std::unordered_map<uint64_t, ContactManifold>& getCurrent() const { return mCurrent; }
    [[nodiscard]] std::unordered_map<uint64_t, ContactManifold>& getCurrent() { return mCurrent; }

    /**
     * @brief Önceki manifold haritasına erişim sağlar.
     */
    [[nodiscard]] const std::unordered_map<uint64_t, ContactManifold>& getPrevious() const { return mPrevious; }
};

} // namespace Baryon::collision
