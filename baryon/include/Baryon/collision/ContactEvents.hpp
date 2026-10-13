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

#include "CollisionListener.hpp"
#include "ContactManifold.hpp"
#include <unordered_map>
#include <cstdint>

namespace Baryon::collision {

/**
 * @class ContactEvents
 * @brief Kareler arası temas ve tetikleyici giriş/çıkış olaylarını tespit edip dinleyiciye bildiren sistem.
 */
class ContactEvents {
public:
    /**
     * @brief Mevcut ve önceki kare manifold haritalarını karşılaştırarak olayları iletir.
     * @param listener Kullanıcı dinleyici göstericisi (nullptr ise işlem yapılmaz).
     * @param current Mevcut kare temas haritası.
     * @param previous Önceki kare temas haritası.
     */
    static void dispatch(
        CollisionListener* listener,
        const std::unordered_map<uint64_t, ContactManifold>& current,
        const std::unordered_map<uint64_t, ContactManifold>& previous) {
        if (!listener) return;

        // Yeni başlayan temaslar (Enter)
        for (const auto& [key, manifold] : current) {
            if (!previous.contains(key)) {
                if (manifold.isTriggerPair) {
                    listener->onTriggerEnter(manifold.entityA, manifold.entityB);
                } else {
                    listener->onCollisionEnter(manifold.entityA, manifold.entityB, manifold);
                }
            }
        }

        // Biten temaslar (Exit)
        for (const auto& [key, oldManifold] : previous) {
            if (!current.contains(key)) {
                if (oldManifold.isTriggerPair) {
                    listener->onTriggerExit(oldManifold.entityA, oldManifold.entityB);
                } else {
                    listener->onCollisionExit(oldManifold.entityA, oldManifold.entityB);
                }
            }
        }
    }
};

} // namespace Baryon::collision
