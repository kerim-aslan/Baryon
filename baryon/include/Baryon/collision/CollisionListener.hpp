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

#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/collision/ContactManifold.hpp"

namespace Baryon {

/**
 * @class CollisionListener
 * @brief Çarpışma ve tetikleyici olaylarını dinleyen kullanıcı arayüzü.
 * @details Oyun geliştiricileri bu sınıfı miras alarak (override) nesnelerin temas anlarında 
 *          (başlama ve bitme) özel oyun mantıklarını (ses, can azaltma, puan verme) çalıştırabilir.
 */
class CollisionListener {
public:
    virtual ~CollisionListener() = default;

    /**
     * @brief İki katı fiziksel cisim temas etmeye başladığında çağrılır.
     * @param entityA Birinci varlık.
     * @param entityB İkinci varlık.
     * @param manifold Temas noktaları ve yüzey normali verisi.
     */
    virtual void onCollisionEnter(ecs::Entity entityA, ecs::Entity entityB, const collision::ContactManifold& manifold) {
        (void)entityA; (void)entityB; (void)manifold;
    }

    /**
     * @brief Birbiriyle temas halindeki iki katı cisim ayrıldığında çağrılır.
     * @param entityA Birinci varlık.
     * @param entityB İkinci varlık.
     */
    virtual void onCollisionExit(ecs::Entity entityA, ecs::Entity entityB) {
        (void)entityA; (void)entityB;
    }

    /**
     * @brief Bir nesne bir tetikleyici (isTrigger) hacmine girdiğinde çağrılır.
     * @param trigger Tetikleyici özelliğine sahip varlık.
     * @param other İçeri giren diğer varlık.
     */
    virtual void onTriggerEnter(ecs::Entity trigger, ecs::Entity other) {
        (void)trigger; (void)other;
    }

    /**
     * @brief Bir nesne bir tetikleyici hacminden çıktığında çağrılır.
     * @param trigger Tetikleyici özelliğine sahip varlık.
     * @param other Dışarı çıkan diğer varlık.
     */
    virtual void onTriggerExit(ecs::Entity trigger, ecs::Entity other) {
        (void)trigger; (void)other;
    }
};

} // namespace Baryon
