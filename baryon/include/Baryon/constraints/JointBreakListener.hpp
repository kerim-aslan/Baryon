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

namespace Baryon {

/**
 * @file JointBreakListener.hpp
 * @brief Eklem ve kısıtlama kırılma bildirim dinleyicisi arayüzü.
 * @details Bir eklem üzerine binen reaksiyon kuvveti veya açısal tork tanımlanan
 *          kırılma eşiğini (breakForce / breakTorque / breakImpulse) aştığında
 *          bu dinleyici otomatik olarak tetiklenir.
 */
class JointBreakListener {
public:
    virtual ~JointBreakListener() = default;

    /**
     * @brief Bir eklem aşırı yük altında kırıldığında çağrılır.
     * @param entityA Eklemin bağlı olduğu birinci varlık (Entity).
     * @param entityB Eklemin bağlı olduğu ikinci varlık (Entity).
     * @param breakImpulse Eklemin kırıldığı anda ölçülen son reaksiyon itmesi (N*s).
     */
    virtual void onJointBroken(ecs::Entity entityA, ecs::Entity entityB, float breakImpulse) {
        (void)entityA;
        (void)entityB;
        (void)breakImpulse;
    }
};

} // namespace Baryon
