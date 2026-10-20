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

#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"

namespace Baryon::collision {

/**
 * @struct SweepHit
 * @brief Geometrik süpürme (sweep / shape cast) sorgusunun sonucunu tutan yapı.
 * @details Süpürülen bir şekil bir cisme çarptığında ilk temas noktası,
 *          yüzey normali, kat edilen mesafe ve çarpılan varlığı içerir.
 */
struct SweepHit {
    bool hasHit{false};          ///< Çarpışma gerçekleşti mi?
    Vector3 point{0.0f, 0.0f, 0.0f};  ///< Dünya koordinatlarındaki temas noktası (hedef yüzey üzerinde).
    Vector3 normal{0.0f, 1.0f, 0.0f}; ///< Çarpılan yüzeyin dışa bakan normal vektörü.
    float distance{0.0f};        ///< Başlangıç noktasından çarpışmaya kadar kat edilen mesafe (metre).
    float fraction{1.0f};        ///< Mesafe / maxDistance oranı [0, 1].
    ecs::Entity entity{0};       ///< Çarpılan varlık.
};

} // namespace Baryon::collision
