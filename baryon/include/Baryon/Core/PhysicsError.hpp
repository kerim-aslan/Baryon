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

#include <cstdint>

namespace Baryon {

/**
 * @brief Fizik ve ECS işlemlerinde oluşabilecek hata kodları.
 */
enum class PhysicsError {
    EntityDead,       ///< Varlık silinmiş veya geçersiz.
    ComponentMissing, ///< Varlıkta istenilen bileşen bulunamadı.
    InvalidValue,     ///< Geçersiz parametre değeri.
    CapacityExceeded  ///< Maksimum varlık kapasitesi aşıldı.
};

} // namespace Baryon
