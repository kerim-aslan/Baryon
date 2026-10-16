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
 * @struct SimulationSettings
 * @brief Fizik dünyasının adım, çözücü iterasyonu ve optimizasyon parametrelerini toplayan yapılandırma.
 */
struct SimulationSettings {
    /// @brief Sabit fizik zaman adımı (saniye). Varsayılan: 1/60s (~0.01667s).
    float fixedTimeStep{1.0f / 60.0f};

    /// @brief Tek bir karede izin verilen maksimum alt adım sayısı (Ölüm spirali koruması).
    int maxSubSteps{8};

    /// @brief Çözücü (Sequential Impulses) hız iterasyonu sayısı. Varsayılan: 12.
    uint32_t velocityIterations{12};

    /// @brief Çözücü pozisyon düzeltme iterasyonu sayısı. Varsayılan: 4.
    uint32_t positionIterations{4};

    /// @brief Geniş fazda hızlı cisimlerin AABB genişletme (velocity expansion) katsayısı (saniye). Varsayılan: 0.02s.
    float broadPhaseVelocityExpansion{0.02f};

    /// @brief Uykuya geçiş için maksimum çizgisel hız eşiği (m/s). Varsayılan: 0.05 m/s.
    float sleepLinearThreshold{0.05f};

    /// @brief Uykuya geçiş için maksimum açısal hız eşiği (rad/s). Varsayılan: 0.05 rad/s.
    float sleepAngularThreshold{0.05f};

    /// @brief Uykuya geçiş hız eşiklerinin altında kalınması gereken süre (saniye). Varsayılan: 0.5s.
    float timeToSleep{0.5f};
};

} // namespace Baryon
