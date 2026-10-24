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

#include "Baryon/collision/ConstraintBase.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/math/Quaternion.hpp"
#include "Baryon/math/Vector3.hpp"

namespace Baryon::collision {

/**
 * @file PrismaticConstraint.hpp
 * @brief Prizmatik / Kayar Eklem (Prismatic / Slider Joint) bileşeni.
 */

/**
 * @struct PrismaticConstraint
 * @brief Tek bir eksen boyunca kaymaya izin veren, dönmeyi ve eksen dışı kaymayı kilitleyen eklem.
 * @details Asansör, hidrolik piston, süspansiyon rayı ve kayar kapı mekanizmaları için kullanılır.
 *          Alt/üst sınır (limit) ve doğrusal motor desteği içerir.
 */
struct PrismaticConstraint : public ConstraintBase {
    /// @brief A cisminin yerel uzayında tanımlı kayma ekseni (birim vektör).
    Vector3 localAxisA{1.0f, 0.0f, 0.0f};

    /// @brief Bağlantı anındaki başlangıç bağıl yönelim (qA^-1 * qB).
    Quaternion initialRelativeOrientation{0.0f, 0.0f, 0.0f, 1.0f};

    /// @brief Sınırların devrede olup olmadığını belirler.
    bool enableLimits{false};

    /// @brief Kayma ekseni boyunca minimum öteleme mesafesi (metre).
    float lowerLimit{0.0f};

    /// @brief Kayma ekseni boyunca maksimum öteleme mesafesi (metre).
    float upperLimit{0.0f};

    /// @brief Doğrusal motorun devrede olup olmadığını belirler.
    bool enableMotor{false};

    /// @brief Hedef doğrusal motor hızı (m/s).
    float motorSpeed{0.0f};

    /// @brief Motorun uygulayabileceği maksimum itme kuvveti (Newton).
    float maxMotorForce{0.0f};

    /// @brief Eksen dışı kilitli 2 doğrusal serbestlik derecesinde birikimli itme.
    Vector3 accumulatedImpulseLinear{0.0f, 0.0f, 0.0f};

    /// @brief Kilitli 3 açısal serbestlik derecesinde birikimli açısal itme.
    Vector3 accumulatedImpulseAngular{0.0f, 0.0f, 0.0f};

    /// @brief Motordan uygulanan birikimli itme.
    float motorImpulse{0.0f};

    /// @brief Sınırlardan (limit) uygulanan birikimli itme.
    float limitImpulse{0.0f};

    /// @name Kurucular
    /// @{

    /**
     * @brief Varsayılan kurucu.
     */
    PrismaticConstraint() = default;

    /**
     * @brief İki varlık, yerel ankorlar, yerel kayma ekseni ve bağıl yönelim ile kısıt oluşturur.
     * @param a Birinci varlık (Entity).
     * @param b İkinci varlık (Entity).
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param axisA A cismi yerel uzayında kayma ekseni.
     * @param relOrientation Başlangıç bağıl yönelim.
     */
    PrismaticConstraint(ecs::Entity a, ecs::Entity b,
                        const Vector3& anchorA, const Vector3& anchorB,
                        const Vector3& axisA,
                        const Quaternion& relOrientation)
        : ConstraintBase(a, b, anchorA, anchorB),
          localAxisA(axisA.getNormalized()),
          initialRelativeOrientation(relOrientation) {}

    /// @}
};

} // namespace Baryon::collision
