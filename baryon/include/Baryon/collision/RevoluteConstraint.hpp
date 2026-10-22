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

#include "ConstraintBase.hpp"

/**
 * @file RevoluteConstraint.hpp
 * @brief Menteşe eklemi (Revolute Constraint) yapısı.
 * @details İki cismi belirli bir noktadan birbirine bağlayarak, yalnızca tek bir eksen 
 *          etrafında dönmelerine izin veren fiziksel bağlantıdır. Kapı menteşesi, 
 *          tekerlek aksı veya robot kolu gibi mekanizmalar oluşturmak için kullanılır.
 */
namespace Baryon::collision {

/**
 * @struct RevoluteConstraint
 * @brief Menteşe ekleminin fiziksel özelliklerini ve sınırlarını tutan yapı.
 */
struct RevoluteConstraint : public ConstraintBase {
    Vector3 hingeAxisLocalA{1.0f, 0.0f, 0.0f}; ///< A cisminin kendi yerel uzayındaki dönme ekseni.
    Vector3 hingeAxisLocalB{1.0f, 0.0f, 0.0f}; ///< B cisminin kendi yerel uzayındaki dönme ekseni.

    bool enableLimit{false};       ///< Dönüş açısı sınırlandırmasının aktif olup olmadığı.
    float lowerAngleLimit{0.0f};   ///< İzin verilen minimum dönüş açısı (radyan cinsinden).
    float upperAngleLimit{0.0f};   ///< İzin verilen maksimum dönüş açısı (radyan cinsinden).

    Vector3 accumulatedImpulsePoint{0.0f, 0.0f, 0.0f}; ///< Noktasal bağlantı için birikmiş itme kuvveti.
    Vector3 accumulatedImpulseHinge{0.0f, 0.0f, 0.0f}; ///< Eksen dışı kilit için birikmiş açısal itme kuvveti.
    float accumulatedMotorImpulse{0.0f};               ///< Motor için birikmiş tork itmesi.

    /**
     * @brief Varsayılan yapıcı metot.
     */
    RevoluteConstraint() = default;

    /**
     * @brief Yeni bir menteşe eklemi oluşturur.
     * @param a Birinci varlık.
     * @param b İkinci varlık.
     * @param anchorA A cismi üzerindeki bağlantı noktası.
     * @param anchorB B cismi üzerindeki bağlantı noktası.
     * @param axisA A cisminin dönme ekseni (vektör otomatik olarak normalize edilir).
     * @param axisB B cisminin dönme ekseni (vektör otomatik olarak normalize edilir).
     */
    RevoluteConstraint(ecs::Entity a, ecs::Entity b, 
                      const Vector3& anchorA, const Vector3& anchorB, 
                      const Vector3& axisA, const Vector3& axisB)
        : ConstraintBase(a, b, anchorA, anchorB),
          hingeAxisLocalA(axisA.getNormalized()), hingeAxisLocalB(axisB.getNormalized()) {}
};

} // namespace Baryon::collision