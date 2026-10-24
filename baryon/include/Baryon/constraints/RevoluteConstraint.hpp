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

    Quaternion initialRelativeOrientation{0.0f, 0.0f, 0.0f, 1.0f}; ///< Başlangıç bağıl yönelim (qA^-1 * qB).

    bool enableLimit{false};       ///< Dönüş açısı sınırlandırmasının aktif olup olmadığı.
    float lowerAngleLimit{0.0f};   ///< İzin verilen minimum dönüş açısı (radyan cinsinden).
    float upperAngleLimit{0.0f};   ///< İzin verilen maksimum dönüş açısı (radyan cinsinden).

    bool enableMotor{false};       ///< Açısal motorun aktif olup olmadığı.
    float motorSpeed{0.0f};        ///< Hedef açısal motor hızı (rad/s).
    float maxMotorTorque{0.0f};    ///< Maksimum motor torku (N*m).

    Vector3 accumulatedImpulsePoint{0.0f, 0.0f, 0.0f}; ///< Noktasal bağlantı için birikmiş itme kuvveti.
    Vector3 accumulatedImpulseHinge{0.0f, 0.0f, 0.0f}; ///< Eksen dışı kilit için birikmiş açısal itme kuvveti.
    float accumulatedMotorImpulse{0.0f};               ///< Motor için birikmiş tork itmesi.
    float accumulatedLimitImpulse{0.0f};               ///< Açısal limit için birikmiş tork itmesi.

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
     * @param initialRelOrient Başlangıç anındaki bağıl kuaterniyon yönelimi (qA^-1 * qB).
     */
    RevoluteConstraint(ecs::Entity a, ecs::Entity b, 
                      const Vector3& anchorA, const Vector3& anchorB, 
                      const Vector3& axisA, const Vector3& axisB,
                      const Quaternion& initialRelOrient = Quaternion::identity())
        : ConstraintBase(a, b, anchorA, anchorB),
          hingeAxisLocalA(axisA.getNormalized()), hingeAxisLocalB(axisB.getNormalized()),
          initialRelativeOrientation(initialRelOrient) {}

    /**
     * @brief Dönüş açısı alt ve üst sınırlarını atar.
     * @param lower Minimum açı (radyan).
     * @param upper Maksimum açı (radyan).
     */
    void setLimits(float lower, float upper) {
        enableLimit = true;
        lowerAngleLimit = lower;
        upperAngleLimit = upper;
    }

    /**
     * @brief Açısal motor parametrelerini atar.
     * @param speed Hedef açısal hız (rad/s).
     * @param maxTorque Maksimum motor torku (N*m).
     */
    void setMotor(float speed, float maxTorque) {
        enableMotor = true;
        motorSpeed = speed;
        maxMotorTorque = maxTorque;
    }
};

} // namespace Baryon::collision