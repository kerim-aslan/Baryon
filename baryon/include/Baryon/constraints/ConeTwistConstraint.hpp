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
#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Quaternion.hpp"

namespace Baryon::collision {

/**
 * @file ConeTwistConstraint.hpp
 * @brief Koni ve Burulma Kısıtı (Cone-Twist / Ragdoll Joint) bileşeni.
 * @details İki rijit cismi noktasal bir ankorla bağlayarak (3 çizgisel serbestlik derecesi kilitli),
 *          koni şeklinde açısal salınıma (swing) ve kendi ekseni etrafında burulmaya (twist)
 *          belirli sınırlar dahilinde izin veren anatomik eklemdir.
 *          İnsan omuzu, kalça mafsalı ve ragdoll karakter fiziği için kullanılır.
 */
struct ConeTwistConstraint : public ConstraintBase {
    Vector3 twistAxisLocalA{1.0f, 0.0f, 0.0f}; ///< A cisminin yerel uzayında birincil burulma ekseni (birim vektör).
    Vector3 twistAxisLocalB{1.0f, 0.0f, 0.0f}; ///< B cisminin yerel uzayında birincil burulma ekseni (birim vektör).

    Quaternion initialRelativeOrientation{0.0f, 0.0f, 0.0f, 1.0f}; ///< Başlangıç bağıl yönelim (qA^-1 * qB).

    bool enableSwingLimit{true};        ///< Koni salınım sınırının devrede olup olmadığı.
    float swingLimitAngle{0.785398f};   ///< Maksimum koni salınım yarı-açısı (radyan, varsayılan: pi/4 = 45 derece).

    bool enableTwistLimit{true};        ///< Eksen etrafında burulma sınırının devrede olup olmadığı.
    float lowerTwistLimit{-0.523599f};  ///< Minimum burulma açısı (radyan, varsayılan: -pi/6 = -30 derece).
    float upperTwistLimit{0.523599f};   ///< Maksimum burulma açısı (radyan, varsayılan: +pi/6 = +30 derece).

    // Çözücü birikimli itmeleri
    Vector3 accumulatedImpulseLinear{0.0f, 0.0f, 0.0f}; ///< Çizgisel ankor kilidi birikmiş itmesi.
    float accumulatedSwingImpulse{0.0f};                ///< Koni salınım sınırı birikmiş açısal itmesi.
    float accumulatedTwistImpulse{0.0f};                ///< Eksen burulma sınırı birikmiş açısal itmesi.

    /**
     * @brief Varsayılan yapıcı metot.
     */
    ConeTwistConstraint() = default;

    /**
     * @brief Yeni bir koni-burulma ragdoll eklemi oluşturur.
     * @param a Birinci varlık (Entity).
     * @param b İkinci varlık (Entity).
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param twistA A cismi yerel burulma ekseni.
     * @param twistB B cismi yerel burulma ekseni.
     * @param initialRelOrient Başlangıç bağıl kuaterniyon yönelimi.
     * @param swingLimit Maksimum koni salınım açısı (radyan).
     * @param lowerTwist Minimum burulma açısı (radyan).
     * @param upperTwist Maksimum burulma açısı (radyan).
     */
    ConeTwistConstraint(ecs::Entity a, ecs::Entity b,
                        const Vector3& anchorA, const Vector3& anchorB,
                        const Vector3& twistA, const Vector3& twistB,
                        const Quaternion& initialRelOrient = Quaternion::identity(),
                        float swingLimit = 0.785398f,
                        float lowerTwist = -0.523599f,
                        float upperTwist = 0.523599f)
        : ConstraintBase(a, b, anchorA, anchorB),
          twistAxisLocalA(twistA.getNormalized()),
          twistAxisLocalB(twistB.getNormalized()),
          initialRelativeOrientation(initialRelOrient),
          enableSwingLimit(true),
          swingLimitAngle(swingLimit),
          enableTwistLimit(true),
          lowerTwistLimit(lowerTwist),
          upperTwistLimit(upperTwist) {}

    /**
     * @brief Koni ve burulma limit açılarını günceller.
     * @param swingLimit Maksimum salınım koni yarı-açısı (radyan).
     * @param lowerTwist Minimum burulma açısı (radyan).
     * @param upperTwist Maksimum burulma açısı (radyan).
     */
    void setLimits(float swingLimit, float lowerTwist, float upperTwist) {
        enableSwingLimit = (swingLimit >= 0.0f);
        swingLimitAngle = swingLimit;
        enableTwistLimit = (lowerTwist <= upperTwist);
        lowerTwistLimit = lowerTwist;
        upperTwistLimit = upperTwist;
    }
};

} // namespace Baryon::collision
