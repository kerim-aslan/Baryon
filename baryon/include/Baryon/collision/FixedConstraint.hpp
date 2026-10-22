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
#include "../math/Quaternion.hpp"

/**
 * @file FixedConstraint.hpp
 * @brief Sabit / Kaynak eklemi (Fixed / Weld Constraint) yapısı.
 * @details İki cismi birbirine tamamen kaynaklayarak 6 serbestlik derecesinin tümünü
 *          (3 çizgisel eksen + 3 açısal eksen) kilitler. İki cisim tek bir katı gövde
 *          gibi birlikte hareket eder.
 */
namespace Baryon::collision {

/**
 * @struct FixedConstraint
 * @brief İki varlık arasındaki sabit (kaynak) bağlantısının fiziksel verilerini tutar.
 */
struct FixedConstraint : public ConstraintBase {
    Quaternion initialRelativeOrientation{Quaternion::identity()}; ///< A cisminin yerel uzayında B'nin hedef bağıl yönelimi (qA⁻¹ * qB).

    Vector3 accumulatedImpulseLinear{0.0f, 0.0f, 0.0f};  ///< Çizgisel bağlantı için birikmiş itme kuvveti.
    Vector3 accumulatedImpulseAngular{0.0f, 0.0f, 0.0f}; ///< Açısal kilit için birikmiş tork itmesi.

    /**
     * @brief Varsayılan yapıcı metot.
     */
    FixedConstraint() = default;

    /**
     * @brief Yeni bir sabit kaynak eklemi oluşturur.
     * @param a Birinci varlık.
     * @param b İkinci varlık.
     * @param anchorA A cismi üzerindeki bağlantı noktası.
     * @param anchorB B cismi üzerindeki bağlantı noktası.
     * @param relOrientation Başlangıçtaki bağıl rotasyon (qA⁻¹ * qB).
     */
    FixedConstraint(ecs::Entity a, ecs::Entity b,
                    const Vector3& anchorA = Vector3(0.0f, 0.0f, 0.0f),
                    const Vector3& anchorB = Vector3(0.0f, 0.0f, 0.0f),
                    const Quaternion& relOrientation = Quaternion::identity())
        : ConstraintBase(a, b, anchorA, anchorB),
          initialRelativeOrientation(relOrientation) {}
};

} // namespace Baryon::collision
