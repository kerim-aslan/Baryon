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
#include "Baryon/math/Vector3.hpp"

namespace Baryon::collision {

/**
 * @file BallSocketConstraint.hpp
 * @brief Küresel / Bilyeli Mafsal Eklemi (Ball-Socket / Spherical Joint) bileşeni.
 */

/**
 * @struct BallSocketConstraint
 * @brief 3 çizgisel serbestlik derecesini kilitleyen, 3 açısal serbestlik derecesini serbest bırakan eklem.
 * @details İki cismin ankor noktalarını uzayda aynı noktada çakıştırır. Sarkaç, bez simülasyonu,
 *          ragdoll omuz/kalça mafsalları gibi sistemlerde kullanılır.
 */
struct BallSocketConstraint : public ConstraintBase {
    /// @brief Hız çözücüsünde birikimli çizgisel tepki itmesi (Warm-starting için).
    Vector3 accumulatedImpulseLinear{0.0f, 0.0f, 0.0f};

    /// @name Kurucular
    /// @{

    /**
     * @brief Varsayılan kurucu.
     */
    BallSocketConstraint() = default;

    /**
     * @brief İki varlık ve yerel ankor noktaları ile kısıt oluşturur.
     * @param a Birinci varlık (Entity).
     * @param b İkinci varlık (Entity).
     * @param anchorA A cisminin yerel uzayındaki bağlantı noktası.
     * @param anchorB B cisminin yerel uzayındaki bağlantı noktası.
     */
    BallSocketConstraint(ecs::Entity a, ecs::Entity b,
                         const Vector3& anchorA, const Vector3& anchorB)
        : ConstraintBase(a, b, anchorA, anchorB) {}

    /**
     * @brief İki varlık ve dünya uzayındaki ortak pivot noktası ile kısıt oluşturur.
     * @param a Birinci varlık (Entity).
     * @param b İkinci varlık (Entity).
     * @param worldPivot Dünya uzayındaki bağlantı noktası.
     * @param poseA A cisminin anlık konumu/oryantasyonu.
     * @param poseB B cisminin anlık konumu/oryantasyonu.
     */
    BallSocketConstraint(ecs::Entity a, ecs::Entity b,
                         const Vector3& worldPivot,
                         const Pose& poseA, const Pose& poseB)
        : ConstraintBase(a, b,
                         poseA.orientation.getConjugate() * (worldPivot - poseA.position),
                         poseB.orientation.getConjugate() * (worldPivot - poseB.position)) {}

    /// @}
};

} // namespace Baryon::collision
