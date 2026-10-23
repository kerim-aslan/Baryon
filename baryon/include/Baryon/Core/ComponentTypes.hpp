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

#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/collision/BallSocketConstraint.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/FixedConstraint.hpp"
#include "Baryon/collision/ConeTwistConstraint.hpp"
#include "Baryon/collision/PrismaticConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/collision/SpringConstraint.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/math/Pose.hpp"
#include <tuple>
#include <type_traits>
#include <variant>

/**
 * @file ComponentTypes.hpp
 * @brief Baryon motorundaki tüm ECS bileşen türlerinin merkezi tek tanım listesi.
 * @details Yeni bir bileşen eklendiğinde sadece bu listeye eklemek yeterlidir;
 * Registry depolaması, yapıcı başlatması, destroyEntity temizliği ve Snapshot
 * save/load döngüsü bu liste üzerinden otomatik olarak yürütülür.
 */
namespace Baryon::Core {

using ComponentTypes = std::tuple<
    Pose,
    Motion,
    MassProps,
    Material,
    BodyState,
    Linkage,
    ecs::ColliderData,
    collision::DistanceConstraint,
    collision::RevoluteConstraint,
    collision::FixedConstraint,
    collision::BallSocketConstraint,
    collision::PrismaticConstraint,
    collision::SpringConstraint,
    collision::ConeTwistConstraint
>;

/**
 * @brief Tüm eklem ve kısıtlama bileşen türlerinin merkezi listesi.
 */
using ConstraintTypes = std::tuple<
    collision::DistanceConstraint,
    collision::RevoluteConstraint,
    collision::FixedConstraint,
    collision::BallSocketConstraint,
    collision::PrismaticConstraint,
    collision::SpringConstraint,
    collision::ConeTwistConstraint
>;

template <typename Tuple>
struct TupleToPtrVariant;

template <typename... Ts>
struct TupleToPtrVariant<std::tuple<Ts...>> {
    using type = std::variant<Ts*...>;
};

/**
 * @brief Ada çözücüsü için kısıtlama gösterici varyantı.
 */
using ConstraintPtrVariant = typename TupleToPtrVariant<ConstraintTypes>::type;

template <typename T, typename Tuple>
struct TupleContains;

template <typename T, typename... Ts>
struct TupleContains<T, std::tuple<Ts...>> : std::disjunction<std::is_same<T, Ts>...> {};

template <typename T>
inline constexpr bool isRegisteredComponent = TupleContains<T, ComponentTypes>::value;

} // namespace Baryon::Core

