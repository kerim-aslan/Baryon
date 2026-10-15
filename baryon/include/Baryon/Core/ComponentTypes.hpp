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
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/math/Pose.hpp"
#include <tuple>
#include <type_traits>

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
    collision::RevoluteConstraint
>;

template <typename T, typename Tuple>
struct TupleContains;

template <typename T, typename... Ts>
struct TupleContains<T, std::tuple<Ts...>> : std::disjunction<std::is_same<T, Ts>...> {};

template <typename T>
inline constexpr bool isRegisteredComponent = TupleContains<T, ComponentTypes>::value;

} // namespace Baryon::Core
