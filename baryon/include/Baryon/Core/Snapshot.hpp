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

#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/ecs/ColliderComponents.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"

#include "Baryon/collision/ContactManifold.hpp"
#include <unordered_map>
#include <tuple>
#include <utility>

/**
 * @file Snapshot.hpp
 * @brief Fizik motorunun o anki tam durumunu kaydeden Anlık Görüntü (Snapshot) yapısı.
 * @details Sahnedeki her şeyin bir kopyasını tek bir pakette toplayarak oyunu zaman 
 *          içinde geri sarma (rewind), tekrar izleme (replay), ağ üzerinden çok oyunculu 
 *          senkronizasyon yapma veya hamle geri alma (undo) gibi sistemler kurmanızı sağlar.
 */
namespace Baryon::Core {

/**
 * @brief Tek bir merkezden yönetilen tüm bileşen tipleri listesi (B1).
 * Yeni bileşen eklemek için sadece bu listeye ekleme yapmak yeterlidir.
 */
using ComponentTypeList = std::tuple<
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

template<typename Tuple>
struct ComponentStatesFromTypeList;

template<typename... Comps>
struct ComponentStatesFromTypeList<std::tuple<Comps...>> {
    using type = std::tuple<typename ecs::SparseSet<Comps>::State...>;
};

using ComponentStatesTuple = typename ComponentStatesFromTypeList<ComponentTypeList>::type;

/**
 * @struct Snapshot
 * @brief Fizik dünyasındaki tüm varlıkların, bileşenlerin ve simülasyon durumunun kopyası.
 */
struct Snapshot {
    ecs::EntityManager::State entityManagerState; ///< Varlıkların durumu.
    ComponentStatesTuple componentStates;         ///< Tüm bileşenlerin durumları (Variadic tuple - B1).

    // B2: Determinizm ve geri sarma için simülatör durumları
    float accumulator{0.0f};                      ///< Zaman adımı biriktiricisi.
    std::unordered_map<uint64_t, collision::ContactManifold> manifolds; ///< Warm-start temas manifoldları.

    template<typename T>
    [[nodiscard]] const auto& getComponentState() const {
        return std::get<typename ecs::SparseSet<T>::State>(componentStates);
    }

    template<typename T>
    [[nodiscard]] auto& getComponentState() {
        return std::get<typename ecs::SparseSet<T>::State>(componentStates);
    }
};

} // namespace Baryon::Core