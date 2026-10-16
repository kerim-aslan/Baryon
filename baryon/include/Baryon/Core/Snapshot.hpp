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

#include "Baryon/Core/ComponentTypes.hpp"
#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/ecs/SparseSet.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include <tuple>
#include <unordered_map>

/**
 * @file Snapshot.hpp
 * @brief Fizik motorunun o anki tam durumunu kaydeden Anlık Görüntü (Snapshot) yapısı.
 * @details Sahnedeki her şeyin bir kopyasını tek bir pakette toplayarak oyunu zaman 
 *          içinde geri sarma (rewind), tekrar izleme (replay), ağ üzerinden çok oyunculu 
 *          senkronizasyon yapma veya hamle geri alma (undo) gibi sistemler kurmanızı sağlar.
 */
namespace Baryon::Core {

template <typename Tuple>
struct MapToSparseSetState;

template <typename... Ts>
struct MapToSparseSetState<std::tuple<Ts...>> {
    using type = std::tuple<typename ecs::SparseSet<Ts>::State...>;
};

/**
 * @struct BasicSnapshot
 * @brief Fizik dünyasındaki tüm varlıkların ve onlara ait bileşenlerin o anki kopyasını tutan jenerik veri yapısı.
 * @details Registry sınıfındaki save() fonksiyonu ile doldurulur ve load() fonksiyonu 
 *          ile sahneyi tam olarak bu kayıtlı anın durumuna geri döndürmek için kullanılır.
 */
template <typename ComponentsTuple = ComponentTypes>
struct BasicSnapshot {
    ecs::EntityManager::State entityManagerState; ///< Varlıkların durumu.
    typename MapToSparseSetState<ComponentsTuple>::type componentStates; ///< Bileşen havuzlarının durumları.

    // B2: Determinizm ve geri sarma için simülatör durumları
    float accumulator{0.0f};                      ///< Zaman adımı biriktiricisi.
    std::unordered_map<uint64_t, collision::ContactManifold> manifolds; ///< Warm-start temas manifoldları.

    template <typename T>
    [[nodiscard]] const typename ecs::SparseSet<T>::State& getState() const {
        static_assert(TupleContains<T, ComponentsTuple>::value, "Component is not in Snapshot type list!");
        return std::get<typename ecs::SparseSet<T>::State>(componentStates);
    }

    template <typename T>
    [[nodiscard]] typename ecs::SparseSet<T>::State& getState() {
        static_assert(TupleContains<T, ComponentsTuple>::value, "Component is not in Snapshot type list!");
        return std::get<typename ecs::SparseSet<T>::State>(componentStates);
    }
};

using Snapshot = BasicSnapshot<ComponentTypes>;

} // namespace Baryon::Core