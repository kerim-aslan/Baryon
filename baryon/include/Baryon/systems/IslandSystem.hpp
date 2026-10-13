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

#include "Baryon/Core/Registry.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include <vector>
#include <unordered_map>

/**
 * @file IslandSystem.hpp
 * @brief Fiziksel Ada (Island) tespiti ve uyku (sleep) yönetim sistemi.
 * @details Birbirine temas eden nesneleri bağımsız gruplara (adalar) ayırarak 
 *          simülasyonu optimize eder. Hareketsiz kalan adaları "uyku" moduna 
 *          alarak gereksiz işlemci (CPU) kullanımını engeller.
 */
namespace Baryon::systems {

/**
 * @struct Island
 * @brief Birbirine temas eden veya kısıtlarla bağlı nesnelerin ve manifoldların bağımsız çözüm grubu.
 */
struct Island {
    std::vector<ecs::Entity> bodies;
    std::vector<collision::ContactManifold*> manifolds;
    std::vector<collision::DistanceConstraint*> distanceConstraints;
    std::vector<collision::RevoluteConstraint*> revoluteConstraints;
};

/**
 * @class IslandSystem
 * @brief Nesne gruplandırma, çözücü dağıtımı ve enerji tasarrufu sistemi.
 */
class IslandSystem {
private:
    Core::Registry& mRegistry; ///< ECS kayıt defteri referansı.
    float mSleepLinearThresholdSq{0.0025f};
    float mSleepAngularThresholdSq{0.0025f};
    float mTimeToSleep{0.5f};

    // D3: Yeniden kullanılan kalıcı tamponlar (Her adımda yığın tahsisini engeller)
    std::unordered_map<uint32_t, std::vector<ecs::Entity>> mAdjList;
    std::unordered_map<uint32_t, size_t> mEntityToIsland;
    std::vector<Island> mActiveIslands;

    void buildAdjacencyList(
        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        std::unordered_map<uint32_t, std::vector<ecs::Entity>>& adjList);

    void findIslandsBFS(
        const std::unordered_map<uint32_t, std::vector<ecs::Entity>>& adjList,
        float deltaTime,
        std::vector<Island>& activeIslands,
        std::unordered_map<uint32_t, size_t>& entityToIsland);

    void processIslandSleep(
        std::vector<ecs::Entity>& islandBodies,
        bool canSleep,
        float deltaTime,
        std::vector<Island>& activeIslands,
        std::unordered_map<uint32_t, size_t>& entityToIsland);

    void assignManifoldsAndConstraints(
        std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        const std::unordered_map<uint32_t, size_t>& entityToIsland,
        std::vector<Island>& activeIslands);

public:
    /** 
     * @brief Sistemi başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit IslandSystem(Core::Registry& registry);

    /**
     * @brief Adaların uykuya geçiş eşiklerini ayarlar.
     * @param linearVelThreshold Uykuya geçiş için maksimum çizgisel hız (m/s).
     * @param angularVelThreshold Uykuya geçiş için maksimum açısal hız (rad/s).
     * @param timeToSleep Bu hızların altında kalınması gereken süre (saniye).
     */
    void setSleepThresholds(float linearVelThreshold, float angularVelThreshold, float timeToSleep = 0.5f) {
        mSleepLinearThresholdSq = linearVelThreshold * linearVelThreshold;
        mSleepAngularThresholdSq = angularVelThreshold * angularVelThreshold;
        mTimeToSleep = timeToSleep;
    }

    [[nodiscard]] float getSleepLinearThresholdSq() const { return mSleepLinearThresholdSq; }
    [[nodiscard]] float getSleepAngularThresholdSq() const { return mSleepAngularThresholdSq; }
    [[nodiscard]] float getTimeToSleep() const { return mTimeToSleep; }

    /**
     * @brief Nesneleri gruplara ayırır, manifold/kısıtları adalara bağlar ve uyku durumlarını kontrol eder.
     * @param manifolds Mevcut aktif temas verileri.
     * @param deltaTime Geçen zaman adımı.
     * @return Çözücüye gönderilecek olan aktif (uyumayan) adaların listesi.
     */
    std::vector<Island> step(
        std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime);
};

} // namespace Baryon::systems