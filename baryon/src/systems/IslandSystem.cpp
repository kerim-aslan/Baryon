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

/**
 * @file IslandSystem.cpp
 * @brief Fiziksel Ada (Island) tespiti ve uyku (Sleep) yönetimi.
 * @details Temas noktaları ve eklem bağlantıları üzerinden etkileşim grafiği
 * oluşturur. Birbirine bağlı cisimleri gruplandırarak simülasyonun bağımsız
 * bölümlerini ayırır ve hareketsiz grupları uyku moduna alarak performansı
 * artırır.
 */

#include "Baryon/systems/IslandSystem.hpp"
#include "Baryon/Core/DebugManager.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include <queue>
#include <unordered_set>

namespace Baryon::systems {

IslandSystem::IslandSystem(Core::Registry &registry) : mRegistry(registry) {}

/**
 * @brief Temas ve kısıtlama grafiğinden aktif adaları (bağlantılı dinamik nesne gruplarını) oluşturur.
 */
std::vector<std::vector<ecs::Entity>> IslandSystem::buildIslands(
    const std::unordered_map<uint64_t, collision::ContactManifold> &manifolds) {

  std::vector<std::vector<ecs::Entity>> activeIslands;

  std::unordered_map<uint32_t, std::vector<ecs::Entity>> adjList;
  adjList.reserve(mRegistry.getComponentPool<Core::BodyState>().getAllEntities().size());

  auto addEdge = [&](ecs::Entity a, ecs::Entity b) {
    if (mRegistry.hasComponent<Core::BodyState>(a) &&
        mRegistry.getComponent<Core::BodyState>(a).type ==
            Core::BodyType::Dynamic)
      adjList[a.id].push_back(b);
    if (mRegistry.hasComponent<Core::BodyState>(b) &&
        mRegistry.getComponent<Core::BodyState>(b).type ==
            Core::BodyType::Dynamic)
      adjList[b.id].push_back(a);
  };

  // Temas halindeki cisimler arasında bağ kur
  for (const auto &[key, manifold] : manifolds) {
    if (!manifold.isTriggerPair)
      addEdge(manifold.entityA, manifold.entityB);
  }

  // Sabit mesafe kısıtlamaları olan cisimler arasında bağ kur
  auto &distConstraints =
      mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
  for (const auto &constraint : distConstraints)
    addEdge(constraint.entityA, constraint.entityB);

  // Menteşe/Dönme kısıtlamaları olan cisimler arasında bağ kur
  auto &revConstraints =
      mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
  for (const auto &constraint : revConstraints)
    addEdge(constraint.entityA, constraint.entityB);

  // BFS algoritması ile bağımsız adaları bul
  std::unordered_set<uint32_t> visited;
  auto &entities =
      mRegistry.getComponentPool<Core::BodyState>().getAllEntities();

  for (ecs::Entity startEntity : entities) {
    if (!mRegistry.hasComponent<Core::BodyState>(startEntity))
      continue;
    auto &startState = mRegistry.getComponent<Core::BodyState>(startEntity);

    if (startState.type != Core::BodyType::Dynamic ||
        visited.contains(startEntity.id))
      continue;

    std::vector<ecs::Entity> island;
    std::queue<ecs::Entity> q;

    q.push(startEntity);
    visited.insert(startEntity.id);

    while (!q.empty()) {
      ecs::Entity curr = q.front();
      q.pop();
      island.push_back(curr);

      for (ecs::Entity neighbor : adjList[curr.id]) {
        if (!mRegistry.hasComponent<Core::BodyState>(neighbor))
          continue;
        auto neighborType =
            mRegistry.getComponent<Core::BodyState>(neighbor).type;

        if (neighborType == Core::BodyType::Dynamic) {
          if (!visited.contains(neighbor.id)) {
            visited.insert(neighbor.id);
            q.push(neighbor);
          }
        }
        // Statik veya Kinematik nesneler yapraktır (leaf). 
        // Arama bunların üzerinden devam etmez ve adaya dahil edilmezler.
      }
    }

    // Adadaki herhangi bir cisim uyanıksa tüm ada uyanık olmalıdır
    bool hasAwakeBody = false;
    for (ecs::Entity ent : island) {
      if (mRegistry.hasComponent<Core::BodyState>(ent)) {
        if (!mRegistry.getComponent<Core::BodyState>(ent).isSleeping) {
          hasAwakeBody = true;
          break;
        }
      }
    }

    if (hasAwakeBody) {
      for (ecs::Entity ent : island) {
        if (mRegistry.hasComponent<Core::BodyState>(ent)) {
          auto &state = mRegistry.getComponent<Core::BodyState>(ent);
          state.isSleeping = false;
        }
      }
      activeIslands.push_back(std::move(island));
    }
  }

  return activeIslands;
}

/**
 * @brief Çözücü ve konum entegrasyonu sonrasında adaların uyku durumunu günceller.
 */
void IslandSystem::updateSleep(
    const std::vector<std::vector<ecs::Entity>> &activeIslands,
    float deltaTime) {

  const float kSleepLinearThresholdSq = mSleepLinearThresholdSq;
  const float kSleepAngularThresholdSq = mSleepAngularThresholdSq;
  const float kTimeToSleep = mTimeToSleep;

  for (const auto &island : activeIslands) {
    bool canSleep = true;

    // Adadaki tüm dinamik cisimlerin hızlarını kontrol et
    for (ecs::Entity ent : island) {
      if (mRegistry.hasComponent<Core::Motion>(ent)) {
        const auto &motion = mRegistry.getComponent<Core::Motion>(ent);
        if (motion.linearVelocity.lengthSquare() >= kSleepLinearThresholdSq ||
            motion.angularVelocity.lengthSquare() >= kSleepAngularThresholdSq) {
          canSleep = false;
          break;
        }
      }
    }

    if (canSleep) {
      // Tüm ada sakin: Sayaçları artır
      for (ecs::Entity ent : island) {
        if (!mRegistry.hasComponent<Core::BodyState>(ent))
          continue;
        auto &state = mRegistry.getComponent<Core::BodyState>(ent);
        if (state.type != Core::BodyType::Dynamic)
          continue;

        state.sleepTimer += deltaTime;
        if (state.sleepTimer >= kTimeToSleep) {
          if (!state.isSleeping) {
            BARYON_LOG_SLEEP("Entity ", ent.id, " fell asleep.");
          }
          state.isSleeping = true;

          // Uyuyan cismin enerjisini sıfırla
          if (mRegistry.hasComponent<Core::Motion>(ent)) {
            auto &motion = mRegistry.getComponent<Core::Motion>(ent);
            motion.linearVelocity = Vector3(0, 0, 0);
            motion.angularVelocity = Vector3(0, 0, 0);
            motion.splitLinearVelocity = Vector3(0, 0, 0);
            motion.splitAngularVelocity = Vector3(0, 0, 0);
          }
        }
      }
    } else {
      // Ada hareketli: Sayaçları ve uyku durumunu sıfırla
      for (ecs::Entity ent : island) {
        if (!mRegistry.hasComponent<Core::BodyState>(ent))
          continue;
        auto &state = mRegistry.getComponent<Core::BodyState>(ent);
        state.sleepTimer = 0.0f;
        state.isSleeping = false;
      }
    }
  }
}

/**
 * @brief Geriye dönük uyumluluk adımı.
 */
std::vector<std::vector<ecs::Entity>> IslandSystem::step(
    const std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime) {
  auto islands = buildIslands(manifolds);
  updateSleep(islands, deltaTime);
  return islands;
}

} // namespace Baryon::systems