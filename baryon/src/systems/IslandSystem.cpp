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

void IslandSystem::buildAdjacencyList(
    const std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    std::unordered_map<uint32_t, std::vector<ecs::Entity>> &adjList) {
  auto addEdge = [&](ecs::Entity a, ecs::Entity b) {
    if (mRegistry.hasComponent<Core::BodyState>(a) &&
        mRegistry.getComponent<Core::BodyState>(a).type == Core::BodyType::Dynamic)
      adjList[a.id].push_back(b);
    if (mRegistry.hasComponent<Core::BodyState>(b) &&
        mRegistry.getComponent<Core::BodyState>(b).type == Core::BodyType::Dynamic)
      adjList[b.id].push_back(a);
  };

  for (const auto &[key, manifold] : manifolds) {
    if (!manifold.isTriggerPair)
      addEdge(manifold.entityA, manifold.entityB);
  }

  auto &distConstraints =
      mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
  for (const auto &constraint : distConstraints)
    addEdge(constraint.entityA, constraint.entityB);

  auto &revConstraints =
      mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
  for (const auto &constraint : revConstraints)
    addEdge(constraint.entityA, constraint.entityB);
}

void IslandSystem::processIslandSleep(
    std::vector<ecs::Entity> &islandBodies,
    bool canSleep,
    float deltaTime,
    std::vector<Island> &activeIslands,
    std::unordered_map<uint32_t, size_t> &entityToIsland) {
  if (canSleep) {
    bool fellAsleep = false;
    for (ecs::Entity ent : islandBodies) {
      auto &state = mRegistry.getComponent<Core::BodyState>(ent);
      if (state.type != Core::BodyType::Dynamic)
        continue;

      state.sleepTimer += deltaTime;
      if (state.sleepTimer >= mTimeToSleep) {
        if (!state.isSleeping)
          BARYON_LOG_SLEEP("Entity ", ent.id, " fell asleep.");
        state.isSleeping = true;
        fellAsleep = true;

        if (mRegistry.hasComponent<Core::Motion>(ent)) {
          auto &motion = mRegistry.getComponent<Core::Motion>(ent);
          motion.linearVelocity = Vector3(0, 0, 0);
          motion.angularVelocity = Vector3(0, 0, 0);
        }
      }
    }
    if (!fellAsleep) {
      size_t islandIdx = activeIslands.size();
      for (ecs::Entity ent : islandBodies) {
        entityToIsland[ent.id] = islandIdx;
      }
      Island newIsland;
      newIsland.bodies = std::move(islandBodies);
      activeIslands.push_back(std::move(newIsland));
    }
  } else {
    for (ecs::Entity ent : islandBodies) {
      auto &state = mRegistry.getComponent<Core::BodyState>(ent);
      state.sleepTimer = 0.0f;
      state.isSleeping = false;
    }
    size_t islandIdx = activeIslands.size();
    for (ecs::Entity ent : islandBodies) {
      entityToIsland[ent.id] = islandIdx;
    }
    Island newIsland;
    newIsland.bodies = std::move(islandBodies);
    activeIslands.push_back(std::move(newIsland));
  }
}

void IslandSystem::findIslandsBFS(
    const std::unordered_map<uint32_t, std::vector<ecs::Entity>> &adjList,
    float deltaTime,
    std::vector<Island> &activeIslands,
    std::unordered_map<uint32_t, size_t> &entityToIsland) {
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

    std::vector<ecs::Entity> islandBodies;
    std::queue<ecs::Entity> q;

    q.push(startEntity);
    visited.insert(startEntity.id);

    bool canSleep = true;
    const float kSleepLinearThresholdSq = mSleepLinearThresholdSq;
    const float kSleepAngularThresholdSq = mSleepAngularThresholdSq;

    while (!q.empty()) {
      ecs::Entity curr = q.front();
      q.pop();
      islandBodies.push_back(curr);

      if (mRegistry.hasComponent<Core::Motion>(curr)) {
        auto &currMotion = mRegistry.getComponent<Core::Motion>(curr);
        if (currMotion.linearVelocity.lengthSquare() >=
                kSleepLinearThresholdSq ||
            currMotion.angularVelocity.lengthSquare() >=
                kSleepAngularThresholdSq) {
          canSleep = false;
        }
      }

      auto it = adjList.find(curr.id);
      if (it != adjList.end()) {
        for (ecs::Entity neighbor : it->second) {
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
        }
      }
    }

    processIslandSleep(islandBodies, canSleep, deltaTime, activeIslands, entityToIsland);
  }
}

void IslandSystem::assignManifoldsAndConstraints(
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    const std::unordered_map<uint32_t, size_t> &entityToIsland,
    std::vector<Island> &activeIslands) {
  for (auto &[key, manifold] : manifolds) {
    if (manifold.isTriggerPair)
      continue;
    auto itA = entityToIsland.find(manifold.entityA.id);
    if (itA != entityToIsland.end()) {
      activeIslands[itA->second].manifolds.push_back(&manifold);
    } else {
      auto itB = entityToIsland.find(manifold.entityB.id);
      if (itB != entityToIsland.end()) {
        activeIslands[itB->second].manifolds.push_back(&manifold);
      }
    }
  }

  if (mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData().size() > 0) {
    auto &allDist = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
    for (auto &constraint : allDist) {
      auto itA = entityToIsland.find(constraint.entityA.id);
      if (itA != entityToIsland.end()) {
        activeIslands[itA->second].distanceConstraints.push_back(&constraint);
      } else {
        auto itB = entityToIsland.find(constraint.entityB.id);
        if (itB != entityToIsland.end()) {
          activeIslands[itB->second].distanceConstraints.push_back(&constraint);
        }
      }
    }
  }

  if (mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData().size() > 0) {
    auto &allRev = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
    for (auto &constraint : allRev) {
      auto itA = entityToIsland.find(constraint.entityA.id);
      if (itA != entityToIsland.end()) {
        activeIslands[itA->second].revoluteConstraints.push_back(&constraint);
      } else {
        auto itB = entityToIsland.find(constraint.entityB.id);
        if (itB != entityToIsland.end()) {
          activeIslands[itB->second].revoluteConstraints.push_back(&constraint);
        }
      }
    }
  }
}

std::vector<Island> IslandSystem::step(
    std::unordered_map<uint64_t, collision::ContactManifold> &manifolds,
    float deltaTime) {
  mActiveIslands.clear();
  mAdjList.clear();
  mEntityToIsland.clear();

  mAdjList.reserve(mRegistry.getComponentPool<Core::BodyState>().getAllEntities().size());
  buildAdjacencyList(manifolds, mAdjList);

  findIslandsBFS(mAdjList, deltaTime, mActiveIslands, mEntityToIsland);

  if (!mActiveIslands.empty()) {
    assignManifoldsAndConstraints(manifolds, mEntityToIsland, mActiveIslands);
  }

  return mActiveIslands;
}

} // namespace Baryon::systems