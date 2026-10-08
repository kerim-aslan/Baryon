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

namespace Baryon::systems {

/**
 * @class SequentialImpulsesKernel
 * @brief Ardışık İtme (Sequential Impulses) çözücü çekirdeği.
 * @details Temas noktalarını ve eklem kısıtlamalarını iteratif Gauss-Seidel yaklaşımıyla çözer.
 *          Warm Starting, çift teğetli Coulomb sürtünmesi ve Split Impulse pozisyon düzeltmesi içerir.
 */
class SequentialImpulsesKernel {
private:
    Core::Registry& mRegistry;

public:
    explicit SequentialImpulsesKernel(Core::Registry& registry);

    void execute(
        const std::vector<ecs::Entity>& island,
        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime,
        int velocityIterations = 10,
        int positionIterations = 4
    );

    void solveIsland(
        const std::vector<ecs::Entity>& island,
        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime
    );

    void solveConstraintsForIsland(
        const std::vector<ecs::Entity>& island,
        float deltaTime
    );

    void preStep(collision::ContactManifold& manifold, float deltaTime);
    void resolveCollision(collision::ContactManifold& manifold, float deltaTime);
    void solvePositionConstraints(collision::ContactManifold& manifold, float deltaTime);
    void resolveDistanceConstraint(collision::DistanceConstraint& constraint, float deltaTime);
    void resolveRevoluteConstraint(collision::RevoluteConstraint& constraint, float deltaTime);
};

} // namespace Baryon::systems
