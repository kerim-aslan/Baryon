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
