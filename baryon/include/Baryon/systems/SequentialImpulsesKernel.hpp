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
#include "Baryon/systems/IslandSystem.hpp"
#include <vector>
#include <unordered_map>

namespace Baryon::systems {

struct SolverBody {
    ecs::Entity entity{};
    Vector3 position{0.0f, 0.0f, 0.0f};
    Quaternion orientation{0.0f, 0.0f, 0.0f, 1.0f};
    Vector3 linearVelocity{0.0f, 0.0f, 0.0f};
    Vector3 angularVelocity{0.0f, 0.0f, 0.0f};
    Vector3 splitLinearVelocity{0.0f, 0.0f, 0.0f};
    Vector3 splitAngularVelocity{0.0f, 0.0f, 0.0f};
    float invMass{0.0f};
    Matrix3x3 invInertiaTensor{0, 0, 0, 0, 0, 0, 0, 0, 0};
    Core::BodyType type{Core::BodyType::Dynamic};
};

struct SolverContact {
    uint32_t bodyAIndex{0};
    uint32_t bodyBIndex{0};
    collision::ContactManifold* manifold{nullptr};
    float friction{0.4f};
};

struct SolverDistanceConstraint {
    uint32_t bodyAIndex{0};
    uint32_t bodyBIndex{0};
    collision::DistanceConstraint* constraint{nullptr};
};

struct SolverRevoluteConstraint {
    uint32_t bodyAIndex{0};
    uint32_t bodyBIndex{0};
    collision::RevoluteConstraint* constraint{nullptr};
};

/**
 * @class SequentialImpulsesKernel
 * @brief Ardışık İtme (Sequential Impulses) çözücü çekirdeği.
 * @details Temas noktalarını ve eklem kısıtlamalarını iteratif Gauss-Seidel yaklaşımıyla çözer.
 *          Warm Starting, çift teğetli Coulomb sürtünmesi ve Split Impulse pozisyon düzeltmesi içerir.
 */
class SequentialImpulsesKernel {
private:
    Core::Registry& mRegistry;
    std::vector<SolverBody> mSolverBodies;
    std::vector<SolverContact> mSolverContacts;
    std::vector<SolverDistanceConstraint> mSolverDistanceConstraints;
    std::vector<SolverRevoluteConstraint> mSolverRevoluteConstraints;
    std::unordered_map<uint32_t, uint32_t> mEntityToBodyMap;

    uint32_t getOrAddBody(ecs::Entity entity);
    void prepareIslandData(Island& island, float deltaTime);
    void solveVelocities(int velocityIterations, float deltaTime);
    void solvePositions(int positionIterations, float deltaTime);
    void writeBackVelocities();

public:
    explicit SequentialImpulsesKernel(Core::Registry& registry);

    /**
     * @brief Bir adaya ait temas ve kısıtları çözer.
     * @param island Adaya ait cisimler, manifoldlar ve kısıtlar.
     * @param deltaTime Zaman adımı.
     * @param velocityIterations Hız iterasyon sayısı.
     * @param positionIterations Konum (Split Impulse) iterasyon sayısı.
     */
    void execute(
        Island& island,
        float deltaTime,
        int velocityIterations = 10,
        int positionIterations = 4
    );

    void preStep(collision::ContactManifold& manifold, SolverBody& bodyA, SolverBody& bodyB, float deltaTime);
    void resolveCollision(collision::ContactManifold& manifold, SolverBody& bodyA, SolverBody& bodyB, float friction, float deltaTime);
    void solvePositionConstraints(collision::ContactManifold& manifold, SolverBody& bodyA, SolverBody& bodyB, float deltaTime);
    void resolveDistanceConstraint(collision::DistanceConstraint& constraint, SolverBody& bodyA, SolverBody& bodyB, float deltaTime);
    void resolveRevoluteConstraint(collision::RevoluteConstraint& constraint, SolverBody& bodyA, SolverBody& bodyB, float deltaTime);
};

} // namespace Baryon::systems
