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
#include "Baryon/collision/BallSocketConstraint.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include "Baryon/collision/ConeTwistConstraint.hpp"
#include "Baryon/collision/DistanceConstraint.hpp"
#include "Baryon/collision/FixedConstraint.hpp"
#include "Baryon/collision/PrismaticConstraint.hpp"
#include "Baryon/collision/RevoluteConstraint.hpp"
#include "Baryon/collision/SpringConstraint.hpp"
#include "Baryon/collision/JointBreakListener.hpp"
#include <vector>
#include <unordered_map>
#include <functional>

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
    JointBreakListener* mJointBreakListener{nullptr};
    std::function<void(ecs::Entity, ecs::Entity, float)> mJointBreakCallback{nullptr};

    void collectIslandManifolds(
        const std::vector<ecs::Entity>& island,
        std::unordered_map<uint64_t, collision::ContactManifold>& manifoldMap,
        std::vector<collision::ContactManifold*>& localManifolds);

    std::vector<std::vector<collision::ContactManifold*>> routeAndSortIslandManifolds(
        std::unordered_map<uint64_t, collision::ContactManifold>& manifoldMap,
        const std::unordered_map<uint64_t, size_t>& entityToIsland,
        size_t islandCount);

    using ConstraintRef = Core::ConstraintPtrVariant;

    void collectIslandConstraints(
        const std::vector<ecs::Entity>& island,
        std::vector<ConstraintRef>& localConstraints);

    std::vector<std::vector<ConstraintRef>> routeIslandConstraints(
        const std::unordered_map<uint64_t, size_t>& entityToIsland,
        size_t islandCount);

    void solveSingleIsland(
        const std::vector<collision::ContactManifold*>& localManifolds,
        const std::vector<ConstraintRef>& localConstraints,
        float deltaTime, int velocityIterations, int positionIterations);

    void prepareConstraint(collision::DistanceConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::DistanceConstraint& constraint);
    void solveConstraintVelocity(collision::DistanceConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::DistanceConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::RevoluteConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::RevoluteConstraint& constraint);
    void solveConstraintVelocity(collision::RevoluteConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::RevoluteConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::FixedConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::FixedConstraint& constraint);
    void solveConstraintVelocity(collision::FixedConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::FixedConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::BallSocketConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::BallSocketConstraint& constraint);
    void solveConstraintVelocity(collision::BallSocketConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::BallSocketConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::PrismaticConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::PrismaticConstraint& constraint);
    void solveConstraintVelocity(collision::PrismaticConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::PrismaticConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::SpringConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::SpringConstraint& constraint);
    void solveConstraintVelocity(collision::SpringConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::SpringConstraint& constraint, float deltaTime);

    void prepareConstraint(collision::ConeTwistConstraint& constraint, float deltaTime);
    void warmStartConstraint(collision::ConeTwistConstraint& constraint);
    void solveConstraintVelocity(collision::ConeTwistConstraint& constraint, float deltaTime);
    void solveConstraintPosition(collision::ConeTwistConstraint& constraint, float deltaTime);

    void resolveSpringConstraint(collision::SpringConstraint& constraint, float deltaTime);
    void solveSpringConstraintPosition(collision::SpringConstraint& constraint, float deltaTime);
    void resolveConeTwistConstraint(collision::ConeTwistConstraint& constraint, float deltaTime);
    void solveConeTwistConstraintPosition(collision::ConeTwistConstraint& constraint, float deltaTime);

    void collectIslandConstraints(
        const std::vector<ecs::Entity>& island,
        std::vector<collision::DistanceConstraint*>& localDistance,
        std::vector<collision::RevoluteConstraint*>& localRevolute);

    std::vector<std::vector<collision::DistanceConstraint*>> routeIslandDistanceConstraints(
        const std::unordered_map<uint64_t, size_t>& entityToIsland,
        size_t islandCount);

    std::vector<std::vector<collision::RevoluteConstraint*>> routeIslandRevoluteConstraints(
        const std::unordered_map<uint64_t, size_t>& entityToIsland,
        size_t islandCount);

    void solveSingleIsland(
        const std::vector<collision::ContactManifold*>& localManifolds,
        const std::vector<collision::DistanceConstraint*>& localDist,
        const std::vector<collision::RevoluteConstraint*>& localRev,
        float deltaTime, int velocityIterations, int positionIterations);

    void solveContactNormal(
        collision::ContactPoint& contact,
        const Vector3& rA, const Vector3& rB, const Vector3& n,
        const Vector3& invMassA, const Vector3& invMassB,
        const Matrix3x3& invIA, const Matrix3x3& invIB,
        Core::Motion& motionA, Core::Motion& motionB);

    void solveContactFriction(
        collision::ContactPoint& contact,
        const Vector3& rA, const Vector3& rB,
        const Vector3& t1, const Vector3& t2,
        float friction,
        const Vector3& invMassA, const Vector3& invMassB,
        const Matrix3x3& invIA, const Matrix3x3& invIB,
        Core::Motion& motionA, Core::Motion& motionB);

    void applyRollingResistance(
        float friction, float totalNormalImpulse,
        const Matrix3x3& invIA, const Matrix3x3& invIB,
        Core::Motion& motionA, Core::Motion& motionB);

public:
    explicit SequentialImpulsesKernel(Core::Registry& registry);

    /**
     * @brief Eklem kırılma olaylarını dinleyecek kullanıcı dinleyicisini bağlar.
     * @param listener Dinleyici nesnesi göstericisi.
     */
    void setJointBreakListener(JointBreakListener* listener) {
        mJointBreakListener = listener;
    }

    /**
     * @brief Eklem kırılma olaylarında çağrılacak lambda/fonksiyonu bağlar.
     * @param callback Geri çağırım fonksiyonu.
     */
    void setJointBreakCallback(std::function<void(ecs::Entity, ecs::Entity, float)> callback) {
        mJointBreakCallback = std::move(callback);
    }

    void execute(
        const std::vector<ecs::Entity>& island,
        std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime,
        int velocityIterations = 10,
        int positionIterations = 4
    );

    void executeIslands(
        const std::vector<std::vector<ecs::Entity>>& islands,
        std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime,
        int velocityIterations = 10,
        int positionIterations = 4
    );

    void solveIsland(
        const std::vector<ecs::Entity>& island,
        std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
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
    void solveRevoluteConstraintPosition(collision::RevoluteConstraint& constraint, float deltaTime);
    void resolveFixedConstraint(collision::FixedConstraint& constraint, float deltaTime);
    void solveFixedConstraintPosition(collision::FixedConstraint& constraint, float deltaTime);
    void resolveBallSocketConstraint(collision::BallSocketConstraint& constraint, float deltaTime);
    void solveBallSocketConstraintPosition(collision::BallSocketConstraint& constraint, float deltaTime);
    void resolvePrismaticConstraint(collision::PrismaticConstraint& constraint, float deltaTime);
    void solvePrismaticConstraintPosition(collision::PrismaticConstraint& constraint, float deltaTime);
};

} // namespace Baryon::systems
