/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include <Baryon/Simulator.hpp>
#include <string>
#include <vector>

namespace Baryon::Debugger {

enum class SceneType {
    CompoundShapes,
    OverlapQueries,
    SweepQueries,
    TriggerSensors,
    CollisionFiltering,
    RevoluteHinge,
    RollingResistance,
    RaycastShowcase,
    TunnelingStress,
    BoxStacking,
    NewtonsCradle,
    JointPendulum,
    FixedConstraintWeld,
    BallSocketPendulum,
    PrismaticElevator,
    SpringSuspension,
    ConeTwistRagdoll,
    JointBreakage,
    DOFLock,
    SubsteppingTowerChain,
    SpeculativeCCD,
    DzhanibekovEffect,
    GravityScaleZeroG,
    MultithreadedStress,
    Sandbox,
    Count
};

class TestScenes {
public:
    static void loadScene(Simulator& sim, SceneType type);
    static const char* getSceneName(SceneType type);
    static const char* getSceneDescription(SceneType type);

private:
    static void setupCompoundShapes(Simulator& sim);
    static void setupOverlapQueries(Simulator& sim);
    static void setupSweepQueries(Simulator& sim);
    static void setupTunneling(Simulator& sim);
    static void setupBoxStacking(Simulator& sim);
    static void setupNewtonsCradle(Simulator& sim);
    static void setupJointPendulum(Simulator& sim);
    static void setupFixedConstraintWeld(Simulator& sim);
    static void setupBallSocketPendulum(Simulator& sim);
    static void setupPrismaticElevator(Simulator& sim);
    static void setupSpringSuspension(Simulator& sim);
    static void setupConeTwistRagdoll(Simulator& sim);
    static void setupJointBreakage(Simulator& sim);
    static void setupDOFLock(Simulator& sim);
    static void setupSubsteppingTowerChain(Simulator& sim);
    static void setupSpeculativeCCD(Simulator& sim);
    static void setupDzhanibekovEffect(Simulator& sim);
    static void setupGravityScaleZeroG(Simulator& sim);
    static void setupMultithreadedStress(Simulator& sim);
    static void setupSandbox(Simulator& sim);
    static void setupTriggerSensors(Simulator& sim);
    static void setupCollisionFiltering(Simulator& sim);
    static void setupRevoluteHinge(Simulator& sim);
    static void setupRollingResistance(Simulator& sim);
    static void setupRaycastShowcase(Simulator& sim);
};

} // namespace Baryon::Debugger
