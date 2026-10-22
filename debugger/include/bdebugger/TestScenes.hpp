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
    static void setupSandbox(Simulator& sim);
    static void setupTriggerSensors(Simulator& sim);
    static void setupCollisionFiltering(Simulator& sim);
    static void setupRevoluteHinge(Simulator& sim);
    static void setupRollingResistance(Simulator& sim);
    static void setupRaycastShowcase(Simulator& sim);
};

} // namespace Baryon::Debugger
