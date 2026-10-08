/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include "DumpRecorder.hpp"
#include "DumpReader.hpp"
#include "TestScenes.hpp"
#include "Renderer3D.hpp"

#include <Baryon/Simulator.hpp>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;

namespace Baryon::Debugger {

class DebuggerEventLogger : public CollisionListener {
public:
    struct EventLogItem {
        std::string message;
        bool isTrigger;
    };
    std::vector<EventLogItem> logItems;

    void onCollisionEnter(ecs::Entity a, ecs::Entity b, const collision::ContactManifold& manifold) override {
        (void)manifold;
        logItems.push_back({"[CARPISMA GIRIS] Varlik #" + std::to_string(a.id) + " <-> #" + std::to_string(b.id), false});
        if (logItems.size() > 80) logItems.erase(logItems.begin());
    }
    void onCollisionExit(ecs::Entity a, ecs::Entity b) override {
        logItems.push_back({"[CARPISMA CIKIS] Varlik #" + std::to_string(a.id) + " <-> #" + std::to_string(b.id), false});
        if (logItems.size() > 80) logItems.erase(logItems.begin());
    }
    void onTriggerEnter(ecs::Entity trigger, ecs::Entity other) override {
        logItems.push_back({"[TETIKLEYICI GIRIS] Alan #" + std::to_string(trigger.id) + " <- Nesne #" + std::to_string(other.id), true});
        if (logItems.size() > 80) logItems.erase(logItems.begin());
    }
    void onTriggerExit(ecs::Entity trigger, ecs::Entity other) override {
        logItems.push_back({"[TETIKLEYICI CIKIS] Alan #" + std::to_string(trigger.id) + " <- Nesne #" + std::to_string(other.id), true});
        if (logItems.size() > 80) logItems.erase(logItems.begin());
    }
};

enum class AppMode {
    LiveSimulation,
    ReplayPlayback
};

class DebuggerApp {
private:
    GLFWwindow* mWindow{nullptr};
    int mWindowWidth{1600};
    int mWindowHeight{900};

    // State
    AppMode mMode{AppMode::LiveSimulation};
    bool mIsPaused{false};
    float mSimSpeed{1.0f};
    bool mStepOnce{false};

    // Physics Engine & Recording
    std::unique_ptr<Simulator> mSimulator;
    DebuggerEventLogger mEventLogger;
    DumpRecorder mRecorder;
    DumpReader mReader;

    // 3D Rendering
    Renderer3D mRenderer;
    int mViewportWidth{1280};
    int mViewportHeight{720};
    bool mIsViewportHovered{false};

    // Active Scene
    SceneType mCurrentScene{SceneType::TriggerSensors};

    // Visualization Options
    bool mShowGrid{true};
    bool mShowAxes{true};
    bool mShowShapes{true};
    bool mShowContacts{true};
    bool mShowVelocities{true};

    // Live Raycast state
    float mRaycastAngle{0.0f};
    bool mRaycastAutoSweep{true};

    // File IO status messages
    std::string mStatusMessage{"Ready."};
    float mStatusTimer{0.0f};

    // Selected Entity for inspection
    uint32_t mSelectedEntityId{0xFFFFFFFF};

public:
    DebuggerApp();
    ~DebuggerApp();

    bool init();
    void run();

private:
    void processInput(float dt);
    void updatePhysics(float dt);
    void render3DView();

    // UI Panels
    void renderUI();
    void renderMenuBar();
    void renderViewportPanel();
    void renderControlPanel();
    void renderDiagnosticPanel();
    void renderInspectorPanel();
    void renderTimelinePanel();

    void loadScene(SceneType scene);
    void setStatus(const std::string& msg);
};

} // namespace Baryon::Debugger
