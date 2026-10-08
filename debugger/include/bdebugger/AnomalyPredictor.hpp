/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include "DumpFormat.hpp"
#include <unordered_map>
#include <cmath>

namespace Baryon::Debugger {

struct AnomalyThresholds {
    float maxSafeVelocityVsExtentRatio{0.75f}; // If vel*dt > 0.75 * minDimension, risk of tunneling
    float maxAllowedPenetration{0.08f};        // > 8cm penetration considered excessive
    float energyExplosionFactor{5.0f};         // 5x unexpected increase in kinetic energy
    float maxImpulseJitterThreshold{80.0f};     // Normal impulse variance indicator
    float sleepSpeedThreshold{0.05f};          // Moving faster than this while sleeping is an anomaly
    float constraintToleranceRatio{0.15f};     // 15% stretch on distance constraint is strain
};

class AnomalyPredictor {
private:
    AnomalyThresholds mThresholds;
    float mPreviousTotalKineticEnergy{0.0f};
    bool mHasPreviousFrame{false};
    
    struct EntityPrevState {
        float posX{0.0f}, posY{0.0f}, posZ{0.0f};
        float linVelX{0.0f}, linVelY{0.0f}, linVelZ{0.0f};
    };
    std::unordered_map<uint32_t, EntityPrevState> mPrevEntityStates;

public:
    AnomalyPredictor() = default;
    explicit AnomalyPredictor(const AnomalyThresholds& thresholds) : mThresholds(thresholds) {}

    void setThresholds(const AnomalyThresholds& th) { mThresholds = th; }
    [[nodiscard]] const AnomalyThresholds& getThresholds() const { return mThresholds; }

    void reset() {
        mPreviousTotalKineticEnergy = 0.0f;
        mHasPreviousFrame = false;
        mPrevEntityStates.clear();
    }

    /**
     * @brief Evaluates the frame and appends any detected anomalies directly to frame.anomalies.
     */
    void evaluate(DumpFrame& frame);
};

} // namespace Baryon::Debugger
