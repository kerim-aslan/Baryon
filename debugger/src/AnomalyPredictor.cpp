/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/AnomalyPredictor.hpp"
#include <cmath>
#include <sstream>

namespace Baryon::Debugger {

static inline float getVectorLength(float x, float y, float z) {
    return std::sqrt(x * x + y * y + z * z);
}

void AnomalyPredictor::evaluate(DumpFrame& frame) {
    // 1. Check for Numerical Instability (NaN / Inf) and Kinematics
    for (const auto& e : frame.entities) {
        if (std::isnan(e.posX) || std::isnan(e.posY) || std::isnan(e.posZ) ||
            std::isinf(e.posX) || std::isinf(e.posY) || std::isinf(e.posZ) ||
            std::isnan(e.linVelX) || std::isnan(e.linVelY) || std::isnan(e.linVelZ) ||
            std::isinf(e.linVelX) || std::isinf(e.linVelY) || std::isinf(e.linVelZ)) {
            
            AnomalyRecord a;
            a.frameIndex = frame.frameIndex;
            a.entityId = e.id;
            a.type = AnomalyType::NumericalInstability;
            a.severity = AnomalySeverity::Critical;
            a.description = "Entity state contains NaN or Infinity values!";
            frame.anomalies.push_back(a);
        }

        // 2. Tunneling Risk Check
        if (e.bodyType == 2 && !e.isSleeping) { // Dynamic
            float speed = getVectorLength(e.linVelX, e.linVelY, e.linVelZ);
            float displacement = speed * frame.timeStep;
            
            // Minimum shape dimension
            float minExtent = 0.5f;
            if (e.shapeKind == ShapeKind::Box) {
                minExtent = std::min({e.shapeDimX, e.shapeDimY, e.shapeDimZ}) * 2.0f;
            } else if (e.shapeKind == ShapeKind::Sphere || e.shapeKind == ShapeKind::Capsule) {
                minExtent = e.shapeDimX * 2.0f; // radius * 2
            }

            if (minExtent > 0.001f && (displacement > minExtent * mThresholds.maxSafeVelocityVsExtentRatio)) {
                AnomalyRecord a;
                a.frameIndex = frame.frameIndex;
                a.entityId = e.id;
                a.type = AnomalyType::TunnelingRisk;
                a.severity = (!e.useCCD) ? AnomalySeverity::Critical : AnomalySeverity::Warning;
                
                std::ostringstream ss;
                ss << "High speed (" << speed << " m/s) with single-step displacement (" << displacement 
                   << "m) exceeds safety limit (" << (minExtent * mThresholds.maxSafeVelocityVsExtentRatio) 
                   << "m). CCD: " << (e.useCCD ? "ENABLED" : "DISABLED (HIGH TUNNELING RISK)");
                a.description = ss.str();
                frame.anomalies.push_back(a);
            }
        }

        // 3. Sleeping Anomaly Check
        if (e.isSleeping) {
            float speed = getVectorLength(e.linVelX, e.linVelY, e.linVelZ);
            if (speed > mThresholds.sleepSpeedThreshold) {
                AnomalyRecord a;
                a.frameIndex = frame.frameIndex;
                a.entityId = e.id;
                a.type = AnomalyType::SleepingAnomaly;
                a.severity = AnomalySeverity::Warning;
                std::ostringstream ss;
                ss << "Entity is marked sleeping but has non-trivial velocity (" << speed << " m/s)";
                a.description = ss.str();
                frame.anomalies.push_back(a);
            }
        }
    }

    // 4. Contact & Solver Penetration Checks
    for (const auto& c : frame.contacts) {
        if (c.penetration > mThresholds.maxAllowedPenetration) {
            AnomalyRecord a;
            a.frameIndex = frame.frameIndex;
            a.entityId = c.entityA;
            a.type = AnomalyType::ExcessivePenetration;
            a.severity = (c.penetration > mThresholds.maxAllowedPenetration * 2.0f) ? 
                         AnomalySeverity::Critical : AnomalySeverity::Warning;
            std::ostringstream ss;
            ss << "Excessive penetration depth: " << (c.penetration * 100.0f) 
               << " cm between Entity " << c.entityA << " and Entity " << c.entityB;
            a.description = ss.str();
            frame.anomalies.push_back(a);
        }

        if (c.normalImpulse > mThresholds.maxImpulseJitterThreshold) {
            AnomalyRecord a;
            a.frameIndex = frame.frameIndex;
            a.entityId = c.entityA;
            a.type = AnomalyType::SolverJitter;
            a.severity = AnomalySeverity::Warning;
            std::ostringstream ss;
            ss << "High impulse spike (" << c.normalImpulse << " N*s) indicates possible contact jitter or bounce!";
            a.description = ss.str();
            frame.anomalies.push_back(a);
        }
    }

    // 5. Total Energy Explosion Check
    if (mHasPreviousFrame && mPreviousTotalKineticEnergy > 0.05f) {
        float ratio = frame.totalKineticEnergy / mPreviousTotalKineticEnergy;
        if (ratio > mThresholds.energyExplosionFactor && frame.totalKineticEnergy > 5.0f) {
            AnomalyRecord a;
            a.frameIndex = frame.frameIndex;
            a.entityId = 0xFFFFFFFF;
            a.type = AnomalyType::NumericalInstability;
            a.severity = AnomalySeverity::Critical;
            std::ostringstream ss;
            ss << "Sudden kinetic energy spike (" << mPreviousTotalKineticEnergy << " J -> " 
               << frame.totalKineticEnergy << " J, x" << ratio << ") detected!";
            a.description = ss.str();
            frame.anomalies.push_back(a);
        }
    }

    // Update history
    mPreviousTotalKineticEnergy = frame.totalKineticEnergy;
    mHasPreviousFrame = true;
    mPrevEntityStates.clear();
    for (const auto& e : frame.entities) {
        mPrevEntityStates[e.id] = { e.posX, e.posY, e.posZ, e.linVelX, e.linVelY, e.linVelZ };
    }
}

} // namespace Baryon::Debugger
