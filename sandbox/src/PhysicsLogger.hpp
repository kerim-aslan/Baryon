#pragma once
#include <iostream>
#include <string>
#include <cmath>
#include <algorithm>

namespace PhysicsLogger {

    enum class LogLevel { INFO, WARNING, ERROR };

    inline void log(LogLevel level, const std::string& msg) {
        switch (level) {
            case LogLevel::INFO:    std::cout << "[INFO] " << msg << "\n"; break;
            case LogLevel::WARNING: std::cout << "\033[33m[WARNING] " << msg << "\033[0m\n"; break;
            case LogLevel::ERROR:   std::cout << "\033[31m[ERROR] " << msg << "\033[0m\n"; break;
        }
    }

    // Görev 1: Scale Validation Logger
    // Fiziksel kutu/şekil ile görsel kutunun (scale) arasındaki farkı kontrol eder.
    template<typename Vec3>
    inline void validateScale(const std::string& objectName, const Vec3& physScale, const Vec3& gfxScale, float threshold = 0.05f) {
        float diffX = std::abs(physScale.x - gfxScale.x) / std::max(physScale.x, 0.0001f);
        float diffY = std::abs(physScale.y - gfxScale.y) / std::max(physScale.y, 0.0001f);
        float diffZ = std::abs(physScale.z - gfxScale.z) / std::max(physScale.z, 0.0001f);

        if (diffX > threshold || diffY > threshold || diffZ > threshold) {
            log(LogLevel::WARNING, "SCALE MISMATCH in " + objectName + 
                "!\n  Phys: (" + std::to_string(physScale.x) + ", " + std::to_string(physScale.y) + ", " + std::to_string(physScale.z) + ")" +
                "\n  Gfx : (" + std::to_string(gfxScale.x) + ", " + std::to_string(gfxScale.y) + ", " + std::to_string(gfxScale.z) + ")" +
                "\n  Diff: > " + std::to_string(threshold * 100.0f) + "%");
        }
    }

    // Görev 1: AABB Print Logger
    template<typename Vec3>
    inline void logAABB(const std::string& objectName, const Vec3& minBounds, const Vec3& maxBounds) {
        log(LogLevel::INFO, "AABB [" + objectName + "]\n  Min: (" + 
            std::to_string(minBounds.x) + ", " + std::to_string(minBounds.y) + ", " + std::to_string(minBounds.z) + 
            ")\n  Max: (" + 
            std::to_string(maxBounds.x) + ", " + std::to_string(maxBounds.y) + ", " + std::to_string(maxBounds.z) + ")");
    }

    // Görev 5: Velocity & Tunneling Check Logger
    // Objelerin bir karede ne kadar hareket ettiğini ve içinden geçme riski olup olmadığını raporlar.
    template<typename Vec3>
    inline void checkVelocityTunneling(const std::string& objectName, const Vec3& velocity, float dt, float minThickness) {
        float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
        float distancePerFrame = speed * dt;
        
        if (distancePerFrame > minThickness) {
            log(LogLevel::WARNING, "TUNNELING RISK for " + objectName + 
                "! Moves " + std::to_string(distancePerFrame) + " units per frame, larger than thickness " + std::to_string(minThickness) + 
                ". Sub-stepping or CCD is recommended.");
        }
    }
    
    // Görev 4: EPA Normal Logger
    // Çarpışma sırasında elde edilen normal ve penetrasyon (içe geçme) miktarını raporlar.
    template<typename Vec3>
    inline void logCollisionNormal(const std::string& bodyA, const std::string& bodyB, const Vec3& normal, float penetration) {
        log(LogLevel::INFO, "EPA NORMAL [" + bodyA + " <-> " + bodyB + "]\n  Normal: (" + 
            std::to_string(normal.x) + ", " + std::to_string(normal.y) + ", " + std::to_string(normal.z) + 
            ")\n  Penetration: " + std::to_string(penetration));
    }

} // namespace PhysicsLogger
