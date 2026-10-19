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

#include "Scenario.hpp"
#include "Baryon/collision/CollisionShape.hpp"
#include <iostream>
#include <cmath>
#include <vector>

/**
 * @file BaselineScenarios.hpp
 * @brief Bölüm 2.4 referans regresyon senaryolarının somut uygulamaları.
 */
namespace Baryon::harness {

/**
 * @class SlideFrictionScenario
 * @brief Kutu, µ=0.5, v0=5 m/s zeminde kayar senaryosu.
 */
class SlideFrictionScenario : public IScenario {
private:
    ecs::Entity mBoxEntity{0};
    double mFinalX{0.0};

public:
    std::string getName() const override { return "slide_friction"; }
    std::string getDescription() const override { return "Kutu, mu=0.5, v0=5 m/s zeminde kayar (durdugu x ~= 2.51 m, teorik 2.55)"; }
    int getDefaultSteps() const override { return 300; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        // Zemin: y = -0.5, half-extent 0.5 -> üst yüzey y = 0
        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);
        (void)ground.setMaterial(0.5f, 0.0f);

        // Kutu: half-extents 0.5 (1x1x1), merkez y = 0.5, v0 = 5 m/s
        Body box = sim.createBody(Pose(Vector3(0, 0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
        (void)box.setMass(1.0f);
        (void)box.setMaterial(0.5f, 0.0f);
        // Hava sürtünmesi olmaksızın saf coulomb sürtünmesi (teorik 2.55)
        sim.getRegistry().getComponent<Core::Motion>(box.getEntity()).linearDamping = 0.0f;
        (void)box.setLinearVelocity(Vector3(5.0f, 0.0f, 0.0f));

        mBoxEntity = box.getEntity();
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)step;
        (void)metrics;
        if (sim.getRegistry().isAlive(mBoxEntity)) {
            mFinalX = sim.getRegistry().getComponent<Pose>(mBoxEntity).position.x;
        }
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        // Beklenen: durduğu x ≈ 2.51 - 2.53 m (teorik 2.55, ayrık zaman sapması)
        if (mFinalX >= 2.45 && mFinalX <= 2.60) {
            outMessage = "OK: Final X = " + std::to_string(mFinalX) + " m (beklenen ~2.51-2.55 m)";
            return true;
        }
        outMessage = "FAIL: Final X = " + std::to_string(mFinalX) + " m aralik disinda [2.45, 2.60]";
        return false;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"final_x", mFinalX} };
    }
};

/**
 * @class SphereRestitutionScenario
 * @brief Küre, e=0.8, 5 m'den düşer, 2. tepe (1. sıçrama apeksi) ≈ 3.10 m senaryosu.
 */
class SphereRestitutionScenario : public IScenario {
private:
    ecs::Entity mSphereEntity{0};
    double mPeak1{0.0};
    double mPeak2{0.0};
    int mBounceCount{0};
    bool mWasMovingDown{true};

public:
    std::string getName() const override { return "sphere_restitution"; }
    std::string getDescription() const override { return "Kure, e=0.8, 5 m'den duser, 2. tepe ~= 3.10 m (sonumleme etkisi)"; }
    int getDefaultSteps() const override { return 300; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);
        (void)ground.setMaterial(0.0f, 0.8f);

        // Küre R = 0.5, zemin üstünden 5m -> başlangıç y = 5.5
        Body ball = sim.createBody(Pose(Vector3(0, 5.5f, 0)), collision::CollisionShape{collision::SphereShape(0.5f)});
        (void)ball.setBodyType(Core::BodyType::Dynamic);
        (void)ball.setMass(1.0f);
        (void)ball.setMaterial(0.0f, 0.8f);
        (void)ball.setCCD(false);

        mSphereEntity = ball.getEntity();
        mBounceCount = 0;
        mPeak1 = 0.0;
        mPeak2 = 0.0;
        mWasMovingDown = true;
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)step;
        (void)metrics;
        if (!sim.getRegistry().isAlive(mSphereEntity)) return;

        const auto& pose = sim.getRegistry().getComponent<Pose>(mSphereEntity);
        const auto& motion = sim.getRegistry().getComponent<Core::Motion>(mSphereEntity);

        if (motion.linearVelocity.y > 0.05f) {
            mWasMovingDown = false;
        } else if (!mWasMovingDown && motion.linearVelocity.y <= 0.0f) {
            mBounceCount++;
            double apex = pose.position.y - 0.5f;
            if (mBounceCount == 1) mPeak1 = apex;
            if (mBounceCount == 2) mPeak2 = apex;
            mWasMovingDown = true;
        }
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        // Peak1 (ilk sıçrama apeksi) ≈ 3.10 - 3.12 m
        if (mPeak1 >= 3.00 && mPeak1 <= 3.25) {
            outMessage = "OK: Peak 1 = " + std::to_string(mPeak1) + " m (beklenen ~3.10 m)";
            return true;
        }
        outMessage = "FAIL: Peak 1 = " + std::to_string(mPeak1) + " m aralik disinda [3.00, 3.25]";
        return false;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"peak_1", mPeak1}, {"peak_2", mPeak2} };
    }
};

/**
 * @class DistanceJointScenario
 * @brief Mesafe eklemi, uzunluk 2.0m, asılı kalma (drift kontrolü) senaryosu.
 */
class DistanceJointScenario : public IScenario {
private:
    ecs::Entity mAnchorEntity{0};
    ecs::Entity mBobEntity{0};
    double mMinDist{1e9};
    double mMaxDist{-1e9};
    double mFinalDist{0.0};

public:
    std::string getName() const override { return "distance_joint"; }
    std::string getDescription() const override { return "Mesafe eklemi, uzunluk 2.0 (uzunluk 2.000-2.013 arasi drift)"; }
    int getDefaultSteps() const override { return 180; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        Body anchor = sim.createBody(Pose(Vector3(0, 10.0f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
        (void)anchor.setBodyType(Core::BodyType::Static);

        float targetDist = 2.0f;
        Body bob = sim.createBody(Pose(Vector3(0.0f, 10.0f - targetDist, 0)), collision::CollisionShape{collision::SphereShape(0.3f)});
        (void)bob.setBodyType(Core::BodyType::Dynamic);
        (void)bob.setMass(2.0f);

        sim.createDistanceConstraint(anchor, bob, targetDist);

        mAnchorEntity = anchor.getEntity();
        mBobEntity = bob.getEntity();
        mMinDist = 1e9;
        mMaxDist = -1e9;
        mFinalDist = 0.0;
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)step;
        (void)metrics;
        if (sim.getRegistry().isAlive(mAnchorEntity) && sim.getRegistry().isAlive(mBobEntity)) {
            Vector3 pA = sim.getRegistry().getComponent<Pose>(mAnchorEntity).position;
            Vector3 pB = sim.getRegistry().getComponent<Pose>(mBobEntity).position;
            double d = (pB - pA).length();
            if (d < mMinDist) mMinDist = d;
            if (d > mMaxDist) mMaxDist = d;
            mFinalDist = d;
        }
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        if (mMaxDist <= 2.02 && mMinDist >= 1.98) {
            outMessage = "OK: Distance range [" + std::to_string(mMinDist) + ", " + std::to_string(mMaxDist) + "] m";
            return true;
        }
        outMessage = "FAIL: Distance range [" + std::to_string(mMinDist) + ", " + std::to_string(mMaxDist) + "] m [1.98, 2.02] disinda";
        return false;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"min_distance", mMinDist}, {"max_distance", mMaxDist}, {"final_distance", mFinalDist} };
    }
};

/**
 * @class HeavyOnLightStackScenario
 * @brief 100:1 kütle oranlı 2 kutu yığını (alt y=0.482, üst y=1.471) senaryosu.
 */
class HeavyOnLightStackScenario : public IScenario {
private:
    ecs::Entity mBox1Entity{0};
    ecs::Entity mBox2Entity{0};
    double mFinalY1{0.0};
    double mFinalY2{0.0};

public:
    std::string getName() const override { return "heavy_on_light_stack"; }
    std::string getDescription() const override { return "100:1 kutle oranli 2 kutu yigini (alt y=0.482, ust y=1.471)"; }
    int getDefaultSteps() const override { return 300; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(10.0f, 0.5f, 10.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        // Alt kutu 1 kg, y = 0.5
        Body box1 = sim.createBody(Pose(Vector3(0, 0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)box1.setBodyType(Core::BodyType::Dynamic);
        (void)box1.setMass(1.0f);

        // Üst kutu 100 kg, y = 1.5
        Body box2 = sim.createBody(Pose(Vector3(0, 1.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)box2.setBodyType(Core::BodyType::Dynamic);
        (void)box2.setMass(100.0f);

        mBox1Entity = box1.getEntity();
        mBox2Entity = box2.getEntity();
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)step;
        (void)metrics;
        if (sim.getRegistry().isAlive(mBox1Entity) && sim.getRegistry().isAlive(mBox2Entity)) {
            mFinalY1 = sim.getRegistry().getComponent<Pose>(mBox1Entity).position.y;
            mFinalY2 = sim.getRegistry().getComponent<Pose>(mBox2Entity).position.y;
        }
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        // Alt y in [0.470, 0.495], Üst y in [1.455, 1.485]
        bool ok1 = (mFinalY1 >= 0.470 && mFinalY1 <= 0.495);
        bool ok2 = (mFinalY2 >= 1.455 && mFinalY2 <= 1.485);
        if (ok1 && ok2) {
            outMessage = "OK: Alt y=" + std::to_string(mFinalY1) + ", Ust y=" + std::to_string(mFinalY2);
            return true;
        }
        outMessage = "FAIL: Alt y=" + std::to_string(mFinalY1) + ", Ust y=" + std::to_string(mFinalY2);
        return false;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"bottom_y", mFinalY1}, {"top_y", mFinalY2} };
    }
};

/**
 * @class CcdBulletScenario
 * @brief CCD, 500 m/s küre, 0.1 m kalınlıklı duvar senaryosu (geçmez, x≈9.86).
 */
class CcdBulletScenario : public IScenario {
private:
    ecs::Entity mBulletEntity{0};
    double mFinalX{0.0};

public:
    std::string getName() const override { return "ccd_bullet"; }
    std::string getDescription() const override { return "CCD, 500 m/s kure, 0.1 m kalinlikli duvar (gecmez, x~=9.86)"; }
    int getDefaultSteps() const override { return 15; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

        // Duvar x = 10.0, kalınlık 0.1 (half-extent x = 0.05) -> sol yüzey x = 9.95
        Body wall = sim.createBody(Pose(Vector3(10.0f, 0, 0)), collision::CollisionShape{collision::BoxShape(Vector3(0.05f, 5.0f, 5.0f))});
        (void)wall.setBodyType(Core::BodyType::Static);

        // Mermi: R = 0.1, x = 0, vx = 500 m/s, CCD açık
        Body bullet = sim.createBody(Pose(Vector3(0, 0, 0)), collision::CollisionShape{collision::SphereShape(0.1f)});
        (void)bullet.setBodyType(Core::BodyType::Dynamic);
        (void)bullet.setCCD(true);
        (void)bullet.setLinearVelocity(Vector3(500.0f, 0, 0));

        mBulletEntity = bullet.getEntity();
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)step;
        (void)metrics;
        if (sim.getRegistry().isAlive(mBulletEntity)) {
            mFinalX = sim.getRegistry().getComponent<Pose>(mBulletEntity).position.x;
        }
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        // x <= 9.95 (duvarın sol yüzeyi) ve x >= 9.80 (yaklaşık 9.86)
        if (mFinalX >= 9.80 && mFinalX <= 9.95) {
            outMessage = "OK: Bullet stopped at x = " + std::to_string(mFinalX) + " m (Wall front = 9.95)";
            return true;
        }
        outMessage = "FAIL: Bullet penetrated wall! Final x = " + std::to_string(mFinalX);
        return false;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"final_x", mFinalX} };
    }
};

/**
 * @class Determinism40Scenario
 * @brief Determinizm doğrulaması (40 cisim, 600 adım, hash karşılaştırması).
 */
class Determinism40Scenario : public IScenario {
private:
    uint64_t mFinalHash{0};

public:
    std::string getName() const override { return "determinism_40"; }
    std::string getDescription() const override { return "Determinizm (40 cisim, 600 adim, 2 kosuda ayni hash)"; }
    int getDefaultSteps() const override { return 600; }

    void setup(Simulator& sim) override {
        sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

        // Geniş zemin
        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), collision::CollisionShape{collision::BoxShape(Vector3(30.0f, 0.5f, 30.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);

        // 40 dinamik cisim (20 kutu, 20 küre), ızgara düzeninde serbest bırakılır
        for (int i = 0; i < 40; ++i) {
            float x = static_cast<float>((i % 5) - 2) * 1.5f;
            float z = (static_cast<float>((i / 5) % 4) - 1.5f) * 1.5f;
            float y = 1.0f + static_cast<float>(i / 8) * 1.2f;

            if (i % 2 == 0) {
                Body box = sim.createBody(Pose(Vector3(x, y, z)), collision::CollisionShape{collision::BoxShape(Vector3(0.4f, 0.4f, 0.4f))});
                (void)box.setBodyType(Core::BodyType::Dynamic);
                (void)box.setMass(1.0f);
            } else {
                Body sphere = sim.createBody(Pose(Vector3(x, y, z)), collision::CollisionShape{collision::SphereShape(0.35f)});
                (void)sphere.setBodyType(Core::BodyType::Dynamic);
                (void)sphere.setMass(1.2f);
            }
        }
    }

    void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) override {
        (void)sim;
        (void)step;
        mFinalHash = metrics.stateHash;
    }

    bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const override {
        (void)history;
        outMessage = "OK: Final hash = " + hashToHexString(mFinalHash);
        return true;
    }

    std::map<std::string, double> getCustomMetrics() const override {
        return { {"final_hash_low", static_cast<double>(mFinalHash & 0xFFFFFFFF)},
                 {"final_hash_high", static_cast<double>(mFinalHash >> 32)} };
    }

    [[nodiscard]] uint64_t getFinalHash() const { return mFinalHash; }
};

} // namespace Baryon::harness
