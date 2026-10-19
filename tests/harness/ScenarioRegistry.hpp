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

#include "BaselineScenarios.hpp"
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <functional>

/**
 * @file ScenarioRegistry.hpp
 * @brief Senaryo kayıt ve fabrika yöneticisi.
 */
namespace Baryon::harness {

class ScenarioRegistry {
public:
    using FactoryFunc = std::function<std::unique_ptr<IScenario>()>;

private:
    std::map<std::string, FactoryFunc> mFactories;

    ScenarioRegistry() {
        registerScenario<SlideFrictionScenario>("slide_friction");
        registerScenario<SphereRestitutionScenario>("sphere_restitution");
        registerScenario<DistanceJointScenario>("distance_joint");
        registerScenario<HeavyOnLightStackScenario>("heavy_on_light_stack");
        registerScenario<CcdBulletScenario>("ccd_bullet");
        registerScenario<Determinism40Scenario>("determinism_40");
    }

public:
    static ScenarioRegistry& instance() {
        static ScenarioRegistry reg;
        return reg;
    }

    template <typename T>
    void registerScenario(const std::string& name) {
        mFactories[name] = []() { return std::make_unique<T>(); };
    }

    [[nodiscard]] std::unique_ptr<IScenario> create(const std::string& name) const {
        auto it = mFactories.find(name);
        if (it != mFactories.end()) {
            return it->second();
        }
        return nullptr;
    }

    [[nodiscard]] std::vector<std::string> getRegisteredNames() const {
        std::vector<std::string> names;
        names.reserve(mFactories.size());
        for (const auto& [name, _] : mFactories) {
            names.push_back(name);
        }
        return names;
    }

    [[nodiscard]] bool hasScenario(const std::string& name) const {
        return mFactories.find(name) != mFactories.end();
    }
};

} // namespace Baryon::harness
