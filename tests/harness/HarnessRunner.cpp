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

#include "HarnessRunner.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace Baryon::harness {

HarnessResult HarnessRunner::run(IScenario& scenario, const HarnessOptions& options) {
    HarnessResult result;
    result.scenarioName = scenario.getName();

    Simulator sim;
    sim.setAccelerationField(scenario.getGravity());
    float dt = scenario.getTimeStep();
    sim.setFixedTimeStep(dt);

    scenario.setup(sim);

    int totalSteps = (options.steps > 0) ? options.steps : scenario.getDefaultSteps();
    result.history.reserve(totalSteps);

    for (int step = 0; step < totalSteps; ++step) {
        StepMetrics sm;
        sm.stepIndex = step;
        sm.time = static_cast<double>(step) * static_cast<double>(dt);
        sm.kineticEnergy = computeKineticEnergy(sim);
        sm.potentialEnergy = computePotentialEnergy(sim, scenario.getGravity());
        sm.totalEnergy = sm.kineticEnergy + sm.potentialEnergy;
        sm.linearMomentum = computeLinearMomentum(sim);
        sm.angularMomentum = computeAngularMomentum(sim);
        sm.maxPenetration = computeMaxPenetration(sim);
        sm.stateHash = computeStateHash(sim);

        scenario.stepCallback(sim, step, sm);
        result.history.push_back(sm);

        sim.step(dt);
    }

    result.passed = scenario.evaluate(result.history, result.message);
    result.customMetrics = scenario.getCustomMetrics();

    if (options.verbose) {
        std::cout << "[Harness] Scenario: " << result.scenarioName
                  << " | Steps: " << totalSteps
                  << " | Result: " << (result.passed ? "PASSED" : "FAILED")
                  << " | " << result.message << std::endl;
    }

    if (!options.jsonPath.empty()) {
        exportJson(result, options.jsonPath);
    }
    if (!options.csvPath.empty()) {
        exportCsv({result}, options.csvPath);
    }

    return result;
}

bool HarnessRunner::exportJson(const HarnessResult& result, const std::string& path) {
    return exportJsonMulti({result}, path);
}

bool HarnessRunner::exportJsonMulti(const std::vector<HarnessResult>& results, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "[Harness] Dosya yazilamadi: " << path << std::endl;
        return false;
    }

    out << "{\n  \"scenarios\": [\n";
    for (size_t r = 0; r < results.size(); ++r) {
        const auto& res = results[r];
        out << "    {\n";
        out << "      \"name\": \"" << res.scenarioName << "\",\n";
        out << "      \"passed\": " << (res.passed ? "true" : "false") << ",\n";
        out << "      \"message\": \"" << res.message << "\",\n";
        out << "      \"steps_count\": " << res.history.size() << ",\n";

        // Custom metrics
        out << "      \"custom_metrics\": {\n";
        size_t cmIdx = 0;
        for (const auto& [k, v] : res.customMetrics) {
            out << "        \"" << k << "\": " << std::setprecision(8) << v;
            if (++cmIdx < res.customMetrics.size()) out << ",";
            out << "\n";
        }
        out << "      },\n";

        // Steps array
        out << "      \"steps\": [\n";
        for (size_t i = 0; i < res.history.size(); ++i) {
            const auto& s = res.history[i];
            out << "        {\n"
                << "          \"step\": " << s.stepIndex << ",\n"
                << "          \"time\": " << std::setprecision(6) << s.time << ",\n"
                << "          \"kinetic_energy\": " << std::setprecision(6) << s.kineticEnergy << ",\n"
                << "          \"potential_energy\": " << std::setprecision(6) << s.potentialEnergy << ",\n"
                << "          \"total_energy\": " << std::setprecision(6) << s.totalEnergy << ",\n"
                << "          \"linear_momentum\": [" << s.linearMomentum.x << ", " << s.linearMomentum.y << ", " << s.linearMomentum.z << "],\n"
                << "          \"linear_momentum_mag\": " << s.linearMomentum.length() << ",\n"
                << "          \"angular_momentum\": [" << s.angularMomentum.x << ", " << s.angularMomentum.y << ", " << s.angularMomentum.z << "],\n"
                << "          \"angular_momentum_mag\": " << s.angularMomentum.length() << ",\n"
                << "          \"max_penetration\": " << std::setprecision(6) << s.maxPenetration << ",\n"
                << "          \"state_hash\": \"" << hashToHexString(s.stateHash) << "\"\n"
                << "        }";
            if (i + 1 < res.history.size()) out << ",";
            out << "\n";
        }
        out << "      ]\n";
        out << "    }";
        if (r + 1 < results.size()) out << ",";
        out << "\n";
    }
    out << "  ]\n}\n";

    return true;
}

bool HarnessRunner::exportCsv(const std::vector<HarnessResult>& results, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "[Harness] CSV dosyasi yazilamadi: " << path << std::endl;
        return false;
    }

    out << "scenario,step,time,kinetic_energy,potential_energy,total_energy,"
        << "linear_momentum_x,linear_momentum_y,linear_momentum_z,linear_momentum_mag,"
        << "angular_momentum_x,angular_momentum_y,angular_momentum_z,angular_momentum_mag,"
        << "max_penetration,state_hash\n";

    for (const auto& res : results) {
        for (const auto& s : res.history) {
            out << res.scenarioName << ","
                << s.stepIndex << ","
                << std::setprecision(6) << s.time << ","
                << s.kineticEnergy << ","
                << s.potentialEnergy << ","
                << s.totalEnergy << ","
                << s.linearMomentum.x << ","
                << s.linearMomentum.y << ","
                << s.linearMomentum.z << ","
                << s.linearMomentum.length() << ","
                << s.angularMomentum.x << ","
                << s.angularMomentum.y << ","
                << s.angularMomentum.z << ","
                << s.angularMomentum.length() << ","
                << s.maxPenetration << ","
                << hashToHexString(s.stateHash) << "\n";
        }
    }

    return true;
}

} // namespace Baryon::harness
