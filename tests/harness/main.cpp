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

#include "ScenarioRegistry.hpp"
#include "HarnessRunner.hpp"
#include <iostream>
#include <string>
#include <vector>

void printHelp() {
    std::cout << "Baryon Headless Physics Harness\n"
              << "Kullanim: BaryonHarness [secenekler]\n\n"
              << "Secenekler:\n"
              << "  --scenario <ad>    Belirtilen senaryoyu calistirir (or: slide_friction)\n"
              << "  --all              Kayitli tum senaryolari calistirir\n"
              << "  --steps <n>        Simulasyon adim sayisini belirler\n"
              << "  --json <dosya>     Metrik sonuclarini JSON formatinda kaydeder\n"
              << "  --csv <dosya>      Metrik sonuclarini CSV formatinda kaydeder\n"
              << "  --list             Kayitli tum senaryolari listeler\n"
              << "  --help             Bu yardim mesajini gosterir\n";
}

int main(int argc, char* argv[]) {
    using namespace Baryon::harness;
    auto& registry = ScenarioRegistry::instance();

    std::string scenarioName;
    bool runAll = false;
    int overrideSteps = -1;
    std::string jsonPath;
    std::string csvPath;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        } else if (arg == "--list" || arg == "-l") {
            std::cout << "Kayitli Senaryolar:\n";
            for (const auto& name : registry.getRegisteredNames()) {
                auto scen = registry.create(name);
                std::cout << "  - " << name << ": " << (scen ? scen->getDescription() : "") << "\n";
            }
            return 0;
        } else if (arg == "--all") {
            runAll = true;
        } else if (arg == "--scenario" && i + 1 < argc) {
            scenarioName = argv[++i];
        } else if (arg == "--steps" && i + 1 < argc) {
            overrideSteps = std::stoi(argv[++i]);
        } else if (arg == "--json" && i + 1 < argc) {
            jsonPath = argv[++i];
        } else if (arg == "--csv" && i + 1 < argc) {
            csvPath = argv[++i];
        } else {
            std::cerr << "Bilinmeyen secenek: " << arg << "\n";
            printHelp();
            return 1;
        }
    }

    if (!runAll && scenarioName.empty()) {
        std::cerr << "Lutfen bir senaryo secin (--scenario <ad>) veya --all / --list kullanin.\n";
        printHelp();
        return 1;
    }

    std::vector<std::string> toRun;
    if (runAll) {
        toRun = registry.getRegisteredNames();
    } else {
        if (!registry.hasScenario(scenarioName)) {
            std::cerr << "Hata: '" << scenarioName << "' adinda bir senaryo bulunamadi!\n";
            return 1;
        }
        toRun.push_back(scenarioName);
    }

    std::vector<HarnessResult> allResults;
    bool allPassed = true;

    for (const auto& name : toRun) {
        auto scen = registry.create(name);
        if (!scen) continue;

        HarnessOptions opt;
        opt.scenarioName = name;
        opt.steps = overrideSteps;
        opt.verbose = true;

        HarnessResult res = HarnessRunner::run(*scen, opt);
        if (!res.passed) {
            allPassed = false;
        }
        allResults.push_back(std::move(res));
    }

    if (!jsonPath.empty()) {
        HarnessRunner::exportJsonMulti(allResults, jsonPath);
        std::cout << "[Harness] JSON ciktisi kaydedildi: " << jsonPath << std::endl;
    }
    if (!csvPath.empty()) {
        HarnessRunner::exportCsv(allResults, csvPath);
        std::cout << "[Harness] CSV ciktisi kaydedildi: " << csvPath << std::endl;
    }

    std::cout << "\n=== HARNESS OZET: " << (allPassed ? "TUM SENARYOLAR BASARILI" : "BAZI SENARYOLAR BASARISIZ") << " ===\n";
    return allPassed ? 0 : 1;
}
