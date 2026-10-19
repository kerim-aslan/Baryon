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
#include <string>
#include <vector>
#include <map>

/**
 * @file HarnessRunner.hpp
 * @brief Senaryo koşturucu, metrik kayıt ve JSON/CSV raporlayıcı.
 */
namespace Baryon::harness {

/**
 * @struct HarnessOptions
 * @brief Senaryo koşturma seçenekleri ve dosya çıktı parametreleri.
 */
struct HarnessOptions {
    std::string scenarioName;  ///< Çalıştırılacak senaryo adı.
    int steps{-1};             ///< Adım sayısı (-1 ise senaryonun varsayılanı).
    std::string jsonPath;      ///< JSON çıktısı için dosya yolu (boşsa yazılmaz).
    std::string csvPath;       ///< CSV çıktısı için dosya yolu (boşsa yazılmaz).
    bool verbose{false};       ///< Ayrıntılı konsol çıktısı.
};

/**
 * @struct HarnessResult
 * @brief Senaryo çalıştırma sonucu ve toplanan metrikler.
 */
struct HarnessResult {
    std::string scenarioName;
    bool passed{false};
    std::string message;
    std::vector<StepMetrics> history;
    std::map<std::string, double> customMetrics;
};

/**
 * @class HarnessRunner
 * @brief Senaryoları izole bir Simulator içinde çalıştırıp metrikleri toplayan sınıf.
 */
class HarnessRunner {
public:
    /**
     * @brief Belirtilen senaryoyu koşturur ve sonuçları döndürür.
     */
    static HarnessResult run(IScenario& scenario, const HarnessOptions& options);

    /**
     * @brief Sonuçları JSON formatında dosyaya yazar.
     */
    static bool exportJson(const HarnessResult& result, const std::string& path);

    /**
     * @brief Birden fazla senaryo sonucunu tek bir JSON dosyasına yazar.
     */
    static bool exportJsonMulti(const std::vector<HarnessResult>& results, const std::string& path);

    /**
     * @brief Sonuçları CSV formatında dosyaya yazar (her satır: senaryo adı, adım, metrikler).
     */
    static bool exportCsv(const std::vector<HarnessResult>& results, const std::string& path);
};

} // namespace Baryon::harness
