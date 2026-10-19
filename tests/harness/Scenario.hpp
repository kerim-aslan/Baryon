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

#include "Metrics.hpp"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>

/**
 * @file Scenario.hpp
 * @brief Headless test harness senaryo arayüzü ve adım metrik veri yapıları.
 * @details Her test senaryosu için kurulum, adım ilerletme, metrik toplama ve
 *          sonuç doğrulama yaşam döngüsünü soyutlar.
 */
namespace Baryon::harness {

/**
 * @struct StepMetrics
 * @brief Tek bir simülasyon adımında toplanan fiziksel metrikler.
 */
struct StepMetrics {
    int stepIndex{0};                ///< Simülasyon adım indeksi (0, 1, 2, ...).
    double time{0.0};                ///< Geçen toplam simülasyon zamanı (saniye).
    float kineticEnergy{0.0f};       ///< Toplam kinetik enerji (Joule).
    float potentialEnergy{0.0f};     ///< Toplam potansiyel enerji (Joule).
    float totalEnergy{0.0f};         ///< Toplam mekanik enerji (Joule).
    Vector3 linearMomentum{0, 0, 0}; ///< Çizgisel momentum vektörü (kg*m/s).
    Vector3 angularMomentum{0, 0, 0};///< Açısal momentum vektörü (kg*m^2/s).
    float maxPenetration{0.0f};      ///< Maksimum penetrasyon derinliği (metre).
    uint64_t stateHash{0};           ///< Durum hash'i (FNV-1a 64-bit).
};

/**
 * @class IScenario
 * @brief Test senaryoları için temel soyut sınıf.
 */
class IScenario {
public:
    virtual ~IScenario() = default;

    /** @brief Senaryonun benzersiz kimlik adı (örn: slide_friction). */
    [[nodiscard]] virtual std::string getName() const = 0;

    /** @brief Senaryonun kısa açıklaması. */
    [[nodiscard]] virtual std::string getDescription() const = 0;

    /** @brief Varsayılan adım sayısı. */
    [[nodiscard]] virtual int getDefaultSteps() const { return 300; }

    /** @brief Sabit zaman adımı boyutu (saniye, varsayılan 1/60). */
    [[nodiscard]] virtual float getTimeStep() const { return 1.0f / 60.0f; }

    /** @brief Senaryonun yerçekimi alanı (varsayılan: 0, -9.81, 0). */
    [[nodiscard]] virtual Vector3 getGravity() const { return Vector3(0.0f, -9.81f, 0.0f); }

    /**
     * @brief Fizik dünyasını ve cisimleri senaryoya göre kurar.
     * @param sim Simülatör referansı.
     */
    virtual void setup(Simulator& sim) = 0;

    /**
     * @brief Her simülasyon adımından sonra çağrılan geri çağırma fonksiyonu.
     * @param sim Simülatör referansı.
     * @param step Adım indeksi.
     * @param metrics O adımda ölçülen metrikler.
     */
    virtual void stepCallback(Simulator& sim, int step, const StepMetrics& metrics) {
        (void)sim;
        (void)step;
        (void)metrics;
    }

    /**
     * @brief Simülasyon tamamlandığında sonuçların doğruluğunu değerlendirir.
     * @param history Tüm adımların metrik geçmişi.
     * @param outMessage Başarı veya hata mesajı çıktısı.
     * @return Senaryo beklentileri karşıladıysa true, aksi halde false.
     */
    virtual bool evaluate(const std::vector<StepMetrics>& history, std::string& outMessage) const {
        (void)history;
        outMessage = "OK";
        return true;
    }

    /**
     * @brief Senaryoya özel anahtar-değer sayısal çıktıları döndürür.
     */
    [[nodiscard]] virtual std::map<std::string, double> getCustomMetrics() const {
        return {};
    }
};

} // namespace Baryon::harness
