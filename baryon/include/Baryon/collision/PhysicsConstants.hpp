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

/**
 * @file PhysicsConstants.hpp
 * @brief Fizik motorunun global sabitlerini ve tolerans değerlerini barındırır.
 * @details Sistemdeki "sihirli sayıları" (magic numbers) ortadan kaldırmak ve 
 *          farklı ölçeklerdeki projelerde fizik davranışlarını kolayca ayarlayabilmek 
 *          için tüm temel sabitler burada toplanmıştır.
 */

namespace Baryon::physics {

struct PhysicsConstants {
    // --- Çözücü (Solver) Sabitleri ---
    
    /** 
     * @brief Konum tolerans payı (Slop). 
     * @details Nesnelerin birbirinin içine geçmesine izin verilen maksimum mesafedir.
     *          Üst üste duran nesnelerdeki mikroskobik titreşimleri (jitter) önler.
     */
    static constexpr float LinearSlop = 0.01f; 

    /**
     * @brief İç içe geçme durumlarındaki pozisyon düzeltme oranı.
     * @details Kalan iç içe geçme mesafesinin her adımda yüzde kaçının düzeltileceğini belirler.
     *          Değerin çok yüksek olması yaylanmaya, düşük olması ise yavaş ayrılmaya neden olur.
     */
   static constexpr float PositionCorrectionFactor = 0.2f;

    /**
     * @brief Tek bir adımda yapılabilecek maksimum konum düzeltme miktarı.
     * @details Birbirine çok fazla girmiş nesnelerin, düzeltme sırasında aniden 
     *          uzaya fırlamasını (teleportasyon) engeller.
     */
    static constexpr float MaxLinearCorrection = 0.2f;

    // --- Uyku Sistemi (Sleep) Eşikleri ---
    
    /**
     * @brief Cismin uyku moduna geçmesi için gereken maksimum doğrusal hızın karesi.
     * @details Hareketi durma noktasına gelen cisimleri uyutarak işlemci yükünü azaltır (~0.05 m/s).
     */
    static constexpr float SleepLinearThresholdSq = 0.0025f; 

    /**
     * @brief Cismin uyku moduna geçmesi için gereken maksimum açısal hızın karesi.
     * @details Dönme hareketi bitmek üzere olan cisimleri uyutmak için kullanılır (~0.05 rad/s).
     */
    static constexpr float SleepAngularThresholdSq = 0.0025f;

    /**
     * @brief Mikro hareketleri yok saymak için kullanılan ölü bölge (dead-zone) eşiği.
     * @details Çok düşük hızları sıfır kabul ederek gereksiz hesaplamaları ve titreşimleri önler.
     */
    static constexpr float VelocityDeadZoneSq = 0.000001f; // 0.001 m/s

    /**
     * @brief Cisimlerin sekmesi (restitution) için gereken minimum çarpışma hızı eşiği.
     * @details Çarpışma hızı bu değerden düşükse cisim sekmez. Bu sayede, yere düşüp 
     *          durmuş cisimlerin kendi ağırlıklarıyla sonsuza kadar titreyerek zıplaması engellenir.
     */
    static constexpr float RestitutionVelocityThreshold = 1.0f;

    // --- Dar Faz, Manifold ve Geometri Eşikleri ---

    /**
     * @brief İki temas normalinin aynı yüzeyi temsil ettiği kabulü için benzerlik eşiği (cos(~18 deg) = 0.95).
     */
    static constexpr float NormalSimilarityThreshold = 0.95f;

    /**
     * @brief Warm-start temas eşleştirmesi için karesel mesafe toleransı (~0.2m karesi).
     */
    static constexpr float WarmStartDistanceSqThreshold = 0.04f;

    /**
     * @brief Statik mesh üçgen sorgusu için minimum AABB arama yarıçapı (metre).
     */
    static constexpr float MinMeshQueryExtent = 3.0f;

    /**
     * @brief Yuvarlanma direnci katsayısı (Rolling resistance).
     */
    static constexpr float RollingResistanceCoefficient = 0.02f;

    /**
     * @brief Geniş faz AABB hız süpürmesi varsayılan zaman katsayısı (saniye).
     */
    static constexpr float DefaultVelocityExpansion = 0.02f;

    /**
     * @brief Bozulmuş veya sıfıra yakın yön vektörlerini tespit için karesel eşik.
     */
    static constexpr float DegenerateVectorThresholdSq = 0.0001f;

    /**
     * @brief Geometrik aramalarda sonsuz uzaklık / sınır değeri olarak kullanılan büyük sayı.
     */
    static constexpr float InfinityDistance = 1e10f;
};

} // namespace Baryon::physics