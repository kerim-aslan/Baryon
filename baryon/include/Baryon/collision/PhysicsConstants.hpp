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
     * @brief Cismin uyku moduna geçmesi için gereken maksimum doğrusal hız eşiği (0.05 m/s).
     */
    static constexpr float SleepLinearThreshold = 0.05f;
    static constexpr float SleepLinearThresholdSq = SleepLinearThreshold * SleepLinearThreshold; 

    /**
     * @brief Cismin uyku moduna geçmesi için gereken maksimum açısal hız eşiği (0.05 rad/s).
     */
    static constexpr float SleepAngularThreshold = 0.05f;
    static constexpr float SleepAngularThresholdSq = SleepAngularThreshold * SleepAngularThreshold;

    /**
     * @brief Cismin uyku moduna geçmesi için hareketsiz kalması gereken süre (0.5 saniye).
     */
    static constexpr float TimeToSleep = 0.5f;

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

    // --- Çarpışma ve Manifold Eşleşme Sabitleri (Magic Numbers Centralization) ---

    /**
     * @brief Manifold tutarlılığı ve ardışık temas normali yönelim uyum eşiği.
     */
    static constexpr float ManifoldNormalAlignmentThreshold = 0.95f;

    /**
     * @brief Warm-starting temas noktası eşleme mesafesinin karesi ((0.20m)^2 = 0.04m^2).
     */
    static constexpr float ContactPointMatchRadiusSq = 0.04f;

    /**
     * @brief AABB genişletmesi ve hız kestirimi için varsayılan zaman faktörü (0.02s).
     */
    static constexpr float AabbVelocityInflationTime = 0.02f;

    /**
     * @brief Yuvarlanma sürtünmesi (rolling friction) katsayısı.
     */
    static constexpr float RollingFrictionCoefficient = 0.02f;

    /**
     * @brief Yuvarlanma sürtünmesinin devreye girmesi için minimum bağıl açısal hız eşiği.
     */
    static constexpr float RollingFrictionAngularVelocityThreshold = 1e-4f;

    /**
     * @brief Statik mesh genişlik sorgusu varsayılan yarıçapı.
     */
    static constexpr float DefaultMeshQueryExtent = 3.0f;

    /**
     * @brief Sayısal sonsuz/büyük değer sabiti.
     */
    static constexpr float Infinity = 1e10f;
};

} // namespace Baryon::physics