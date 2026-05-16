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
    static constexpr float LinearSlop = 0.005f; 

    /**
     * @brief İç içe geçme durumlarındaki pozisyon düzeltme oranı.
     * @details Kalan iç içe geçme mesafesinin her adımda yüzde kaçının düzeltileceğini belirler.
     *          Değerin çok yüksek olması yaylanmaya, düşük olması ise yavaş ayrılmaya neden olur.
     */
    static constexpr float PositionCorrectionFactor = 0.5f;

    /**
     * @brief Tek bir adımda yapılabilecek maksimum konum düzeltme miktarı.
     * @details Birbirine çok fazla girmiş nesnelerin, düzeltme sırasında aniden 
     *          uzaya fırlamasını (teleportasyon) engeller.
     */
    static constexpr float MaxLinearCorrection = 0.2f;

    // --- Uyku Sistemi (Sleep) Eşikleri ---
    
    /**
     * @brief Cismin uyku moduna geçmesi için gereken maksimum doğrusal hızın karesi.
     * @details Hareketi durma noktasına gelen cisimleri uyutarak işlemci yükünü azaltır (~0.31 m/s).
     */
    static constexpr float SleepLinearThresholdSq = 0.1f; 

    /**
     * @brief Cismin uyku moduna geçmesi için gereken maksimum açısal hızın karesi.
     * @details Dönme hareketi bitmek üzere olan cisimleri uyutmak için kullanılır (~0.2 rad/s).
     */
    static constexpr float SleepAngularThresholdSq = 0.04f;

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
};

} // namespace Baryon::physics