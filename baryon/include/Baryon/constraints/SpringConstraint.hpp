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

#include "Baryon/collision/ConstraintBase.hpp"
#include "Baryon/math/Vector3.hpp"

namespace Baryon::collision {

/**
 * @file SpringConstraint.hpp
 * @brief Yay ve Yumuşak Mesafe Kısıtı (Spring / Damped Soft Distance Joint) bileşeni.
 * @details İki rijit cisim arasındaki mesafeyi Hooke Yasası ve sönümlü harmonik salınıcı
 *          (damped harmonic oscillator) modeli ile elastik biçimde korur.
 *          Doğal yay uzunluğu (rest length), yay sertliği (stiffness k, N/m) ve
 *          sönüm katsayısı (damping c, N*s/m) veya alternatif olarak doğal frekans (Hz)
 *          ve sönümleme oranı (damping ratio zeta) parametrizasyonlarını destekler.
 */
struct SpringConstraint : public ConstraintBase {
    Vector3 localAnchorA{0.0f, 0.0f, 0.0f}; ///< A cisminin kütle merkezine göre yerel bağlantı noktası.
    Vector3 localAnchorB{0.0f, 0.0f, 0.0f}; ///< B cisminin kütle merkezine göre yerel bağlantı noktası.

    float restLength{1.0f};   ///< Yayın gerilmesiz doğal serbest uzunluğu (metre).
    float stiffness{100.0f};  ///< Yay sertlik katsayısı (k, N/m).
    float damping{5.0f};      ///< Viskoz sönümleme katsayısı (c, N*s/m).

    // Yumuşak kısıt (soft constraint) çözücü parametreleri
    float gamma{0.0f};        ///< Yumuşak kısıt uyumluluk katsayısı (constraint softness gamma).
    float biasFactor{0.0f};   ///< Baumgarte konum hatası bias katsayısı.
    float accumulatedImpulse{0.0f}; ///< Önceki adımdan aktarılan skaler itme (warm starting).

    /**
     * @brief Varsayılan yapıcı.
     */
    SpringConstraint() = default;

    /**
     * @brief İki cisim arasında yay kısıtı oluşturur.
     * @param a Birinci cismin ECS Entity kimliği.
     * @param b İkinci cismin ECS Entity kimliği.
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param targetRestLength Yayın serbest uzunluğu.
     * @param springStiffness Yay sabiti (N/m).
     * @param springDamping Sönüm sabiti (N*s/m).
     */
    SpringConstraint(ecs::Entity a, ecs::Entity b,
                     const Vector3& anchorA, const Vector3& anchorB,
                     float targetRestLength, float springStiffness = 100.0f, float springDamping = 5.0f)
        : ConstraintBase(a, b),
          localAnchorA(anchorA),
          localAnchorB(anchorB),
          restLength(targetRestLength),
          stiffness(springStiffness),
          damping(springDamping),
          gamma(0.0f),
          biasFactor(0.0f),
          accumulatedImpulse(0.0f) {}

    /**
     * @brief Doğal frekans ve sönüm oranı parametreleri ile yay katsayılarını hesaplar.
     * @details $$k = m_{\text{eff}} \omega^2, \quad c = 2 m_{\text{eff}} \zeta \omega, \quad \omega = 2\pi f$$
     * @param frequencyHz Yayın titreşim frekansı (Hertz).
     * @param dampingRatio Sönümleme oranı zeta (0: sönümsüz, 1: kritik sönümlü, >1: aşırı sönümlü).
     * @param effectiveMass Cismin etkin indirgenmiş kütlesi (kg).
     */
    void setFrequencyAndDamping(float frequencyHz, float dampingRatio, float effectiveMass = 1.0f) {
        float omega = 2.0f * 3.14159265f * frequencyHz;
        stiffness = effectiveMass * omega * omega;
        damping = 2.0f * effectiveMass * dampingRatio * omega;
    }
};

} // namespace Baryon::collision
