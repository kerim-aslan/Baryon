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

#include "Baryon/Core/Registry.hpp"
#include "Baryon/math/Vector3.hpp"

/**
 * @file Integrator.hpp
 * @brief Sayısal Entegrasyon (Hareket) Sistemi.
 * @details Newton'un hareket yasalarını kullanarak kuvvetleri hıza, hızları ise 
 *          yeni konumlara dönüştürür. Simülasyon kararlılığı için iki aşamalı 
 *          "Yarı-Örtük Euler" (Semi-implicit Euler) yöntemi uygulanır.
 */
namespace Baryon::systems {

/**
 * @class Integrator
 * @brief Fiziksel cisimlerin hız ve konum güncellemelerini hesaplayan sistem.
 */
class Integrator {
private:
    Core::Registry& mRegistry;                         ///< ECS kayıt defteri referansı.
    Vector3 mAccelerationField{0.0f, -9.81f, 0.0f};   ///< Sahne genelindeki sabit ivme alanı (Varsayılan: Yerçekimi).

public:
    /** 
     * @brief Entegrasyon sistemini başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit Integrator(Core::Registry& registry);

    /** 
     * @brief Sahne genelindeki ivme alanını (m/s²) günceller. 
     * @param accelerationField Yeni ivme vektörü (Örn: Yerçekimi yönü ve şiddeti).
     */
    void setAccelerationField(const Vector3& accelerationField);

    /**
     * @brief 1. AŞAMA: Hız Entegrasyonu.
     * @details Çözücüden (solver) önce çalışır. Cisimlere uygulanan kuvvetleri ve 
     *          ivmeyi kullanarak doğrusal ve açısal hızları hesaplar. 
     *          Hava direnci (damping) ve mikro hareketlerin durdurulması bu aşamada yapılır.
     * @param deltaTime Geçen zaman adımı (saniye).
     */
    void integrateVelocities(float deltaTime);

    /**
     * @brief 2. AŞAMA: Konum ve Dönüş Entegrasyonu.
     * @details Çözücüden (solver) sonra çalışır. Güncellenmiş hızları kullanarak 
     *          cisimlerin dünya üzerindeki yeni konumlarını ve bakış yönlerini (rotasyon) hesaplar.
     *          Ayrıca cismin dönme direncini (eylemsizlik tensörü) dünya koordinatlarına uyarlar.
     * @param deltaTime Geçen zaman adımı (saniye).
     */
    void integratePositions(float deltaTime);
};

} // namespace Baryon::systems