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

#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"

/**
 * @file ConstraintBase.hpp
 * @brief Genel kısıtlama (Constraint) temel veri yapısı.
 * @details Sanal fonksiyon içermeyen, veri odaklı (data-oriented) kısıtlama temel yapısıdır.
 *          Tüm eklem türleri (Mesafe, Menteşe, Sabit vb.) bu temel yapıyı miras alır.
 */
namespace Baryon::collision {

/**
 * @struct ConstraintBase
 * @brief İki varlık (entity) arasındaki kısıtlamanın ortak fiziksel ve mantıksal verilerini tutar.
 */
struct ConstraintBase {
    ecs::Entity entityA;           ///< Birinci bağlı varlık kimliği.
    ecs::Entity entityB;           ///< İkinci bağlı varlık kimliği.

    Vector3 localAnchorA{0.0f, 0.0f, 0.0f}; ///< A cisminin kendi yerel uzayındaki bağlantı noktası.
    Vector3 localAnchorB{0.0f, 0.0f, 0.0f}; ///< B cisminin kendi yerel uzayındaki bağlantı noktası.

    float breakImpulse{1e10f};     ///< Eklemin kırılması için gereken maksimum itme kuvveti eşiği (N*s).
    float breakForce{1e10f};       ///< Eklemin kırılması için gereken maksimum çizgisel kuvvet eşiği (Newton).
    float breakTorque{1e10f};      ///< Eklemin kırılması için gereken maksimum açısal tork eşiği (N*m).

    float appliedImpulseLinear{0.0f};  ///< Bu adımda ekleme binen toplam çizgisel itme (N*s).
    float appliedImpulseAngular{0.0f}; ///< Bu adımda ekleme binen toplam açısal itme (N*m*s).

    bool isBroken{false};          ///< Eklemin kırılmış olup olmadığı.
    bool isEnabled{true};          ///< Eklemin fiziksel olarak aktif olup olmadığı.

    /**
     * @brief Varsayılan yapıcı metot.
     */
    ConstraintBase() = default;

    /**
     * @brief Parametreli temel kısıt yapıcı metodu.
     * @param a Birinci varlık.
     * @param b İkinci varlık.
     * @param anchorA A cismi üzerindeki bağlantı noktası.
     * @param anchorB B cismi üzerindeki bağlantı noktası.
     * @param breakImp Kırılma eşiği (varsayılan: 1e10 N*s).
     */
    ConstraintBase(ecs::Entity a, ecs::Entity b,
                   const Vector3& anchorA = Vector3(0.0f, 0.0f, 0.0f),
                   const Vector3& anchorB = Vector3(0.0f, 0.0f, 0.0f),
                   float breakImp = 1e10f)
        : entityA(a), entityB(b),
          localAnchorA(anchorA), localAnchorB(anchorB),
          breakImpulse(breakImp), breakForce(1e10f), breakTorque(1e10f),
          appliedImpulseLinear(0.0f), appliedImpulseAngular(0.0f),
          isBroken(false), isEnabled(true) {}

    /**
     * @brief Ekleme kırılma kuvveti ve tork eşikleri atar.
     * @param maxForce Maksimum çizgisel kuvvet (Newton).
     * @param maxTorque Maksimum açısal tork (Newton*metre).
     */
    void setBreakForce(float maxForce, float maxTorque = 1e10f) {
        breakForce = maxForce;
        breakTorque = maxTorque;
    }

    /**
     * @brief Ekleme kırılma itmesi eşiği atar.
     * @param maxImpulse Maksimum reaksiyon itmesi (N*s).
     */
    void setBreakImpulse(float maxImpulse) {
        breakImpulse = maxImpulse;
    }

    /**
     * @brief Son adımda ekleme binen reaksiyon kuvvetini döndürür.
     * @param dt Zaman adımı (saniye).
     * @return Reaksiyon kuvveti (Newton).
     */
    [[nodiscard]] float getReactionForce(float dt) const {
        return dt > 0.0f ? appliedImpulseLinear / dt : 0.0f;
    }

    /**
     * @brief Son adımda ekleme binen reaksiyon torkunu döndürür.
     * @param dt Zaman adımı (saniye).
     * @return Reaksiyon torku (N*m).
     */
    [[nodiscard]] float getReactionTorque(float dt) const {
        return dt > 0.0f ? appliedImpulseAngular / dt : 0.0f;
    }
};

} // namespace Baryon::collision
