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

#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Matrix3x3.hpp"
#include "Baryon/math/Quaternion.hpp"
#include <cstdint>

/**
 * @file CoreComponents.hpp
 * @brief Fizik motorunun temel ECS bileşenleri (Veri paketleri).
 * @details Bu dosyadaki tüm yapılar, veri odaklı tasarıma (Data-Oriented Design) uygun 
 *          olarak bellekte ardışık dizilebilen, önbellek dostu saf veri paketleri (POD) içerir.
 */
namespace Baryon::Core {

/**
 * @enum BodyType
 * @brief Fiziksel cismin temel davranış türünü belirler.
 */
enum class BodyType {
    Static,    ///< Dış güçlerden etkilenmeyen, tamamen hareketsiz cisim (örn. Zemin, Duvar).
    Kinematic, ///< Fiziksel çarpışmalara neden olan ancak sadece kod ile hareket ettirilen cisim (örn. Hareketli platform).
    Dynamic    ///< Yerçekimi, çarpışma ve dış güçlerden tamamen etkilenen fiziksel cisim.
};

/**
 * @struct Motion
 * @brief Cismin hareket hızını ve üzerindeki anlık kuvvetleri tutan bileşen.
 * @details Uygulanan dış kuvvetler ve torklar her fizik adımının (step) sonunda otomatik olarak sıfırlanır.
 */
struct Motion {
    Vector3 linearVelocity{0.0f, 0.0f, 0.0f};   ///< Cismin doğrusal hızı (m/s).
    Vector3 angularVelocity{0.0f, 0.0f, 0.0f};  ///< Cismin dönme (açısal) hızı (rad/s).

    Vector3 splitLinearVelocity{0.0f, 0.0f, 0.0f};  ///< Split impulse çizgisel hız (yalnızca konum entegrasyonu için).
    Vector3 splitAngularVelocity{0.0f, 0.0f, 0.0f}; ///< Split impulse açısal hız (yalnızca yönelim entegrasyonu için).

    float linearDamping{0.05f};                    ///< Doğrusal hava direnci sönümleme katsayısı.
    float angularDamping{0.05f};                   ///< Açısal hava direnci sönümleme katsayısı.
    float gravityScale{1.0f};                      ///< Gövde bazlı yerçekimi ivmesi ölçek katsayısı (varsayılan 1.0f).
    
    Vector3 externalForce{0.0f, 0.0f, 0.0f};    ///< Bu karede cisme uygulanan toplam itme kuvveti (Newton).
    Vector3 externalTorque{0.0f, 0.0f, 0.0f};   ///< Bu karede cisme uygulanan toplam dönme kuvveti (N·m).
};

/**
 * @struct MassProps
 * @brief Cismin kütle ve dönme direncini (eylemsizlik) tanımlayan bileşen.
 * @details Statik ve Kinematik cisimler fiziksel tepki vermediği için bu cisimlerin 
 *          ters kütlesi ve ters eylemsizlik değerleri her zaman sıfır kabul edilir.
 */
struct MassProps {
    float mass{1.0f};          ///< Cismin toplam kütlesi (kg).
    float inverseMass{1.0f};   ///< Ters kütle (1 / kütle). İşlem optimizasyonu için saklanır.

    Matrix3x3 localInertiaTensor{Matrix3x3::identity()};        ///< Cismin kendi yerel eksenlerindeki dönme direnci.
    Matrix3x3 inverseLocalInertiaTensor{Matrix3x3::identity()}; ///< Cismin yerel ters eylemsizlik matrisi (önceden hesaplanır).
    Matrix3x3 inverseInertiaTensor{Matrix3x3::identity()};      ///< Cismin dünya uzayındaki döndürülmüş ters eylemsizlik tensörü.
};

/**
 * @struct Material
 * @brief Cismin yüzey sürtünme ve esneklik özelliklerini tutan bileşen.
 */
struct Material {
    float friction{0.4f};      ///< Sürtünme katsayısı [0.0 - 1.0]. Yüksek değerler kaymayı zorlaştırır.
    float restitution{0.0000000000005f}; ///< Zıplama/Esneklik katsayısı [0.0 - 1.0]. 1.0 objenin tamamen sekmesini sağlar.
};

/**
 * @enum LockAxis
 * @brief Cismin kilitli çizgisel ve açısal serbestlik dereceleri (DOF Lock).
 * @details Bitmask olarak kombine edilebilir. Kilitlenen eksenler boyunca cisim
 *          fiziksel olarak hareket edemez veya dönemez (sonsuz eylemsizlik).
 */
enum LockAxis : uint8_t {
    None         = 0,
    LinearX      = 1 << 0, ///< X ekseni boyunca çizgisel öteleme kilitli.
    LinearY      = 1 << 1, ///< Y ekseni boyunca çizgisel öteleme kilitli.
    LinearZ      = 1 << 2, ///< Z ekseni boyunca çizgisel öteleme kilitli.
    AngularX     = 1 << 3, ///< X ekseni etrafında dönme (Pitch) kilitli.
    AngularY     = 1 << 4, ///< Y ekseni etrafında dönme (Yaw) kilitli.
    AngularZ     = 1 << 5, ///< Z ekseni etrafında dönme (Roll) kilitli.

    // Kullanışlı kombinasyonlar
    LinearAll    = LinearX | LinearY | LinearZ,   ///< Tüm çizgisel hareketler kilitli.
    AngularAll   = AngularX | AngularY | AngularZ,///< Tüm dönmeler kilitli (tam diklik).
    All          = LinearAll | AngularAll,        ///< 6 DOF tamamen kilitli.
    Lock2D       = LinearZ | AngularX | AngularY  ///< 2D düzlem (XY) kısıtı: Z hareketi ve X/Y devrilmesi kilitli.
};

constexpr inline LockAxis operator|(LockAxis a, LockAxis b) noexcept {
    return static_cast<LockAxis>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
constexpr inline LockAxis operator&(LockAxis a, LockAxis b) noexcept {
    return static_cast<LockAxis>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
constexpr inline LockAxis operator^(LockAxis a, LockAxis b) noexcept {
    return static_cast<LockAxis>(static_cast<uint8_t>(a) ^ static_cast<uint8_t>(b));
}
constexpr inline LockAxis operator~(LockAxis a) noexcept {
    return static_cast<LockAxis>(~static_cast<uint8_t>(a));
}

/**
 * @struct BodyState
 * @brief Cismin genel simülasyon durumunu ve özelliklerini tutan bileşen.
 */
struct BodyState {
    BodyType type{BodyType::Dynamic};   ///< Cismin hareket davranış tipi (Static, Kinematic, Dynamic).
    uint8_t lockedAxes{0};              ///< Kilitli serbestlik dereceleri (LockAxis bitmask).
    bool isSleeping{false};             ///< Cisim hareketsiz kalıp işlem tasarrufu için uyku moduna geçti mi?
    float sleepTimer{0.0f};             ///< Cismin uyku moduna geçmesi için geçen hareketsiz süre sayacı.
    bool useCCD{true};                  ///< Sürekli Çarpışma Algılama (CCD - hızlı cisimlerin tünellemesini önler) aktif mi?
    float ccdMotionThreshold{0.0f};     ///< CCD'nin devreye girmesi için cismin o karede ulaşması gereken minimum hız eşiği.
    bool ccdIntegrated{false};          ///< CCD bu adımda konum entegrasyonunu tamamladı mı?
    bool useSpeculativeContacts{false}; ///< Spekülatif kontak (öngörülü çarpışma - CCD olmadan tünelleme önleme) aktif mi?
    bool useGyroscopicTorque{false};    ///< Jiroskopik tork (Euler hareket denklemleri) entegrasyonu aktif mi?
};

/**
 * @struct Linkage
 * @brief Cisimler arasındaki ebeveyn-çocuk (parent-child) hiyerarşi bağlantısı.
 * @details Bir cismin dünya pozisyonu yerine, bağlı olduğu ebeveynine göre nerede durduğunu tanımlar.
 */
struct Linkage {
    uint64_t parentId{0xFFFFFFFFFFFFFFFFULL};               ///< Bağlı olduğu ebeveyn varlığın kimliği (0xFFFFFFFFFFFFFFFF = Kök/Bağımsız cisim).
    Vector3 localPosition{0.0f, 0.0f, 0.0f};               ///< Ebeveyn merkezine göre yerel konumu.
    Quaternion localOrientation{0.0f, 0.0f, 0.0f, 1.0f};   ///< Ebeveyn eksenine göre yerel dönüş (rotasyon) değeri.
};


} // namespace Baryon::Core
