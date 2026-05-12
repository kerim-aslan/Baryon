#pragma once

#include <cstdint>
#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Matrix3x3.hpp"

/**
 * @file BaryonTypes.hpp
 * @brief Baryon Fizik Motoru'nun temel tip tanımlamaları ve cisim oluşturma yapısı.
 * @details Motorun dışarıya açık (public API) tiplerini içerir.
 *          BodyID, BodyType ve BodyDef burada tanımlanmıştır.
 */
namespace Baryon {

/// @brief Cisim kimlik numarası tipi. Her cisim benzersiz bir uint32_t ID'ye sahiptir.
using BodyID = uint32_t;

/// @brief Geçersiz cisim ID'si sabiti (0xFFFFFFFF). "Böyle bir cisim yok" anlamına gelir.
constexpr BodyID INVALID_BODY_ID = 0xFFFFFFFF;

/**
 * @enum BodyType
 * @brief Fizik motorundaki üç temel cisim tipini tanımlar.
 *
 * @details Her cisim tipi, fizik simülasyonunda farklı davranış sergiler:
 *          - **Static**: Asla hareket etmez (duvar, zemin). Ters kütlesi 0'dır.
 *          - **Kinematic**: Sadece kod ile hareket eder. Fiziksel kuvvetlerden etkilenmez.
 *          - **Dynamic**: Tam fizik simülasyonuna tabidir. Düşer, çarpar, sekebilir.
 */
enum class BodyType {
    Static,     ///< Hareketsiz cisim (duvar, zemin). Sonsuz kütleli.
    Kinematic,  ///< Kodla kontrol edilen cisim. Kuvvetlerden etkilenmez.
    Dynamic     ///< Tam fizik simülasyonuna tabi cisim. Kuvvetlere tepki verir.
};

/**
 * @struct BodyDef
 * @brief Yeni bir fizik cismi oluşturmak için kullanılan tanımlama yapısı.
 *
 * @details Kullanıcı bu yapıyı doldurarak Simulator::createBody() fonksiyonuna
 *          gönderir. Tüm alanlar makul varsayılan değerlerle başlatılmıştır.
 */
struct BodyDef {
    BodyType type{BodyType::Dynamic};           ///< Cisim tipi (varsayılan: Dynamic).
    float mass{1.0f};                            ///< Cismin kütlesi (kg). Static için etkisizdir.
    float friction{0.4f};                        ///< Sürtünme katsayısı [0, 1]. 0: kaygan buz, 1: kauçuk.
    float restitution{0.0000000000005f};         ///< Esneklik/zıplama katsayısı. 0: hiç sekmez, 1: tam elastik.
    float linearDamping{0.01f};                  ///< Doğrusal sönümleme. Hava direncini simüle eder.
    float angularDamping{1.0f};                  ///< Açısal sönümleme. Dönme hızını zamanla azaltır.
    bool useCCD{true};                           ///< Sürekli Çarpışma Algılama (CCD) aktif mi?
    void* userData{nullptr};                     ///< Kullanıcı tarafından atanabilecek özel veri işaretçisi.
};

} // namespace Baryon
