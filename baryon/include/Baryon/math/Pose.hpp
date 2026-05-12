#pragma once

#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"

/**
 * @file Pose.hpp
 * @brief Bir objenin uzaydaki tam konumunu (pozisyon) ve duruşunu (rotasyon) tutan veri yapısı.
 * @details Oyun dünyasındaki her fiziksel objenin nerede olduğunu ve nereye baktığını belirler. 
 *          Ebeveyn-çocuk (parent-child) bağlantıları ve uzay dönüşümleri (kameraya göre 
 *          yön bulma vb.) bu yapı üzerinden gerçekleştirilir.
 */
namespace Baryon {

/**
 * @struct Pose
 * @brief Pozisyon ve yönelim (rotasyon) verilerini tek bir pakette birleştirir.
 * @details Objelerin sahnedeki tam durumunu ifade eder.
 */
struct Pose {
    /// @brief Objenin oyun dünyasındaki konumu (metre cinsinden).
    Vector3 position;

    /// @brief Objenin oyun dünyasındaki duruşu / yönelimi.
    Quaternion orientation;

    /// @name Kurucular
    /// @{

    /**
     * @brief Varsayılan kurucu. Objeyi merkeze (0,0,0) yerleştirir ve rotasyonunu sıfırlar.
     */
    constexpr Pose() = default;

    /**
     * @brief Belirli bir pozisyon ve rotasyon ile Pose oluşturur.
     * @param position Objenin dünyadaki konumu.
     * @param orientation Objenin baktığı yön.
     */
    constexpr Pose(const Vector3& position, const Quaternion& orientation)
        : position(position), orientation(orientation) {}

    /**
     * @brief Sadece pozisyon belirterek Pose oluşturur (Rotasyon sıfır kabul edilir).
     * @param position Objenin dünyadaki konumu.
     */
    constexpr explicit Pose(const Vector3& position)
        : position(position), orientation(Quaternion::identity()) {}

    /// @}

    /// @name Statik Fabrika Fonksiyonları
    /// @{

    /**
     * @brief Tamamen sıfırlanmış (merkezde ve hiç dönmemiş) bir başlangıç durumu üretir.
     * @return Orijinde ve rotasyonsuz Pose.
     */
    [[nodiscard]] static constexpr Pose identity() {
        return Pose(Vector3(0.0f, 0.0f, 0.0f), Quaternion::identity());
    }

    /// @}

    /// @name Uzay Dönüşümleri
    /// @{

    /**
     * @brief Objenin kendi (yerel) uzayındaki bir noktayı, dünya uzayına çevirir.
     * @details Bu işlem noktayı önce objenin baktığı yöne göre döndürür, ardından objenin 
     *          dünyadaki pozisyonunu ekler. 
     *          Örnek: Bir arabanın içindeki vites kolunun oyun dünyasındaki kesin koordinatını bulmak.
     * @param localPoint Objeye göre yerel konum (örn: arabanın merkezinden 1 metre sağda).
     * @return Noktanın oyun dünyasındaki gerçek konumu.
     */
    [[nodiscard]] constexpr Vector3 operator*(const Vector3& localPoint) const {
        // Önce noktayı döndür, sonra pozisyonu ekle
        return (orientation * localPoint) + position;
    }

    /**
     * @brief İki farklı durumu (Pose) birleştirir (Ebeveyn-Çocuk / Hiyerarşi dönüşümü).
     * @details Bir objenin başka bir objeye göre olan konumunu, dünya konumuna çevirmek için kullanılır.
     *          Örnek: Bir karakterin (A) elinde tuttuğu kılıcın (B) dünyadaki tam durumunu 
     *          bulmak için: Karakterin_Pose'u * Kılıcın_Karaktere_Göre_Pose'u hesaplanır.
     * @param other Çocuk objenin (alt objenin) ebeveyne göre durumu.
     * @return Birleştirilmiş nihai dünya durumu.
     */
    [[nodiscard]] constexpr Pose operator*(const Pose& other) const {
        return Pose(
            *this * other.position,          // Çocuğun pozisyonunu kendi uzayıma göre çevir
            orientation * other.orientation  // İki rotasyonu birbirine ekle
        );
    }

    /**
     * @brief Pose'un tam tersini (inverse) hesaplar.
     * @details Dünya koordinatlarındaki bir noktayı, bu objenin kendi (yerel) koordinatlarına çevirmek 
     *          için kullanılır. "Dünyadaki şu düşman, bu kameranın tam olarak neresinde (sağında mı, 
     *          solunda mı) duruyor?" sorusunun yanıtını bulmak için gereklidir.
     * @return Ters çevrilmiş durum (Pose).
     */
    [[nodiscard]] constexpr Pose getInverse() const {
        Quaternion invOrientation = orientation.getInverse();
        // Pozisyonu ters rotasyonla döndürüp eksi yöne çekiyoruz
        Vector3 invPosition = invOrientation * (-position);
        return Pose(invPosition, invOrientation);
    }

    /// @}

    /// @name Yardımcı Fizik Fonksiyonları
    /// @{

    /**
     * @brief Kuaterniyon formatındaki rotasyon verisini 3x3 dönüşüm matrisine çevirir.
     * @details Motorun fizik çözücüsünde ve eylemsizlik (dönme direnci) hesaplamalarında, 
     *          hızlı matris çarpımları yapabilmek için kullanılır.
     * @return Objeyi döndüren 3x3 rotasyon matrisi.
     */
    [[nodiscard]] Matrix3x3 getOrientationMatrix() const {
        float x = orientation.x; float y = orientation.y; float z = orientation.z; float w = orientation.w;
        float x2 = x + x; float y2 = y + y; float z2 = z + z;
        float xx = x * x2; float xy = x * y2; float xz = x * z2;
        float yy = y * y2; float yz = y * z2; float zz = z * z2;
        float wx = w * x2; float wy = w * y2; float wz = w * z2;

        return Matrix3x3(
            1.0f - (yy + zz), xy - wz, xz + wy,
            xy + wz, 1.0f - (xx + zz), yz - wx,
            xz - wy, yz + wx, 1.0f - (xx + yy)
        );
    }

    /// @}
};

} // namespace Baryon