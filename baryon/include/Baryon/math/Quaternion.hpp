#pragma once

#include "Vector3.hpp"
#include <cmath>
#include <cassert>
#include <utility>

/**
 * @file Quaternion.hpp
 * @brief Kuaterniyon (Quaternion) yapısı - 3B Rotasyon Kontrolü.
 * @details Nesnelerin 3 boyutlu uzayda dönme hareketlerini hesaplamak için kullanılır. 
 *          Euler açılarının aksine "Eksen Kilitlenmesi" (Gimbal Lock) sorunu yaratmaz 
 *          ve rotasyon matrislerinden çok daha az bellek harcar.
 */
namespace Baryon {

/**
 * @struct Quaternion
 * @brief 3B rotasyonları temsil eden 4 bileşenli matematiksel yapı.
 * @details Kuaterniyonlar; dönme eksenini temsil eden bir vektör (x, y, z) ve 
 *          dönme miktarını temsil eden bir skaler (w) bileşenden oluşur. 
 *          Fizik motorunda tutarlı sonuçlar için her zaman "birim uzunlukta" tutulur.
 */
struct Quaternion {
    /// @name Veri Bileşenleri
    /// @{

    float x{0.0f}; ///< Rotasyon ekseni X ağırlığı.
    float y{0.0f}; ///< Rotasyon ekseni Y ağırlığı.
    float z{0.0f}; ///< Rotasyon ekseni Z ağırlığı.
    float w{1.0f}; ///< Dönme miktarı (reel kısım). w=1 değeri "sıfır rotasyon" demektir.

    /// @}

    /// @name Kurucular
    /// @{

    /**
     * @brief Varsayılan olarak hiçbir rotasyon uygulamayan (0,0,0,1) kimlik kuaterniyonu oluşturur.
     */
    constexpr Quaternion() = default;
    
    /**
     * @brief Doğrudan x, y, z ve w bileşenlerini belirleyerek kuaterniyon oluşturur.
     */
    constexpr Quaternion(float x, float y, float z, float w)
        : x(x), y(y), z(z), w(w) {}

    /**
     * @brief Belirli bir eksen ve açı (radyan) kullanarak rotasyon oluşturur.
     * @param axis Dönüşün etrafında gerçekleşeceği eksen (örn: Vector3(0,1,0) Y ekseni için).
     * @param angleRadian Dönüş miktarı (radyan cinsinden).
     */
    Quaternion(const Vector3& axis, float angleRadian) {
        float halfAngle = angleRadian * 0.5f;
        float sinHalfAngle = std::sin(halfAngle);
        
        x = axis.x * sinHalfAngle;
        y = axis.y * sinHalfAngle;
        z = axis.z * sinHalfAngle;
        w = std::cos(halfAngle);
    }

    /// @}

    /// @name Statik Yardımcılar
    /// @{

    /**
     * @brief "Sıfır Rotasyon" (Identity) değerini döndürür. 
     * @details Bir objeyi başlangıç/hiç dönmemiş haline getirmek için kullanılır.
     */
    [[nodiscard]] static constexpr Quaternion identity() {
        return Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    }

    /// @}

    /// @name Erişim Operatörleri
    /// @{

    /**
     * @brief Bileşenlere sayısal indeks (0-3) ile erişim sağlar (0:x, 1:y, 2:z, 3:w).
     */
    [[nodiscard]] constexpr auto&& operator[](this auto&& self, std::size_t index) {
        assert(index < 4 && "Kuaterniyon indeks hatasi! (0-3 arasi olmalidir)");
        if (index == 0) return std::forward<decltype(self)>(self).x;
        if (index == 1) return std::forward<decltype(self)>(self).y;
        if (index == 2) return std::forward<decltype(self)>(self).z;
        return std::forward<decltype(self)>(self).w;
    }

    /// @}

    /// @name Operatörler ve Matematik
    /// @{

    /**
     * @brief İki rotasyonu birleştirir.
     * @details 'C = A * B' işlemi, önce B rotasyonunu sonra A rotasyonunu uygular.
     * @param other Üzerine eklenecek ikinci rotasyon.
     * @return Birleşmiş yeni rotasyon değeri.
     */
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w,
            w * other.w - x * other.x - y * other.y - z * other.z
        );
    }

    /**
     * @brief Bir vektörü (nokta veya yön) bu kuaterniyonun açısına göre döndürür.
     * @param point Dönüştürülecek vektör.
     * @return Döndürülmüş yeni vektör.
     */
    [[nodiscard]] constexpr Vector3 operator*(const Vector3& point) const {
        Vector3 qVec(x, y, z);
        Vector3 t = 2.0f * qVec.cross(point);
        return point + (w * t) + qVec.cross(t);
    }

    /**
     * @brief Kuaterniyonun matematiksel uzunluğunun karesini hesaplar.
     */
    [[nodiscard]] constexpr float lengthSquare() const {
        return x * x + y * y + z * z + w * w;
    }

    /**
     * @brief Kuaterniyonun matematiksel uzunluğunu hesaplar.
     */
    [[nodiscard]] float length() const {
        return std::sqrt(lengthSquare());
    }

    /// @}

    /// @name Normalizasyon ve Dönüşüm
    /// @{

    /**
     * @brief Kuaterniyonu birim uzunluğa (1.0) eşitler.
     * @details Fizik hesaplamaları sırasında oluşan ufak kaymaları temizleyerek 
     *          rotasyonun bozulmasını engeller.
     */
    void normalize() {
        float lenSq = lengthSquare();
        if (lenSq > 1e-12f) {
            float invLen = 1.0f / std::sqrt(lenSq);
            x *= invLen;
            y *= invLen;
            z *= invLen;
            w *= invLen;
        } else {
            // Tamamen bozulmuşsa sıfır rotasyon değerine dön
            x = 0.0f; y = 0.0f; z = 0.0f; w = 1.0f;
        }
    }

    /**
     * @brief Mevcut rotasyonun tam tersini temsil eden "eşlenik" kuaterniyonu döndürür.
     */
    [[nodiscard]] constexpr Quaternion getConjugate() const {
        return Quaternion(-x, -y, -z, w);
    }

    /**
     * @brief Rotasyonu tamamen geri alan ters (inverse) kuaterniyonu hesaplar.
     */
    [[nodiscard]] constexpr Quaternion getInverse() const {
        float lenSq = lengthSquare();
        assert(lenSq > 1e-12f && "Sifir uzunluktaki kuaterniyonun tersi alinamaz!");
        float invLenSq = 1.0f / lenSq;
        return Quaternion(-x * invLenSq, -y * invLenSq, -z * invLenSq, w * invLenSq);
    }

    /**
     * @brief İki kuaterniyon arasındaki benzerliği (nokta çarpımı) hesaplar.
     * @return 1'e ne kadar yakınsa rotasyonlar o kadar benzerdir.
     */
    [[nodiscard]] constexpr float dot(const Quaternion& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    /// @}
};

} // namespace Baryon