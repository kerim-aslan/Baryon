#pragma once

#include <cmath>
#include <cassert>
#include <utility>

/**
 * @file Vector3.hpp
 * @brief 3 Boyutlu Vektör yapısı.
 * @details Fizik motorunun temel matematik birimi olup; pozisyon, hız ve kuvvet 
 *          gibi vektörel büyüklükleri temsil eder. Performans için fonksiyonlar 
 *          mümkün olduğunca derleme zamanında (constexpr) hesaplanır.
 */
namespace Baryon {

/**
 * @struct Vector3
 * @brief 3B uzayda (x, y, z) bileşenlerini yöneten temel yapı.
 */
struct Vector3 {
    /// @name Veri Üyeleri
    /// @{
    float x{0.0f}; ///< X ekseni (Sağ-Sol)
    float y{0.0f}; ///< Y ekseni (Yukarı-Aşağı)
    float z{0.0f}; ///< Z ekseni (İleri-Geri)
    /// @}

    /// @name Kurucular
    /// @{

    /** @brief Varsayılan kurucu. Bileşenleri (0,0,0) olarak başlatır. */
    constexpr Vector3() = default;

    /** 
     * @brief Parametreli kurucu.
     * @param x X bileşeni, @param y Y bileşeni, @param z Z bileşeni.
     */
    constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    /// @}

    /// @name Erişim Operatörleri
    /// @{

    /**
     * @brief İndeks (0,1,2) üzerinden bileşenlere erişim sağlar.
     * @details C++23 "Deducing this" özelliği ile const ve mutable erişim tek yapıda birleştirilmiştir.
     * @param index Erişim indeksi (0:x, 1:y, 2:z).
     * @return İlgili bileşene referans.
     */
    [[nodiscard]] constexpr auto&& operator[](this auto&& self, std::size_t index) {
        assert(index < 3 && "Vektor indeksi sinir disinda!");
        if (index == 0) return std::forward<decltype(self)>(self).x;
        if (index == 1) return std::forward<decltype(self)>(self).y;
        return std::forward<decltype(self)>(self).z;
    }

    /// @}

    /// @name Aritmetik Operatörler
    /// @{

    /** @brief İki vektörün toplamını hesaplar. */
    [[nodiscard]] constexpr Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    /** @brief İki vektörün farkını hesaplar. */
    [[nodiscard]] constexpr Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    /** @brief Vektörü bir skaler değerle çarpar. */
    [[nodiscard]] constexpr Vector3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    
    /** @brief Vektörün yönünü tersine çevirir. */
    [[nodiscard]] constexpr Vector3 operator-() const {
        return {-x, -y, -z};
    }

    /** @brief Vektörü bir skaler değere böler. */
    [[nodiscard]] constexpr Vector3 operator/(float scalar) const {
        assert(scalar != 0.0f && "Vektor islemlerinde sifira bolme hatasi!");
        float invScalar = 1.0f / scalar;
        return {x * invScalar, y * invScalar, z * invScalar};
    }

    /// @}

    /// @name Bileşik Atama Operatörleri
    /// @{

    constexpr Vector3& operator+=(const Vector3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    constexpr Vector3& operator-=(const Vector3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    constexpr Vector3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    constexpr Vector3& operator/=(float scalar) {
        assert(scalar != 0.0f && "Vektor islemlerinde sifira bolme hatasi!");
        float invScalar = 1.0f / scalar;
        x *= invScalar; y *= invScalar; z *= invScalar;
        return *this;
    }

    /// @}

    /// @name Fizik ve Geometri Fonksiyonları
    /// @{

    /**
     * @brief İç Çarpım (Dot Product) hesaplar.
     * @details İki vektör arasındaki paralellik oranını veya izdüşümü bulmak için kullanılır.
     * @return Skaler çarpım sonucu.
     */
    [[nodiscard]] constexpr float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /**
     * @brief Vektörel Çarpım (Cross Product) hesaplar.
     * @details Her iki vektöre de dik olan yeni bir vektör üretir (Normal hesaplama, tork vb.).
     * @return Dik vektör sonucu.
     */
    [[nodiscard]] constexpr Vector3 cross(const Vector3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    /**
     * @brief Vektör uzunluğunun karesini hesaplar.
     * @details Karekök işlemi içermediği için mesafe karşılaştırmalarında daha performanslıdır.
     */
    [[nodiscard]] constexpr float lengthSquare() const {
        return x * x + y * y + z * z;
    }

    /** @brief Vektörün büyüklüğünü (Öklid uzunluğu) hesaplar. */
    [[nodiscard]] float length() const {
        return std::sqrt(lengthSquare());
    }

    /**
     * @brief Vektörü normalize ederek birim vektöre (uzunluk = 1) çevirir.
     * @details Çok küçük vektörler için sayısal kararlılık koruması içerir.
     */
    void normalize() {
        float lenSq = lengthSquare();
        if (lenSq > 1e-12f) {
            float invLen = 1.0f / std::sqrt(lenSq);
            x *= invLen; y *= invLen; z *= invLen;
        }
    }

    /** @brief Orijinal vektörü bozmadan normalize edilmiş bir kopyasını döndürür. */
    [[nodiscard]] Vector3 getNormalized() const {
        Vector3 result = *this;
        result.normalize();
        return result;
    }

    /// @}
};

/** @brief Skaler * Vektör çarpımı için global operatör. */
[[nodiscard]] constexpr Vector3 operator*(float scalar, const Vector3& vec) {
    return vec * scalar;
}

} // namespace Baryon