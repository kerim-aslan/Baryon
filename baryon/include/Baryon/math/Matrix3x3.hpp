#pragma once

#include "Vector3.hpp"
#include <cassert>
#include <utility>

/**
 * @file Matrix3x3.hpp
 * @brief 3x3 Matris (Matrix) veri yapısı.
 * @details Fizik motorunda objelerin dönüş (rotasyon) işlemlerini ve 
 *          eylemsizlik (dönme direnci) hesaplamalarını yapmak için kullanılır.
 */
namespace Baryon {

/**
 * @struct Matrix3x3
 * @brief 3x3 boyutlu standart matris yapısı.
 * @details Satır öncelikli (row-major) çalışır. Elemanlara matrix[satır][sütun] 
 *          şeklinde kolayca erişilebilir. Ağırlıklı olarak rotasyon ve fiziksel 
 *          eylemsizlik (inertia) işlemlerinde kullanılır.
 */
struct Matrix3x3 {
    /// @brief 3 adet Vector3 satırı (satır-öncelikli yapı).
    Vector3 rows[3];

    /// @name Kurucular
    /// @{

    /**
     * @brief İçi tamamen sıfırlarla dolu (sıfır matrisi) bir matris oluşturur.
     */
    constexpr Matrix3x3() = default;

    /**
     * @brief 3 ayrı satır vektörü belirterek matris oluşturur.
     * @param row0 Birinci satır (Vector3).
     * @param row1 İkinci satır (Vector3).
     * @param row2 Üçüncü satır (Vector3).
     */
    constexpr Matrix3x3(const Vector3& row0, const Vector3& row1, const Vector3& row2)
        : rows{row0, row1, row2} {}

    /**
     * @brief 9 ayrı sayısal (skaler) değer belirterek matris oluşturur.
     */
    constexpr Matrix3x3(float m00, float m01, float m02,
                        float m10, float m11, float m12,
                        float m20, float m21, float m22)
        : rows{
            Vector3(m00, m01, m02),
            Vector3(m10, m11, m12),
            Vector3(m20, m21, m22)
        } {}

    /// @}

    /// @name Statik Fabrika Fonksiyonları
    /// @{

    /**
     * @brief Birim (identity) matris üretir.
     * @details Matematikteki "etkisiz eleman"dır. Bir vektör veya matris bu matrisle 
     *          çarpıldığında hiçbir değişikliğe uğramaz (Sıfır rotasyon).
     * @return 3x3 birim matris.
     */
    [[nodiscard]] static constexpr Matrix3x3 identity() {
        return Matrix3x3(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    /// @}

    /// @name Erişim Operatörleri
    /// @{

    /**
     * @brief İstenilen satıra (Vector3 olarak) doğrudan erişim sağlar.
     * @details Döndürülen Vector3 üzerinden sütun elemanlarına da kolayca erişilebilir:
     *          `matrix[satır].x`, `matrix[satır].y`, `matrix[satır].z`
     * @param rowIndex Satır numarası (0, 1 veya 2).
     * @return İlgili satırın referansı.
     */
    [[nodiscard]] constexpr auto&& operator[](this auto&& self, std::size_t rowIndex) {
        assert(rowIndex < 3 && "Matrix3x3 satir indeksi sinirlarin disinda!");
        return std::forward<decltype(self)>(self).rows[rowIndex];
    }

    /// @}

    /// @name Matris Operatörleri
    /// @{

    /**
     * @brief Vektörü bu matrisin temsil ettiği dönüşüme (örn. rotasyona) göre değiştirir.
     * @param vec Dönüştürülecek hedef vektör.
     * @return Dönüşüm uygulanmış yeni vektör.
     */
    [[nodiscard]] constexpr Vector3 operator*(const Vector3& vec) const {
        return Vector3(
            rows[0].dot(vec),
            rows[1].dot(vec),
            rows[2].dot(vec)
        );
    }

    /**
     * @brief İki farklı matrisi (dönüşümü) birbiriyle birleştirir.
     * @param other Çarpılacak diğer matris.
     * @return İki dönüşümün birleşimi olan yeni matris.
     */
    [[nodiscard]] constexpr Matrix3x3 operator*(const Matrix3x3& other) const {
        return Matrix3x3(
            rows[0].x * other[0].x + rows[0].y * other[1].x + rows[0].z * other[2].x,
            rows[0].x * other[0].y + rows[0].y * other[1].y + rows[0].z * other[2].y,
            rows[0].x * other[0].z + rows[0].y * other[1].z + rows[0].z * other[2].z,

            rows[1].x * other[0].x + rows[1].y * other[1].x + rows[1].z * other[2].x,
            rows[1].x * other[0].y + rows[1].y * other[1].y + rows[1].z * other[2].y,
            rows[1].x * other[0].z + rows[1].y * other[1].z + rows[1].z * other[2].z,

            rows[2].x * other[0].x + rows[2].y * other[1].x + rows[2].z * other[2].x,
            rows[2].x * other[0].y + rows[2].y * other[1].y + rows[2].z * other[2].y,
            rows[2].x * other[0].z + rows[2].y * other[1].z + rows[2].z * other[2].z
        );
    }

    /// @}

    /// @name Matris Dönüşüm Fonksiyonları
    /// @{

    /**
     * @brief Matrisin transpozunu (satır ve sütunların yer değiştirmiş hali) döndürür.
     * @details Rotasyon matrislerinde bu işlem, dönüşümü tersine çevirmek için kullanılır 
     *          ve normal matematiksel ters alma işleminden çok daha performanslıdır.
     * @return Transpoze edilmiş yeni matris.
     */
    [[nodiscard]] constexpr Matrix3x3 getTranspose() const {
        return Matrix3x3(
            rows[0].x, rows[1].x, rows[2].x,
            rows[0].y, rows[1].y, rows[2].y,
            rows[0].z, rows[1].z, rows[2].z
        );
    }

    /**
     * @brief Matrisin determinantını hesaplar.
     * @details Matrisin hacimsel değişim oranını verir. Sonuç 0 çıkarsa matris 
     *          tersine çevrilemez (singüler) demektir.
     * @return Determinant değeri.
     */
    [[nodiscard]] constexpr float getDeterminant() const {
        return rows[0].x * (rows[1].y * rows[2].z - rows[2].y * rows[1].z) -
               rows[0].y * (rows[1].x * rows[2].z - rows[2].x * rows[1].z) +
               rows[0].z * (rows[1].x * rows[2].y - rows[2].x * rows[1].y);
    }

    /**
     * @brief Matrisin tam tersi olan (inverse) dönüşümü hesaplar.
     * @details Fizik motorunda eylemsizlik hesaplamalarında ve eklem (kısıtlama) 
     *          çözücülerinde sıklıkla kullanılır.
     * @return Ters çevrilmiş matris.
     * @pre Matrisin determinantı sıfır olmamalıdır (Aksi halde oyun çöker).
     */
    [[nodiscard]] Matrix3x3 getInverse() const {
        float det = getDeterminant();
        assert(det != 0.0f && "Matrix3x3 determinanti sifir, tersi alinamadi!");
        float invDet = 1.0f / det;

        return Matrix3x3(
            (rows[1].y * rows[2].z - rows[2].y * rows[1].z) * invDet,
            (rows[0].z * rows[2].y - rows[0].y * rows[2].z) * invDet,
            (rows[0].y * rows[1].z - rows[0].z * rows[1].y) * invDet,

            (rows[1].z * rows[2].x - rows[1].x * rows[2].z) * invDet,
            (rows[0].x * rows[2].z - rows[0].z * rows[2].x) * invDet,
            (rows[1].x * rows[0].z - rows[0].x * rows[1].z) * invDet,

            (rows[1].x * rows[2].y - rows[2].x * rows[1].y) * invDet,
            (rows[2].x * rows[0].y - rows[0].x * rows[2].y) * invDet,
            (rows[0].x * rows[1].y - rows[1].x * rows[0].y) * invDet
        );
    }

    /// @}
};

} // namespace Baryon