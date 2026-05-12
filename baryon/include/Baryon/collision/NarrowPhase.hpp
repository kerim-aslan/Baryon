#pragma once

#include "../math/Vector3.hpp"
#include "../math/Pose.hpp"
#include "CollisionShape.hpp"
#include <array>

#include <initializer_list>
#include "Ray.hpp"

/**
 * @file NarrowPhase.hpp
 * @brief Dar faz (Narrow Phase) çarpışma algılama sistemi.
 * @details Geniş fazdan (Broad Phase) gelen şüpheli cisim çiftlerinin gerçekten 
 *          çarpışıp çarpışmadığını kesin olarak belirleyen algoritmaları (GJK, EPA) içerir.
 */
namespace Baryon::collision {

/**
 * @struct CollisionInfo
 * @brief Kesin çarpışma testinin sonucunu tutan yapı.
 */
struct CollisionInfo {
    bool hasCollision{false}; ///< Çarpışma gerçekleşti mi?
    Vector3 normal;           ///< Çarpışma yüzeyinin normali (ayrılma yönü). B'den A'ya doğru bakar.
    float penetration{0};     ///< İki cismin birbirine ne kadar iç içe geçtiği (metre cinsinden).
    Vector3 contactPoint;     ///< Dünya koordinatlarında çarpışmanın gerçekleştiği temas noktası.
};

/**
 * @struct Simplex
 * @brief GJK ve EPA algoritmalarının kullandığı geçici geometrik yapı (nokta, çizgi, üçgen veya dörtyüzlü).
 * @details GJK algoritması kesişimi bulduğunda bu yapıyı oluşturur, ardından EPA 
 *          algoritması bu yapıyı kullanarak temas noktalarını ve derinliği hesaplar.
 */
struct Simplex {
private:
    std::array<Vector3, 4> mPoints;
    int mSize{0};

public:
    Simplex() = default;

    /** @brief Yeni bir noktayı en başa (LIFO mantığıyla) ekler. */
    void push_front(const Vector3& point) {
        mPoints = {point, mPoints[0], mPoints[1], mPoints[2]};
        mSize = std::min(mSize + 1, 4);
    }

    Vector3& operator[](int i) { return mPoints[i]; }
    const Vector3& operator[](int i) const { return mPoints[i]; }
    [[nodiscard]] int size() const { return mSize; }

    /** @brief Simpleks içeriğini verilen noktalarla sıfırdan doldurur. */
    void assign(std::initializer_list<Vector3> list) {
        mSize = 0;
        for (const auto& v : list) {
            mPoints[mSize++] = v;
        }
    }
};

/**
 * @class NarrowPhase
 * @brief Gelişmiş çarpışma testlerini barındıran statik yardımcı sınıf.
 * @details Herhangi bir durum (state) tutmaz, sadece verilen şekiller üzerinde 
 *          matematiksel çarpışma hesaplamaları yapar.
 */
class NarrowPhase {
public:
    /**
     * @brief İki şeklin kesişip kesişmediğini GJK algoritması ile test eder.
     * @param shapeA Test edilecek birinci şekil.
     * @param transformA Birinci şeklin dünya konumu ve yönelimi.
     * @param shapeB Test edilecek ikinci şekil.
     * @param transformB İkinci şeklin dünya konumu ve yönelimi.
     * @param[out] outSimplex Çarpışma varsa, EPA algoritmasına aktarılacak olan sonuç simpleksi.
     * @return Cisimler kesişiyorsa true, aksi halde false.
     */
    static bool GJK(const CollisionShape& shapeA, const Pose& transformA,
                    const CollisionShape& shapeB, const Pose& transformB,
                    Simplex& outSimplex);

    /**
     * @brief İki şekil arasındaki en kısa mesafeyi bulur.
     * @details Özellikle Sürekli Çarpışma Algılama (CCD) sisteminde cisimlerin 
     *          çarpışmadan hemen önceki anlarını bulmak için kullanılır.
     * @param[out] outPointA A şekli üzerindeki en yakın nokta (dünya koordinatlarında).
     * @param[out] outPointB B şekli üzerindeki en yakın nokta (dünya koordinatlarında).
     * @return İki şekil arasındaki mesafe (kesişiyorlarsa 0 döner).
     */
    static float GJK_distance(const CollisionShape& shapeA, const Pose& transformA,
                              const CollisionShape& shapeB, const Pose& transformB,
                              Vector3& outPointA, Vector3& outPointB);

    /**
     * @brief Bir ışının (ray) belirtilen şekle çarpıp çarpmadığını test eder.
     * @param shape Hedef şekil.
     * @param transform Şeklin dünya konumu.
     * @param ray Fırlatılan ışın.
     * @param[out] hit Çarpışma detaylarını (mesafe, normal, vb.) tutacak yapı.
     * @return Işın şekle çarptıysa true.
     */
    static bool raycast(const CollisionShape& shape, const Pose& transform, 
                        const Ray& ray, RaycastHit& hit);

    /**
     * @brief GJK sonrası iç içe geçme derinliğini ve ayrılma yönünü hesaplar (EPA Algoritması).
     * @details GJK algoritması sadece "çarpışma var" diyebilir. EPA ise bu çarpışmayı 
     *          çözebilmek (cisimleri ayırmak) için gereken fiziksel verileri üretir.
     * @param simplex GJK fonksiyonundan elde edilen başlangıç simpleksi.
     * @return Çarpışma yüzeyi normali, derinlik ve temas noktası bilgilerini içeren veri.
     */
    static CollisionInfo EPA(const Simplex& simplex, 
                             const CollisionShape& shapeA, const Pose& transformA,
                             const CollisionShape& shapeB, const Pose& transformB);

    /**
     * @brief Verilen bir yönde, şeklin en uç (en dış) noktasını bulur.
     * @details Şeklin yapısına (küre, kutu, vb.) göre o yöndeki en uzak noktayı hesaplar.
     *          GJK ve EPA algoritmalarının temel tarama fonksiyonudur.
     * @param shape Sorgulanacak şekil.
     * @param transform Şeklin dünya konumu.
     * @param direction Arama yapılacak yön vektörü.
     * @return Şeklin belirtilen yöndeki en uç noktası (dünya koordinatlarında).
     */
    static Vector3 getSupportPoint(const CollisionShape& shape, const Pose& transform, const Vector3& direction);

private:
    /**
     * @brief GJK algoritmasının iç durumunu güncelleyen yardımcı fonksiyon.
     * @details Eldeki noktalara bakarak bir sonraki adımda hangi yöne doğru 
     *          arama yapılması gerektiğini belirler.
     * @param simplex Güncellenecek simpleks.
     * @param direction Bir sonraki arama yönü (çıktı olarak güncellenir).
     * @return Orijin simpleksin içindeyse (kesişim kesinleştiyse) true döner.
     */
    static bool handleSimplex(Simplex& simplex, Vector3& direction);
};

} // namespace Baryon::collision