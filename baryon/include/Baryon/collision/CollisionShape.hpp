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

#include "AABB.hpp"
#include "../math/Vector3.hpp"
#include "../math/Matrix3x3.hpp"
#include "../math/Pose.hpp"
#include <variant>
#include <vector>
#include <memory>

/**
 * @file CollisionShape.hpp
 * @brief Çarpışma geometrisi şekilleri ve sarmalayıcı (wrapper) sınıf.
 * @details Sanal fonksiyon (vtable) maliyeti yaratmadan, std::variant kullanılarak 
 *          farklı çarpışma şekillerini güvenle yönetmeyi sağlar.
 */
namespace Baryon::collision {

/// @name Temel Geometri Şekilleri
/// @{

/**
 * @struct SphereShape
 * @brief Küre çarpışma geometrisi.
 * @details Yarıçap değeri ile tanımlanan en hızlı ve basit çarpışma şeklidir.
 */
struct SphereShape {
    float radius; ///< Kürenin yarıçapı (metre).

    /**
     * @brief Küre oluşturur.
     * @param r Yarıçap.
     */
    constexpr explicit SphereShape(float r) : radius(r) {}
};

/**
 * @struct BoxShape
 * @brief Kutu çarpışma geometrisi.
 * @details Merkezden dışa doğru yarım boyutlar (half-extents) ile tanımlanır.
 */
struct BoxShape {
    Vector3 halfExtents; ///< Kutunun her eksendeki yarım boyutu (tam boyutun yarısı).

    /**
     * @brief Kutu oluşturur.
     * @param he Yarım boyutlar vektörü.
     */
    constexpr explicit BoxShape(const Vector3& he) : halfExtents(he) {}
};

/**
 * @struct CapsuleShape
 * @brief Kapsül çarpışma geometrisi.
 * @details İki ucu yarımküre olan silindirik bir yapıdır. Genellikle karakter 
 *          kontrolcüleri (character controller) için kullanılır.
 */
struct CapsuleShape {
    float radius; ///< Kapsülün uçlarındaki yarımkürelerin yarıçapı.
    float height; ///< Orta silindir kısmının yüksekliği.

    /**
     * @brief Kapsül oluşturur.
     * @param r Yarıçap.
     * @param h Silindir yüksekliği.
     */
    constexpr CapsuleShape(float r, float h) : radius(r), height(h) {}
};

/**
 * @struct CylinderShape
 * @brief Silindir çarpışma geometrisi.
 * @details Yarıçap ve yerel Y ekseni boyunca yarım yükseklik (halfHeight) ile tanımlanır.
 *          Düz dairesel taban ve tavan kapaklarına sahiptir.
 */
struct CylinderShape {
    float radius;     ///< Silindirin yarıçapı (metre).
    float halfHeight; ///< Y ekseni boyunca yarım yükseklik (metre, tam yükseklik h = 2 * halfHeight).

    /**
     * @brief Silindir oluşturur.
     * @param r Yarıçap.
     * @param hh Yarım yükseklik.
     */
    constexpr CylinderShape(float r, float hh) : radius(r), halfHeight(hh) {}

    /** @brief Tam yüksekliği döndürür (h = 2 * halfHeight). */
    [[nodiscard]] constexpr float getHeight() const { return halfHeight * 2.0f; }
};

/**
 * @struct ConeShape
 * @brief Koni çarpışma geometrisi.
 * @details Yarıçap (radius) ve toplam yükseklik (height) ile tanımlanır.
 *          Ağırlık merkezi tabandan h/4 mesafede yerel orijinde (0,0,0) konumlandırılmıştır.
 *          Dairesel taban y = -0.25 * height, tepe noktası (apex) (0, 0.75 * height, 0) seviyesindedir.
 */
struct ConeShape {
    float radius; ///< Koni taban yarıçapı (metre).
    float height; ///< Koni toplam yüksekliği (metre).

    /**
     * @brief Koni oluşturur.
     * @param r Taban yarıçapı.
     * @param h Toplam yükseklik.
     */
    constexpr ConeShape(float r = 0.5f, float h = 1.0f) : radius(r), height(h) {}

    /** @brief Taban dairesinin yerel Y koordinatını döndürür (-h/4). */
    [[nodiscard]] constexpr float getBaseY() const { return -0.25f * height; }

    /** @brief Tepe noktasının yerel Y koordinatını döndürür (3h/4). */
    [[nodiscard]] constexpr float getApexY() const { return 0.75f * height; }
};

/**
 * @struct ConvexHullShape
 * @brief Dışbükey örtü (convex hull) çarpışma geometrisi.
 * @details Bir nokta bulutu (vertex dizisi) ile tanımlanan dışbükey hacimdir.
 */
struct ConvexHullShape {
    std::vector<Vector3> vertices; ///< Geometriyi oluşturan köşe noktaları.

    /**
     * @brief Dışbükey örtü oluşturur.
     * @param verts Köşe noktaları listesi.
     */
    explicit ConvexHullShape(const std::vector<Vector3>& verts) : vertices(verts) {}
};

/**
 * @struct TriangleShape
 * @brief Tekil üçgen çarpışma geometrisi.
 */
struct TriangleShape {
    Vector3 v0; ///< Birinci köşe.
    Vector3 v1; ///< İkinci köşe.
    Vector3 v2; ///< Üçüncü köşe.

    /**
     * @brief Üçgen oluşturur.
     */
    constexpr TriangleShape(const Vector3& a, const Vector3& b, const Vector3& c)
        : v0(a), v1(b), v2(c) {}
};

/// @}

/// @name Statik Mesh (Üçgen Ağı) Sistemi
/// @{

/**
 * @struct MeshTriangleData
 * @brief Bir ağ (mesh) içerisindeki üçgeni ve o üçgenin sınır kutusunu tutar.
 */
struct MeshTriangleData {
    TriangleShape shape;  ///< Üçgen geometrisi.
    AABB bounds;          ///< Üçgenin kendi sınırlayıcı kutusu.
};

/**
 * @class StaticMesh
 * @brief Statik üçgen ağı (triangle mesh) çarpışma geometrisi.
 * @details 3B modellerin (vertex ve index) verilerinden fizik üçgenleri oluşturur. 
 *          Geniş zeminler, binalar ve harita geometrileri için tasarlanmıştır.
 */
class StaticMesh {
private:
    std::vector<MeshTriangleData> mTriangles; ///< Üçgenler ve onlara ait AABB'ler.
    AABB mMeshBounds;                          ///< Tüm mesh'i saran genel AABB.

public:
    /**
     * @brief Vertex ve index dizilerinden statik mesh oluşturur.
     * @param vertices Modelin köşe noktaları.
     * @param indices Üçgen indeks dizisi (her 3 eleman bir üçgeni temsil eder).
     */
    StaticMesh(const std::vector<Vector3>& vertices, const std::vector<uint32_t>& indices) {
        Vector3 minB(1e10f, 1e10f, 1e10f);
        Vector3 maxB(-1e10f, -1e10f, -1e10f);

        for (size_t i = 0; i < indices.size(); i += 3) {
            TriangleShape tri(vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]]);
            
            Vector3 tMin(std::min({tri.v0.x, tri.v1.x, tri.v2.x}), std::min({tri.v0.y, tri.v1.y, tri.v2.y}), std::min({tri.v0.z, tri.v1.z, tri.v2.z}));
            Vector3 tMax(std::max({tri.v0.x, tri.v1.x, tri.v2.x}), std::max({tri.v0.y, tri.v1.y, tri.v2.y}), std::max({tri.v0.z, tri.v1.z, tri.v2.z}));
            
            mTriangles.push_back({tri, AABB(tMin, tMax)});

            minB.x = std::min(minB.x, tMin.x); minB.y = std::min(minB.y, tMin.y); minB.z = std::min(minB.z, tMin.z);
            maxB.x = std::max(maxB.x, tMax.x); maxB.y = std::max(maxB.y, tMax.y); maxB.z = std::max(maxB.z, tMax.z);
        }
        mMeshBounds = AABB(minB, maxB);
    }

    /**
     * @brief Tüm modeli kapsayan ana sınırlayıcı kutuyu döndürür.
     */
    [[nodiscard]] const AABB& getBounds() const { return mMeshBounds; }
    
    /**
     * @brief Belirtilen bir AABB ile kesişen üçgenleri bulur ve callback ile döndürür.
     * @details Performans optimizasyonu için tüm mesh yerine sadece kesişen üçgenler işlenir.
     * @tparam F Callback fonksiyon tipi.
     * @param targetAABB Kesişim testi yapılacak hedef AABB.
     * @param callback Kesişen her üçgen için çağrılacak fonksiyon.
     */
    template <typename F>
    void queryTriangles(const AABB& targetAABB, F&& callback) const {
        for (const auto& t : mTriangles) {
            if (t.bounds.testCollision(targetAABB)) {
                callback(t.shape);
            }
        }
    }
};

/**
 * @struct StaticMeshShape
 * @brief Bellek tasarrufu için StaticMesh nesnesini paylaşımlı (shared) tutan sarmalayıcı.
 */
struct StaticMeshShape {
    std::shared_ptr<StaticMesh> mesh; ///< Paylaşımlı mesh verisi.

    /**
     * @brief StaticMeshShape oluşturur.
     */
    explicit StaticMeshShape(std::shared_ptr<StaticMesh> m) : mesh(std::move(m)) {}
};

/**
 * @class HeightfieldShape
 * @brief Düzenli ızgara (heightfield/terrain) arazi çarpışma geometrisi.
 * @details X ve Z eksenlerinde düzenli aralıklarla yerleşen yükseklik (Y) örneklerinden oluşur.
 *          Geniş faz için tek bir AABB üretirken, dar fazda sadece hedef AABB ile kesişen
 *          ızgara hücrelerinin üçgenlerini O(1) indeks aralığı hesaplayarak hızlıca sorgular.
 */
class HeightfieldShape {
private:
    int32_t mWidthSamples{2};   ///< X eksenindeki örnek nokta sayısı (nx >= 2).
    int32_t mLengthSamples{2};  ///< Z eksenindeki örnek nokta sayısı (nz >= 2).
    float mCellSize{1.0f};      ///< Her bir hücrenin genişliği ve uzunluğu (dx = dz > 0).
    std::vector<float> mHeights;///< Yükseklik verileri (row-major: z * nx + x).
    AABB mBounds;               ///< Yerel sınırlayıcı kutu.
    bool mCentered{true};       ///< Orijinin ızgara merkezinde olup olmadığı.

    void recomputeBounds() {
        if (mHeights.empty()) {
            mBounds = AABB(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
            return;
        }
        float minH = 1e10f;
        float maxH = -1e10f;
        for (float h : mHeights) {
            if (!std::isnan(h) && !std::isinf(h)) {
                if (h < minH) minH = h;
                if (h > maxH) maxH = h;
            }
        }
        if (minH > maxH) {
            minH = 0.0f;
            maxH = 0.0f;
        }

        float totalW = static_cast<float>(mWidthSamples - 1) * mCellSize;
        float totalL = static_cast<float>(mLengthSamples - 1) * mCellSize;

        Vector3 minB, maxB;
        if (mCentered) {
            float halfW = totalW * 0.5f;
            float halfL = totalL * 0.5f;
            minB = Vector3(-halfW, minH, -halfL);
            maxB = Vector3(halfW, maxH, halfL);
        } else {
            minB = Vector3(0.0f, minH, 0.0f);
            maxB = Vector3(totalW, maxH, totalL);
        }
        mBounds = AABB(minB, maxB);
    }

public:
    HeightfieldShape() = default;

    /**
     * @brief Düzenli ızgara arazi geometrisi oluşturur.
     * @param nx X eksenindeki örnek nokta sayısı (nx >= 2).
     * @param nz Z eksenindeki örnek nokta sayısı (nz >= 2).
     * @param cellSize Hücre boyutu (metre cinsinden > 0).
     * @param heights Yükseklik dizisi (en az nx * nz eleman).
     * @param centered Orijinin arazinin yatay merkezinde olup olmadığı (varsayılan true).
     */
    HeightfieldShape(int32_t nx, int32_t nz, float cellSize, std::vector<float> heights, bool centered = true)
        : mWidthSamples(std::max(2, nx)),
          mLengthSamples(std::max(2, nz)),
          mCellSize(cellSize > 1e-4f ? cellSize : 1.0f),
          mHeights(std::move(heights)),
          mCentered(centered) {
        
        size_t requiredSize = static_cast<size_t>(mWidthSamples) * static_cast<size_t>(mLengthSamples);
        if (mHeights.size() < requiredSize) {
            mHeights.resize(requiredSize, 0.0f);
        }
        for (auto& h : mHeights) {
            if (std::isnan(h) || std::isinf(h)) {
                h = 0.0f;
            }
        }
        recomputeBounds();
    }

    [[nodiscard]] int32_t getWidthSamples() const noexcept { return mWidthSamples; }
    [[nodiscard]] int32_t getLengthSamples() const noexcept { return mLengthSamples; }
    [[nodiscard]] float getCellSize() const noexcept { return mCellSize; }
    [[nodiscard]] const std::vector<float>& getHeights() const noexcept { return mHeights; }
    [[nodiscard]] const AABB& getBounds() const noexcept { return mBounds; }
    [[nodiscard]] bool isCentered() const noexcept { return mCentered; }

    [[nodiscard]] float getHeight(int32_t x, int32_t z) const {
        if (x < 0) x = 0;
        if (x >= mWidthSamples) x = mWidthSamples - 1;
        if (z < 0) z = 0;
        if (z >= mLengthSamples) z = mLengthSamples - 1;
        return mHeights[static_cast<size_t>(z) * static_cast<size_t>(mWidthSamples) + static_cast<size_t>(x)];
    }

    [[nodiscard]] Vector3 getVertex(int32_t x, int32_t z) const {
        float h = getHeight(x, z);
        float totalW = static_cast<float>(mWidthSamples - 1) * mCellSize;
        float totalL = static_cast<float>(mLengthSamples - 1) * mCellSize;
        float vx = static_cast<float>(x) * mCellSize;
        float vz = static_cast<float>(z) * mCellSize;
        if (mCentered) {
            vx -= totalW * 0.5f;
            vz -= totalL * 0.5f;
        }
        return Vector3(vx, h, vz);
    }

    /**
     * @brief Hedef AABB ile kesişen ızgara hücrelerinin üçgenlerini O(1) hücre aralığı ile bulup çağırır.
     * @tparam F Callback fonksiyonu (void(const TriangleShape&)).
     */
    template <typename F>
    void queryTriangles(const AABB& targetAABB, F&& callback) const {
        if (!mBounds.testCollision(targetAABB))
            return;

        float totalW = static_cast<float>(mWidthSamples - 1) * mCellSize;
        float totalL = static_cast<float>(mLengthSamples - 1) * mCellSize;
        float offsetX = mCentered ? (totalW * 0.5f) : 0.0f;
        float offsetZ = mCentered ? (totalL * 0.5f) : 0.0f;

        float minGridX = (targetAABB.minBounds.x + offsetX) / mCellSize;
        float maxGridX = (targetAABB.maxBounds.x + offsetX) / mCellSize;
        float minGridZ = (targetAABB.minBounds.z + offsetZ) / mCellSize;
        float maxGridZ = (targetAABB.maxBounds.z + offsetZ) / mCellSize;

        int32_t i0 = std::clamp(static_cast<int32_t>(std::floor(minGridX)), 0, mWidthSamples - 2);
        int32_t i1 = std::clamp(static_cast<int32_t>(std::floor(maxGridX)), 0, mWidthSamples - 2);
        int32_t j0 = std::clamp(static_cast<int32_t>(std::floor(minGridZ)), 0, mLengthSamples - 2);
        int32_t j1 = std::clamp(static_cast<int32_t>(std::floor(maxGridZ)), 0, mLengthSamples - 2);

        for (int32_t j = j0; j <= j1; ++j) {
            for (int32_t i = i0; i <= i1; ++i) {
                Vector3 p00 = getVertex(i, j);
                Vector3 p10 = getVertex(i + 1, j);
                Vector3 p01 = getVertex(i, j + 1);
                Vector3 p11 = getVertex(i + 1, j + 1);

                // Tri 1: (p00, p01, p11) - normal yukari (+Y)
                TriangleShape tri1(p00, p01, p11);
                Vector3 t1Min(std::min({p00.x, p01.x, p11.x}), std::min({p00.y, p01.y, p11.y}), std::min({p00.z, p01.z, p11.z}));
                Vector3 t1Max(std::max({p00.x, p01.x, p11.x}), std::max({p00.y, p01.y, p11.y}), std::max({p00.z, p01.z, p11.z}));
                if (targetAABB.testCollision(AABB(t1Min, t1Max))) {
                    callback(tri1);
                }

                // Tri 2: (p00, p11, p10) - normal yukari (+Y)
                TriangleShape tri2(p00, p11, p10);
                Vector3 t2Min(std::min({p00.x, p11.x, p10.x}), std::min({p00.y, p11.y, p10.y}), std::min({p00.z, p11.z, p10.z}));
                Vector3 t2Max(std::max({p00.x, p11.x, p10.x}), std::max({p00.y, p11.y, p10.y}), std::max({p00.z, p11.z, p10.z}));
                if (targetAABB.testCollision(AABB(t2Min, t2Max))) {
                    callback(tri2);
                }
            }
        }
    }
};

class CollisionShape;

/**
 * @struct CompoundChild
 * @brief CompoundShape içindeki tekil alt şekil ve yerel transformu.
 */
struct CompoundChild {
    std::shared_ptr<CollisionShape> shapePtr; ///< Alt şekil (küre, kutu vb.).
    Pose localPose;                          ///< Ana gövdeye göre yerel konum ve yönelim.

    CompoundChild() = default;
    CompoundChild(const CompoundChild&) = default;
    CompoundChild(CompoundChild&&) noexcept = default;
    CompoundChild& operator=(const CompoundChild&) = default;
    CompoundChild& operator=(CompoundChild&&) noexcept = default;

    CompoundChild(CollisionShape s, const Pose& p = Pose());
    [[nodiscard]] const CollisionShape& getShape() const { return *shapePtr; }
    [[nodiscard]] CollisionShape& getShape() { return *shapePtr; }
};

/**
 * @struct CompoundShape
 * @brief Birden fazla alt şekli birleştiren bileşik çarpışma geometrisi.
 * @note İç içe bileşik şekillere (derinlik > 1) izin verilmez.
 */
struct CompoundShape {
    std::vector<CompoundChild> children; ///< Alt şekil listesi.

    CompoundShape() = default;
    CompoundShape(const CompoundShape&) = default;
    CompoundShape(CompoundShape&&) noexcept = default;
    CompoundShape& operator=(const CompoundShape&) = default;
    CompoundShape& operator=(CompoundShape&&) noexcept = default;
    explicit CompoundShape(std::vector<CompoundChild> ch) : children(std::move(ch)) {}

    /**
     * @brief Çocukların hacim ağırlıklı kütle merkezini hesaplar.
     */
    [[nodiscard]] Vector3 computeCenterOfMass() const;

    /**
     * @brief Paralel eksen teoremi (Huygens-Steiner) ile birleşik atalet tensörünü hesaplar.
     * @param totalMass Bileşik cismin toplam kütlesi.
     */
    [[nodiscard]] Matrix3x3 computeInertiaTensor(float totalMass) const;
};

/// @}

/// @name Çarpışma Şekli Sarmalayıcısı
/// @{

/**
 * @brief Desteklenen tüm çarpışma şekillerini içeren variant tipi.
 */
using ShapeVariant = std::variant<SphereShape, BoxShape, CapsuleShape, CylinderShape, ConeShape, ConvexHullShape, TriangleShape, StaticMeshShape, HeightfieldShape, CompoundShape>;

/**
 * @class CollisionShape
 * @brief Tüm çarpışma geometrilerini tek bir tip altında güvenle tutan sınıf.
 */
class CollisionShape {
private:
    ShapeVariant mShape; ///< İçerilen aktif geometri.

public:
    /**
     * @brief Herhangi bir desteklenen şekilden CollisionShape oluşturur.
     */
    template<typename T>
    constexpr explicit CollisionShape(T&& shape) : mShape(std::forward<T>(shape)) {}

    /**
     * @brief İçerilen şeklin variant nesnesine erişim sağlar.
     */
    [[nodiscard]] const ShapeVariant& getVariant() const { return mShape; }

    /**
     * @brief Şeklin kendi yerel koordinat sistemindeki sınırlayıcı kutusunu (AABB) hesaplar.
     * @return Yerel uzaydaki AABB.
     */
    [[nodiscard]] AABB computeLocalAABB() const {
        return std::visit([](const auto& shape) -> AABB {
            using T = std::decay_t<decltype(shape)>;

            if constexpr (std::is_same_v<T, SphereShape>) {
                return AABB(Vector3(-shape.radius, -shape.radius, -shape.radius), Vector3(shape.radius, shape.radius, shape.radius));
            } else if constexpr (std::is_same_v<T, BoxShape>) {
                return AABB(-shape.halfExtents, shape.halfExtents);
            } else if constexpr (std::is_same_v<T, CapsuleShape>) {
                Vector3 r(shape.radius, shape.radius + (shape.height * 0.5f), shape.radius);
                return AABB(-r, r);
            } else if constexpr (std::is_same_v<T, CylinderShape>) {
                return AABB(Vector3(-shape.radius, -shape.halfHeight, -shape.radius), 
                            Vector3(shape.radius, shape.halfHeight, shape.radius));
            } else if constexpr (std::is_same_v<T, ConeShape>) {
                return AABB(Vector3(-shape.radius, shape.getBaseY(), -shape.radius),
                            Vector3(shape.radius, shape.getApexY(), shape.radius));
            } else if constexpr (std::is_same_v<T, ConvexHullShape>) {
                if (shape.vertices.empty()) return AABB(Vector3(0,0,0), Vector3(0,0,0));
                Vector3 minB = shape.vertices[0], maxB = shape.vertices[0];
                for (const auto& v : shape.vertices) {
                    minB.x = std::min(minB.x, v.x); minB.y = std::min(minB.y, v.y); minB.z = std::min(minB.z, v.z);
                    maxB.x = std::max(maxB.x, v.x); maxB.y = std::max(maxB.y, v.y); maxB.z = std::max(maxB.z, v.z);
                }
                return AABB(minB, maxB);
            } else if constexpr (std::is_same_v<T, TriangleShape>) {
                Vector3 minB(std::min({shape.v0.x, shape.v1.x, shape.v2.x}), std::min({shape.v0.y, shape.v1.y, shape.v2.y}), std::min({shape.v0.z, shape.v1.z, shape.v2.z}));
                Vector3 maxB(std::max({shape.v0.x, shape.v1.x, shape.v2.x}), std::max({shape.v0.y, shape.v1.y, shape.v2.y}), std::max({shape.v0.z, shape.v1.z, shape.v2.z}));
                return AABB(minB, maxB);
            } 
            else if constexpr (std::is_same_v<T, StaticMeshShape>) {
                return shape.mesh ? shape.mesh->getBounds() : AABB(Vector3(0,0,0), Vector3(0,0,0));
            }
            else if constexpr (std::is_same_v<T, HeightfieldShape>) {
                return shape.getBounds();
            }
            else if constexpr (std::is_same_v<T, CompoundShape>) {
                if (shape.children.empty()) return AABB(Vector3(0,0,0), Vector3(0,0,0));
                Vector3 globalMin(1e10f, 1e10f, 1e10f);
                Vector3 globalMax(-1e10f, -1e10f, -1e10f);

                for (const auto& child : shape.children) {
                    AABB childLocal = child.getShape().computeLocalAABB();
                    Vector3 minB = childLocal.minBounds;
                    Vector3 maxB = childLocal.maxBounds;
                    Vector3 corners[8] = {
                        Vector3(minB.x, minB.y, minB.z), Vector3(maxB.x, minB.y, minB.z),
                        Vector3(minB.x, maxB.y, minB.z), Vector3(maxB.x, maxB.y, minB.z),
                        Vector3(minB.x, minB.y, maxB.z), Vector3(maxB.x, minB.y, maxB.z),
                        Vector3(minB.x, maxB.y, maxB.z), Vector3(maxB.x, maxB.y, maxB.z)
                    };
                    for (const auto& c : corners) {
                        Vector3 transC = child.localPose.position + (child.localPose.orientation * c);
                        globalMin.x = std::min(globalMin.x, transC.x);
                        globalMin.y = std::min(globalMin.y, transC.y);
                        globalMin.z = std::min(globalMin.z, transC.z);
                        globalMax.x = std::max(globalMax.x, transC.x);
                        globalMax.y = std::max(globalMax.y, transC.y);
                        globalMax.z = std::max(globalMax.z, transC.z);
                    }
                }
                return AABB(globalMin, globalMax);
            }
            return AABB();
        }, mShape);
    }
};

inline CompoundChild::CompoundChild(CollisionShape s, const Pose& p)
    : shapePtr(std::make_shared<CollisionShape>(std::move(s))), localPose(p) {}

inline Vector3 CompoundShape::computeCenterOfMass() const {
    if (children.empty()) return Vector3(0.0f, 0.0f, 0.0f);

    float totalVolume = 0.0f;
    Vector3 weightedSum(0.0f, 0.0f, 0.0f);

    for (const auto& child : children) {
        float vol = std::visit([](const auto& s) -> float {
            using ST = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<ST, SphereShape>) {
                return (4.0f / 3.0f) * 3.14159265f * s.radius * s.radius * s.radius;
            } else if constexpr (std::is_same_v<ST, BoxShape>) {
                Vector3 size = s.halfExtents * 2.0f;
                return size.x * size.y * size.z;
            } else if constexpr (std::is_same_v<ST, CapsuleShape>) {
                float r = s.radius;
                float h = s.height;
                return 3.14159265f * r * r * h + (4.0f / 3.0f) * 3.14159265f * r * r * r;
            } else if constexpr (std::is_same_v<ST, CylinderShape>) {
                float r = s.radius;
                float h = s.getHeight();
                return 3.14159265f * r * r * h;
            } else if constexpr (std::is_same_v<ST, ConeShape>) {
                float r = s.radius;
                float h = s.height;
                return (1.0f / 3.0f) * 3.14159265f * r * r * h;
            } else if constexpr (std::is_same_v<ST, HeightfieldShape>) {
                return 0.0f;
            } else {
                return 1.0f;
            }
        }, child.getShape().getVariant());

        totalVolume += vol;
        weightedSum += child.localPose.position * vol;
    }

    if (totalVolume > 1e-8f) {
        return weightedSum / totalVolume;
    }
    return Vector3(0.0f, 0.0f, 0.0f);
}

inline Matrix3x3 CompoundShape::computeInertiaTensor(float totalMass) const {
    if (children.empty() || totalMass <= 0.0f) {
        return Matrix3x3::identity();
    }

    Vector3 com = computeCenterOfMass();

    std::vector<float> volumes;
    volumes.reserve(children.size());
    float totalVolume = 0.0f;

    for (const auto& child : children) {
        float vol = std::visit([](const auto& s) -> float {
            using ST = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<ST, SphereShape>) {
                return (4.0f / 3.0f) * 3.14159265f * s.radius * s.radius * s.radius;
            } else if constexpr (std::is_same_v<ST, BoxShape>) {
                Vector3 size = s.halfExtents * 2.0f;
                return size.x * size.y * size.z;
            } else if constexpr (std::is_same_v<ST, CapsuleShape>) {
                float r = s.radius;
                float h = s.height;
                return 3.14159265f * r * r * h + (4.0f / 3.0f) * 3.14159265f * r * r * r;
            } else if constexpr (std::is_same_v<ST, CylinderShape>) {
                float r = s.radius;
                float h = s.getHeight();
                return 3.14159265f * r * r * h;
            } else if constexpr (std::is_same_v<ST, ConeShape>) {
                float r = s.radius;
                float h = s.height;
                return (1.0f / 3.0f) * 3.14159265f * r * r * h;
            } else if constexpr (std::is_same_v<ST, HeightfieldShape>) {
                return 0.0f;
            } else {
                return 1.0f;
            }
        }, child.getShape().getVariant());

        volumes.push_back(vol);
        totalVolume += vol;
    }

    Matrix3x3 totalI;
    totalI[0] = Vector3(0, 0, 0);
    totalI[1] = Vector3(0, 0, 0);
    totalI[2] = Vector3(0, 0, 0);

    for (size_t i = 0; i < children.size(); ++i) {
        const auto& child = children[i];
        float m_i = (totalVolume > 1e-8f) ? (totalMass * (volumes[i] / totalVolume))
                                          : (totalMass / static_cast<float>(children.size()));

        Matrix3x3 I_local;
        std::visit([&I_local, m_i](const auto& s) {
            using ST = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<ST, SphereShape>) {
                float val = 0.4f * m_i * s.radius * s.radius;
                I_local[0].x = val; I_local[1].y = val; I_local[2].z = val;
            } else if constexpr (std::is_same_v<ST, BoxShape>) {
                Vector3 size = s.halfExtents * 2.0f;
                I_local[0].x = (1.0f / 12.0f) * m_i * (size.y * size.y + size.z * size.z);
                I_local[1].y = (1.0f / 12.0f) * m_i * (size.x * size.x + size.z * size.z);
                I_local[2].z = (1.0f / 12.0f) * m_i * (size.x * size.x + size.y * size.y);
            } else if constexpr (std::is_same_v<ST, CapsuleShape>) {
                float r = s.radius;
                float h = s.height;
                float cylMass = m_i * (h / (h + (4.0f / 3.0f) * r));
                float capMass = m_i - cylMass;
                float ix = cylMass * ((1.0f / 12.0f) * h * h + (1.0f / 4.0f) * r * r) +
                           capMass * ((2.0f / 5.0f) * r * r + (3.0f / 8.0f) * h * r + (1.0f / 4.0f) * h * h);
                float iy = (1.0f / 2.0f) * cylMass * r * r + (2.0f / 5.0f) * capMass * r * r;
                I_local[0].x = ix; I_local[1].y = iy; I_local[2].z = ix;
            } else if constexpr (std::is_same_v<ST, CylinderShape>) {
                float r = s.radius;
                float h = s.getHeight();
                float ixz = (1.0f / 12.0f) * m_i * (3.0f * r * r + h * h);
                float iy = 0.5f * m_i * r * r;
                I_local[0].x = ixz; I_local[1].y = iy; I_local[2].z = ixz;
            } else if constexpr (std::is_same_v<ST, ConeShape>) {
                float r = s.radius;
                float h = s.height;
                float ixz = (3.0f / 20.0f) * m_i * (r * r + 0.25f * h * h);
                float iy = 0.3f * m_i * r * r;
                I_local[0].x = ixz; I_local[1].y = iy; I_local[2].z = ixz;
            } else if constexpr (std::is_same_v<ST, HeightfieldShape>) {
                I_local = Matrix3x3::identity();
            } else {
                I_local[0].x = 1.0f / 6.0f * m_i;
                I_local[1].y = 1.0f / 6.0f * m_i;
                I_local[2].z = 1.0f / 6.0f * m_i;
            }
        }, child.getShape().getVariant());

        Matrix3x3 R = child.localPose.getOrientationMatrix();
        Matrix3x3 I_rotated = R * I_local * R.getTranspose();

        Vector3 d = child.localPose.position - com;
        Matrix3x3 parallelAxis;
        parallelAxis[0].x = m_i * (d.y * d.y + d.z * d.z);
        parallelAxis[0].y = -m_i * (d.x * d.y);
        parallelAxis[0].z = -m_i * (d.x * d.z);

        parallelAxis[1].x = -m_i * (d.x * d.y);
        parallelAxis[1].y = m_i * (d.x * d.x + d.z * d.z);
        parallelAxis[1].z = -m_i * (d.y * d.z);

        parallelAxis[2].x = -m_i * (d.x * d.z);
        parallelAxis[2].y = -m_i * (d.y * d.z);
        parallelAxis[2].z = m_i * (d.x * d.x + d.y * d.y);

        totalI[0] += I_rotated[0] + parallelAxis[0];
        totalI[1] += I_rotated[1] + parallelAxis[1];
        totalI[2] += I_rotated[2] + parallelAxis[2];
    }

    return totalI;
}

/// @}

} // namespace Baryon::collision