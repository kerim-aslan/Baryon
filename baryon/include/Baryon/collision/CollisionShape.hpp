#pragma once

#include "AABB.hpp"
#include "../math/Vector3.hpp"
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

/// @}

/// @name Çarpışma Şekli Sarmalayıcısı
/// @{

/**
 * @brief Desteklenen tüm çarpışma şekillerini içeren variant tipi.
 */
using ShapeVariant = std::variant<SphereShape, BoxShape, CapsuleShape, ConvexHullShape, TriangleShape, StaticMeshShape>;

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
            return AABB();
        }, mShape);
    }
};

/// @}

} // namespace Baryon::collision