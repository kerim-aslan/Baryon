#pragma once
#include "../math/Vector3.hpp"
#include "../ecs/EntityManager.hpp"
#include <array>
#include <cstdint>
#include <algorithm>

/**
 * @file ContactManifold.hpp
 * @brief Çarpışma sonrası temas verilerini tutan temel yapılar.
 * @details Fizik çözücüsünün (solver) kullandığı temas noktası (ContactPoint) ve bu 
 *          noktaları gruplayan manifold (ContactManifold) yapılarını tanımlar.
 */
namespace Baryon::collision {

/**
 * @struct ContactPoint
 * @brief Tekil bir çarpışma temas noktasının verisi.
 * @details Çarpışan iki cisim üzerindeki yerel/dünya koordinatlarını, iç içe geçme 
 *          miktarını ve fizik çözücüsü için gerekli itme (impulse) değerlerini saklar.
 */
struct ContactPoint {
    Vector3 localPointA;    ///< A cisminin yerel uzayındaki temas noktası.
    Vector3 localPointB;    ///< B cisminin yerel uzayındaki temas noktası.
    Vector3 worldPosition;  ///< Temasın dünya koordinatlarındaki konumu.
    float penetration;      ///< İç içe geçme derinliği (metre). Pozitif değerler iç içe geçmeyi belirtir.
    
    /**
     * @brief Normal yönlü birikmiş itme (impulse) kuvveti.
     * @details Warm-starting (kareler arası çözücü hızlandırması) için saklanır.
     */
    float normalImpulse{0.0f};

    /**
     * @brief Teğet (sürtünme) yönündeki birikmiş itme kuvveti.
     */
    float tangentImpulse{0.0f};
};

/**
 * @struct ContactManifold
 * @brief İki cisim arasındaki temas noktalarını (en fazla 4) gruplayan yapı.
 * @details Sürekli çarpışma çözümü (warm-starting) için temas noktalarının 
 *          kareler (frames) arasında korunmasını ve yönetilmesini sağlar.
 */
struct ContactManifold {
    ecs::Entity entityA;   ///< Birinci cismin varlık kimliği.
    ecs::Entity entityB;   ///< İkinci cismin varlık kimliği.
    
    Vector3 normal;        ///< Temas normal vektörü (B'den A'ya doğru yönelir).
    float penetration{0};  ///< Bu manifolddaki en derin iç içe geçme miktarı.
    
    bool isTriggerPair{false}; ///< Bu temasın fiziksel tepki vermeyen bir tetikleyici (trigger) olup olmadığı.

    std::array<ContactPoint, 4> contacts; ///< En fazla 4 adet temas noktası dizisi.
    uint32_t contactCount{0};              ///< Dizideki aktif temas noktası sayısı.

    /**
     * @brief İki varlık için başlangıç durumunda boş bir temas manifoldu oluşturur.
     */
    ContactManifold(ecs::Entity a, ecs::Entity b) 
        : entityA(a), entityB(b), normal(0, 0, 0), penetration(0), isTriggerPair(false) {}
        
    /**
     * @brief Manifold'a yeni bir temas noktası ekler veya mevcut olanı günceller.
     * @details Eğer aynı noktada (çok yakın) bir temas zaten varsa onu günceller. 
     *          Kapasite (4) doluysa, yapıyı korumak için en sığ noktayı yenisiyle değiştirir.
     * @param newPoint Eklenmek istenen yeni temas noktası.
     */
    void addContact(const ContactPoint& newPoint) {
        // En yüksek penetrasyon derinliğini güncelle
        penetration = std::max(penetration, newPoint.penetration);

        // Noktalar birbirine çok yakınsa eskisini güncelle (Warm-starting koruması)
        for (uint32_t i = 0; i < contactCount; ++i) {
            if ((contacts[i].localPointA - newPoint.localPointA).lengthSquare() < 0.001f) {
                contacts[i] = newPoint;
                return;
            }
        }

        // Kapasite varsa doğrudan ekle
        if (contactCount < 4) {
            contacts[contactCount++] = newPoint;
        } else {
            // Kapasite doluysa en sığ (en az iç içe geçen) noktayı bul ve değiştir
            uint32_t shallowest = 0;
            float minPen = contacts[0].penetration;
            
            for(uint32_t i = 1; i < 4; ++i) {
                if(contacts[i].penetration < minPen) {
                    minPen = contacts[i].penetration;
                    shallowest = i;
                }
            }
            contacts[shallowest] = newPoint;
        }
    }
};

} // namespace Baryon::collision