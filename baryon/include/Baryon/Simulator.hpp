#pragma once

#include "Baryon/memory/MemoryManager.hpp"
#include "Baryon/Core/Registry.hpp"
#include "Baryon/systems/Integrator.hpp"
#include "Baryon/systems/HierarchySystem.hpp"
#include "Baryon/systems/SpatialPartitioning.hpp"
#include "Baryon/systems/CollisionSystem.hpp"
#include "Baryon/systems/SequentialImpulsesKernel.hpp"
#include "Baryon/systems/IslandSystem.hpp"
#include "Baryon/body/Body.hpp"
#include "Baryon/collision/Ray.hpp"

#include <vector>

/**
 * @file Simulator.hpp
 * @brief Baryon Fizik Motoru'nun ana yönetim sınıfı.
 * @details Simulator, tüm fizik alt sistemlerini (hareket, çarpışma, eklemler) 
 *          koordine eden en üst düzey sınıftır. Kullanıcının fizik dünyasıyla 
 *          iletişim kurmasını sağlayan ana arayüzdür.
 *
 *          Fiziksel İşlem Sırası (Pipeline):
 *          1. Hız Güncelleme (Kuvvetlerin hıza dönüştürülmesi)
 *          2. Sürekli Çarpışma Algılama (CCD - Hızlı objelerin takibi)
 *          3. Kaba Tarama (Broad Phase - AABB Ağacı ile hızlı filtreleme)
 *          4. Hassas Tarama (Narrow Phase - GJK/EPA ile kesin çarpışma testi)
 *          5. Gruplandırma (Island System - Temas halindeki objeleri ayırma)
 *          6. Çözücü (Solver - Çarpışma ve eklem kuvvetlerinin uygulanması)
 *          7. Konum Güncelleme (Hızın pozisyona dönüştürülmesi)
 *          8. Hiyerarşi Güncelleme (Bağlı nesnelerin konum takibi)
 */
namespace Baryon {

/**
 * @class Simulator
 * @brief Fizik dünyasını simüle eden ve tüm kuralları işleten ana merkez.
 * @details Zamanı sabit adımlarla (fixed timestep) ilerleterek her bilgisayarda 
 *          aynı fiziksel sonuçların alınmasını garanti eder. Varsayılan hız 60 Hz'dir.
 */
class Simulator {
private:
    memory::MemoryManager mMemoryManager;       ///< Bellek yönetiminden sorumlu birim.
    Core::Registry mRegistry;                   ///< Tüm varlıkların ve verilerin tutulduğu defter.

    /// @name Fizik Alt Sistemleri
    /// @{
    systems::Integrator mIntegrator;             ///< Hareket ve ivme hesaplayıcı.
    systems::HierarchySystem mHierarchySystem;   ///< Bağlı (parent-child) nesne yöneticisi.
    systems::SpatialPartitioning mSpatialPartitioning; ///< Uzaysal arama ve filtreleme sistemi.
    systems::CollisionSystem mCollisionSystem;   ///< Kesin çarpışma tespit birimi.
    systems::SequentialImpulsesKernel mSolver;   ///< Çarpışma ve eklem kuvveti uygulayıcı.
    systems::IslandSystem mIslandSystem;         ///< Optimizasyon ve uyku (sleep) yöneticisi.
    /// @}

    float mFixedTimeStep{1.0f / 60.0f};         ///< Sabit fizik adım boyutu (Varsayılan: 16.67ms).
    float mAccumulator{0.0f};                    ///< Kareler arası zamanı dengeleyen biriktirici.
    
public:
    /**
     * @brief Simülatörü ve tüm alt fizik sistemlerini başlatır.
     */
    Simulator();

    // Güvenlik: Motorun merkezi birimleri kopyalanamaz.
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    
    /**
     * @brief Fizik dünyasını geçen süre kadar simüle eder.
     * @details Sabit zaman adımı kullanarak fiziksel tutarlılığı korur. 
     *          Eğer bilgisayar çok yavaşlarsa, simülasyonun kilitlenmesini 
     *          önlemek için "performans kaybı koruması" (0.1s sınırı) devreye girer.
     * @param deltaTime Son kareden bu yana geçen gerçek süre (saniye).
     */
    void step(float deltaTime);

    /**
     * @brief Dünyaya bir "lazer" (ışın) fırlatarak objeleri arar.
     * @details Başlangıç noktasından belirtilen yöne doğru fırlatılan ışının 
     *          çarptığı en yakın objenin bilgilerini döndürür.
     * @param origin Işının çıkış noktası.
     * @param direction Işının fırlatıldığı yön.
     * @param maxDist Işının gidebileceği maksimum mesafe (Varsayılan: 1000).
     * @return Çarpışma sonucu (RaycastHit). Çarpma olup olmadığı kontrol edilmelidir.
     */
    collision::RaycastHit raycast(const Vector3& origin, const Vector3& direction, float maxDist = 1000.0f);

    /**
     * @brief Belirli bir objeye ait o anki tüm temas bilgilerini döndürür.
     * @details Karakter kontrolcüleri gibi özel sistemlerin çevredeki objelerle 
     *          olan temas detaylarını okuması için kullanılır.
     */
    std::vector<collision::ContactManifold> getContactsForEntity(ecs::Entity entity) const;

    /// @name Varlık ve Cisim Yönetimi
    /// @{

    /**
     * @brief Dünyada yeni bir fiziksel cisim (Body) oluşturur.
     * @details Belirtilen konum, dönüş ve şekil ile bir cisim yaratır. 
     *          Kütle ve dönme direnci gibi fiziksel özellikler otomatik hesaplanır.
     * @param transform Cismin başlangıç konumu ve bakış yönü.
     * @param shape Cismin geometrik şekli (Kutu, küre vb.).
     * @return Oluşturulan cismi yönetmek için kullanılan Body nesnesi.
     */
    Body createBody(const Pose& transform, const collision::CollisionShape& shape); 

    /**
     * @brief Bir cismi fizik dünyasından tamamen kaldırır.
     * @param body Silinecek cisim.
     */
    void destroyBody(Body& body);

    /// @}

    /// @name Eklem ve Kısıtlama Sistemleri
    /// @{

    /**
     * @brief İki cisim arasına sabit mesafeli bir bağlantı (ip/çubuk) ekler.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param distance Aradaki korunacak mesafe.
     */
    void createDistanceConstraint(Body& a, Body& b, float distance) {
        collision::DistanceConstraint constraint(a.getEntity(), b.getEntity(), distance);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cismi menteşe (kapı, tekerlek aksı vb.) ile birbirine bağlar.
     * @details Nesnelerin sadece belirli bir eksen etrafında dönmesine izin verir.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param anchorA Birinci cisim üzerindeki bağlantı noktası.
     * @param anchorB İkinci cisim üzerindeki bağlantı noktası.
     * @param axisA Birinci cisim üzerindeki dönüş ekseni.
     * @param axisB İkinci cisim üzerindeki dönüş ekseni.
     */
    void createRevoluteConstraint(Body& a, Body& b, 
                             const Vector3& anchorA, const Vector3& anchorB, 
                             const Vector3& axisA, const Vector3& axisB) {
        collision::RevoluteConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, axisA, axisB);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /// @}

    /// @name Ayarlar ve Durum Kaydı
    /// @{

    /**
     * @brief Dünyadaki genel ivmeyi (Yerçekimi vb.) ayarlar.
     * @param accelerationField İvme vektörü (Varsayılan: Y ekseninde -9.81).
     */
    void setAccelerationField(const Vector3& accelerationField);

    /**
     * @brief Fizik adımlarının çalışma sıklığını ayarlar.
     * @param timeStep Adım boyutu (Örn: 1.0f/120.0f değeri 120 Hz hız sağlar).
     */
    void setFixedTimeStep(float timeStep) { mFixedTimeStep = timeStep; }

    /**
     * @brief Veritabanına (Registry) doğrudan erişim sağlar (İleri düzey kullanım).
     */
    [[nodiscard]] Core::Registry& getRegistry() { return mRegistry; }

    /**
     * @brief Dünyanın o anki tam durumunu kopyalar (Snapshot).
     * @details Zamanı geri sarma veya oyun kaydetme sistemleri için kullanılır.
     * @return Kaydedilen dünya durumu.
     */
    [[nodiscard]] Core::Snapshot saveSnapshot() const {
        return mRegistry.save();
    }

    /**
     * @brief Dünyayı daha önce kaydedilmiş bir duruma geri döndürür.
     * @param snap Yüklenecek durum kaydı.
     */
    void restoreSnapshot(const Core::Snapshot& snap) {
        mRegistry.load(snap);
        mSpatialPartitioning.rebuildTree();
    }

    /**
     * @brief Tüm varlıkların AABB sınırlarını konsola yazdırır.
     * @details Çarpışma algılayıcı ve fizik motorunun görsellerle boyut uyumunu 
     *          doğrulamak için (Hata ayıklama amaçlı) kullanılır.
     */
    void printAABBs();

    /// @}
};

} // namespace Baryon