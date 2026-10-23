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

#include "Baryon/memory/MemoryManager.hpp"
#include "Baryon/Core/Registry.hpp"
#include "Baryon/SimulationSettings.hpp"
#include "Baryon/systems/Integrator.hpp"
#include "Baryon/systems/HierarchySystem.hpp"
#include "Baryon/systems/SpatialPartitioning.hpp"
#include "Baryon/systems/CollisionSystem.hpp"
#include "Baryon/systems/SequentialImpulsesKernel.hpp"
#include "Baryon/systems/IslandSystem.hpp"
#include "Baryon/body/Body.hpp"
#include "Baryon/collision/Ray.hpp"
#include "Baryon/collision/Sweep.hpp"
#include "Baryon/collision/CollisionListener.hpp"

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

    SimulationSettings mSettings;               ///< Simülasyon yapılandırma ayarları.
    float mFixedTimeStep{1.0f / 60.0f};         ///< Sabit fizik adım boyutu (Varsayılan: 16.67ms).
    float mAccumulator{0.0f};                    ///< Kareler arası zamanı dengeleyen biriktirici.
    int mMaxSubSteps{8};                         ///< Tek bir karede izin verilen maksimum alt adım sayısı.
    
public:
    /**
     * @brief Simülatörü ve tüm alt fizik sistemlerini başlatır.
     */
    Simulator();

    // Güvenlik: Motorun merkezi birimleri kopyalanamaz.
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    /**
     * @brief Simülasyon yapılandırma ayarlarını döndürür.
     */
    [[nodiscard]] const SimulationSettings& getSettings() const { return mSettings; }
    [[nodiscard]] SimulationSettings& getSettings() { return mSettings; }

    /**
     * @brief Simülasyon ayarlarını günceller ve alt sistemlere iletir.
     */
    void setSettings(const SimulationSettings& settings);
    
    /**
     * @brief Fizik dünyasını geçen süre kadar simüle eder.
     * @details Sabit zaman adımı kullanarak fiziksel tutarlılığı korur. 
     *          Eğer bilgisayar çok yavaşlarsa, simülasyonun kilitlenmesini 
     *          önlemek için "performans kaybı koruması" (0.1s sınırı) devreye girer.
     * @param deltaTime Son kareden bu yana geçen gerçek süre (saniye).
     */
    void step(float deltaTime);

    /**
     * @brief Tek bir karede çalıştırılabilecek maksimum alt adım sayısını ayarlar.
     * @param maxSubSteps Maksimum alt adım sayısı (varsayılan 8).
     */
    void setMaxSubSteps(int maxSubSteps) {
        mSettings.maxSubSteps = std::max(1, maxSubSteps);
        mMaxSubSteps = mSettings.maxSubSteps;
    }

    /**
     * @brief Mevcut maksimum alt adım sayısını döndürür.
     */
    [[nodiscard]] int getMaxSubSteps() const {
        return mSettings.maxSubSteps;
    }

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
     * @brief Dünya üzerinde ışın fırlatarak ışının kestiği TÜM objeleri tespit eder.
     * @details İade edilen liste mesafeye göre (en yakından en uzağa) küçükten büyüğe sıralıdır.
     * @param origin Işının çıkış noktası.
     * @param direction Işının fırlatıldığı yön.
     * @param maxDist Işının gidebileceği maksimum mesafe (Varsayılan: 1000.0f).
     * @param layerMask Filtrelenecek katman bitmaskesi (Varsayılan: 0xFFFFFFFF).
     * @return Kesişen tüm cisimlerin çarpışma detaylarını içeren sıralı liste.
     */
    [[nodiscard]] std::vector<collision::RaycastHit> raycastAll(const Vector3& origin, const Vector3& direction, 
                                                                float maxDist = 1000.0f, uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Belirli bir objeye ait o anki tüm temas bilgilerini döndürür.
     * @details Karakter kontrolcüleri gibi özel sistemlerin çevredeki objelerle 
     *          olan temas detaylarını okuması için kullanılır.
     */
    std::vector<collision::ContactManifold> getContactsForEntity(ecs::Entity entity) const;

    /// @name Hacim ve Çakışma Sorguları (Overlap Queries)
    /// @{

    /**
     * @brief Belirtilen küre hacmi içindeki tüm varlıkları sorgular.
     * @param center Kürenin dünya koordinatlarındaki merkezi.
     * @param radius Kürenin yarıçapı.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Kesişen varlıkların (Entity) listesi.
     */
    [[nodiscard]] std::vector<ecs::Entity> overlapSphere(const Vector3& center, float radius, uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Belirtilen yönelimli kutu hacmi içindeki tüm varlıkları sorgular.
     * @param center Kutunun dünya koordinatlarındaki merkezi.
     * @param halfExtents Kutunun yarım boyutları (x, y, z).
     * @param orientation Kutunun dönüş yönelimi (varsayılan: birim kuaterniyon).
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Kesişen varlıkların (Entity) listesi.
     */
    [[nodiscard]] std::vector<ecs::Entity> overlapBox(const Vector3& center, const Vector3& halfExtents, 
                                                      const Quaternion& orientation = Quaternion(), 
                                                      uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Herhangi bir çarpışma şekli ile hacim çakışma sorgusu yapar.
     * @param shape Sorgulanacak çarpışma geometrisi (Küre, Kutu, Kapsül, CompoundShape vb.).
     * @param pose Şeklin dünya konumu ve yönelimi.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Kesişen varlıkların (Entity) listesi.
     */
    [[nodiscard]] std::vector<ecs::Entity> overlapShape(const collision::CollisionShape& shape, const Pose& pose, 
                                                        uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief overlapSphere sorgusunu Body listesi olarak döndürür.
     */
    [[nodiscard]] std::vector<Body> overlapSphereBodies(const Vector3& center, float radius, uint32_t layerMask = 0xFFFFFFFF);

    /**
     * @brief overlapBox sorgusunu Body listesi olarak döndürür.
     */
    [[nodiscard]] std::vector<Body> overlapBoxBodies(const Vector3& center, const Vector3& halfExtents, 
                                                     const Quaternion& orientation = Quaternion(), 
                                                     uint32_t layerMask = 0xFFFFFFFF);

    /**
     * @brief overlapShape sorgusunu Body listesi olarak döndürür.
     */
    [[nodiscard]] std::vector<Body> overlapShapeBodies(const collision::CollisionShape& shape, const Pose& pose, 
                                                       uint32_t layerMask = 0xFFFFFFFF);

    /// @}

    /// @name Süpürme ve Şekil Fırlatma Sorguları (Sweep Queries / Shape Cast)
    /// @{

    /**
     * @brief Küre şeklini belirtilen doğrultuda fırlatarak ilk temas noktasını sorgular.
     * @param center Kürenin başlangıç merkezi.
     * @param radius Kürenin yarıçapı.
     * @param direction Süpürme yönü.
     * @param maxDistance Maksimum süpürme mesafesi.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Süpürme temas sonucu (SweepHit).
     */
    [[nodiscard]] collision::SweepHit sweepSphere(const Vector3& center, float radius, 
                                                  const Vector3& direction, float maxDistance, 
                                                  uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Yönelimli kutu şeklini belirtilen doğrultuda fırlatarak ilk temas noktasını sorgular.
     * @param center Kutunun başlangıç merkezi.
     * @param halfExtents Kutunun yarım boyutları (x, y, z).
     * @param orientation Kutunun dönüş yönelimi.
     * @param direction Süpürme yönü.
     * @param maxDistance Maksimum süpürme mesafesi.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Süpürme temas sonucu (SweepHit).
     */
    [[nodiscard]] collision::SweepHit sweepBox(const Vector3& center, const Vector3& halfExtents, 
                                               const Quaternion& orientation, const Vector3& direction, 
                                               float maxDistance, uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Yönelimli kapsül şeklini belirtilen doğrultuda fırlatarak ilk temas noktasını sorgular.
     * @param center Kapsülün başlangıç merkezi.
     * @param radius Kapsülün yarıçapı.
     * @param height Kapsülün silindir yüksekliği.
     * @param orientation Kapsülün dönüş yönelimi.
     * @param direction Süpürme yönü.
     * @param maxDistance Maksimum süpürme mesafesi.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Süpürme temas sonucu (SweepHit).
     */
    [[nodiscard]] collision::SweepHit sweepCapsule(const Vector3& center, float radius, float height, 
                                                   const Quaternion& orientation, const Vector3& direction, 
                                                   float maxDistance, uint32_t layerMask = 0xFFFFFFFF) const;

    /**
     * @brief Herhangi bir geometrik şekli belirtilen doğrultuda fırlatarak ilk temas noktasını sorgular.
     * @param shape Fırlatılacak şekil (Küre, Kutu, Kapsül, CompoundShape vb.).
     * @param startPose Şeklin başlangıç pozu (konum ve yönelim).
     * @param direction Süpürme yönü.
     * @param maxDistance Maksimum süpürme mesafesi.
     * @param layerMask Filtrelenecek katman bitmaskesi (varsayılan: 0xFFFFFFFF).
     * @return Süpürme temas sonucu (SweepHit).
     */
    [[nodiscard]] collision::SweepHit sweepShape(const collision::CollisionShape& shape, const Pose& startPose, 
                                                 const Vector3& direction, float maxDistance, 
                                                 uint32_t layerMask = 0xFFFFFFFF) const;

    /// @}

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
    void setMaxEntityCapacity(uint32_t maxCapacity) { mRegistry.setMaxCapacity(maxCapacity); }
    [[nodiscard]] uint32_t getMaxEntityCapacity() const { return mRegistry.getMaxCapacity(); }


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
        Quaternion relOrientation = Quaternion::identity();
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            relOrientation = poseA.orientation.getInverse() * poseB.orientation;
        }
        collision::RevoluteConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, axisA, axisB, relOrientation);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cismi birbirine tamamen sabitler (kaynaklar / weld).
     * @details 6 serbestlik derecesinin tümünü kilitler; cisimler tek bir katı gövde gibi davranır.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param anchorA Birinci cisim üzerindeki yerel bağlantı noktası.
     * @param anchorB İkinci cisim üzerindeki yerel bağlantı noktası.
     */
    void createFixedConstraint(Body& a, Body& b, 
                              const Vector3& anchorA, const Vector3& anchorB) {
        Quaternion relOrientation = Quaternion::identity();
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            relOrientation = poseA.orientation.getInverse() * poseB.orientation;
        }
        collision::FixedConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, relOrientation);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cismi mevcut konum ve yönelimlerinde birbirine kaynaklar.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     */
    void createFixedConstraint(Body& a, Body& b) {
        Vector3 anchorA(0.0f, 0.0f, 0.0f);
        Vector3 anchorB(0.0f, 0.0f, 0.0f);
        Quaternion relOrientation = Quaternion::identity();
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            Vector3 worldMid = (poseA.position + poseB.position) * 0.5f;
            anchorA = poseA.orientation.getInverse() * (worldMid - poseA.position);
            anchorB = poseB.orientation.getInverse() * (worldMid - poseB.position);
            relOrientation = poseA.orientation.getInverse() * poseB.orientation;
        }
        collision::FixedConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, relOrientation);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cisim arasına bilyeli mafsal / küresel eklem (Ball-Socket) bağlar.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param anchorA Birinci cisim üzerindeki yerel ankor noktası.
     * @param anchorB İkinci cisim üzerindeki yerel ankor noktası.
     */
    void createBallSocketConstraint(Body& a, Body& b,
                                    const Vector3& anchorA, const Vector3& anchorB) {
        collision::BallSocketConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cisim arasına dünya uzayındaki ortak bir pivot noktası üzerinden bilyeli mafsal bağlar.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param worldPivot Dünya koordinatlarındaki ortak mafsal noktası.
     */
    void createBallSocketConstraint(Body& a, Body& b, const Vector3& worldPivot) {
        Vector3 anchorA(0.0f, 0.0f, 0.0f);
        Vector3 anchorB(0.0f, 0.0f, 0.0f);
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            anchorA = poseA.orientation.getConjugate() * (worldPivot - poseA.position);
            anchorB = poseB.orientation.getConjugate() * (worldPivot - poseB.position);
        }
        collision::BallSocketConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cisim arasına prizmatik (kayar) eklem bağlar.
     * @param a Birinci cisim (ray/kılavuz gövdesi).
     * @param b İkinci cisim (kayıcı gövde).
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param axisA A cismi yerel koordinatlarındaki kayma ekseni.
     */
    void createPrismaticConstraint(Body& a, Body& b,
                                   const Vector3& anchorA, const Vector3& anchorB,
                                   const Vector3& axisA = Vector3(1.0f, 0.0f, 0.0f)) {
        Quaternion relOrientation = Quaternion::identity();
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            relOrientation = poseA.orientation.getInverse() * poseB.orientation;
        }
        collision::PrismaticConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, axisA, relOrientation);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief Prizmatik ekleme alt ve üst kayma sınırları (limit) atar.
     * @param body Eklem bileşenine sahip cisim.
     * @param lowerLimit Minimum kayma mesafesi (metre).
     * @param upperLimit Maksimum kayma mesafesi (metre).
     */
    void setPrismaticLimits(Body& body, float lowerLimit, float upperLimit) {
        if (mRegistry.hasComponent<collision::PrismaticConstraint>(body.getEntity())) {
            auto& pc = mRegistry.getComponent<collision::PrismaticConstraint>(body.getEntity());
            pc.enableLimits = true;
            pc.lowerLimit = lowerLimit;
            pc.upperLimit = upperLimit;
        }
    }

    /**
     * @brief Prizmatik ekleme doğrusal motor parametreleri atar.
     * @param body Eklem bileşenine sahip cisim.
     * @param motorSpeed Hedef doğrusal kayma hızı (m/s).
     * @param maxMotorForce Maksimum motor kuvveti (Newton).
     */
    void setPrismaticMotor(Body& body, float motorSpeed, float maxMotorForce) {
        if (mRegistry.hasComponent<collision::PrismaticConstraint>(body.getEntity())) {
            auto& pc = mRegistry.getComponent<collision::PrismaticConstraint>(body.getEntity());
            pc.enableMotor = true;
            pc.motorSpeed = motorSpeed;
            pc.maxMotorForce = maxMotorForce;
        }
    }

    /**
     * @brief Menteşe (Revolute) eklemine açısal dönüş sınırları (limit) atar.
     * @param body Eklem bileşenine sahip cisim.
     * @param lowerAngleLimit İzin verilen minimum dönüş açısı (radyan).
     * @param upperAngleLimit İzin verilen maksimum dönüş açısı (radyan).
     */
    void setRevoluteLimits(Body& body, float lowerAngleLimit, float upperAngleLimit) {
        if (mRegistry.hasComponent<collision::RevoluteConstraint>(body.getEntity())) {
            auto& rc = mRegistry.getComponent<collision::RevoluteConstraint>(body.getEntity());
            rc.setLimits(lowerAngleLimit, upperAngleLimit);
        }
    }

    /**
     * @brief Menteşe (Revolute) eklemine açısal motor parametreleri atar.
     * @param body Eklem bileşenine sahip cisim.
     * @param motorSpeed Hedef açısal dönüş hızı (rad/s).
     * @param maxMotorTorque Uygulanabilecek maksimum tork limiti (Newton*metre).
     */
    void setRevoluteMotor(Body& body, float motorSpeed, float maxMotorTorque) {
        if (mRegistry.hasComponent<collision::RevoluteConstraint>(body.getEntity())) {
            auto& rc = mRegistry.getComponent<collision::RevoluteConstraint>(body.getEntity());
            rc.setMotor(motorSpeed, maxMotorTorque);
        }
    }

    /**
     * @brief İki cisim arasına sönümlü yay / elastik mesafe kısıtı bağlar.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param restLength Yayın gerilmesiz doğal serbest uzunluğu (metre).
     * @param stiffness Yay sertlik katsayısı (k, N/m).
     * @param damping Sönümleme katsayısı (c, N*s/m).
     */
    void createSpringConstraint(Body& a, Body& b,
                                const Vector3& anchorA, const Vector3& anchorB,
                                float restLength, float stiffness = 100.0f, float damping = 5.0f) {
        collision::SpringConstraint constraint(a.getEntity(), b.getEntity(), anchorA, anchorB, restLength, stiffness, damping);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief İki cismin kütle merkezleri arasına varsayılan serbest mesafeli yay bağlar.
     * @param a Birinci cisim.
     * @param b İkinci cisim.
     * @param restLength Yayın gerilmesiz doğal serbest uzunluğu (metre).
     * @param stiffness Yay sertlik katsayısı (k, N/m).
     * @param damping Sönümleme katsayısı (c, N*s/m).
     */
    void createSpringConstraint(Body& a, Body& b,
                                float restLength, float stiffness = 100.0f, float damping = 5.0f) {
        createSpringConstraint(a, b, Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), restLength, stiffness, damping);
    }

    /**
     * @brief Yay kısıtına doğal frekans (Hz) ve sönüm oranı (zeta) parametreleri atar.
     * @param body Yay bileşenine sahip cisim.
     * @param frequencyHz Titreşim frekansı (Hz).
     * @param dampingRatio Sönümleme oranı (1.0: kritik sönüm).
     * @param effectiveMass Etkin indirgenmiş kütle (kg).
     */
    void setSpringFrequencyAndDamping(Body& body, float frequencyHz, float dampingRatio, float effectiveMass = 1.0f) {
        if (mRegistry.hasComponent<collision::SpringConstraint>(body.getEntity())) {
            auto& sc = mRegistry.getComponent<collision::SpringConstraint>(body.getEntity());
            sc.setFrequencyAndDamping(frequencyHz, dampingRatio, effectiveMass);
        }
    }

    /**
     * @brief İki cisim arasına koni-burulma (Cone-Twist / Ragdoll) eklemi bağlar.
     * @param a Birinci cisim (örneğin omuz/gövde).
     * @param b İkinci cisim (örneğin kol).
     * @param anchorA A cismi yerel ankor noktası.
     * @param anchorB B cismi yerel ankor noktası.
     * @param twistA A cismi yerel burulma ekseni.
     * @param twistB B cismi yerel burulma ekseni.
     * @param swingLimit Maksimum koni salınım yarı-açısı (radyan).
     * @param lowerTwist Minimum burulma açısı (radyan).
     * @param upperTwist Maksimum burulma açısı (radyan).
     */
    void createConeTwistConstraint(Body& a, Body& b,
                                  const Vector3& anchorA, const Vector3& anchorB,
                                  const Vector3& twistA, const Vector3& twistB,
                                  float swingLimit = 0.785398f,
                                  float lowerTwist = -0.523599f,
                                  float upperTwist = 0.523599f) {
        Quaternion relOrientation = Quaternion::identity();
        if (mRegistry.hasComponent<Pose>(a.getEntity()) && mRegistry.hasComponent<Pose>(b.getEntity())) {
            const auto& poseA = mRegistry.getComponent<Pose>(a.getEntity());
            const auto& poseB = mRegistry.getComponent<Pose>(b.getEntity());
            relOrientation = poseA.orientation.getInverse() * poseB.orientation;
        }
        collision::ConeTwistConstraint constraint(a.getEntity(), b.getEntity(),
                                                  anchorA, anchorB, twistA, twistB,
                                                  relOrientation, swingLimit, lowerTwist, upperTwist);
        mRegistry.addComponent(a.getEntity(), constraint);
    }

    /**
     * @brief Koni-burulma (Cone-Twist) eklemine koni salınım ve burulma limitleri atar.
     * @param body Eklem bileşenine sahip cisim.
     * @param swingLimit Maksimum salınım koni yarı-açısı (radyan).
     * @param lowerTwist Minimum eksenel burulma açısı (radyan).
     * @param upperTwist Maksimum eksenel burulma açısı (radyan).
     */
    void setConeTwistLimits(Body& body, float swingLimit, float lowerTwist, float upperTwist) {
        if (mRegistry.hasComponent<collision::ConeTwistConstraint>(body.getEntity())) {
            auto& ctc = mRegistry.getComponent<collision::ConeTwistConstraint>(body.getEntity());
            ctc.setLimits(swingLimit, lowerTwist, upperTwist);
        }
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
    void setFixedTimeStep(float timeStep) {
        mFixedTimeStep = timeStep;
        mSettings.fixedTimeStep = timeStep;
    }
    [[nodiscard]] float getFixedTimeStep() const { return mFixedTimeStep; }

    /**
     * @brief Adaların ve cisimlerin uykuya geçiş eşiklerini ayarlar.
     * @param linearVelThreshold Uykuya geçiş için maksimum çizgisel hız (m/s).
     * @param angularVelThreshold Uykuya geçiş için maksimum açısal hız (rad/s).
     * @param timeToSleep Bu hızların altında kalınması gereken süre (saniye).
     */
    void setSleepThresholds(float linearVelThreshold, float angularVelThreshold, float timeToSleep = 0.5f) {
        mSettings.sleepLinearThreshold = linearVelThreshold;
        mSettings.sleepAngularThreshold = angularVelThreshold;
        mSettings.timeToSleep = timeToSleep;
        mIslandSystem.setSleepThresholds(linearVelThreshold, angularVelThreshold, timeToSleep);
    }

    /**
     * @brief Veritabanına (Registry) doğrudan erişim sağlar (İleri düzey kullanım).
     */
    [[nodiscard]] Core::Registry& getRegistry() { return mRegistry; }
    [[nodiscard]] const Core::Registry& getRegistry() const { return mRegistry; }

    /**
     * @brief Aktif temas manifoldlarına doğrudan salt okunur erişim sağlar.
     */
    [[nodiscard]] const auto& getManifolds() const { return mCollisionSystem.getManifolds(); }

    /**
     * @brief Uzaysal bölümleme sistemine doğrudan salt okunur erişim sağlar.
     */
    [[nodiscard]] const systems::SpatialPartitioning& getSpatialPartitioning() const { return mSpatialPartitioning; }

    /**
     * @brief Çarpışma ve tetikleyici olaylarını dinleyecek kullanıcı arayüzünü bağlar.
     */
    void setCollisionListener(CollisionListener* listener) {
        mCollisionSystem.setListener(listener);
    }

    /**
     * @brief Dünyanın o anki tam durumunu kopyalar (Snapshot).
     * @details Zamanı geri sarma veya oyun kaydetme sistemleri için kullanılır.
     * @return Kaydedilen dünya durumu.
     */
    [[nodiscard]] Core::Snapshot saveSnapshot() const {
        Core::Snapshot snap = mRegistry.save();
        snap.accumulator = mAccumulator;
        snap.manifolds = mCollisionSystem.getManifolds();
        return snap;
    }

    /**
     * @brief Dünyayı daha önce kaydedilmiş bir duruma geri döndürür (B2: Deterministik geri yükleme).
     * @param snap Yüklenecek durum kaydı.
     */
    void restoreSnapshot(const Core::Snapshot& snap) {
        mRegistry.load(snap);
        mAccumulator = snap.accumulator;
        mCollisionSystem.getManifolds() = snap.manifolds;
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