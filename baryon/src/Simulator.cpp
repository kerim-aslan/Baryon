/**
 * @file Simulator.cpp
 * @brief Simulator sınıfının uygulama (implementation) dosyası.
 * @details Fizik simülasyonunun ana döngüsünü, varlık yönetimini ve 
 *          dünya sorgularını (raycast) koordine eden merkezi mantığı içerir.
 */

#include "Baryon/Simulator.hpp"
#include "Baryon/collision/NarrowPhase.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include <ranges>

namespace Baryon {

/**
 * @brief Simulator kurucusu. Tüm fizik sistemlerini hiyerarşik sırayla başlatır.
 * @details Sistemler arasındaki bağımlılıklar nedeniyle başlatma sırası önemlidir: 
 *          Bellek yöneticisi ve kayıt defteri (Registry) tüm sistemlerin temelidir.
 */
Simulator::Simulator()
    : mRegistry(mMemoryManager),
      mIntegrator(mRegistry),
      mHierarchySystem(mRegistry),
      mSpatialPartitioning(mRegistry, mMemoryManager),
      mCollisionSystem(mRegistry, mSpatialPartitioning),
      mSolver(mRegistry),
      mIslandSystem(mRegistry)
{
}

/**
 * @brief Fizik dünyasında yeni bir cisim (Body) oluşturur.
 * @details İşlem sırası:
 *          1. ECS üzerinde yeni bir varlık (Entity) yaratılır.
 *          2. Hareket, kütle ve çarpışma verilerini tutan bileşenler atanır.
 *          3. Geniş faz (AABB Ağacı) kaydı yapılır.
 *          4. Cismin geometrisine göre eylemsizlik tensörü (inertia tensor) otomatik hesaplanır.
 *             Hesaplamada şu formül temel alınır:
 *             $$I_{xx} = \frac{m}{12}(h^2 + d^2), \quad I_{yy} = \frac{m}{12}(w^2 + d^2), \quad I_{zz} = \frac{m}{12}(w^2 + h^2)$$
 *
 * @param transform Cismin başlangıç konumu ve yönelimi.
 * @param shape Cismin çarpışma geometrisi.
 * @return Oluşturulan cismi yönetmek için kullanılan Body nesnesi.
 */
Body Simulator::createBody(const Pose& transform, const collision::CollisionShape& shape) {
    ecs::Entity entity = mRegistry.createEntity();

    mRegistry.addComponent(entity, transform);
    mRegistry.addComponent(entity, ecs::ColliderData{shape});
    
    // Temel fizik bileşenlerinin ilk değerlerle atanması
    mRegistry.addComponent(entity, Core::Motion{});
    mRegistry.addComponent(entity, Core::MassProps{});
    mRegistry.addComponent(entity, Core::Material{});
    mRegistry.addComponent(entity, Core::BodyState{});

    // Performans için uzaysal bölümleme ağacına ekleme
    mSpatialPartitioning.addEntityToTree(entity);

    Body rb(entity, mRegistry);

    // Kütle ve eylemsizlik tensörü hesaplamaları (varsayılan: 1kg)
    collision::AABB localAABB = shape.computeLocalAABB();
    Vector3 size = localAABB.maxBounds - localAABB.minBounds;
    (void)rb.setMass(1.0f);
    
    // Kutu yaklaşımı ile eylemsizlik tensörü oluşturma
    Matrix3x3 I = Matrix3x3::identity();
    I[0].x = (1.0f / 12.0f) * 1.0f * (size.y * size.y + size.z * size.z);
    I[1].y = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.z * size.z);
    I[2].z = (1.0f / 12.0f) * 1.0f * (size.x * size.x + size.y * size.y);
    (void)rb.setLocalInertiaTensor(I);

    return rb;
}

/**
 * @brief Bir cismi fizik dünyasından ve bellekten tamamen siler.
 * @param body Silinecek cisim.
 */
void Simulator::destroyBody(Body& body) {
    ecs::Entity entity = body.getEntity();
    if (mRegistry.isAlive(entity)) {
        mSpatialPartitioning.removeEntityFromTree(entity);
        mRegistry.destroyEntity(entity);
    }
}

/**
 * @brief Fizik simülasyonunu ana döngü içerisinde ilerletir.
 * @details Sabit zaman adımı (fixed timestep) ve biriktirici (accumulator) mantığı kullanılır. 
 *          Bu sayede FPS dalgalanmalarından bağımsız, deterministik bir fizik dünyası sağlanır.
 *
 * Fiziksel İşlem Akışı:
 * 1. Hız Entegrasyonu (Kuvvet -> Hız).
 * 2. CCD İşlemleri (Tünelleme önleyici).
 * 3. Geniş Faz (Broad Phase) Güncellemesi.
 * 4. Dar Faz (Narrow Phase) ve Temas Üretimi.
 * 5. Ada Sistemi (Island) ve Uyku Yönetimi.
 * 6. Kısıtlama Çözücü (Constraint Solver).
 * 7. Konum Entegrasyonu (Hız -> Konum).
 * 8. Hiyerarşi Güncellemesi.
 */
void Simulator::step(float deltaTime) {
    if (deltaTime <= 0.0f) return;

    mAccumulator += deltaTime;

    // Ölüm Spirali (Spiral of Death) Koruması: Bilgisayar çok yavaşlarsa 
    // simülasyonun sonsuz döngüye girmesini engellemek için adımları sınırlar.
    if (mAccumulator > 0.1f) {
        mAccumulator = 0.1f;
    }

    while (mAccumulator >= mFixedTimeStep) {
        float dt = mFixedTimeStep;

        // 1. Dinamik kuvvetleri hıza aktar
        mIntegrator.integrateVelocities(dt);

        // 1b. Hızlı hareket eden nesneler için sürekli çarpışma kontrolü
        {
            auto& statePool = mRegistry.getComponentPool<Core::BodyState>();
            auto& states = statePool.getAllData();
            auto& entities = statePool.getAllEntities();
            for (auto&& [entity, state] : std::views::zip(entities, states)) {
                if (state.type != Core::BodyType::Dynamic || !state.useCCD || state.isSleeping) continue;
                mCollisionSystem.applyCCD(entity, dt);
            }
        }

        // 2. Sınır kutularını (AABB) güncelle ve adayları filtrele
        mSpatialPartitioning.step();

        // 3. Kesin çarpışma testlerini yap ve temas noktalarını üret
        mCollisionSystem.step();

        // 4. Etkileşim halindeki nesne gruplarını ayır
        auto islands = mIslandSystem.step(mCollisionSystem.getManifolds(), dt);

        // 5. Her nesne grubu için fiziksel tepkileri hesapla (12 iterasyon kararlılık için idealdir)
        for (const auto& island : islands) {
            mSolver.execute(island, mCollisionSystem.getManifolds(), dt, 12);
        }

        // 6. Güncellenmiş hızları kullanarak konumları değiştir
        mIntegrator.integratePositions(dt);

        // 7. Bağlı nesne hiyerarşilerini (ebeveyn-çocuk) uyarla
        mHierarchySystem.step();

        // 8. O kareye özel kullanılan geçici bellek alanlarını temizle
        mMemoryManager.resetSingleFrameAllocator();

        mAccumulator -= mFixedTimeStep;
    }
}

/**
 * @brief Sahne genelindeki yerçekimi veya benzeri ivme alanını ayarlar.
 */
void Simulator::setAccelerationField(const Vector3& accelerationField) {
    mIntegrator.setAccelerationField(accelerationField);
}

/**
 * @brief Dünya üzerinde ışın fırlatarak en yakın objeyi tespit eder.
 * @details Performans için önce AABB ağacıyla kaba bir eleme yapılır, 
 *          ardından adaylar üzerinde hassas geometrik test uygulanır.
 *
 * @param origin Işın başlangıç noktası.
 * @param direction Işın yönü.
 * @param maxDist Maksimum menzil.
 * @return Çarpışma detaylarını içeren RaycastHit yapısı.
 */
collision::RaycastHit Simulator::raycast(const Vector3& origin, const Vector3& direction, float maxDist) {
    collision::RaycastHit closestHit;
    closestHit.hasHit = false;
    closestHit.distance = maxDist;

    collision::Ray ray(origin, direction, maxDist);
    
    mSpatialPartitioning.getTree().query(ray.computeAABB(), [&](ecs::Entity obstacle) {
        if (!mRegistry.hasComponent<Pose>(obstacle) || !mRegistry.hasComponent<ecs::ColliderData>(obstacle)) return;

        const auto& obsTrans = mRegistry.getComponent<Pose>(obstacle);
        const auto& obsColl = mRegistry.getComponent<ecs::ColliderData>(obstacle);

        collision::RaycastHit hit;
        if (collision::NarrowPhase::raycast(obsColl.shape, obsTrans, ray, hit)) {
            if (hit.distance < closestHit.distance) {
                closestHit = hit;
                closestHit.entity = obstacle;
            }
        }
    });

    return closestHit;
}

/**
 * @brief Belirli bir nesneye ait tüm aktif temas noktalarını listeler.
 */
std::vector<collision::ContactManifold> Simulator::getContactsForEntity(ecs::Entity entity) const {
    std::vector<collision::ContactManifold> result;
    const auto& manifolds = mCollisionSystem.getManifolds();
    for (const auto& [key, manifold] : manifolds) {
        if (manifold.entityA.id == entity.id || manifold.entityB.id == entity.id) {
            result.push_back(manifold);
        }
    }
    return result;
}

} // namespace Baryon