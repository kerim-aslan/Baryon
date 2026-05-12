/**
 * @file IslandSystem.cpp
 * @brief Fiziksel Ada (Island) tespiti ve uyku (Sleep) yönetimi.
 * @details Temas noktaları ve eklem bağlantıları üzerinden etkileşim grafiği oluşturur. 
 *          Birbirine bağlı cisimleri gruplandırarak simülasyonun bağımsız bölümlerini ayırır 
 *          ve hareketsiz grupları uyku moduna alarak performansı artırır.
 */

#include "Baryon/systems/IslandSystem.hpp"
#include "Baryon/collision/PhysicsConstants.hpp"
#include <queue>

namespace Baryon::systems {

IslandSystem::IslandSystem(Core::Registry& registry)
    : mRegistry(registry) {}

/**
 * @brief Ada tespiti yapar, uyku sürelerini yönetir ve aktif iş yükünü belirler.
 * @details Süreç üç ana aşamadan oluşur:
 *
 *          1. BAĞLANTI GRAFİĞİ: Temas verileri ve kısıtlamalar (joint) taranarak 
 *             dinamik cisimler arasındaki komşuluk ilişkileri haritalanır.
 *
 *          2. ADA TESPİTİ (BFS): Ziyaret edilmemiş her dinamik cisimden başlanarak 
 *             Genişlik Öncelikli Arama (BFS) yapılır ve birbirine bağlı tüm nesneler 
 *             aynı "ada" içerisine toplanır.
 *
 *          3. UYKU YÖNETİMİ: Bir adadaki tüm cisimlerin hızı belirlenen eşiklerin 
 *             (0.20 m/s ve 0.35 rad/s) altında kalırsa, ada uyku moduna girmeye aday olur. 
 *             0.5 saniye boyunca bu durum korunursa, ada tamamen uyutulur ve hesaplamadan çıkarılır.
 *
 * @param manifolds Mevcut aktif temas verileri.
 * @param deltaTime Zaman adımı.
 * @return Çözücüye gönderilecek olan aktif (uyumayan) adaların listesi.
 */
std::vector<std::vector<ecs::Entity>> IslandSystem::step(
    const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
    float deltaTime) {

    std::vector<std::vector<ecs::Entity>> activeIslands;

    // AŞAMA 1: Komşuluk listesi (bağlantı grafiği) oluştur
    std::unordered_map<uint32_t, std::vector<ecs::Entity>> adjList;

    auto addEdge = [&](ecs::Entity a, ecs::Entity b) {
        if (mRegistry.hasComponent<Core::BodyState>(a) && mRegistry.getComponent<Core::BodyState>(a).type == Core::BodyType::Dynamic)
            adjList[a.id].push_back(b);
        if (mRegistry.hasComponent<Core::BodyState>(b) && mRegistry.getComponent<Core::BodyState>(b).type == Core::BodyType::Dynamic)
            adjList[b.id].push_back(a);
    };

    // Temas halindeki cisimler arasında bağ kur
    for (const auto& [key, manifold] : manifolds) {
        if (!manifold.isTriggerPair) addEdge(manifold.entityA, manifold.entityB);
    }
    
    // Sabit mesafe kısıtlamaları olan cisimler arasında bağ kur
    auto& distConstraints = mRegistry.getComponentPool<collision::DistanceConstraint>().getAllData();
    for (const auto& constraint : distConstraints) addEdge(constraint.entityA, constraint.entityB);
    
    // Menteşe/Dönme kısıtlamaları olan cisimler arasında bağ kur
    auto& revConstraints = mRegistry.getComponentPool<collision::RevoluteConstraint>().getAllData();
    for (const auto& constraint : revConstraints) addEdge(constraint.entityA, constraint.entityB);

    // AŞAMA 2: BFS algoritması ile bağımsız adaları bul
    std::vector<bool> visited(10000, false); 
    auto& entities = mRegistry.getComponentPool<Core::BodyState>().getAllEntities();

    for (ecs::Entity startEntity : entities) {
        if (!mRegistry.hasComponent<Core::BodyState>(startEntity)) continue;
        auto& startState = mRegistry.getComponent<Core::BodyState>(startEntity);
        
        if (startState.type != Core::BodyType::Dynamic || visited[startEntity.id]) continue;

        std::vector<ecs::Entity> island;
        std::queue<ecs::Entity> q;

        q.push(startEntity);
        visited[startEntity.id] = true;

        bool canSleep = true;
        const float kSleepLinearThresholdSq = physics::PhysicsConstants::SleepLinearThresholdSq;
        const float kSleepAngularThresholdSq = physics::PhysicsConstants::SleepAngularThresholdSq;

        while (!q.empty()) {
            ecs::Entity curr = q.front();
            q.pop();
            island.push_back(curr);

            // Hareket Kontrolü: Adadaki herhangi bir cisim hızlıysa ada uyanık kalmalıdır
            if (mRegistry.hasComponent<Core::Motion>(curr)) {
                auto& currMotion = mRegistry.getComponent<Core::Motion>(curr);
                if (currMotion.linearVelocity.lengthSquare() >= kSleepLinearThresholdSq || 
                    currMotion.angularVelocity.lengthSquare() >= kSleepAngularThresholdSq) {
                    canSleep = false;
                }
            }

            for (ecs::Entity neighbor : adjList[curr.id]) {
                if (!visited[neighbor.id]) {
                    visited[neighbor.id] = true;
                    q.push(neighbor);
                }
            }
        }

        // AŞAMA 3: Adanın uyku durumuna karar ver
        if (canSleep) {
            // Ada hareketsiz: Tüm nesnelerin uyku sayaçlarını artır
            bool fellAsleep = false;
            for (ecs::Entity ent : island) {
                auto& state = mRegistry.getComponent<Core::BodyState>(ent);
                if (state.type != Core::BodyType::Dynamic) continue;
                
                state.sleepTimer += deltaTime;
                if (state.sleepTimer >= 0.5f) { // Belirlenen süre sonunda uykuya dal
                    state.isSleeping = true;
                    fellAsleep = true;
                    
                    // Uyuyan cismin enerjisini tamamen sıfırla (Titremeyi önler)
                    if (mRegistry.hasComponent<Core::Motion>(ent)) {
                        auto& motion = mRegistry.getComponent<Core::Motion>(ent);
                        motion.linearVelocity = Vector3(0,0,0);
                        motion.angularVelocity = Vector3(0,0,0);
                    }
                }
            }
            // Eğer henüz hepsi uykuya dalmadıysa aktif iş yükünde tut
            if (!fellAsleep) {
                activeIslands.push_back(std::move(island));
            }
        } else {
            // Ada uyanık veya yeni uyandı: Sayaçları sıfırla ve aktif listeye ekle
            for (ecs::Entity ent : island) {
                auto& state = mRegistry.getComponent<Core::BodyState>(ent);
                if (state.type != Core::BodyType::Dynamic) continue;
                state.sleepTimer = 0.0f;
                state.isSleeping = false; 
            }
            activeIslands.push_back(std::move(island));
        }
    }
    return activeIslands;
}

} // namespace Baryon::systems