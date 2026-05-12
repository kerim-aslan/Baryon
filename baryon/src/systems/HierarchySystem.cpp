/**
 * @file HierarchySystem.cpp
 * @brief Nesne hiyerarşisi ve koordinat güncelleme sistemi.
 * @details Sahnedeki ebeveyn-çocuk ilişkilerini (Linkage) analiz eder. 
 *          BFS (Genişlik Öncelikli Arama) kullanarak, bir ebeveynin her zaman 
 *          çocuklarından önce güncellenmesini garanti eder.
 */

#include "Baryon/systems/HierarchySystem.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace Baryon::systems {

HierarchySystem::HierarchySystem(Core::Registry& registry)
    : mRegistry(registry) {}

/**
 * @brief Tüm hiyerarşik bağları ve dünya konumlarını günceller.
 * @details İşlem üç ana aşamadan oluşur:
 * 
 *          1. BAĞLANTI GRAFİĞİ: Hangi ebeveynin hangi çocuklara sahip olduğu haritalanır.
 *          2. SIRALAMA (BFS): Kök nesnelerden başlanarak aşağı doğru bir güncelleme sırası oluşturulur.
 *          3. DÜNYA KONUMU HESAPLAMA: Çocuk nesnelerin konum ve rotasyonları ebeveynlerine göre güncellenir.
 */
void HierarchySystem::step() {
    auto& linkagePool = mRegistry.getComponentPool<Core::Linkage>();
    auto& linkages = linkagePool.getAllData();
    auto& entities = linkagePool.getAllEntities();

    if (entities.empty()) return;

    // AŞAMA 1: Ebeveyn -> Çocuklar haritasını oluştur
    std::unordered_map<uint32_t, std::vector<ecs::Entity>> childrenMap;
    std::vector<ecs::Entity> roots;
    std::vector<ecs::Entity> potentialRoots;

    for (size_t i = 0; i < entities.size(); ++i) {
        auto& entity = entities[i];
        auto& linkage = linkages[i];

        if (linkage.parentId == 0xFFFFFFFF) {
            // Hiçbir yere bağlı olmayan ana nesne (Kök)
            roots.push_back(entity);
        } else {
            childrenMap[linkage.parentId].push_back(entity);
            
            // Eğer ebeveynin kendisi bir yere bağlı değilse, o hiyerarşinin köküdür
            ecs::Entity parentEnt{linkage.parentId};
            if (!mRegistry.hasComponent<Core::Linkage>(parentEnt)) {
                potentialRoots.push_back(parentEnt);
            }
        }
    }

    // Tekrarlanan kök düğümleri temizle ve listeye ekle
    std::sort(potentialRoots.begin(), potentialRoots.end(), [](const ecs::Entity& a, const ecs::Entity& b) {
        return a.id < b.id;
    });
    potentialRoots.erase(std::unique(potentialRoots.begin(), potentialRoots.end(), [](const ecs::Entity& a, const ecs::Entity& b) {
        return a.id == b.id;
    }), potentialRoots.end());
    
    for (auto& pr : potentialRoots) {
        roots.push_back(pr);
    }

    // AŞAMA 2: Güncelleme sırasını belirlemek için kuyruk (Queue) yapısını başlat
    std::queue<ecs::Entity> bfsQueue;
    for (auto& root : roots) {
        bfsQueue.push(root);
    }

    // AŞAMA 3: Ebeveynden çocuğa doğru dünya koordinatlarını hesapla
    while (!bfsQueue.empty()) {
        ecs::Entity current = bfsQueue.front();
        bfsQueue.pop();

        auto it = childrenMap.find(current.id);
        if (it == childrenMap.end()) continue;

        if (!mRegistry.hasComponent<Pose>(current)) continue;
        const auto& parentPose = mRegistry.getComponent<Pose>(current);

        for (auto& child : it->second) {
            if (!mRegistry.hasComponent<Pose>(child) || !mRegistry.hasComponent<Core::Linkage>(child)) continue;

            auto& childPose = mRegistry.getComponent<Pose>(child);
            const auto& childLinkage = mRegistry.getComponent<Core::Linkage>(child);

            // Rotasyon Güncelleme: Ebeveyn Rotasyonu * Yerel Rotasyon
            childPose.orientation = parentPose.orientation * childLinkage.localOrientation;
            
            // Konum Güncelleme: Ebeveyn Konumu + (Ebeveyn Rotasyonu * Yerel Konum)
            childPose.position = parentPose.position + (parentPose.orientation * childLinkage.localPosition);

            // Çocuğun çocuklarını da güncellemek üzere kuyruğa ekle
            bfsQueue.push(child);
        }
    }
}

} // namespace Baryon::systems