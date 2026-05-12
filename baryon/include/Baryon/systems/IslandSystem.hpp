#pragma once

#include "Baryon/Core/Registry.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include <vector>
#include <unordered_map>

/**
 * @file IslandSystem.hpp
 * @brief Fiziksel Ada (Island) tespiti ve uyku (sleep) yönetim sistemi.
 * @details Birbirine temas eden nesneleri bağımsız gruplara (adalar) ayırarak 
 *          simülasyonu optimize eder. Hareketsiz kalan adaları "uyku" moduna 
 *          alarak gereksiz işlemci (CPU) kullanımını engeller.
 */
namespace Baryon::systems {

/**
 * @class IslandSystem
 * @brief Nesne gruplandırma, çözücü dağıtımı ve enerji tasarrufu sistemi.
 */
class IslandSystem {
private:
    Core::Registry& mRegistry; ///< ECS kayıt defteri referansı.

public:
    /** 
     * @brief Sistemi başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit IslandSystem(Core::Registry& registry);

    /**
     * @brief Nesneleri gruplara ayırır ve uyku durumlarını kontrol eder.
     * @details 
     * 1. Temas halindeki tüm nesneleri analiz ederek bir bağlantı haritası çıkarır.
     * 2. Birbirine değen nesneleri "ada" adı verilen bağımsız gruplara atar.
     * 3. Her adayı kendi içinde fizik çözücüsüne (solver) gönderir.
     * 4. Enerji Tasarrufu: Bir adadaki tüm nesneler durma noktasına gelirse, 
     *    ada tamamen uyutulur ve tekrar hareket edene kadar hesaplanmaz.
     * 
     * @param manifolds Mevcut aktif temas verileri.
     * @param deltaTime Geçen zaman adımı.
     * @return Tespit edilen ve işlenen adaların (nesne gruplarının) listesi.
     */
    std::vector<std::vector<ecs::Entity>> step(
        const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
        float deltaTime);
};

} // namespace Baryon::systems