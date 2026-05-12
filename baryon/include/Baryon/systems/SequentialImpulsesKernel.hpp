#pragma once

#include "Baryon/Core/Registry.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include <vector>
#include <unordered_map>

/**
 * @file SequentialImpulsesKernel.hpp
 * @brief Ardışık İtme (Sequential Impulses) kısıtlama çözücüsü.
 * @details Çarpışmaları ve eklem (joint) bağlantılarını çözmek için kullanılan ana algoritmadır. 
 *          Nesnelerin birbirini itmesini, sürtünmeyi ve birbirine bağlı kalmasını 
 *          sağlayan itme kuvvetlerini (impulse) iteratif olarak hesaplar.
 */
namespace Baryon::systems {

/**
 * @class SequentialImpulsesKernel
 * @brief Fiziksel kısıtlamaları çözen ve simülasyonun kararlılığını sağlayan çekirdek sınıf.
 * @details Hız entegrasyonu ile pozisyon güncellemesi arasında çalışarak, nesnelerin 
 *          birbirinin içinden geçmesini engeller ve gerçekçi tepkiler üretir.
 */
class SequentialImpulsesKernel {
private:
    Core::Registry& mRegistry; ///< ECS kayıt defteri referansı.

public:
    /** 
     * @brief Çözücü çekirdeğini başlatır. 
     * @param registry ECS kayıt defteri. 
     */
    explicit SequentialImpulsesKernel(Core::Registry& registry);

    /**
     * @brief Bir nesne grubu (ada) içindeki tüm çarpışma ve bağlantıları çözer.
     * @details 
     * 1. Hız İterasyonları: Nesnelerin hızlarını, birbirlerini itecek ve sürtünme 
     *    yaratacak şekilde günceller.
     * 2. Eklem Çözümü: Mesafe ve menteşe gibi eklemlerin gerginliğini ayarlar.
     * 3. Pozisyon İterasyonları: Hızdan bağımsız olarak, nesnelerin iç içe geçmiş 
     *    kısımlarını doğrudan birbirinden ayırır.
     * 
     * @param island İşlem görecek nesne grubu (ada).
     * @param manifolds Tüm aktif temas verileri.
     * @param deltaTime Geçen zaman.
     * @param velocityIterations Hız hassasiyeti (iterasyon sayısı).
     * @param positionIterations Konum düzeltme hassasiyeti (varsayılan: 3).
     */
    void execute(const std::vector<ecs::Entity>& island,
                  const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds,
                  float deltaTime, int velocityIterations, int positionIterations = 3);

    /** @brief Belirli bir nesne grubundaki çarpışmaları çözer. */
    void solveIsland(const std::vector<ecs::Entity>& island, 
                      const std::unordered_map<uint64_t, collision::ContactManifold>& manifolds, 
                      float deltaTime);

    /** @brief Belirli bir nesne grubundaki eklem (joint) kısıtlamalarını çözer. */
    void solveConstraintsForIsland(const std::vector<ecs::Entity>& island, float deltaTime);

private:
    /**
     * @brief Tek bir temas noktası için itme kuvvetlerini (itme ve sürtünme) hesaplar.
     * @details Nesnelerin kütlelerini, esnekliklerini ve çarpışma açılarını 
     *          hesaba katarak hızlarını anlık olarak değiştirir.
     */
    void resolveCollision(collision::ContactManifold& manifold, float deltaTime);

    /**
     * @brief Pozisyon Düzeltme (Anti-Titreme).
     * @details Hızlardan bağımsız olarak, cisimlerin birbirine giren kısımlarını 
     *          fiziksel olarak birbirinden uzaklaştırır. Bu işlem, nesnelerin 
     *          titremesini (jitter) ve üst üste binmesini önler.
     */
    void solvePositionConstraints(collision::ContactManifold& manifold);

    /** @brief İki nesne arasındaki sabit mesafe kısıtlamasını çözer. */
    void resolveDistanceConstraint(collision::DistanceConstraint& constraint, float deltaTime);

    /** @brief İki nesne arasındaki menteşe (dönme) kısıtlamasını çözer. */
    void resolveRevoluteConstraint(collision::RevoluteConstraint& constraint, float deltaTime);
};

} // namespace Baryon::systems