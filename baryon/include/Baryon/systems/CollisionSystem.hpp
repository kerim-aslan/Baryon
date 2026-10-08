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

#include "Baryon/Core/Registry.hpp"
#include "SpatialPartitioning.hpp"
#include "Baryon/collision/ContactManifold.hpp"
#include <unordered_map>

/**
 * @file CollisionSystem.hpp
 * @brief Dar Faz (Narrow Phase) Çarpışma Yönetim Sistemi.
 * @details Geniş fazdan (Broad Phase) gelen potansiyel çarpışma adaylarını kesin sonuç 
 *          için test eder. GJK ve EPA algoritmalarını kullanarak temas noktalarını 
 *          belirler ve hızlı hareket eden objelerin "tünelleme" (içinden geçme) 
 *          sorununu CCD sistemi ile çözer.
 */
namespace Baryon {
class CollisionListener;
}

namespace Baryon::systems {

/**
 * @class CollisionSystem
 * @brief Çarpışma algılama, temas noktası hesaplama ve manifold yönetiminden sorumlu ana sistem.
 */
class CollisionSystem {
private:
    Core::Registry& mRegistry;                   ///< ECS kayıt defteri referansı.
    SpatialPartitioning& mSpatialPartitioning;   ///< Geniş faz (uzaysal bölümleme) sistem referansı.
    CollisionListener* mListener{nullptr};       ///< Çarpışma ve tetikleyici olaylarını dinleyen kullanıcı sınıfı.

    /**
     * @brief Aktif temas bilgilerini (manifold) tutan harita.
     * @details Çarpışan her bir cisim çifti için oluşturulan temas verileri, 
     *          benzersiz bir anahtar ile burada saklanır ve kareler boyunca güncellenir.
     */
    std::unordered_map<uint64_t, collision::ContactManifold> mManifoldMap;

public:
    /**
     * @brief Çarpışma sistemini başlatır.
     * @param registry ECS kayıt defteri.
     * @param sp Uzaysal bölümleme (geniş faz) sistemi.
     */
    CollisionSystem(Core::Registry& registry, SpatialPartitioning& sp);

    /**
     * @brief Her fizik adımında çarpışma testlerini gerçekleştirir.
     * @details 
     * 1. Önceki kareden kalan temas verilerini (manifold) analiz eder.
     * 2. Geniş fazdan gelen aday çiftleri kesin kesişim testine (GJK) sokar.
     * 3. Çarpışma varsa, iç içe geçme miktarını ve normali (EPA) hesaplar.
     * 4. Temas noktalarını günceller ve fizik çözücüsü için hazır hale getirir.
     * 5. Statik modeller (StaticMesh) gibi karmaşık yapıları üçgen bazlı test eder.
     */
    void step();

    /**
     * @brief Hızlı hareket eden cisimler için Sürekli Çarpışma Algılama (CCD) uygular.
     * @details Mermi gibi yüksek hızlı objelerin, ince engellerin (duvar vb.) içinden 
     *          hesaplanmadan geçip gitmesini engellemek için "Etki Anı" (TOI) hesaplaması yapar.
     *          Çarpışma anı bulunduğunda cismi tam temas anında durdurur.
     * 
     * @param entity CCD uygulanacak fiziksel varlık.
     * @param deltaTime Geçen zaman adımı.
     */
    void applyCCD(ecs::Entity entity, float deltaTime);

    /** 
     * @brief Mevcut aktif temas manifold'larına erişim sağlar. 
     */
    [[nodiscard]] const auto& getManifolds() const { return mManifoldMap; }
    [[nodiscard]] auto& getManifolds() { return mManifoldMap; }

    void setListener(CollisionListener* listener) { mListener = listener; }
};

} // namespace Baryon::systems