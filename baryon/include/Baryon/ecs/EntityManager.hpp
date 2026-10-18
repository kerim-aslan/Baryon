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

#include "../memory/MemoryManager.hpp"
#include "../Core/PhysicsError.hpp"
#include <deque>
#include <vector>
#include <cstdint>
#include <cassert>
#include <expected>

/**
 * @file EntityManager.hpp
 * @brief Varlık (Entity) yönetim sistemi ve kimlik yapıları.
 * @details Sahnedeki her objeyi temsil eden hafif kimlik (ID) numaralarının 
 *          üretilmesi, silinmesi ve güvenli bir şekilde geri dönüştürülmesi işlemlerini yönetir.
 */
namespace Baryon::ecs {

/**
 * @struct Entity
 * @brief Sahnedeki bir objeyi temsil eden 8 baytlık (64-bit) eşsiz kimlik (ID) yapısı.
 * @details İçerisinde varlığın hafızadaki sırasını (32-bit indeks) ve bu sıranın kaçıncı 
 *          kez kullanıldığını (32-bit nesil/versiyon) barındırır. Bu sayede silinmiş bir 
 *          varlığın kimliği ile yanlışlıkla işlem yapılması engellenir.
 */
struct Entity {
    uint64_t id{0}; ///< Varlık kimlik numarası (32-bit sıra + 32-bit versiyon bilgisi).
    constexpr bool operator==(const Entity& other) const = default;
    constexpr auto operator<=>(const Entity& other) const = default;
    constexpr Entity() = default;
    constexpr explicit Entity(uint64_t inId) : id(inId) {}
};


/**
 * @class EntityManager
 * @brief Varlıkların yaşam döngüsünü kontrol eden merkez sınıf.
 * @details Silinen varlıkların numaralarını saklayıp yeni varlıklar için tekrar kullanarak 
 *          bellek ve performans tasarrufu (geri dönüşüm) sağlar.
 */
class EntityManager {
public:
    static constexpr uint32_t DEFAULT_MAX_CAPACITY = 0xFFFFFFFFULL;

private:
    std::pmr::deque<uint32_t> mFreeIndices;   ///< Silinip geri dönüşüme atılmış müsait numaralar kuyruğu.
    std::pmr::vector<uint32_t> mGenerations;  ///< Her bir numaranın kaçıncı versiyonda (nesilde) olduğunu tutan dizi (32-bit).
    uint32_t mCurrentIndex{0};                ///< Geri dönüşümde numara yoksa verilecek bir sonraki yeni numara.
    uint32_t mMaxCapacity{DEFAULT_MAX_CAPACITY}; ///< İzin verilen maksimum varlık kapasitesi.

public:
    /**
     * @brief Varlık yöneticisini bellek havuzu ile başlatır.
     */
    explicit EntityManager(memory::MemoryManager& memoryManager)
        : mFreeIndices(memoryManager.getPoolResource()),
          mGenerations(memoryManager.getPoolResource()) {}

    /**
     * @brief İzin verilen maksimum varlık kapasitesini ayarlar.
     */
    void setMaxCapacity(uint32_t maxCapacity) { mMaxCapacity = maxCapacity; }

    /**
     * @brief Mevcut maksimum varlık kapasitesini döndürür.
     */
    [[nodiscard]] uint32_t getMaxCapacity() const { return mMaxCapacity; }

    /**
     * @brief Sahneye yeni ve eşsiz bir varlık kimliği oluşturur.
     * @details Mümkünse eski (silinmiş) bir numarayı geri dönüştürür ve versiyonunu artırarak yeni bir kimlik verir.
     * @return Oluşturulan yeni varlığın kimliği veya kapasite aşıldığında PhysicsError::CapacityExceeded.
     */
    [[nodiscard]] std::expected<Entity, PhysicsError> createEntity() {
        if (mFreeIndices.empty() && mCurrentIndex >= mMaxCapacity) {
            return std::unexpected(PhysicsError::CapacityExceeded);
        }
        uint32_t index;
        if (!mFreeIndices.empty()) {
            index = mFreeIndices.front();
            mFreeIndices.pop_front();
        } else {
            index = mCurrentIndex++;
            mGenerations.push_back(0);
        }
        uint64_t fullId = (static_cast<uint64_t>(mGenerations[index]) << 32) |
                          (static_cast<uint64_t>(index) & 0xFFFFFFFFULL);
        return Entity{ fullId };
    }

    /**
     * @brief Varlığı sistemden siler ve kimliğini geri dönüşüme gönderir.
     * @details Silinen numaranın versiyonu (nesli) artırılır, böylece bu varlığa ait 
     *          eski referanslar anında geçersiz (ölü) sayılır.
     * @param entity Yok edilecek varlık kimliği.
     */
    void destroyEntity(Entity entity) {
        assert(isAlive(entity) && "Zaten olu olan bir varligi tekrar silemezsiniz!");
        uint32_t index = getIndex(entity);
        mGenerations[index]++;
        mFreeIndices.push_back(index);
    }

    /**
     * @brief Verilen varlık kimliğinin sahnede hala geçerli (yaşıyor) olup olmadığını kontrol eder.
     * @details Eğer bu kimliğe sahip varlık silinmişse veya numara başkasına devredilmişse false döner.
     * @param entity Kontrol edilecek varlık.
     * @return Varlık yaşıyorsa true, aksi halde false.
     */
    [[nodiscard]] bool isAlive(Entity entity) const {
        uint32_t index = getIndex(entity);
        uint32_t generation = getGeneration(entity);
        return index < mGenerations.size() && mGenerations[index] == generation;
    }

    /**
     * @brief Varlık kimliğinin içinden sadece sıra numarasını (indeksi) ayıklar.
     * @details Motorun iç yapılarındaki dizilere erişmek için kullanılır (32-bit indeks).
     */
    [[nodiscard]] static constexpr uint32_t getIndex(Entity entity) {
        return static_cast<uint32_t>(entity.id & 0xFFFFFFFFULL); 
    }

    /**
     * @brief Varlık kimliğinin içinden versiyon (nesil) bilgisini ayıklar.
     * @details Varlığın silinip silinmediğini doğrulamak için kullanılır (32-bit nesil).
     */
    [[nodiscard]] static constexpr uint32_t getGeneration(Entity entity) {
        return static_cast<uint32_t>((entity.id >> 32) & 0xFFFFFFFFULL); 
    }

    /** 
     * @struct State 
     * @brief Zamanı geri sarma veya oyun kaydetme işlemleri için EntityManager'ın o anki durumu. 
     */
    struct State {
        std::vector<uint32_t> freeIndices;
        std::vector<uint32_t> generations;
        uint32_t currentIndex{0};
        uint32_t maxCapacity{DEFAULT_MAX_CAPACITY};
    };

    /** 
     * @brief Varlık yöneticisinin mevcut durumunu tamamen kopyalar ve kaydeder (Snapshot). 
     */
    [[nodiscard]] State save() const {
        return State{
            std::vector<uint32_t>(mFreeIndices.begin(), mFreeIndices.end()),
            std::vector<uint32_t>(mGenerations.begin(), mGenerations.end()),
            mCurrentIndex,
            mMaxCapacity
        };
    }

    /** 
     * @brief Daha önceden kaydedilmiş bir durumu (Snapshot) varlık yöneticisine geri yükler. 
     */
    void load(const State& state) {
        mFreeIndices.assign(state.freeIndices.begin(), state.freeIndices.end());
        mGenerations.assign(state.generations.begin(), state.generations.end());
        mCurrentIndex = state.currentIndex;
        mMaxCapacity = state.maxCapacity;
    }
};

} // namespace Baryon::ecs

namespace std {
template <> struct hash<Baryon::ecs::Entity> {
    size_t operator()(const Baryon::ecs::Entity& e) const noexcept {
        return hash<uint64_t>{}(e.id);
    }
};
}