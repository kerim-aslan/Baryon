#pragma once

#include "../memory/MemoryManager.hpp"
#include <deque>
#include <vector>
#include <cstdint>
#include <cassert>

/**
 * @file EntityManager.hpp
 * @brief Varlık (Entity) yönetim sistemi ve kimlik yapıları.
 * @details Sahnedeki her objeyi temsil eden hafif kimlik (ID) numaralarının 
 *          üretilmesi, silinmesi ve güvenli bir şekilde geri dönüştürülmesi işlemlerini yönetir.
 */
namespace Baryon::ecs {

/**
 * @struct Entity
 * @brief Sahnedeki bir objeyi temsil eden 4 baytlık eşsiz kimlik (ID) yapısı.
 * @details İçerisinde varlığın hafızadaki sırasını (indeks) ve bu sıranın kaçıncı 
 *          kez kullanıldığını (nesil/versiyon) barındırır. Bu sayede silinmiş bir 
 *          varlığın kimliği ile yanlışlıkla işlem yapılması engellenir.
 */
struct Entity {
    uint32_t id; ///< Varlık kimlik numarası (sıra ve versiyon bilgisinin birleşimi).
    constexpr bool operator==(const Entity& other) const = default;
};

/**
 * @class EntityManager
 * @brief Varlıkların yaşam döngüsünü kontrol eden merkez sınıf.
 * @details Silinen varlıkların numaralarını saklayıp yeni varlıklar için tekrar kullanarak 
 *          bellek ve performans tasarrufu (geri dönüşüm) sağlar.
 */
class EntityManager {
private:
    std::pmr::deque<uint32_t> mFreeIndices;   ///< Silinip geri dönüşüme atılmış müsait numaralar kuyruğu.
    std::pmr::vector<uint8_t> mGenerations;   ///< Her bir numaranın kaçıncı versiyonda (nesilde) olduğunu tutan dizi.
    uint32_t mCurrentIndex{0};                ///< Geri dönüşümde numara yoksa verilecek bir sonraki yeni numara.

public:
    /**
     * @brief Varlık yöneticisini bellek havuzu ile başlatır.
     */
    explicit EntityManager(memory::MemoryManager& memoryManager)
        : mFreeIndices(memoryManager.getPoolResource()),
          mGenerations(memoryManager.getPoolResource()) {}

    /**
     * @brief Sahneye yeni ve eşsiz bir varlık kimliği oluşturur.
     * @details Mümkünse eski (silinmiş) bir numarayı geri dönüştürür ve versiyonunu artırarak yeni bir kimlik verir.
     * @return Oluşturulan yeni varlığın kimliği.
     */
    [[nodiscard]] Entity createEntity() {
        uint32_t index;
        if (!mFreeIndices.empty()) {
            index = mFreeIndices.front();
            mFreeIndices.pop_front();
        } else {
            index = mCurrentIndex++;
            mGenerations.push_back(0);
        }
        return Entity{ (static_cast<uint32_t>(mGenerations[index]) << 24) | index };
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
        uint8_t generation = getGeneration(entity);
        return index < mGenerations.size() && mGenerations[index] == generation;
    }

    /**
     * @brief Varlık kimliğinin içinden sadece sıra numarasını (indeksi) ayıklar.
     * @details Motorun iç yapılarındaki dizilere erişmek için kullanılır.
     */
    [[nodiscard]] static constexpr uint32_t getIndex(Entity entity) {
        return entity.id & 0x00FFFFFF; 
    }

    /**
     * @brief Varlık kimliğinin içinden versiyon (nesil) bilgisini ayıklar.
     * @details Varlığın silinip silinmediğini doğrulamak için kullanılır.
     */
    [[nodiscard]] static constexpr uint8_t getGeneration(Entity entity) {
        return static_cast<uint8_t>((entity.id & 0xFF000000) >> 24); 
    }

    /** 
     * @struct State 
     * @brief Zamanı geri sarma veya oyun kaydetme işlemleri için EntityManager'ın o anki durumu. 
     */
    struct State {
        std::vector<uint32_t> freeIndices;
        std::vector<uint8_t> generations;
        uint32_t currentIndex;
    };

    /** 
     * @brief Varlık yöneticisinin mevcut durumunu tamamen kopyalar ve kaydeder (Snapshot). 
     */
    [[nodiscard]] State save() const {
        return State{
            std::vector<uint32_t>(mFreeIndices.begin(), mFreeIndices.end()),
            std::vector<uint8_t>(mGenerations.begin(), mGenerations.end()),
            mCurrentIndex
        };
    }

    /** 
     * @brief Daha önceden kaydedilmiş bir durumu (Snapshot) varlık yöneticisine geri yükler. 
     */
    void load(const State& state) {
        mFreeIndices.assign(state.freeIndices.begin(), state.freeIndices.end());
        mGenerations.assign(state.generations.begin(), state.generations.end());
        mCurrentIndex = state.currentIndex;
    }
};

} // namespace Baryon::ecs