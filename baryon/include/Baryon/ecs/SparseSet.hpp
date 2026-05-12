#pragma once

#include "EntityManager.hpp"
#include "../memory/MemoryManager.hpp"
#include <vector>
#include <cassert>


/**
 * @file SparseSet.hpp
 * @brief Yüksek performanslı bileşen (Component) veri deposu.
 * @details ECS mimarisinin bel kemiğidir. Herhangi bir bileşen tipini (örneğin Konum, Hız) 
 *          bellekte ardışık ve boşluksuz (sıkı) bir şekilde tutarak, işlemcinin (CPU) 
 *          binlerce objeyi çok yüksek hızda işlemesini sağlar.
 */
namespace Baryon::ecs {

/**
 * @class SparseSet
 * @brief İstenilen herhangi bir bileşen türünü güvenle ve hızlıca depolayan şablon sınıfı.
 * @tparam T Depolanacak bileşen veri tipi (Örn: Pose, Motion).
 */
template <typename T>
class SparseSet {
private:
    std::pmr::vector<uint32_t> mSparseMap; ///< Varlık kimliğinden (ID), verinin dizideki gerçek sırasına giden yönlendirme haritası.
    std::pmr::vector<Entity> mEntities;     ///< Dizideki verinin hangi varlığa (sahibine) ait olduğunu tutan liste.
    std::pmr::vector<T> mData;              ///< Gerçek bileşen verilerinin aralarında boşluk olmadan yan yana dizildiği ana bellek.

    static constexpr uint32_t MAX_ENTITIES = 10000;       ///< Sistemin desteklediği maksimum varlık sayısı.
    static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF; ///< Varlığın bu bileşene sahip olmadığını belirten geçersiz indeks değeri.

public:
    /**
     * @brief Veri deposunu başlatır ve bellek havuzuna bağlanır.
     * @param memoryManager Bellek tahsisleri için kullanılacak yönetici.
     */
    explicit SparseSet(memory::MemoryManager& memoryManager)
        : mSparseMap(MAX_ENTITIES, INVALID_INDEX, memoryManager.getPoolResource()),
          mEntities(memoryManager.getPoolResource()),
          mData(memoryManager.getPoolResource()) {
        mEntities.reserve(1000);
        mData.reserve(1000);
    }

    /**
     * @brief Bir varlığa yeni bir bileşen ekler.
     * @param entity Bileşenin ekleneceği varlık.
     * @param data Eklenecek bileşen verisi.
     * @pre Varlık zaten bu bileşene sahip olmamalıdır.
     */
    void addComponent(Entity entity, const T& data) {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        assert(entityIndex < MAX_ENTITIES && "Varlik indeksi maksimum siniri asiyor!");
        assert(mSparseMap[entityIndex] == INVALID_INDEX && "Varlik zaten bu bilesene sahip!");
        uint32_t denseIndex = static_cast<uint32_t>(mData.size());
        mData.push_back(data);
        mEntities.push_back(entity);
        mSparseMap[entityIndex] = denseIndex;
    }

    /**
     * @brief Bir varlığın sahip olduğu bileşeni sistemden siler.
     * @details Yüksek performans için silinen elemanın oluşturduğu boşluk, dizinin en 
     *          sonundaki eleman oraya taşınarak kapatılır. Böylece bellek her zaman boşluksuz kalır.
     * @param entity Bileşeni silinecek varlık.
     * @pre Varlık bu bileşene sahip olmalıdır.
     */
    void removeComponent(Entity entity) {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        assert(entityIndex < MAX_ENTITIES && "Varlik indeksi maksimum siniri asiyor!");
        uint32_t indexOfRemoved = mSparseMap[entityIndex];
        assert(indexOfRemoved != INVALID_INDEX && "Varlik bu bilesene sahip degil!");
        uint32_t lastDenseIndex = static_cast<uint32_t>(mData.size() - 1);
        
        // Hızlı silme: Boşluğu sondaki elemanla doldur
        if (indexOfRemoved != lastDenseIndex) {
            Entity lastEntity = mEntities[lastDenseIndex];
            mData[indexOfRemoved] = std::move(mData[lastDenseIndex]);
            mEntities[indexOfRemoved] = lastEntity;
            uint32_t lastEntityIndex = EntityManager::getIndex(lastEntity);
            mSparseMap[lastEntityIndex] = indexOfRemoved;
        }
        mData.pop_back();
        mEntities.pop_back();
        mSparseMap[entityIndex] = INVALID_INDEX;
    }

    /**
     * @brief Varlığın bu bileşene (özelliğe) sahip olup olmadığını kontrol eder.
     * @param entity Kontrol edilecek varlık.
     * @return Bileşene sahipse true, aksi halde false.
     */
    [[nodiscard]] bool hasComponent(Entity entity) const {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        return entityIndex < MAX_ENTITIES && mSparseMap[entityIndex] != INVALID_INDEX;
    }

    /**
     * @brief Varlığın bileşen verisine doğrudan erişim sağlar.
     * @param entity Verisine erişilecek varlık.
     * @return Bileşen verisinin referansı.
     * @pre Varlık mutlaka bu bileşene sahip olmalıdır.
     */
    [[nodiscard]] auto&& getData(this auto&& self, Entity entity) {
        uint32_t entityIndex = EntityManager::getIndex(entity);
        assert(self.hasComponent(entity) && "Varlik bu bilesene sahip degil!");
        uint32_t denseIndex = self.mSparseMap[entityIndex];
        return std::forward<decltype(self)>(self).mData[denseIndex];
    }

    /** @struct State @brief Zamanı geri sarma (Snapshot) işlemleri için durum kopyası. */
    struct State {
        std::vector<uint32_t> sparseMap;
        std::vector<Entity> entities;
        std::vector<T> data;
    };

    /** @brief Deponun mevcut durumunu tamamen kopyalar ve kaydeder. */
    [[nodiscard]] State save() const {
        return State{
            std::vector<uint32_t>(mSparseMap.begin(), mSparseMap.end()),
            std::vector<Entity>(mEntities.begin(), mEntities.end()),
            std::vector<T>(mData.begin(), mData.end())
        };
    }

    /** @brief Kaydedilmiş bir durumu (Snapshot) depoya geri yükler. */
    void load(const State& state) {
        mSparseMap.assign(state.sparseMap.begin(), state.sparseMap.end());
        mEntities.assign(state.entities.begin(), state.entities.end());
        mData.assign(state.data.begin(), state.data.end());
    }

    /** @brief Sistem döngüleri (örneğin tüm hızları güncellemek) için tüm verilere topluca erişim sağlar. */
    [[nodiscard]] std::pmr::vector<T>& getAllData() { return mData; }
    [[nodiscard]] const std::pmr::vector<T>& getAllData() const { return mData; }
    
    /** @brief Bu bileşene sahip olan tüm varlıkların kimlik listesine erişim sağlar. */
    [[nodiscard]] std::pmr::vector<Entity>& getAllEntities() { return mEntities; }
    [[nodiscard]] const std::pmr::vector<Entity>& getAllEntities() const { return mEntities; }
    
    /** @brief Dizideki sırasını (indeks) bilerek veriye doğrudan erişim sağlar. */
    [[nodiscard]] T& getDataByIndex(uint32_t index) {
        assert(index < mData.size());
        return mData[index];
    }
};

} // namespace Baryon::ecs