#pragma once
#include "AABB.hpp"
#include "../ecs/EntityManager.hpp"
#include "../memory/MemoryManager.hpp"
#include <vector>
#include <cstdint>

/**
 * @file DynamicAABBTree.hpp
 * @brief Dinamik AABB Ağacı (Geniş faz uzaysal bölümleme).
 * @details Sahnedeki fiziksel cisimleri bir ikili ağaç (BVH) yapısında tutarak 
 *          çarpışma sorgularının (ışın izleme, AABB kesişimi) çok hızlı yapılmasını sağlar.
 */
namespace Baryon::collision {

constexpr int32_t NULL_NODE = -1; ///< Geçersiz veya boş düğümü temsil eden indeks değeri.

/**
 * @struct TreeNode
 * @brief Uzaysal ağaç düğümü.
 * @details Yaprak (en alt) düğümler doğrudan varlıkları ve onların sınır kutularını tutar. 
 *          İç düğümler ise çocuklarının sınır kutularını kapsayacak şekilde genişler.
 */
struct TreeNode {
    AABB aabb;                              ///< Bu düğümün ve altındakilerin sınır kutusu.
    ecs::Entity entity;                     ///< Sadece yaprak düğümlerde geçerli varlık kimliği.
    int32_t parentIndex{NULL_NODE};         ///< Üst (ebeveyn) düğüm indeksi.
    int32_t leftChildIndex{NULL_NODE};      ///< Sol çocuk düğüm indeksi.
    int32_t rightChildIndex{NULL_NODE};     ///< Sağ çocuk düğüm indeksi.
    int32_t height{-1};                     ///< Ağacın bu noktasındaki yüksekliği (-1 boş demektir).

    /** 
     * @brief Düğümün yaprak olup olmadığını kontrol eder. 
     * @details Sol çocuğu olmayan düğüm otomatik olarak yapraktır (çünkü ağaç tam ikilidir).
     */
    [[nodiscard]] constexpr bool isLeaf() const { return leftChildIndex == NULL_NODE; }
};

/**
 * @class DynamicAABBTree
 * @brief Çarpışma optimizasyonları için Dinamik BVH (Sınırlayıcı Hacim Hiyerarşisi) ağacı.
 */
class DynamicAABBTree {
private:
    std::pmr::vector<TreeNode> mNodes; ///< Bellek havuzu (PMR) kullanan düğüm dizisi.
    int32_t mRootIndex{NULL_NODE};     ///< Ağacın kök düğümünün indeksi.
    int32_t mFreeList{NULL_NODE};      ///< Silinen düğümlerin yeniden kullanımı için boş liste başı.

public:
    /** 
     * @brief Ağacı başlatır.
     * @param memoryManager Bellek tahsisleri için kullanılacak PMR yöneticisi. 
     */
    explicit DynamicAABBTree(memory::MemoryManager& memoryManager);

    /** 
     * @brief Ağaca yeni bir varlık (yaprak) ekler.
     * @details Performans için eklenen kutu "şişmanlatılır" (Fat AABB). Bu sayede ufak hareketlerde 
     *          ağacın sürekli güncellenmesi engellenir. En iyi yeri bulmak için SAH (Yüzey Alanı Keşfi) kullanır.
     * @param entity Eklenecek varlık.
     * @param aabb Varlığın orijinal sınır kutusu.
     * @return Yeni oluşturulan düğümün dizideki indeksi. 
     */
    int32_t insertLeaf(ecs::Entity entity, const AABB& aabb);

    /** 
     * @brief Varlığı (yaprağı) ağaçtan tamamen siler ve bellek bloğunu geri verir. 
     */
    void removeLeaf(int32_t leafNodeIndex);

    /** 
     * @brief Varlığın hareketine göre sınır kutusunu günceller.
     * @details Yeni kutu, eski "şişman" kutunun hala içindeyse ağaç yapısı bozulmaz (erken çıkış).
     *          Dışına çıktıysa düğüm ağaçtan çıkartılıp yeniden doğru yere eklenir.
     * @param nodeIndex Güncellenecek düğümün indeksi.
     * @param newAABB Varlığın yeni hedef sınır kutusu.
     * @return Güncelleme sonrası düğümün yeni (veya aynı) indeksi. 
     */
    int32_t updateLeaf(int32_t nodeIndex, const AABB& newAABB);
    
    /** 
     * @brief Tüm ağacı ve düğüm belleklerini sıfırlar. 
     * @details Sahne değişikliklerinde veya geri yükleme (snapshot restore) işlemlerinde kullanılır.
     */
    void clear() {
        mNodes.clear();
        mRootIndex = NULL_NODE;
        mFreeList = NULL_NODE;
    }

    /**
     * @brief Verilen bir bölge (AABB) ile kesişen tüm varlıkları hızlıca bulur.
     * @details Tüm sahneyi taramak yerine ağaç yapısı kullanıldığı için performansı çok yüksektir.
     * @tparam F Geri arama (callback) fonksiyonu tipi. Parametre olarak 'ecs::Entity' almalıdır.
     * @param targetAABB Arama yapılacak hedef bölge (sınır kutusu).
     * @param callback Kesişen her varlık bulunduğunda çağrılacak işlev.
     */
    template <typename F>
    void query(const AABB& targetAABB, F&& callback) const {
        if (mRootIndex == NULL_NODE) return;
        
        std::pmr::vector<int32_t> stack(mNodes.get_allocator());
        stack.reserve(64); // Çoğu ağaç derinliği için yeterli tahsis
        stack.push_back(mRootIndex);
        
        while (!stack.empty()) {
            int32_t nodeIndex = stack.back();
            stack.pop_back();
            
            const auto& node = mNodes[nodeIndex];
            
            // Eğer düğüm hedef bölge ile kesişmiyorsa o dalı tamamen atla (optimizasyon)
            if (node.aabb.testCollision(targetAABB)) {
                if (node.isLeaf()) {
                    callback(node.entity); // Yaprak ise varlığı döndür
                } else {
                    // İç düğüm ise alt çocukları yığına (stack) ekle ve devam et
                    stack.push_back(node.leftChildIndex);
                    stack.push_back(node.rightChildIndex);
                }
            }
        }
    }

    /** 
     * @brief İlgili düğümün sınır kutusu verisine erişim sağlar. 
     */
    [[nodiscard]] const AABB& getNodeAABB(int32_t nodeIndex) const {
        return mNodes[nodeIndex].aabb;
    }

private:
    int32_t allocateNode();                         ///< Yeni bir düğüm belleği tahsis eder (free list veya yeni).
    void freeNode(int32_t nodeIndex);               ///< Kullanılmayan düğümü geri dönüşüm listesine ekler.
    void insertLeafIntoTree(int32_t leafNodeIndex); ///< Düğümü en az maliyetle ağaca bağlar.
    void removeLeafFromTree(int32_t leafNodeIndex); ///< Düğümün ağaçla bağlantısını koparır.
    void fixUpwardsTree(int32_t nodeIndex);         ///< Değişiklik sonrası köke kadar sınır kutularını yeniden boyutlandırır.
};

} // namespace Baryon::collision