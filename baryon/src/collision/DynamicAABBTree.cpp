/**
 * @file DynamicAABBTree.cpp
 * @brief Dinamik AABB Ağacı (BVH) Uygulaması.
 * @details Nesneleri sahne üzerinde gruplandırarak çarpışma sorgularını hızlandırır. 
 *          SAH (Surface Area Heuristic) algoritması ile ağacın dengeli ve verimli 
 *          kalmasını sağlar. Bellek yönetimi için "Free List" (Geri Dönüşüm) yapısı kullanır.
 */

#include "Baryon/collision/DynamicAABBTree.hpp"
#include <cassert>
#include <algorithm>

namespace Baryon::collision {

/**
 * @brief Ağacı başlatır ve bellek havuzuna bağlanır.
 */
DynamicAABBTree::DynamicAABBTree(memory::MemoryManager& memoryManager)
    : mNodes(memoryManager.getPoolResource()) {
    // Performans için başlangıçta 1024 düğümlük yer ayırır.
    mNodes.reserve(1024);
}

// ==================== BELLEK YÖNETİMİ ====================

/**
 * @brief Sistemden yeni bir düğüm (Node) talep eder.
 * @details Eğer silinmiş eski düğümler varsa (Free List), belleği yormamak için 
 *          onları geri dönüştürür; yoksa dizinin sonuna yeni eleman ekler.
 * @return Yeni düğümün indeksi.
 */
int32_t DynamicAABBTree::allocateNode() {
    if (mFreeList != NULL_NODE) {
        int32_t nodeIndex = mFreeList;
        mFreeList = mNodes[nodeIndex].parentIndex; 
        
        mNodes[nodeIndex].parentIndex = NULL_NODE;
        mNodes[nodeIndex].leftChildIndex = NULL_NODE;
        mNodes[nodeIndex].rightChildIndex = NULL_NODE;
        mNodes[nodeIndex].height = 0;
        return nodeIndex;
    }

    mNodes.push_back(TreeNode{});
    return static_cast<int32_t>(mNodes.size() - 1);
}

/**
 * @brief Kullanılmayan bir düğümü geri dönüşüm listesine gönderir.
 * @param nodeIndex Serbest bırakılacak düğüm.
 */
void DynamicAABBTree::freeNode(int32_t nodeIndex) {
    assert(nodeIndex >= 0 && nodeIndex < static_cast<int32_t>(mNodes.size()));
    
    mNodes[nodeIndex].height = -1; // Düğümün boşta olduğunu işaretle
    mNodes[nodeIndex].parentIndex = mFreeList;
    mFreeList = nodeIndex;
}

// ==================== AĞAÇ YAPILANDIRMA ====================

/**
 * @brief Ağaca yeni bir nesne (yaprak düğüm) ekler.
 * @details "Fat AABB" tekniği ile nesne sınırlarını %10 oranında şişirir. 
 *          Bu sayede nesne küçük hareketler yaptığında ağacı sürekli güncellemek 
 *          gerekmez, işlemci yükü azalır.
 * 
 * 
 * 
 * @param entity Varlık kimliği.
 * @param aabb Nesnenin gerçek sınır kutusu.
 * @return Oluşturulan yaprağın indeksi.
 */
int32_t DynamicAABBTree::insertLeaf(ecs::Entity entity, const AABB& aabb) {
    int32_t leafNode = allocateNode();
    
    // Fat AABB: Sınırları her yönde 0.1 birim genişlet
    Vector3 fatExtents(0.1f, 0.1f, 0.1f);
    mNodes[leafNode].aabb.minBounds = aabb.minBounds - fatExtents;
    mNodes[leafNode].aabb.maxBounds = aabb.maxBounds + fatExtents;
    
    mNodes[leafNode].entity = entity;
    mNodes[leafNode].height = 0; 

    insertLeafIntoTree(leafNode);
    return leafNode;
}

/**
 * @brief SAH algoritması ile yaprağı ağaçtaki en verimli konuma yerleştirir.
 * @details Yerleştirme maliyetini (Yüzey Alanı Sezgisi) hesaplayarak ağacın 
 *          derinliğini ve boşluk oranını minimize eder.
 * 
 * 
 */
void DynamicAABBTree::insertLeafIntoTree(int32_t leafNodeIndex) {
    if (mRootIndex == NULL_NODE) {
        mRootIndex = leafNodeIndex;
        mNodes[leafNodeIndex].parentIndex = NULL_NODE;
        return;
    }

    // En uygun yerleşimi (Sibling) bulmak için maliyet analizi yap
    AABB leafAABB = mNodes[leafNodeIndex].aabb;
    int32_t searchIndex = mRootIndex;

    while (!mNodes[searchIndex].isLeaf()) {
        int32_t left = mNodes[searchIndex].leftChildIndex;
        int32_t right = mNodes[searchIndex].rightChildIndex;

        float currentArea = mNodes[searchIndex].aabb.getSurfaceArea();
        AABB combinedAABB = mNodes[searchIndex].aabb;
        combinedAABB.merge(leafAABB);
        float combinedArea = combinedAABB.getSurfaceArea();

        // Buraya eklemenin doğrudan maliyeti
        float costNode = 2.0f * combinedArea;
        // Alt dallara inmenin ek maliyeti
        float inheritanceCost = 2.0f * (combinedArea - currentArea);

        // Sol dal maliyeti
        float costLeft;
        AABB combinedLeft = mNodes[left].aabb;
        combinedLeft.merge(leafAABB);
        costLeft = (mNodes[left].isLeaf()) ? combinedLeft.getSurfaceArea() + inheritanceCost : 
                   (combinedLeft.getSurfaceArea() - mNodes[left].aabb.getSurfaceArea()) + inheritanceCost;

        // Sağ dal maliyeti
        float costRight;
        AABB combinedRight = mNodes[right].aabb;
        combinedRight.merge(leafAABB);
        costRight = (mNodes[right].isLeaf()) ? combinedRight.getSurfaceArea() + inheritanceCost : 
                    (combinedRight.getSurfaceArea() - mNodes[right].aabb.getSurfaceArea()) + inheritanceCost;

        if (costNode < costLeft && costNode < costRight) break;

        searchIndex = (costLeft < costRight) ? left : right;
    }

    int32_t sibling = searchIndex;

    // Yeni bir iç düğüm oluşturup kardeşi ve yaprağı birbirine bağla
    int32_t oldParent = mNodes[sibling].parentIndex;
    int32_t newParent = allocateNode();
    mNodes[newParent].parentIndex = oldParent;
    mNodes[newParent].aabb = mNodes[sibling].aabb;
    mNodes[newParent].aabb.merge(leafAABB);
    mNodes[newParent].height = mNodes[sibling].height + 1;

    if (oldParent != NULL_NODE) {
        if (mNodes[oldParent].leftChildIndex == sibling) mNodes[oldParent].leftChildIndex = newParent;
        else mNodes[oldParent].rightChildIndex = newParent;
    } else {
        mRootIndex = newParent;
    }

    mNodes[newParent].leftChildIndex = sibling;
    mNodes[newParent].rightChildIndex = leafNodeIndex;
    mNodes[sibling].parentIndex = newParent;
    mNodes[leafNodeIndex].parentIndex = newParent;

    // Kök düğüme kadar tüm sınırları ve yükseklikleri güncelle
    fixUpwardsTree(mNodes[leafNodeIndex].parentIndex);
}

/**
 * @brief Belirtilen düğümden yukarı doğru ağacın boyutlarını günceller.
 */
void DynamicAABBTree::fixUpwardsTree(int32_t nodeIndex) {
    while (nodeIndex != NULL_NODE) {
        int32_t left = mNodes[nodeIndex].leftChildIndex;
        int32_t right = mNodes[nodeIndex].rightChildIndex;

        mNodes[nodeIndex].height = 1 + std::max(mNodes[left].height, mNodes[right].height);
        mNodes[nodeIndex].aabb = mNodes[left].aabb;
        mNodes[nodeIndex].aabb.merge(mNodes[right].aabb);

        nodeIndex = mNodes[nodeIndex].parentIndex;
    }
}

// ==================== SİLME VE GÜNCELLEME ====================

/**
 * @brief Bir nesneyi ağaçtan çıkartır.
 * @details Nesne çıkarıldığında, ebeveyni olan iç düğümü de siler ve kardeşini 
 *          üst düğüme doğrudan bağlar.
 * 
 * 
 */
void DynamicAABBTree::removeLeafFromTree(int32_t leafNodeIndex) {
    if (leafNodeIndex == mRootIndex) {
        mRootIndex = NULL_NODE;
        return;
    }

    int32_t parentIndex = mNodes[leafNodeIndex].parentIndex;
    int32_t grandParentIndex = mNodes[parentIndex].parentIndex;
    int32_t siblingIndex = (mNodes[parentIndex].leftChildIndex == leafNodeIndex) 
                           ? mNodes[parentIndex].rightChildIndex : mNodes[parentIndex].leftChildIndex;

    if (grandParentIndex != NULL_NODE) {
        if (mNodes[grandParentIndex].leftChildIndex == parentIndex) mNodes[grandParentIndex].leftChildIndex = siblingIndex;
        else mNodes[grandParentIndex].rightChildIndex = siblingIndex;
        mNodes[siblingIndex].parentIndex = grandParentIndex;
    } else {
        mRootIndex = siblingIndex;
        mNodes[siblingIndex].parentIndex = NULL_NODE;
    }

    freeNode(parentIndex);
    fixUpwardsTree(grandParentIndex);
}

/**
 * @brief Yaprağı ağaçtan ayırır ve belleğini serbest bırakır.
 */
void DynamicAABBTree::removeLeaf(int32_t leafNodeIndex) {
    removeLeafFromTree(leafNodeIndex);
    freeNode(leafNodeIndex);
}

/**
 * @brief Bir nesnenin hareketine göre konumunu ağaçta günceller.
 * @details Eğer nesne hala eski "Fat AABB" (şişirilmiş sınır) içindeyse 
 *          hiçbir işlem yapmaz (erken çıkış). Sınırı aşmışsa ağaçtan silip 
 *          yeni konumuna göre tekrar ekler.
 */
int32_t DynamicAABBTree::updateLeaf(int32_t nodeIndex, const AABB& newAABB) {
    if (mNodes[nodeIndex].aabb.contains(newAABB)) {
        return nodeIndex; 
    }

    ecs::Entity entity = mNodes[nodeIndex].entity;
    removeLeaf(nodeIndex);
    return insertLeaf(entity, newAABB);
}

} // namespace Baryon::collision