#pragma once

#include <memory_resource>

/**
 * @file MemoryManager.hpp
 * @brief Fizik motorunun bellek yönetimini sağlayan merkezi sistem.
 * @details Performans kaybını önlemek ve işlemciyi yormamak adına iki farklı strateji kullanır:
 * 
 *          1. **Kalıcı Veri Havuzu (Pool)**: ECS tabloları ve oyun boyunca silinmeyen 
 *             veriler için kullanılır. Bellek parçalanmasını önleyerek hızı sabit tutar.
 * 
 *          2. **Geçici Görev Belleği (Monotonic)**: Sadece o anki fizik adımında lazım olan 
 *             geçici veriler içindir. Her kare sonunda anında boşaltılır, temizleme maliyeti yoktur.
 */
namespace Baryon::memory {

/**
 * @class MemoryManager
 * @brief Bellek tahsis süreçlerini optimize eden ve sistem kaynaklarını yöneten sınıf.
 * @details Kopyalanamaz ve taşınamaz bir yapıdadır. Tüm fizik motorunun bellek 
 *          ihtiyaçlarını karşılayan ana depoyu temsil eder.
 */
class MemoryManager {
private:
    std::pmr::memory_resource* mUpstream; ///< Ana sistem bellek kaynağı.

    /// @brief Uzun ömürlü veriler için kullanılan bellek havuzu.
    std::pmr::unsynchronized_pool_resource mPoolResource;

    /// @brief Sadece tek bir kare boyunca yaşayan geçici veriler için bellek alanı.
    std::pmr::monotonic_buffer_resource mSingleFrameResource;

public:
    /**
     * @brief Bellek yöneticisini 1 MB'lık geçici tampon alanla başlatır.
     */
    MemoryManager() 
        : mUpstream(std::pmr::new_delete_resource()),
          mPoolResource(mUpstream),
          mSingleFrameResource(1024 * 1024, mUpstream) {}

    // Kopyalama ve taşıma işlemlerine izin verilmez (Güvenlik için).
    MemoryManager(const MemoryManager&) = delete;
    MemoryManager& operator=(const MemoryManager&) = delete;
    MemoryManager(MemoryManager&&) = delete;
    MemoryManager& operator=(MemoryManager&&) = delete;

    /**
     * @brief Uzun vadeli veriler (ECS tabloları vb.) için bellek kaynağı sağlar.
     * @return Kalıcı bellek havuzu işaretçisi.
     */
    [[nodiscard]] std::pmr::memory_resource* getPoolResource() {
        return &mPoolResource;
    }

    /**
     * @brief Geçici hesaplamalar (çarpışma testleri vb.) için hızlı bellek kaynağı sağlar.
     * @return Geçici bellek kaynağı işaretçisi.
     */
    [[nodiscard]] std::pmr::memory_resource* getSingleFrameResource() {
        return &mSingleFrameResource;
    }

    /**
     * @brief Geçici belleği tamamen sıfırlar. 
     * @details Her fizik adımının sonunda bir kez çağrılmalıdır. Tüm geçici verileri 
     *          tek hamlede siler, böylece bireysel veri silme maliyetinden kurtarır.
     */
    void resetSingleFrameAllocator() {
        mSingleFrameResource.release();
    }
};

} // namespace Baryon::memory