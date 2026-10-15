#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <atomic>
#include <cstdlib>
#include <new>

using namespace Baryon;
using namespace Baryon::collision;

// ==================== ALLOCATION TRACKER ====================
static std::atomic<size_t> gAllocationCount{0};
static bool gTrackAllocations = false;

void* operator new(size_t size) {
    if (gTrackAllocations) {
        gAllocationCount.fetch_add(1, std::memory_order_relaxed);
    }
    void* ptr = std::malloc(size);
    if (!ptr) throw std::bad_alloc();
    return ptr;
}

void operator delete(void* ptr) noexcept {
    std::free(ptr);
}

void operator delete(void* ptr, size_t) noexcept {
    std::free(ptr);
}

// 1. Broadphase Leaf Update Test (1000 static + 10 dynamic)
void testBroadphaseUpdateLeafCount() {
    std::cout << "\n--- V6.1: Broadphase updateLeaf Call Count Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // 1000 Statik zemin
    for (int i = 0; i < 1000; ++i) {
        float x = static_cast<float>(i % 50) * 4.0f;
        float z = static_cast<float>(i / 50) * 4.0f;
        Body g = sim.createBody(Pose(Vector3(x, 0, z)), CollisionShape{BoxShape(Vector3(1, 0.2f, 1))});
        (void)g.setBodyType(Core::BodyType::Static);
    }

    // 10 Hareketli dinamik kutu
    for (int i = 0; i < 10; ++i) {
        float x = static_cast<float>(i) * 4.0f;
        Body b = sim.createBody(Pose(Vector3(x, 5.0f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)b.setBodyType(Core::BodyType::Dynamic);
        b.setLinearVelocity(Vector3(1.0f, -2.0f, 0.0f));
    }

    // 1 Adım ısınma
    sim.step(1.0f / 60.0f);

    // Sayacı sıfırla ve 1 adım ölç
    sim.getSpatialPartitioning().getTree().resetUpdateLeafCallCount();
    sim.step(1.0f / 60.0f);

    uint64_t calls = sim.getSpatialPartitioning().getTree().getUpdateLeafCallCount();
    std::cout << "updateLeaf call count: " << calls << " (Expected: <= 10 for 10 moving bodies)" << std::endl;

    CHECK_MSG(calls <= 10, ("1000 statik + 10 hareketli cisimle updateLeaf sayisi <= 10 olmali (olculen: " + std::to_string(calls) + ")").c_str());
}

// 2. Steady-State Step Allocation Test
void testStepAllocations() {
    std::cout << "\n--- V6.2: Steady-State Heap Allocation Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // 50 Statik + 50 Dinamik cisim
    for (int i = 0; i < 50; ++i) {
        float x = static_cast<float>(i % 10) * 4.0f;
        float z = static_cast<float>(i / 10) * 4.0f;
        Body g = sim.createBody(Pose(Vector3(x, 0, z)), CollisionShape{BoxShape(Vector3(1.5f, 0.2f, 1.5f))});
        (void)g.setBodyType(Core::BodyType::Static);

        Body b = sim.createBody(Pose(Vector3(x, 1.0f, z)), CollisionShape{SphereShape(0.5f)});
        (void)b.setBodyType(Core::BodyType::Dynamic);
    }

    // Isınma (PMR bellek havuzu ve dahili vektörlerin kapasite tahsisi için 15 adım)
    for (int s = 0; s < 15; ++s) {
        sim.step(1.0f / 60.0f);
    }

    // Tahsis sayımını başlat
    gAllocationCount.store(0);
    gTrackAllocations = true;

    constexpr int kMeasureSteps = 5;
    for (int s = 0; s < kMeasureSteps; ++s) {
        sim.step(1.0f / 60.0f);
    }

    gTrackAllocations = false;
    size_t totalAllocs = gAllocationCount.load();
    double allocsPerStep = static_cast<double>(totalAllocs) / kMeasureSteps;

    std::cout << "Measured allocations over " << kMeasureSteps << " steps: " << totalAllocs 
              << " (" << allocsPerStep << " allocs/step)" << std::endl;
    std::cout << "Baseline steady-state heap allocations documented: " << allocsPerStep << " allocs/step" << std::endl;

    // V6 ölçüm ve raporlama doğrulaması
    CHECK_MSG(allocsPerStep < 500.0, ("Isinma sonrasi adim basina tahsis olculdu (olculen: " + std::to_string(allocsPerStep) + ")").c_str());
}

int main() {
    std::cout << "=== Baryon V6: Broadphase & Memory Allocation Verification ===" << std::endl;
    testBroadphaseUpdateLeafCount();
    testStepAllocations();
    return reportAndExit();
}
