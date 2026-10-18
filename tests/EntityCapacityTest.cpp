#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <unordered_set>
#include <vector>
#include <random>

using namespace Baryon;
using namespace Baryon::collision;

int main() {
    std::cout << "=== G1: Entity Capacity, Limits and Recycling Test ===" << std::endl;

    // 1. 70,000 cisim yarat ve yinelenen id kontrolü
    {
        std::cout << "\n--- Test 1: 70,000 Bodies Creation and ID Uniqueness ---" << std::endl;
        Simulator sim;
        std::unordered_set<uint64_t> seenIds;
        std::vector<Body> bodies;
        bodies.reserve(70000);

        int duplicateCount = 0;
        for (int i = 0; i < 70000; ++i) {
            Vector3 expectedPos(static_cast<float>(i), static_cast<float>(i) * 0.5f, static_cast<float>(i) * -0.25f);
            Body b = sim.createBody(Pose(expectedPos), CollisionShape{SphereShape(0.5f)});
            uint64_t id = b.getEntity().id;
            if (!seenIds.insert(id).second) {
                duplicateCount++;
            }
            bodies.push_back(b);
        }

        std::cout << "Created 70000 bodies: duplicate ids=" << duplicateCount << std::endl;
        CHECK_MSG(duplicateCount == 0, "Created 70000 bodies: duplicate ids=0 must be satisfied!");

        // 2. Rastgele 1000 cismin pozisyon dogrulugu
        std::cout << "\n--- Test 2: Sample 1000 Bodies Position Verification ---" << std::endl;
        std::mt19937 rng(42);
        std::uniform_int_distribution<size_t> distIndex(0, 69999);
        int wrongPositionCount = 0;

        for (int sample = 0; sample < 1000; ++sample) {
            size_t idx = distIndex(rng);
            Vector3 expectedPos(static_cast<float>(idx), static_cast<float>(idx) * 0.5f, static_cast<float>(idx) * -0.25f);
            auto posRes = bodies[idx].getPosition();
            if (!posRes.has_value()) {
                wrongPositionCount++;
                continue;
            }
            Vector3 actualPos = posRes.value();
            if ((actualPos - expectedPos).lengthSquare() > 1e-4f) {
                wrongPositionCount++;
            }
        }

        std::cout << "Sampled 1000 bodies: wrong positions=" << wrongPositionCount << std::endl;
        CHECK_MSG(wrongPositionCount == 0, "All 1000 sampled bodies must have correct position data!");
    }

    // 3. Aynı slotu 70,000 kez geri dönüştür (ve 65,536 kritik sarma eşiğini sına)
    {
        std::cout << "\n--- Test 3: 70,000 Slot Recycling Generations Test ---" << std::endl;
        memory::MemoryManager mm;
        Core::Registry reg(mm);

        ecs::Entity originalEntity = reg.createEntity().value();
        reg.addComponent<Pose>(originalEntity, Pose(Vector3(1, 2, 3), Quaternion::identity()));
        CHECK(reg.isAlive(originalEntity));

        reg.destroyEntity(originalEntity);
        CHECK(!reg.isAlive(originalEntity));

        // Slotu 70,000 kez geri dönüştür (16-bit 65,536 sarma eşiğini aşarak)
        ecs::Entity currentEntity = originalEntity;
        for (int i = 0; i < 70000; ++i) {
            currentEntity = reg.createEntity().value();
            reg.destroyEntity(currentEntity);
        }

        // Şimdi aynı slota yeni bir varlık oluştur
        ecs::Entity newEntity = reg.createEntity().value();

        reg.addComponent<Pose>(newEntity, Pose(Vector3(99, 99, 99), Quaternion::identity()));

        bool oldAlive = reg.isAlive(originalEntity);
        bool oldHasPose = reg.hasComponent<Pose>(originalEntity);

        std::cout << "After 70,000 recycles: old entity isAlive=" << (oldAlive ? "true" : "false")
                  << ", hasComponent=" << (oldHasPose ? "true" : "false") << std::endl;

        CHECK_MSG(!oldAlive, "Original entity handle must remain dead after 70,000 recycles!");
        CHECK_MSG(!oldHasPose, "Original entity handle must not have components of recycled slot!");
        CHECK_MSG(reg.isAlive(newEntity), "New entity in same slot must be alive!");
    }

    // 4. Kapasite sınırında hata kodu (PhysicsError::CapacityExceeded)
    {
        std::cout << "\n--- Test 4: Capacity Limit Error Reporting ---" << std::endl;
        memory::MemoryManager mm;
        Core::Registry reg(mm);

        reg.setMaxCapacity(3);
        CHECK(reg.getMaxCapacity() == 3);

        auto e1 = reg.createEntity();
        CHECK(e1.has_value());
        auto e2 = reg.createEntity();
        CHECK(e2.has_value());
        auto e3 = reg.createEntity();
        CHECK(e3.has_value());

        // Kapasite doldu, 4. varlık oluşturma başarısız olmalı ve CapacityExceeded dönmeli
        auto e4 = reg.createEntity();
        CHECK_MSG(!e4.has_value(), "createEntity beyond capacity must fail!");
        CHECK_MSG(e4.error() == PhysicsError::CapacityExceeded, "Error must be PhysicsError::CapacityExceeded!");

        // Bir varlığı serbest bırakınca geri dönüşüm üzerinden tekrar oluşturulabilmeli
        reg.destroyEntity(e2.value());
        auto eRecycled = reg.createEntity();
        CHECK_MSG(eRecycled.has_value(), "createEntity after recycling when at capacity must succeed!");

        // Tekrar doldurulunca yine hata dönmeli
        auto eOverflowAgain = reg.createEntity();
        CHECK_MSG(!eOverflowAgain.has_value() && eOverflowAgain.error() == PhysicsError::CapacityExceeded,
                  "createEntity must again return CapacityExceeded when capacity full!");
    }

    return reportAndExit();
}
