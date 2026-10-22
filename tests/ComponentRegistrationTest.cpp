#include "TestCommon.hpp"
#include <Baryon/Core/Registry.hpp>
#include <Baryon/Core/Snapshot.hpp>
#include <Baryon/memory/MemoryManager.hpp>
#include <iostream>

using namespace Baryon;
using namespace Baryon::Core;

// Test için özel harici bileşen
struct CustomGameComponent {
    int score{100};
    float mana{45.5f};
    bool isBuffed{true};

    bool operator==(const CustomGameComponent& other) const {
        return score == other.score && mana == other.mana && isBuffed == other.isBuffed;
    }
};

int main() {
    std::cout << "=== V7: Tek Noktadan Yeni Bilesen Ekleme Testi ===" << std::endl;

    // 1. static_assert: Standart bilesenler kayitli mi?
    static_assert(Registry::isRegistered<Pose>, "Pose bileseni kayitli olmali");
    static_assert(Registry::isRegistered<Motion>, "Motion bileseni kayitli olmali");
    static_assert(Registry::isRegistered<MassProps>, "MassProps bileseni kayitli olmali");
    static_assert(Registry::isRegistered<BodyState>, "BodyState bileseni kayitli olmali");
    static_assert(Registry::isRegistered<collision::DistanceConstraint>, "DistanceConstraint bileseni kayitli olmali");
    static_assert(Registry::isRegistered<collision::RevoluteConstraint>, "RevoluteConstraint bileseni kayitli olmali");
    static_assert(Registry::isRegistered<collision::FixedConstraint>, "FixedConstraint bileseni kayitli olmali");
    static_assert(Registry::isRegistered<collision::BallSocketConstraint>, "BallSocketConstraint bileseni kayitli olmali");
    static_assert(Registry::isRegistered<collision::PrismaticConstraint>, "PrismaticConstraint bileseni kayitli olmali");

    // 2. Custom bileşenin tek noktadan genişletilmiş tip listesi ile kaydı:
    using ExtendedComponentTypes = std::tuple<
        Pose,
        Motion,
        MassProps,
        Material,
        BodyState,
        Linkage,
        ecs::ColliderData,
        collision::DistanceConstraint,
        collision::RevoluteConstraint,
        collision::FixedConstraint,
        collision::BallSocketConstraint,
        collision::PrismaticConstraint,
        CustomGameComponent
    >;

    using ExtendedRegistry = BasicRegistry<ExtendedComponentTypes>;
    using ExtendedSnapshot = BasicSnapshot<ExtendedComponentTypes>;

    static_assert(ExtendedRegistry::isRegistered<CustomGameComponent>,
                  "CustomGameComponent genisletilmis listede kayitli olmali");
    static_assert(!Registry::isRegistered<CustomGameComponent>,
                  "CustomGameComponent standart listede henuz kayitli olmamali");

    memory::MemoryManager mm;
    ExtendedRegistry reg(mm);

    ecs::Entity e1 = reg.createEntity().value();
    ecs::Entity e2 = reg.createEntity().value();

    reg.addComponent<Pose>(e1, Pose{Vector3(1, 2, 3), Quaternion::identity()});
    reg.addComponent<CustomGameComponent>(e1, CustomGameComponent{500, 80.0f, true});

    reg.addComponent<Pose>(e2, Pose{Vector3(10, 20, 30), Quaternion::identity()});
    reg.addComponent<CustomGameComponent>(e2, CustomGameComponent{250, 10.0f, false});

    CHECK_MSG(reg.hasComponent<CustomGameComponent>(e1), "e1 CustomGameComponent'e sahip olmali");
    CHECK_MSG(reg.hasComponent<CustomGameComponent>(e2), "e2 CustomGameComponent'e sahip olmali");
    CHECK_MSG(reg.getComponent<CustomGameComponent>(e1).score == 500, "e1 score 500 olmali");

    // 3. Snapshot Save Turu (Tek noktadan bilesen listesi uzerinden otomatik kaydedilmeli)
    ExtendedSnapshot snap = reg.save();

    // 4. Yok etme yolu (destroyEntity otomatik olarak CustomGameComponent'i temizlemeli)
    reg.destroyEntity(e1);
    CHECK_MSG(!reg.isAlive(e1), "e1 yok edilmis olmali");
    CHECK_MSG(!reg.hasComponent<CustomGameComponent>(e1), "e1 silindikten sonra CustomGameComponent kalmamali");
    CHECK_MSG(!reg.hasComponent<Pose>(e1), "e1 silindikten sonra Pose kalmamali");

    // 5. Snapshot Load Turu (Geri yuklendiginde CustomGameComponent aynen geri gelmeli)
    reg.load(snap);
    CHECK_MSG(reg.isAlive(e1), "e1 snapshot'tan geri yuklenmeli");
    CHECK_MSG(reg.hasComponent<CustomGameComponent>(e1), "e1 yuklendikten sonra CustomGameComponent'e sahip olmali");
    CHECK_MSG(reg.getComponent<CustomGameComponent>(e1).score == 500, "e1 yuklendikten sonra score 500 olmali");
    CHECK_MSG(reg.getComponent<CustomGameComponent>(e1).mana == 80.0f, "e1 yuklendikten sonra mana 80.0f olmali");
    CHECK_MSG(reg.getComponent<CustomGameComponent>(e1).isBuffed == true, "e1 yuklendikten sonra isBuffed true olmali");

    CHECK_MSG(reg.hasComponent<CustomGameComponent>(e2), "e2 CustomGameComponent'e sahip olmali");
    CHECK_MSG(reg.getComponent<CustomGameComponent>(e2).score == 250, "e2 score 250 olmali");

    // 6. 16-bit Entity Generation and Slot Recycling Test
    {
        std::cout << "\n--- 16-bit Entity Generation and Recycling Test ---" << std::endl;
        ecs::Entity originalEntity = reg.createEntity().value();
        reg.addComponent<Pose>(originalEntity, Pose(Vector3(0, 1, 0), Quaternion::identity()));

        CHECK_MSG(reg.isAlive(originalEntity), "Created entity must be alive!");
        CHECK_MSG(reg.hasComponent<Pose>(originalEntity), "Created entity must have Pose component!");

        reg.destroyEntity(originalEntity);
        CHECK_MSG(!reg.isAlive(originalEntity), "Destroyed entity must not be alive!");
        CHECK_MSG(!reg.hasComponent<Pose>(originalEntity), "Destroyed entity must not report having component!");

        static_assert(sizeof(decltype(ecs::EntityManager::getGeneration(originalEntity))) == 4,
                      "Entity generation must be 32-bit (uint32_t)");


        // Aynı slotu 300 kez (8-bit sınırının ötesinde) geri dönüştür
        ecs::Entity latestEntity = originalEntity;
        for (int i = 0; i < 300; ++i) {
            latestEntity = reg.createEntity().value();
            reg.destroyEntity(latestEntity);
        }

        // Şimdi aynı slota yeni bir varlık oluştur
        ecs::Entity newEntityInSameSlot = reg.createEntity().value();
        reg.addComponent<Pose>(newEntityInSameSlot, Pose(Vector3(5, 5, 5), Quaternion::identity()));

        bool oldHandleAlive = reg.isAlive(originalEntity);
        bool oldHandleHasComp = reg.hasComponent<Pose>(originalEntity);
        std::cout << "  -> 300 recycling cycles: oldHandle alive: " << (oldHandleAlive ? "true" : "false")
                  << " (Expected: false), hasComponent: " << (oldHandleHasComp ? "true" : "false") << " (Expected: false)" << std::endl;

        CHECK_MSG(!oldHandleAlive, "Old entity handle must remain dead even after 300 recycles of the same slot!");
        CHECK_MSG(!oldHandleHasComp, "Old entity handle must NOT report having components belonging to new entity in same slot!");
        CHECK_MSG(reg.isAlive(newEntityInSameSlot), "New entity in same slot must be alive!");
    }

    return reportAndExit();
}
