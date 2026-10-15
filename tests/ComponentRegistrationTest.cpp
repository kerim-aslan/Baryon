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

    ecs::Entity e1 = reg.createEntity();
    ecs::Entity e2 = reg.createEntity();

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

    reportAndExit();
}
