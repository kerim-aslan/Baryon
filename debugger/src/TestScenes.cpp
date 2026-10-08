/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/TestScenes.hpp"
#include <Baryon/collision/CollisionShape.hpp>

namespace Baryon::Debugger {

const char* TestScenes::getSceneName(SceneType type) {
    switch (type) {
        case SceneType::TriggerSensors: return "1. Tetikleyici & Hayalet Alan (Trigger Zones)";
        case SceneType::CollisionFiltering: return "2. Katman Filtreleme (Bitmask: Kirmizi vs Mavi)";
        case SceneType::RevoluteHinge: return "3. Mentese Mafsali & Tork (Revolute Joint)";
        case SceneType::RollingResistance: return "4. Yuvarlanma Direnci & Durma Testi";
        case SceneType::RaycastShowcase: return "5. Lazer Isini Kesisimi (Raycast)";
        case SceneType::TunnelingStress: return "6. Yuksek Hizli Tunelleme Testi (CCD)";
        case SceneType::BoxStacking: return "7. Kutu Yigini Kararliligi (10 Kutu)";
        case SceneType::NewtonsCradle: return "8. Esnek Carpisma & Momentum (Newton Besigi)";
        case SceneType::JointPendulum: return "9. Eklem & Mesafe Kisiti Zinciri";
        case SceneType::Sandbox: return "10. Serbest Fizik Test Alani (Sandbox)";
        default: return "Bilinmeyen Sahne";
    }
}

const char* TestScenes::getSceneDescription(SceneType type) {
    switch (type) {
        case SceneType::TriggerSensors:
            return "Tetikleyicileri test eder. Mor renkli alandan kure sifir sekme ve sifir direnc ile gecerken, sagdaki duvara carpan kure geri seker. Tanı panelinde [TETIKLEYICI GIRIS/CIKIS] loglanir.";
        case SceneType::CollisionFiltering:
            return "Bitmask filtrelemeyi test eder. Kirmizi ve Mavi kutular (Z=-2.5) birbirlerinin icinden hayalet gibi gecer. Kirmizi ve Kirmizi kutular (Z=2.5) ise karsi karsiya carpisir!";
        case SceneType::RevoluteHinge:
            return "Dikey Y ekseni etrafinda bagli mentese kapisi. Yercekimi altinda sabit kalarak tork ve acisal eylemsizlik tensoru ile doner.";
        case SceneType::RollingResistance:
            return "Duz zeminde iki kurenin yuvarlanmasi. Yuvarlanma direnci 0 olan kure durmaksizin giderken, direnci olan kure dogal sekilde yavaslayip durur.";
        case SceneType::RaycastShowcase:
            return "Kapsul, Kutu, Kure ve 3D Ucgen Ag (Mesh) uzerine canli 3D lazer isini gonderir. Kesisim noktasi (Sari) ve yuzey dikmesi (Yesil) cizilir.";
        case SceneType::TunnelingStress:
            return "5 cm ince duvara 100 m/s hizla mermiler firlatir. CCD korumasi ile CCD'siz tunelleme riskini kiyaslar.";
        case SceneType::BoxStacking:
            return "10 adet dinamik kutuyu ust uste dizer. Cozucunun kararliligini ve titreme (jitter) onlemesini test eder.";
        case SceneType::NewtonsCradle:
            return "Yanyana kureler serisi. Yuzey sekmesini ve momentum aktarimini test eder.";
        case SceneType::JointPendulum:
            return "Mesafe kisitlari ile bagli zincir sarkaç. Baumgarte stabilizasyonunu dogrular.";
        case SceneType::Sandbox:
            return "Duvarlarla cevrili serbest fizik test arenasi.";
        default: return "";
    }
}

void TestScenes::loadScene(Simulator& sim, SceneType type) {
    // Clear existing bodies safely
    auto entities = sim.getRegistry().getComponentPool<Pose>().getAllEntities();
    for (auto e : entities) {
        if (sim.getRegistry().isAlive(e)) {
            Body b(e, sim.getRegistry());
            sim.destroyBody(b);
        }
    }
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    switch (type) {
        case SceneType::TriggerSensors: setupTriggerSensors(sim); break;
        case SceneType::CollisionFiltering: setupCollisionFiltering(sim); break;
        case SceneType::RevoluteHinge: setupRevoluteHinge(sim); break;
        case SceneType::RollingResistance: setupRollingResistance(sim); break;
        case SceneType::RaycastShowcase: setupRaycastShowcase(sim); break;
        case SceneType::TunnelingStress: setupTunneling(sim); break;
        case SceneType::BoxStacking: setupBoxStacking(sim); break;
        case SceneType::NewtonsCradle: setupNewtonsCradle(sim); break;
        case SceneType::JointPendulum: setupJointPendulum(sim); break;
        case SceneType::Sandbox: setupSandbox(sim); break;
        case SceneType::Count: break;
    }
}

void TestScenes::setupTunneling(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Zero gravity for linear trajectory

    // Static thin wall at X = 5.0 (Thickness = 0.05m)
    Pose wallPose(Vector3(5.0f, 1.0f, 0.0f));
    collision::CollisionShape wallShape{collision::BoxShape(Vector3(0.025f, 3.0f, 5.0f))};
    Body wall = sim.createBody(wallPose, wallShape);
    wall.setBodyType(Core::BodyType::Static);

    // Bullet 1: With CCD (Safe bullet)
    Pose bullet1Pose(Vector3(-10.0f, 1.5f, 0.0f));
    collision::CollisionShape bulletShape{collision::SphereShape(0.2f)};
    Body bullet1 = sim.createBody(bullet1Pose, bulletShape);
    bullet1.setBodyType(Core::BodyType::Dynamic);
    bullet1.setMass(0.5f);
    bullet1.setCCD(true);
    bullet1.setLinearVelocity(Vector3(100.0f, 0.0f, 0.0f));

    // Bullet 2: Without CCD (Vulnerable bullet - will tunnel or be flagged)
    Pose bullet2Pose(Vector3(-10.0f, 0.5f, 1.5f));
    Body bullet2 = sim.createBody(bullet2Pose, bulletShape);
    bullet2.setBodyType(Core::BodyType::Dynamic);
    bullet2.setMass(0.5f);
    bullet2.setCCD(false);
    bullet2.setLinearVelocity(Vector3(100.0f, 0.0f, 0.0f));
}

void TestScenes::setupBoxStacking(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Static Ground
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    collision::CollisionShape groundShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))};
    Body ground = sim.createBody(groundPose, groundShape);
    ground.setBodyType(Core::BodyType::Static);

    // 10 Stacked Boxes
    float boxHalf = 0.5f;
    for (int i = 0; i < 10; ++i) {
        float y = (boxHalf) + i * (boxHalf * 2.0f + 0.02f);
        Pose boxPose(Vector3(0.0f, y, 0.0f));
        collision::CollisionShape boxShape{collision::BoxShape(Vector3(boxHalf, boxHalf, boxHalf))};
        Body box = sim.createBody(boxPose, boxShape);
        box.setBodyType(Core::BodyType::Dynamic);
        box.setMass(1.0f);
    }
}

void TestScenes::setupNewtonsCradle(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Zero gravity

    float radius = 0.5f;
    // Row of 4 resting spheres
    for (int i = 0; i < 4; ++i) {
        float x = (i - 1.5f) * (radius * 2.0f);
        Pose pose(Vector3(x, 1.0f, 0.0f));
        collision::CollisionShape shape{collision::SphereShape(radius)};
        Body body = sim.createBody(pose, shape);
        body.setBodyType(Core::BodyType::Dynamic);
        body.setMass(2.0f);
        // High restitution
        auto& mat = sim.getRegistry().getComponent<Core::Material>(body.getEntity());
        mat.restitution = 1.0f;
    }

    // Fast incoming sphere from left
    Pose strikerPose(Vector3(-6.0f, 1.0f, 0.0f));
    collision::CollisionShape strikerShape{collision::SphereShape(radius)};
    Body striker = sim.createBody(strikerPose, strikerShape);
    striker.setBodyType(Core::BodyType::Dynamic);
    striker.setMass(2.0f);
    striker.setCCD(true);
    striker.setLinearVelocity(Vector3(15.0f, 0.0f, 0.0f));
    auto& mat = sim.getRegistry().getComponent<Core::Material>(striker.getEntity());
    mat.restitution = 1.0f;
}

void TestScenes::setupJointPendulum(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Fixed Anchor at ceiling
    Pose anchorPose(Vector3(0.0f, 10.0f, 0.0f));
    collision::CollisionShape anchorShape{collision::BoxShape(Vector3(0.3f, 0.3f, 0.3f))};
    Body prevBody = sim.createBody(anchorPose, anchorShape);
    prevBody.setBodyType(Core::BodyType::Static);

    // 4 Chain links
    float linkDistance = 1.5f;
    for (int i = 1; i <= 4; ++i) {
        Pose linkPose(Vector3(i * linkDistance * 0.7f, 10.0f - i * linkDistance * 0.7f, 0.0f));
        collision::CollisionShape linkShape{collision::BoxShape(Vector3(0.3f, 0.3f, 0.3f))};
        Body link = sim.createBody(linkPose, linkShape);
        link.setBodyType(Core::BodyType::Dynamic);
        link.setMass(1.0f);

        sim.createDistanceConstraint(prevBody, link, linkDistance);
        prevBody = link;
    }
}

void TestScenes::setupSandbox(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Ground
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    collision::CollisionShape groundShape{collision::BoxShape(Vector3(25.0f, 0.5f, 25.0f))};
    Body ground = sim.createBody(groundPose, groundShape);
    ground.setBodyType(Core::BodyType::Static);

    // 4 Walls
    float arenaRadius = 20.0f;
    float wallHeight = 2.0f;

    auto makeWall = [&](Vector3 pos, Vector3 halfExtents) {
        Body w = sim.createBody(Pose(pos), collision::CollisionShape{collision::BoxShape(halfExtents)});
        w.setBodyType(Core::BodyType::Static);
    };

    makeWall(Vector3(0.0f, wallHeight, arenaRadius), Vector3(arenaRadius, wallHeight, 0.5f));
    makeWall(Vector3(0.0f, wallHeight, -arenaRadius), Vector3(arenaRadius, wallHeight, 0.5f));
    makeWall(Vector3(arenaRadius, wallHeight, 0.0f), Vector3(0.5f, wallHeight, arenaRadius));
    makeWall(Vector3(-arenaRadius, wallHeight, 0.0f), Vector3(0.5f, wallHeight, arenaRadius));

    // Assortment of dynamic bodies
    for (int x = -2; x <= 2; ++x) {
        for (int z = -2; z <= 2; ++z) {
            Vector3 pos(x * 3.0f, 3.0f + (x + z + 4) * 1.5f, z * 3.0f);
            if ((x + z) % 2 == 0) {
                Body b = sim.createBody(Pose(pos), collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
                b.setBodyType(Core::BodyType::Dynamic);
                b.setMass(1.5f);
            } else {
                Body b = sim.createBody(Pose(pos), collision::CollisionShape{collision::SphereShape(0.6f)});
                b.setBodyType(Core::BodyType::Dynamic);
                b.setMass(1.0f);
            }
        }
    }
}

void TestScenes::setupTriggerSensors(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Sifir yercekimi ile dumduz ucus rotasi

    // Zemin referansi
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(30.0f, 0.5f, 15.0f))}).setBodyType(Core::BodyType::Static);

    // 1. Hat (Z = -2.5): MOR TETIKLEYICI ALANI (Trigger Gate)
    Pose gatePose(Vector3(0.0f, 2.0f, -2.5f));
    Body gate = sim.createBody(gatePose, collision::CollisionShape{collision::BoxShape(Vector3(0.6f, 2.0f, 2.0f))});
    gate.setBodyType(Core::BodyType::Static);
    gate.setTrigger(true);

    // Kure 1: Tetikleyici kapisinin tam icinden sifir sekme ve sifir direnc ile gecer!
    Pose runner1Pose(Vector3(-10.0f, 2.0f, -2.5f));
    Body runner1 = sim.createBody(runner1Pose, collision::CollisionShape{collision::SphereShape(0.7f)});
    runner1.setBodyType(Core::BodyType::Dynamic);
    runner1.setMass(2.0f);
    runner1.setLinearVelocity(Vector3(8.0f, 0.0f, 0.0f));

    // 2. Hat (Z = 2.5): KATI DUVAR (Solid Wall - Karsilastirma icin)
    Pose wallPose(Vector3(0.0f, 2.0f, 2.5f));
    Body wall = sim.createBody(wallPose, collision::CollisionShape{collision::BoxShape(Vector3(0.6f, 2.0f, 2.0f))});
    wall.setBodyType(Core::BodyType::Static);
    wall.setTrigger(false);

    // Kure 2: Kati duvara carpar ve geriye seker!
    Pose runner2Pose(Vector3(-10.0f, 2.0f, 2.5f));
    Body runner2 = sim.createBody(runner2Pose, collision::CollisionShape{collision::SphereShape(0.7f)});
    runner2.setBodyType(Core::BodyType::Dynamic);
    runner2.setMass(2.0f);
    runner2.setLinearVelocity(Vector3(8.0f, 0.0f, 0.0f));
}

void TestScenes::setupCollisionFiltering(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Sifir yercekimi - kutular surtunmeden dolayi durmaz!

    // Zemin referansi
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(30.0f, 0.5f, 15.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // KIRMIZI TAKIM (Layer 0x0002): Maske sadece 0x0002 (Yalnizca diger kirmizilarla carpisir, Maviyi yoksayar)
    uint32_t redLayer = 0x0002;
    uint32_t redMask = 0x0002;

    // MAVI TAKIM (Layer 0x0004): Maske sadece 0x0004 (Yalnizca diger mavilerle carpisir, Kirmiziyi yoksayar)
    uint32_t blueLayer = 0x0004;
    uint32_t blueMask = 0x0004;

    // HAT 1 (Z = -2.5): KIRMIZI VS MAVI -> FARKLI KATMANLAR: BIRBIRLERININ ICINDEN GECERLER!
    Pose redPose(Vector3(-8.0f, 2.0f, -2.5f));
    Body redBox = sim.createBody(redPose, collision::CollisionShape{collision::BoxShape(Vector3(0.7f, 0.7f, 0.7f))});
    redBox.setBodyType(Core::BodyType::Dynamic);
    redBox.setMass(2.0f);
    redBox.setCollisionLayer(redLayer);
    redBox.setCollisionMask(redMask);
    redBox.setLinearVelocity(Vector3(5.0f, 0.0f, 0.0f));

    Pose bluePose(Vector3(8.0f, 2.0f, -2.5f));
    Body blueBox = sim.createBody(bluePose, collision::CollisionShape{collision::BoxShape(Vector3(0.7f, 0.7f, 0.7f))});
    blueBox.setBodyType(Core::BodyType::Dynamic);
    blueBox.setMass(2.0f);
    blueBox.setCollisionLayer(blueLayer);
    blueBox.setCollisionMask(blueMask);
    blueBox.setLinearVelocity(Vector3(-5.0f, 0.0f, 0.0f));

    // HAT 2 (Z = 2.5): KIRMIZI VS KIRMIZI -> AYNI KATMAN: KARSILASINCA CARPISIR VE SEKERLER!
    Pose red1Pose(Vector3(-8.0f, 2.0f, 2.5f));
    Body red1 = sim.createBody(red1Pose, collision::CollisionShape{collision::BoxShape(Vector3(0.7f, 0.7f, 0.7f))});
    red1.setBodyType(Core::BodyType::Dynamic);
    red1.setMass(2.0f);
    red1.setCollisionLayer(redLayer);
    red1.setCollisionMask(redMask);
    red1.setLinearVelocity(Vector3(5.0f, 0.0f, 0.0f));

    Pose red2Pose(Vector3(8.0f, 2.0f, 2.5f));
    Body red2 = sim.createBody(red2Pose, collision::CollisionShape{collision::BoxShape(Vector3(0.7f, 0.7f, 0.7f))});
    red2.setBodyType(Core::BodyType::Dynamic);
    red2.setMass(2.0f);
    red2.setCollisionLayer(redLayer);
    red2.setCollisionMask(redMask);
    red2.setLinearVelocity(Vector3(-5.0f, 0.0f, 0.0f));
}

void TestScenes::setupRevoluteHinge(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))}).setBodyType(Core::BodyType::Static);

    // Sabit Dikey Direk (Post / Hinge Anchor)
    Pose postPose(Vector3(0.0f, 2.0f, 0.0f));
    Body post = sim.createBody(postPose, collision::CollisionShape{collision::BoxShape(Vector3(0.2f, 2.0f, 0.2f))});
    post.setBodyType(Core::BodyType::Static);

    // Dikey Kapi Kanadi (Dynamic Door: genislik 1.6m, yukseklik 2.0m, kalinlik 0.1m)
    Pose doorPose(Vector3(1.0f, 2.0f, 0.0f));
    Body door = sim.createBody(doorPose, collision::CollisionShape{collision::BoxShape(Vector3(0.8f, 1.0f, 0.06f))});
    door.setBodyType(Core::BodyType::Dynamic);
    door.setMass(4.0f);

    // Menteşe Kısıtı (RevoluteConstraint - Y ekseni etrafında)
    sim.createRevoluteConstraint(post, door,
        Vector3(0.0f, 0.0f, 0.0f),
        Vector3(-0.8f, 0.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f));

    // Kapiya baslangic donme itmesi (Acisal itme) ver
    door.applyAngularImpulse(Vector3(0.0f, 15.0f, 0.0f));
}

void TestScenes::setupRollingResistance(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Duz zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(50.0f, 0.5f, 10.0f))});
    ground.setBodyType(Core::BodyType::Static);
    ground.setMaterial(0.3f, 0.0f);

    // Kure A: Dusuk surtunme (0.01) - Sonsuza dek yuvarlanmaya devam eder
    Pose sphere1Pose(Vector3(-15.0f, 0.6f, -2.5f));
    Body s1 = sim.createBody(sphere1Pose, collision::CollisionShape{collision::SphereShape(0.6f)});
    s1.setBodyType(Core::BodyType::Dynamic);
    s1.setMass(1.0f);
    s1.setMaterial(0.01f, 0.0f);
    s1.setLinearVelocity(Vector3(8.0f, 0.0f, 0.0f));
    s1.setAngularVelocity(Vector3(0.0f, 0.0f, -13.3f));

    // Kure B: Yuksek surtunme & yuvarlanma kaybi (0.6) - Dogal olarak yavaslayip durur
    Pose sphere2Pose(Vector3(-15.0f, 0.6f, 2.5f));
    Body s2 = sim.createBody(sphere2Pose, collision::CollisionShape{collision::SphereShape(0.6f)});
    s2.setBodyType(Core::BodyType::Dynamic);
    s2.setMass(1.0f);
    s2.setMaterial(0.6f, 0.0f);
    s2.setLinearVelocity(Vector3(8.0f, 0.0f, 0.0f));
    s2.setAngularVelocity(Vector3(0.0f, 0.0f, -13.3f));
}

void TestScenes::setupRaycastShowcase(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Sifir yercekimi

    // 1. Kapsul Sekli (X = -6)
    Pose capPose(Vector3(-6.0f, 2.0f, 0.0f));
    Body cap = sim.createBody(capPose, collision::CollisionShape{collision::CapsuleShape(0.6f, 2.0f)});
    cap.setBodyType(Core::BodyType::Static);

    // 2. Kutu Sekli (X = -2)
    Pose boxPose(Vector3(-2.0f, 2.0f, 0.0f));
    Body box = sim.createBody(boxPose, collision::CollisionShape{collision::BoxShape(Vector3(0.8f, 0.8f, 0.8f))});
    box.setBodyType(Core::BodyType::Static);

    // 3. Kure Sekli (X = 2)
    Pose sphPose(Vector3(2.0f, 2.0f, 0.0f));
    Body sph = sim.createBody(sphPose, collision::CollisionShape{collision::SphereShape(0.8f)});
    sph.setBodyType(Core::BodyType::Static);

    // 4. Statik 3D Ucgen Ag (Mesh Piramit: X = 6)
    std::vector<Vector3> verts = {
        Vector3(-1.0f, 0.0f, -1.0f),
        Vector3(1.0f, 0.0f, -1.0f),
        Vector3(0.0f, 0.0f, 1.0f),
        Vector3(0.0f, 2.0f, 0.0f)
    };
    std::vector<uint32_t> inds = {
        0, 1, 2, // taban
        0, 1, 3, // yan 1
        1, 2, 3, // yan 2
        2, 0, 3  // yan 3
    };
    auto meshObj = std::make_shared<collision::StaticMesh>(verts, inds);
    Pose meshPose(Vector3(6.0f, 1.0f, 0.0f));
    Body meshBody = sim.createBody(meshPose, collision::CollisionShape{collision::StaticMeshShape(meshObj)});
    meshBody.setBodyType(Core::BodyType::Static);
}

} // namespace Baryon::Debugger
