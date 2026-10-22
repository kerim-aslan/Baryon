/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/TestScenes.hpp"
#include <Baryon/collision/CollisionShape.hpp>

namespace Baryon::Debugger {

const char* TestScenes::getSceneName(SceneType type) {
    switch (type) {
        case SceneType::CompoundShapes: return "0. Bilesik Sekiller & Eylemsizlik (Compound Shapes)";
        case SceneType::OverlapQueries: return "1. Hacim & Cakisma Sorgulari (Overlap Queries)";
        case SceneType::SweepQueries: return "2. Supurme & Sekil Firlatma (Sweep Queries / Shape Cast)";
        case SceneType::TriggerSensors: return "3. Tetikleyici & Hayalet Alan (Trigger Zones)";
        case SceneType::CollisionFiltering: return "4. Katman Filtreleme (Bitmask: Kirmizi vs Mavi)";
        case SceneType::RevoluteHinge: return "5. Mentese Mafsali & Tork (Revolute Joint)";
        case SceneType::RollingResistance: return "6. Yuvarlanma Direnci & Durma Testi";
        case SceneType::RaycastShowcase: return "7. Lazer Isini Kesisimi (Raycast)";
        case SceneType::TunnelingStress: return "8. Yuksek Hizli Tunelleme Testi (CCD)";
        case SceneType::BoxStacking: return "9. Kutu Yigini Kararliligi (10 Kutu)";
        case SceneType::NewtonsCradle: return "10. Esnek Carpisma & Momentum (Newton Besigi)";
        case SceneType::JointPendulum: return "11. Eklem & Mesafe Kisiti Zinciri";
        case SceneType::FixedConstraintWeld: return "12. Sabit Kaynak Eklemi (Fixed / Weld Joint)";
        case SceneType::BallSocketPendulum: return "13. Kuresel Mafsal Sarkaci (Ball-Socket Pendulum)";
        case SceneType::PrismaticElevator: return "14. Prizmatik Kayar Eklem & Asansor (Prismatic Slider)";
        case SceneType::SpringSuspension: return "15. Yayli Suspansiyon & Sonumlu Salinim (Spring Joint)";
        case SceneType::Sandbox: return "16. Serbest Fizik Test Alani (Sandbox)";
        default: return "Bilinmeyen Sahne";
    }
}

const char* TestScenes::getSceneDescription(SceneType type) {
    switch (type) {
        case SceneType::CompoundShapes:
            return "Birden fazla alt geometriyi (kure, kutu) tek bir rijit cisimde birlestirir. Agirlik merkezi ve Huygens-Steiner paralel eksen eylemsizlik tensoru ile dambil, L-blok ve masa gibi karmasik sekilleri simule eder.";
        case SceneType::OverlapQueries:
            return "overlapSphere, overlapBox ve overlapShape fonksiyonlarini test eder. 3B uzayda donen tarayici hacim (kure/OBB) hedeflere degdiginde canli olarak algilanir ve tel kafes rengi mavi/kirmizi olarak degisir.";
        case SceneType::SweepQueries:
            return "sweepSphere, sweepBox ve sweepCapsule fonksiyonlarini test eder. 3B uzayda belirli bir dogrultuda firlatilan seklin temas mesafesi, carptigi yuzey normali ve ilk temas ani canli olarak cizilir.";
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
        case SceneType::FixedConstraintWeld:
            return "6 serbestlik derecesinin (3 cizgisel + 3 acisal) kilitlendigi kaynak eklemini test eder. Yercekimi altinda cantilever konsol kiris ve bagli kutular tek bir kati govde gibi davranir.";
        case SceneType::BallSocketPendulum:
            return "3 cizgisel serbestlik derecesinin kilitlendigi, 3 acisal serbestligin serbest oldugu bilyeli mafsal (ball-socket) eklemini gosterir. Cift eklemli 3D kuresel sarkac ve serbest acisal eksende donen jiroskopik bloklar yer alir.";
        case SceneType::PrismaticElevator:
            return "1 serbest dogrusal kayma eksenine (1-DOF) izin verip 2 yanal ve 3 acisal ekseni kilitleyen prizmatik eklemi test eder. Dikey eksende alt/ust sinirli ve motorlu asansor kabini ile yatay rayda hareket eden mekanik kizak yer alir.";
        case SceneType::SpringSuspension:
            return "Hooke Yasasi ve sonumlu harmonik salinici (Damped Harmonic Oscillator) modelini test eder. Sira sira farkli sonumleme oranlarina (sonumsuz zeta=0, kritik sonumlu zeta=1, zayif sonumlu zeta=0.15) sahip yayli kutleler ve 4 tekerlekli araba suspansiyon sasisi yer alir.";
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
        case SceneType::CompoundShapes: setupCompoundShapes(sim); break;
        case SceneType::OverlapQueries: setupOverlapQueries(sim); break;
        case SceneType::SweepQueries: setupSweepQueries(sim); break;
        case SceneType::TriggerSensors: setupTriggerSensors(sim); break;
        case SceneType::CollisionFiltering: setupCollisionFiltering(sim); break;
        case SceneType::RevoluteHinge: setupRevoluteHinge(sim); break;
        case SceneType::RollingResistance: setupRollingResistance(sim); break;
        case SceneType::RaycastShowcase: setupRaycastShowcase(sim); break;
        case SceneType::TunnelingStress: setupTunneling(sim); break;
        case SceneType::BoxStacking: setupBoxStacking(sim); break;
        case SceneType::NewtonsCradle: setupNewtonsCradle(sim); break;
        case SceneType::JointPendulum: setupJointPendulum(sim); break;
        case SceneType::FixedConstraintWeld: setupFixedConstraintWeld(sim); break;
        case SceneType::BallSocketPendulum: setupBallSocketPendulum(sim); break;
        case SceneType::PrismaticElevator: setupPrismaticElevator(sim); break;
        case SceneType::SpringSuspension: setupSpringSuspension(sim); break;
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

    // Silindir C: Yuvarlanan silindir (Z = 0.0)
    Quaternion cylRot(Vector3(1.0f, 0.0f, 0.0f), 3.14159265f * 0.5f);
    Pose cylPose(Vector3(-15.0f, 0.6f, 0.0f), cylRot);
    Body c1 = sim.createBody(cylPose, collision::CollisionShape{collision::CylinderShape(0.6f, 0.8f)});
    c1.setBodyType(Core::BodyType::Dynamic);
    c1.setMass(1.5f);
    c1.setMaterial(0.01f, 0.0f);
    c1.setLinearVelocity(Vector3(8.0f, 0.0f, 0.0f));
    c1.setAngularVelocity(Vector3(0.0f, 0.0f, -13.3f));
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

    // 5. Silindir Sekli (X = 10)
    Pose cylTargetPose(Vector3(10.0f, 2.0f, 0.0f));
    Body cylTarget = sim.createBody(cylTargetPose, collision::CollisionShape{collision::CylinderShape(0.7f, 1.5f)});
    cylTarget.setBodyType(Core::BodyType::Static);

    // 6. Koni Sekli (X = 14)
    Pose coneTargetPose(Vector3(14.0f, 2.0f, 0.0f));
    Body coneTarget = sim.createBody(coneTargetPose, collision::CollisionShape{collision::ConeShape(0.8f, 2.0f)});
    coneTarget.setBodyType(Core::BodyType::Static);
}

void TestScenes::setupCompoundShapes(Simulator& sim) {
    // 1. Static Ground
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(25.0f, 0.5f, 25.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // 2. Dumbbell (2 kure ve baglanti silindiri/kutusu)
    {
        std::vector<collision::CompoundChild> children;
        children.emplace_back(collision::CollisionShape{collision::SphereShape(0.6f)}, Pose(Vector3(-1.5f, 0.0f, 0.0f)));
        children.emplace_back(collision::CollisionShape{collision::SphereShape(0.6f)}, Pose(Vector3(1.5f, 0.0f, 0.0f)));
        children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(1.0f, 0.15f, 0.15f))}, Pose(Vector3(0.0f, 0.0f, 0.0f)));

        collision::CompoundShape dumbbell(std::move(children));
        Pose p(Vector3(-4.0f, 6.0f, 0.0f), Quaternion(Vector3(0.0f, 0.0f, 1.0f), 0.35f));
        Body b = sim.createBody(p, collision::CollisionShape{dumbbell});
        (void)b.setMass(4.0f);
        (void)b.setMaterial(0.4f, 0.4f);
    }

    // 3. L-Shaped Object (2 dik kutu)
    {
        std::vector<collision::CompoundChild> children;
        children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(1.5f, 0.3f, 0.5f))}, Pose(Vector3(0.0f, 0.0f, 0.0f)));
        children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(0.3f, 1.2f, 0.5f))}, Pose(Vector3(-1.2f, 1.2f, 0.0f)));

        collision::CompoundShape lShape(std::move(children));
        Vector3 axis(0.0f, 1.0f, 0.5f);
        axis.normalize();
        Pose p(Vector3(0.0f, 5.0f, 0.0f), Quaternion(axis, 0.5f));
        Body b = sim.createBody(p, collision::CollisionShape{lShape});
        (void)b.setMass(3.0f);
        (void)b.setMaterial(0.4f, 0.2f);
    }

    // 4. Table (4 bacak + 1 masa tablasi)
    {
        std::vector<collision::CompoundChild> children;
        children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(1.2f, 0.1f, 1.2f))}, Pose(Vector3(0.0f, 1.0f, 0.0f)));
        float legX = 1.0f;
        float legZ = 1.0f;
        Vector3 legHalfExtents(0.1f, 0.5f, 0.1f);
        children.emplace_back(collision::CollisionShape{collision::BoxShape(legHalfExtents)}, Pose(Vector3(-legX, 0.5f, -legZ)));
        children.emplace_back(collision::CollisionShape{collision::BoxShape(legHalfExtents)}, Pose(Vector3( legX, 0.5f, -legZ)));
        children.emplace_back(collision::CollisionShape{collision::BoxShape(legHalfExtents)}, Pose(Vector3(-legX, 0.5f,  legZ)));
        children.emplace_back(collision::CollisionShape{collision::BoxShape(legHalfExtents)}, Pose(Vector3( legX, 0.5f,  legZ)));

        Vector3 tableAxis(1.0f, 0.2f, 0.0f);
        tableAxis.normalize();
        Pose p(Vector3(4.0f, 4.0f, 0.0f), Quaternion(tableAxis, 0.2f));
        collision::CompoundShape table(std::move(children));
        Body b = sim.createBody(p, collision::CollisionShape{table});
        (void)b.setMass(5.0f);
        (void)b.setMaterial(0.4f, 0.1f);
    }
}

void TestScenes::setupOverlapQueries(Simulator& sim) {
    // 1. Statik Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(25.0f, 0.5f, 25.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // Dairesel duzende hedefler (Kure, Kutu, Kapsul, Dambil)
    // 2. Kure (0 deg: X=6, Z=0)
    Body s = sim.createBody(Pose(Vector3(6.0f, 1.5f, 0.0f)), collision::CollisionShape{collision::SphereShape(1.0f)});
    s.setBodyType(Core::BodyType::Static);

    // 3. Kutu (90 deg: X=0, Z=6)
    Body b = sim.createBody(Pose(Vector3(0.0f, 1.5f, 6.0f)), collision::CollisionShape{collision::BoxShape(Vector3(1.0f, 1.0f, 1.0f))});
    b.setBodyType(Core::BodyType::Static);

    // 4. Kapsul (180 deg: X=-6, Z=0)
    Body cap = sim.createBody(Pose(Vector3(-6.0f, 1.5f, 0.0f)), collision::CollisionShape{collision::CapsuleShape(0.7f, 1.5f)});
    cap.setBodyType(Core::BodyType::Static);

    // 5. Dambil (270 deg: X=0, Z=-6)
    std::vector<collision::CompoundChild> children;
    children.emplace_back(collision::CollisionShape{collision::SphereShape(0.6f)}, Pose(Vector3(-1.2f, 0.0f, 0.0f)));
    children.emplace_back(collision::CollisionShape{collision::SphereShape(0.6f)}, Pose(Vector3(1.2f, 0.0f, 0.0f)));
    children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(0.8f, 0.15f, 0.15f))}, Pose(Vector3(0.0f, 0.0f, 0.0f)));
    collision::CompoundShape dumbbell(std::move(children));
    Body damb = sim.createBody(Pose(Vector3(0.0f, 1.5f, -6.0f)), collision::CollisionShape{dumbbell});
    damb.setBodyType(Core::BodyType::Static);
}

void TestScenes::setupSweepQueries(Simulator& sim) {
    // 1. Statik Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(25.0f, 0.5f, 25.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // Kapsul (Sol taraf: X = -4.0, Z = 0)
    Body cap = sim.createBody(Pose(Vector3(-4.0f, 1.5f, 0.0f)), collision::CollisionShape{collision::CapsuleShape(0.6f, 2.0f)});
    cap.setBodyType(Core::BodyType::Static);

    // Kutu (Orta-Sol: X = -1.5, Z = 3.0)
    Body box = sim.createBody(Pose(Vector3(-1.5f, 1.5f, 3.0f)), collision::CollisionShape{collision::BoxShape(Vector3(0.8f, 1.0f, 0.8f))});
    box.setBodyType(Core::BodyType::Static);

    // Kure (Orta-Sag: X = 1.5, Z = 3.0)
    Body sph = sim.createBody(Pose(Vector3(1.5f, 1.5f, 3.0f)), collision::CollisionShape{collision::SphereShape(0.9f)});
    sph.setBodyType(Core::BodyType::Static);

    // Dambil (Bilesik Sekil: X = 4.5, Z = 0.0)
    std::vector<collision::CompoundChild> children;
    children.emplace_back(collision::CollisionShape{collision::SphereShape(0.5f)}, Pose(Vector3(-1.0f, 0.0f, 0.0f)));
    children.emplace_back(collision::CollisionShape{collision::SphereShape(0.5f)}, Pose(Vector3(1.0f, 0.0f, 0.0f)));
    children.emplace_back(collision::CollisionShape{collision::BoxShape(Vector3(0.7f, 0.12f, 0.12f))}, Pose(Vector3(0.0f, 0.0f, 0.0f)));
    collision::CompoundShape dumbbell(std::move(children));
    Body damb = sim.createBody(Pose(Vector3(4.5f, 1.5f, 0.0f)), collision::CollisionShape{dumbbell});
    damb.setBodyType(Core::BodyType::Static);
}

void TestScenes::setupFixedConstraintWeld(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // 1. Konsol kiriş (Cantilever): Statik mesnet ve 3 parçalı kaynak kiriş
    Pose postPose(Vector3(-4.0f, 5.0f, 0.0f));
    Body post = sim.createBody(postPose, collision::CollisionShape{collision::BoxShape(Vector3(0.25f, 5.0f, 0.25f))});
    post.setBodyType(Core::BodyType::Static);

    Body prev = post;
    for (int i = 1; i <= 3; ++i) {
        Pose beamPose(Vector3(-4.0f + static_cast<float>(i) * 1.0f, 9.5f, 0.0f));
        Body beam = sim.createBody(beamPose, collision::CollisionShape{collision::BoxShape(Vector3(0.45f, 0.15f, 0.15f))});
        beam.setBodyType(Core::BodyType::Dynamic);
        (void)beam.setMass(1.5f);
        sim.createFixedConstraint(prev, beam);
        prev = beam;
    }

    // 2. T-Şekilli kaynaklı serbest takla atan dinamik gövde
    Pose t1Pose(Vector3(3.0f, 8.0f, 0.0f));
    Body t1 = sim.createBody(t1Pose, collision::CollisionShape{collision::BoxShape(Vector3(0.8f, 0.2f, 0.2f))});
    t1.setBodyType(Core::BodyType::Dynamic);
    (void)t1.setMass(2.0f);

    Pose t2Pose(Vector3(3.0f, 7.2f, 0.0f));
    Body t2 = sim.createBody(t2Pose, collision::CollisionShape{collision::BoxShape(Vector3(0.2f, 0.6f, 0.2f))});
    t2.setBodyType(Core::BodyType::Dynamic);
    (void)t2.setMass(2.0f);

    sim.createFixedConstraint(t1, t2);
    (void)t1.setAngularVelocity(Vector3(0.0f, 0.0f, 2.0f));
}

void TestScenes::setupBallSocketPendulum(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // 1. Çift Mafsallı Küresel Sarkaç (Double Spherical Pendulum)
    Vector3 pivot1(0.0f, 8.0f, 0.0f);
    Pose p1Pose(pivot1);
    Body p1 = sim.createBody(p1Pose, collision::CollisionShape{collision::SphereShape(0.15f)});
    p1.setBodyType(Core::BodyType::Static);

    Vector3 bob1Pos(2.0f, 8.0f, 0.0f);
    Pose b1Pose(bob1Pos);
    Body b1 = sim.createBody(b1Pose, collision::CollisionShape{collision::SphereShape(0.25f)});
    b1.setBodyType(Core::BodyType::Dynamic);
    (void)b1.setMass(1.0f);
    sim.createBallSocketConstraint(p1, b1, pivot1);

    Vector3 bob2Pos(3.5f, 8.0f, 0.5f);
    Pose b2Pose(bob2Pos);
    Body b2 = sim.createBody(b2Pose, collision::CollisionShape{collision::SphereShape(0.25f)});
    b2.setBodyType(Core::BodyType::Dynamic);
    (void)b2.setMass(1.0f);
    sim.createBallSocketConstraint(b1, b2, bob1Pos);

    // 2. Jiroskopik Serbest Dönen Kutu (3-DOF Serbest Rotasyon Gösterimi)
    Vector3 pivotSpin(-4.0f, 7.0f, 0.0f);
    Pose pSpinPose(pivotSpin);
    Body pSpin = sim.createBody(pSpinPose, collision::CollisionShape{collision::SphereShape(0.15f)});
    pSpin.setBodyType(Core::BodyType::Static);

    Vector3 spinnerPos(-4.0f, 5.0f, 0.0f);
    Pose spinPose(spinnerPos);
    Body spinner = sim.createBody(spinPose, collision::CollisionShape{collision::BoxShape(Vector3(0.3f, 0.6f, 0.3f))});
    spinner.setBodyType(Core::BodyType::Dynamic);
    (void)spinner.setMass(2.0f);
    sim.createBallSocketConstraint(pSpin, spinner, pivotSpin);
    (void)spinner.setAngularVelocity(Vector3(2.0f, 5.0f, 1.0f));
}

void TestScenes::setupPrismaticElevator(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // 1. Dikey Kılavuz Ray ve Motorlu Asansör Kabini (Y Ekseni Boyunca 1-DOF Kayma)
    Vector3 towerPos(0.0f, 5.0f, 0.0f);
    Pose towerPose(towerPos);
    Body tower = sim.createBody(towerPose, collision::CollisionShape{collision::BoxShape(Vector3(0.2f, 5.0f, 0.2f))});
    tower.setBodyType(Core::BodyType::Static);

    Vector3 cabinPos(0.8f, 1.0f, 0.0f);
    Pose cabinPose(cabinPos);
    Body cabin = sim.createBody(cabinPose, collision::CollisionShape{collision::BoxShape(Vector3(0.6f, 0.8f, 0.6f))});
    cabin.setBodyType(Core::BodyType::Dynamic);
    (void)cabin.setMass(10.0f);

    sim.createPrismaticConstraint(tower, cabin, Vector3(0.8f, -4.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
    sim.setPrismaticLimits(tower, -2.0f, 6.0f);
    sim.setPrismaticMotor(tower, 2.0f, 400.0f);

    // 2. Yatay Hidrolik Piston (X Ekseni Boyunca Kayma ve Sınır Testi)
    Vector3 cylinderBasePos(-4.0f, 1.5f, 0.0f);
    Pose cylPose(cylinderBasePos);
    Body cylBase = sim.createBody(cylPose, collision::CollisionShape{collision::BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    cylBase.setBodyType(Core::BodyType::Static);

    Vector3 rodPos(-2.5f, 1.5f, 0.0f);
    Pose rodPose(rodPos);
    Body rod = sim.createBody(rodPose, collision::CollisionShape{collision::BoxShape(Vector3(1.0f, 0.3f, 0.3f))});
    rod.setBodyType(Core::BodyType::Dynamic);
    (void)rod.setMass(5.0f);

    sim.createPrismaticConstraint(cylBase, rod, Vector3(1.5f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    sim.setPrismaticLimits(cylBase, 0.0f, 3.0f);
    sim.setPrismaticMotor(cylBase, 1.5f, 200.0f);
}

void TestScenes::setupSpringSuspension(Simulator& sim) {
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, collision::CollisionShape{collision::BoxShape(Vector3(20.0f, 0.5f, 20.0f))});
    ground.setBodyType(Core::BodyType::Static);

    // Üst Statik Taşıyıcı Kiriş
    Pose beamPose(Vector3(0.0f, 8.0f, 0.0f));
    Body beam = sim.createBody(beamPose, collision::CollisionShape{collision::BoxShape(Vector3(6.0f, 0.2f, 0.5f))});
    beam.setBodyType(Core::BodyType::Static);

    // 1. Sol: Sönümsüz Harmonik Salınıcı (k = 60, c = 0)
    Pose bob1Pose(Vector3(-4.0f, 4.0f, 0.0f));
    Body bob1 = sim.createBody(bob1Pose, collision::CollisionShape{collision::SphereShape(0.4f)});
    bob1.setBodyType(Core::BodyType::Dynamic);
    (void)bob1.setMass(2.0f);
    sim.createSpringConstraint(beam, bob1, Vector3(-4.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), 2.5f, 60.0f, 0.0f);

    // 2. Orta: Zayıf Sönümlü Salınıcı (k = 60, c = 2.0)
    Pose bob2Pose(Vector3(0.0f, 4.0f, 0.0f));
    Body bob2 = sim.createBody(bob2Pose, collision::CollisionShape{collision::SphereShape(0.4f)});
    bob2.setBodyType(Core::BodyType::Dynamic);
    (void)bob2.setMass(2.0f);
    sim.createSpringConstraint(beam, bob2, Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), 2.5f, 60.0f, 2.0f);

    // 3. Sağ: Kritik Sönümlü Salınıcı (k = 60, c = 2*sqrt(m*k) = ~22)
    Pose bob3Pose(Vector3(4.0f, 4.0f, 0.0f));
    Body bob3 = sim.createBody(bob3Pose, collision::CollisionShape{collision::SphereShape(0.4f)});
    bob3.setBodyType(Core::BodyType::Dynamic);
    (void)bob3.setMass(2.0f);
    sim.createSpringConstraint(beam, bob3, Vector3(4.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), 2.5f, 60.0f, 22.0f);

    // 4. Ön Taraf: 4 Ayaklı Yaylı Süspansiyon Platformu (Z = 4.0)
    Pose chassisPose(Vector3(0.0f, 3.0f, 4.0f));
    Body chassis = sim.createBody(chassisPose, collision::CollisionShape{collision::BoxShape(Vector3(1.5f, 0.2f, 1.0f))});
    chassis.setBodyType(Core::BodyType::Dynamic);
    (void)chassis.setMass(10.0f);

    float cornerX[4] = {-1.2f, 1.2f, -1.2f, 1.2f};
    float cornerZ[4] = {-0.8f, -0.8f, 0.8f, 0.8f};
    for (int i = 0; i < 4; ++i) {
        Pose postPose(Vector3(cornerX[i], 1.0f, 4.0f + cornerZ[i]));
        Body post = sim.createBody(postPose, collision::CollisionShape{collision::BoxShape(Vector3(0.15f, 1.0f, 0.15f))});
        post.setBodyType(Core::BodyType::Static);

        sim.createSpringConstraint(post, chassis,
                                   Vector3(0.0f, 1.0f, 0.0f),
                                   Vector3(cornerX[i], 0.0f, cornerZ[i]),
                                   1.5f, 120.0f, 15.0f);
    }
}

} // namespace Baryon::Debugger
