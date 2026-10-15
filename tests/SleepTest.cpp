#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

// 1. Varsayılan ayarlarla kutu dinlenince <= 1.5s (90 adım) içinde uyumalı
void testDefaultBoxSleep() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);
    Body b = sim.createBody(Pose(Vector3(0, 0.6f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b.setBodyType(Core::BodyType::Dynamic);

    int slept = -1;
    for (int i = 0; i < 600; ++i) {
        sim.step(1.0f / 60.0f);
        if (slept < 0 && b.isSleeping().value()) {
            slept = i;
        }
    }
    std::cout << "[testDefaultBoxSleep] slept at step: " << slept << std::endl;
    CHECK_MSG(slept >= 0 && slept <= 90, "Dinlenen kutu 1.5 sn icinde uyumali");
}

// 2. Varsayılan ayarlarla küre dinlenince <= 1.5s (90 adım) içinde uyumalı
void testSphereSleep() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);
    Body s = sim.createBody(Pose(Vector3(0, 0.6f, 0)), CollisionShape{SphereShape(0.5f)});
    (void)s.setBodyType(Core::BodyType::Dynamic);

    int slept = -1;
    for (int i = 0; i < 600; ++i) {
        sim.step(1.0f / 60.0f);
        if (slept < 0 && s.isSleeping().value()) {
            slept = i;
        }
    }
    std::cout << "[testSphereSleep] slept at step: " << slept << std::endl;
    CHECK_MSG(slept >= 0 && slept <= 90, "Dinlenen kure 1.5 sn icinde uyumali");
}

// 3. Varsayılan ayarlarla dik kapsül dinlenince <= 1.5s (90 adım) içinde uyumalı
void testUprightCapsuleSleep() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);
    // Yarıçap 0.3, silindir yüksekliği 1.0 -> toplam yarım boy = 0.3 + 0.5 = 0.8.
    // Başlangıç y = 0.85 (zemine çok yakın)
    Body c = sim.createBody(Pose(Vector3(0, 0.85f, 0)), CollisionShape{CapsuleShape(0.3f, 1.0f)});
    (void)c.setBodyType(Core::BodyType::Dynamic);

    int slept = -1;
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        if (slept < 0 && c.isSleeping().value()) {
            slept = i;
        }
    }
    std::cout << "[testUprightCapsuleSleep] slept at step: " << slept << std::endl;
    CHECK_MSG(slept >= 0 && slept <= 90, "Dinlenen dik kapsul 1.5 sn icinde uyumali");
}

// 4. Varsayılan ayarlarla 8'li kule <= 1.5s (90 adım) içinde uyumalı
void testTowerSleep() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);

    std::vector<Body> tower;
    tower.reserve(8);
    for (int j = 0; j < 8; ++j) {
        // Her kutu yarım boyutu 0.5, yani 1.0 yükseklik.
        // Taban y=0'da. Kutu 0 merkez: 0.5f. Kutu 1 merkez: 1.5f ...
        Body b = sim.createBody(Pose(Vector3(0, 0.5f + static_cast<float>(j) * 1.0f, 0)),
                                CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)b.setBodyType(Core::BodyType::Dynamic);
        tower.push_back(b);
    }

    int allSlept = -1;
    for (int i = 0; i < 600; ++i) {
        sim.step(1.0f / 60.0f);
        bool all = true;
        for (const auto& b : tower) {
            if (!b.isSleeping().value()) {
                all = false;
                break;
            }
        }
        if (allSlept < 0 && all) {
            allSlept = i;
        }
    }
    std::cout << "[testTowerSleep] all 8 boxes slept at step: " << allSlept << std::endl;
    CHECK_MSG(allSlept >= 0 && allSlept <= 90, "8'li kule 1.5 sn icinde uyumali");
}

// 5. Frekans bağımsızlığı: 30 Hz ve 120 Hz testleri
void testFrequencies() {
    // 30 Hz testi (1.5 sn = 45 adım)
    {
        Simulator sim;
        sim.setFixedTimeStep(1.0f / 30.0f);
        sim.setAccelerationField(Vector3(0, -9.81f, 0));
        Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
        (void)f.setBodyType(Core::BodyType::Static);
        Body b = sim.createBody(Pose(Vector3(0, 0.6f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)b.setBodyType(Core::BodyType::Dynamic);

        int slept = -1;
        for (int i = 0; i < 300; ++i) {
            sim.step(1.0f / 30.0f);
            if (slept < 0 && b.isSleeping().value()) {
                slept = i;
            }
        }
        std::cout << "[testFrequencies 30Hz] slept at step: " << slept << std::endl;
        CHECK_MSG(slept >= 0 && slept <= 45, "30 Hz'de kutu 1.5 sn icinde uyumali");
    }

    // 120 Hz testi (1.5 sn = 180 adım)
    {
        Simulator sim;
        sim.setFixedTimeStep(1.0f / 120.0f);
        sim.setAccelerationField(Vector3(0, -9.81f, 0));
        Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
        (void)f.setBodyType(Core::BodyType::Static);
        Body b = sim.createBody(Pose(Vector3(0, 0.6f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        (void)b.setBodyType(Core::BodyType::Dynamic);

        int slept = -1;
        for (int i = 0; i < 600; ++i) {
            sim.step(1.0f / 120.0f);
            if (slept < 0 && b.isSleeping().value()) {
                slept = i;
            }
        }
        std::cout << "[testFrequencies 120Hz] slept at step: " << slept << std::endl;
        CHECK_MSG(slept >= 0 && slept <= 180, "120 Hz'de kutu 1.5 sn icinde uyumali");
    }
}

// 6. Uyandırma testleri:
// (a) Uyuyan kutuya hızlı cisim çarpınca uyansın
void testWakeUpFastProjectile() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);
    Body b = sim.createBody(Pose(Vector3(0, 0.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b.setBodyType(Core::BodyType::Dynamic);

    // Kutuyu uyuyana kadar simüle et
    for (int i = 0; i < 90; ++i) sim.step(1.0f / 60.0f);
    CHECK_MSG(b.isSleeping().value(), "Hedef kutu baslangicta uyuyor olmali");

    // Hızlı mermi fırlat (-X yönünden X=0'a doğru hızla gelen küçük küre)
    Body bullet = sim.createBody(Pose(Vector3(-5.0f, 0.5f, 0)), CollisionShape{SphereShape(0.2f)});
    (void)bullet.setBodyType(Core::BodyType::Dynamic);
    (void)bullet.setLinearVelocity(Vector3(50.0f, 0, 0));

    // Birkaç adım ilerlet, çarpışma kutuyu uyandırmalı
    bool wokeUp = false;
    for (int i = 0; i < 20; ++i) {
        sim.step(1.0f / 60.0f);
        if (!b.isSleeping().value()) {
            wokeUp = true;
            break;
        }
    }
    std::cout << "[testWakeUpFastProjectile] woke up: " << (wokeUp ? "true" : "false") << std::endl;
    CHECK_MSG(wokeUp, "Hizli mermi carpan uyuyan kutu uyanmali");
}

// (b) Uyuyan kutunun altındaki cisim yok edilince uyansın
void testWakeUpDestroyUnderneath() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);

    Body b1 = sim.createBody(Pose(Vector3(0, 0.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b1.setBodyType(Core::BodyType::Dynamic);

    Body b2 = sim.createBody(Pose(Vector3(0, 1.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b2.setBodyType(Core::BodyType::Dynamic);

    for (int i = 0; i < 90; ++i) sim.step(1.0f / 60.0f);
    CHECK_MSG(b1.isSleeping().value(), "Alt kutu uyumus olmali");
    CHECK_MSG(b2.isSleeping().value(), "Ust kutu uyumus olmali");

    // Altındaki cismi yok et
    sim.destroyBody(b1);

    // Üstteki kutu uyanmalı ve düşmeli
    sim.step(1.0f / 60.0f);
    bool wokeUp = !b2.isSleeping().value();
    std::cout << "[testWakeUpDestroyUnderneath] b2 woke up after b1 destroyed: " << (wokeUp ? "true" : "false") << std::endl;
    CHECK_MSG(wokeUp, "Altindaki cisim yok edilen kutu uyanmali");

    // Birkaç adım sonra zemine doğru düşmüş olmalı (y < 1.0)
    for (int i = 0; i < 60; ++i) sim.step(1.0f / 60.0f);
    float finalY = b2.getPosition().value().y;
    std::cout << "[testWakeUpDestroyUnderneath] final Y after fall: " << finalY << std::endl;
    CHECK_MSG(finalY < 0.7f, "Altindaki cisim yok edilen kutu serbest dusup zemine inmeli");
}

// (c) applyForce / applyLinearImpulse / setLinearVelocity uyuyan cismi uyandırsın
void testWakeUpApi() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);

    Body b = sim.createBody(Pose(Vector3(0, 0.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b.setBodyType(Core::BodyType::Dynamic);

    // Uyut
    for (int i = 0; i < 90; ++i) sim.step(1.0f / 60.0f);
    CHECK_MSG(b.isSleeping().value(), "Kutu uyumus olmali");

    // 1. applyLinearImpulse
    (void)b.applyLinearImpulse(Vector3(0, 1.0f, 0));
    CHECK_MSG(!b.isSleeping().value(), "applyLinearImpulse sonrasi uyanmali");

    // Tekrar uyut
    for (int i = 0; i < 90; ++i) sim.step(1.0f / 60.0f);
    CHECK_MSG(b.isSleeping().value(), "Kutu tekrar uyumus olmali");

    // 2. setLinearVelocity
    (void)b.setLinearVelocity(Vector3(0.5f, 0, 0));
    CHECK_MSG(!b.isSleeping().value(), "setLinearVelocity sonrasi uyanmali");

    // Tekrar uyut
    for (int i = 0; i < 90; ++i) sim.step(1.0f / 60.0f);
    CHECK_MSG(b.isSleeping().value(), "Kutu tekrar uyumus olmali");

    // 3. applyForce
    (void)b.applyForce(Vector3(0, 50.0f, 0));
    CHECK_MSG(!b.isSleeping().value(), "applyForce sonrasi uyanmali");
}

// 7. N1.5: Yanlış yere uyku kontrolü
// (a) Serbest yukarı fırlatılan cismin tepe noktasında (hız ≈ 0) uyumamalı
void testNoWrongSleepApex() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    // Düşey yukarı 10 m/s ile fırlatılan küre: t_apex = 10 / 9.81 = ~1.02 saniye (~61 adım)
    Body b = sim.createBody(Pose(Vector3(0, 0.5f, 0)), CollisionShape{SphereShape(0.5f)});
    (void)b.setBodyType(Core::BodyType::Dynamic);
    (void)b.setLinearVelocity(Vector3(0, 10.0f, 0));

    bool sleptAtAnyPoint = false;
    for (int i = 0; i < 120; ++i) { // ~2 saniye uçuş
        sim.step(1.0f / 60.0f);
        if (b.isSleeping().value()) {
            sleptAtAnyPoint = true;
            std::cout << "[testNoWrongSleepApex] mistakenly slept at step " << i << std::endl;
            break;
        }
    }
    CHECK_MSG(!sleptAtAnyPoint, "Serbest firlatilan cisim tepe noktasinda uyumamali");
}

// (b) Yuvarlanan 0.3 m/s top 0.5 sn içinde donmamalı
void testNoWrongSleepRollingBall() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);
    (void)f.setMaterial(0.0f, 0.0f); // Sürtünmesiz zemin

    Body ball = sim.createBody(Pose(Vector3(0, 0.5f, 0)), CollisionShape{SphereShape(0.5f)});
    (void)ball.setBodyType(Core::BodyType::Dynamic);
    (void)ball.setMaterial(0.0f, 0.0f);
    (void)ball.setLinearVelocity(Vector3(0.3f, 0, 0));

    // 0.5 saniye = 30 adım. 0.8 saniye (48 adım) boyunca donmamalı
    bool froze = false;
    for (int i = 0; i < 48; ++i) {
        sim.step(1.0f / 60.0f);
        if (ball.isSleeping().value()) {
            froze = true;
            std::cout << "[testNoWrongSleepRollingBall] ball froze at step " << i << std::endl;
            break;
        }
    }
    CHECK_MSG(!froze, "0.3 m/s yuvarlanan top 0.5 sn icinde donmamali");
}

// (c) 45° eğik kapsül devrilmeli (uykuya geçmeden)
void testNoWrongSleepTiltedCapsule() {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    Body f = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(100, 0.5f, 100))});
    (void)f.setBodyType(Core::BodyType::Static);

    // Kapsülü Z ekseni etrafında 45 derece döndür
    float angle = 0.785398f; // pi / 4
    Quaternion rot(Vector3(0, 0, 1), angle);
    Body c = sim.createBody(Pose(Vector3(0, 1.0f, 0), rot), CollisionShape{CapsuleShape(0.3f, 1.0f)});
    (void)c.setBodyType(Core::BodyType::Dynamic);

    bool sleptPrematurely = false;
    bool toppled = false;
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
        Quaternion currentRot = c.getOrientation().value();
        // Devrildiğinde rotasyon açısı belirgin şekilde değişmiş olmalı
        if (std::abs(currentRot.z - rot.z) > 0.2f) {
            toppled = true;
        }
        if (c.isSleeping().value() && !toppled) {
            sleptPrematurely = true;
            std::cout << "[testNoWrongSleepTiltedCapsule] slept prematurely at step " << i << " rot.z=" << currentRot.z << std::endl;
            break;
        }
    }
    CHECK_MSG(!sleptPrematurely, "45 derece egik kapsul devrilmeden once uyumamali");
    CHECK_MSG(toppled, "45 derece egik kapsul devrilmeli");
}

int main() {
    std::cout << "--- N1 UYKU TESTLERI (REPRODUCTION SUITE) ---\n";
    testDefaultBoxSleep();
    testSphereSleep();
    testUprightCapsuleSleep();
    testTowerSleep();
    testFrequencies();
    testWakeUpFastProjectile();
    testWakeUpDestroyUnderneath();
    testWakeUpApi();
    testNoWrongSleepApex();
    testNoWrongSleepRollingBall();
    testNoWrongSleepTiltedCapsule();

    return reportAndExit();
}
