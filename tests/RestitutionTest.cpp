#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

// Tek bir düşme testini koşturup apex oranını döndürür
static float runDropTest(float H, float e, bool useCCD) {
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin: y = -0.5, yarım boyut (100, 0.5, 100) -> üst yüzey y = 0
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)),
                                 CollisionShape{BoxShape(Vector3(100.0f, 0.5f, 100.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);
    (void)ground.setMaterial(0.0f, e);

    // Düşen küre: yarıçap R = 0.5
    // Zemin üstünde dinlenme merkezi: y = 0.5
    // Başlangıç merkezi: y = H + 0.5
    Body ball = sim.createBody(Pose(Vector3(0, H + 0.5f, 0)),
                               CollisionShape{SphereShape(0.5f)});
    (void)ball.setBodyType(Core::BodyType::Dynamic);
    (void)ball.setMass(1.0f);
    (void)ball.setMaterial(0.0f, e);
    (void)ball.setCCD(useCCD);

    float maxPostBounceY = 0.5f;
    bool hitGround = false;

    // Maksimum adım sayısı (H=15m için serbest düşüş ~1.75s, sıçrama ~1.75s, toplam 3.5s ~ 210 adım)
    int maxSteps = static_cast<int>(std::ceil(std::sqrt(2.0f * H / 9.81f) * 2.5f * 60.0f)) + 60;
    if (maxSteps < 120) maxSteps = 120;

    for (int step = 0; step < maxSteps; ++step) {
        sim.step(1.0f / 60.0f);
        Vector3 pos = ball.getPosition().value();
        Vector3 vel = ball.getLinearVelocity().value();

        if (!hitGround) {
            if (pos.y <= 0.51f || vel.y > 0.0f) {
                hitGround = true;
            }
        } else {
            if (pos.y > maxPostBounceY) {
                maxPostBounceY = pos.y;
            }
            if (maxPostBounceY > 0.52f && vel.y < -0.1f) {
                break;
            }
        }
    }

    float bounceHeight = maxPostBounceY - 0.5f;
    return (H > 0.0f) ? (bounceHeight / H) : 0.0f;
}

// 1. Düşme Testleri: H ∈ {1, 2, 3, 5, 8, 15}, e ∈ {0.5, 1.0}, CCD açık ve kapalı
void testRestitutionDrops() {
    const std::vector<float> heights = {1.0f, 2.0f, 3.0f, 5.0f, 8.0f, 15.0f};

    std::cout << "\n--- E = 1.0 RESTITUTION TESTLERI ---" << std::endl;
    for (float H : heights) {
        // CCD kapalı
        float ratioNoCcd = runDropTest(H, 1.0f, false);
        std::cout << "H=" << H << "m | e=1.0 | CCD=OFF -> apex orani: " << ratioNoCcd << std::endl;
        CHECK_MSG(ratioNoCcd >= 0.85f && ratioNoCcd <= 1.05f,
                  ("e=1.0 CCD=OFF H=" + std::to_string(H) + " apex orani [0.85, 1.05] arasinda olmali").c_str());

        // CCD açık
        float ratioCcd = runDropTest(H, 1.0f, true);
        std::cout << "H=" << H << "m | e=1.0 | CCD=ON  -> apex orani: " << ratioCcd << std::endl;
        CHECK_MSG(ratioCcd >= 0.85f && ratioCcd <= 1.05f,
                  ("e=1.0 CCD=ON H=" + std::to_string(H) + " apex orani [0.85, 1.05] arasinda olmali").c_str());
    }

    std::cout << "\n--- E = 0.5 RESTITUTION TESTLERI ---" << std::endl;
    for (float H : heights) {
        // CCD kapalı
        float ratioNoCcd = runDropTest(H, 0.5f, false);
        std::cout << "H=" << H << "m | e=0.5 | CCD=OFF -> apex orani: " << ratioNoCcd << std::endl;
        CHECK_MSG(ratioNoCcd >= 0.20f && ratioNoCcd <= 0.30f,
                  ("e=0.5 CCD=OFF H=" + std::to_string(H) + " apex orani [0.20, 0.30] arasinda olmali").c_str());

        // CCD açık
        float ratioCcd = runDropTest(H, 0.5f, true);
        std::cout << "H=" << H << "m | e=0.5 | CCD=ON  -> apex orani: " << ratioCcd << std::endl;
        CHECK_MSG(ratioCcd >= 0.20f && ratioCcd <= 0.30f,
                  ("e=0.5 CCD=ON H=" + std::to_string(H) + " apex orani [0.20, 0.30] arasinda olmali").c_str());
    }
}

// 2. Momentum Testi: 1 kg küre -> 1 kg kutu (v0 ∈ {4, 6, 10, 20, 100} m/s)
void testMomentumTransfer() {
    const std::vector<float> velocities = {4.0f, 6.0f, 10.0f, 20.0f, 100.0f};

    std::cout << "\n--- MOMENTUM KORUNUMU TESTLERI (1kg kure -> 1kg kutu) ---" << std::endl;
    for (float v0 : velocities) {
        for (bool useCCD : {false, true}) {
            // Yüksek hızlarda (100 m/s) CCD kapalıyken tünelleme kaçınılmazdır, bu yüzden v0 >= 20 iken CCD açık test edilir
            if (!useCCD && v0 > 10.0f) continue;

            Simulator sim;
            sim.setAccelerationField(Vector3(0, 0, 0));

            // Başlangıç: küre merkezi (-2, 0, 0), yarıçap 0.5 (sağ ucu -1.5)
            // Kutu merkezi (0, 0, 0), yarım boy 0.5 (sol yüzeyi -0.5)
            // Temas mesafesi: 1.0 m katedilmeli
            Body sphere = sim.createBody(Pose(Vector3(-2.0f, 0, 0)),
                                         CollisionShape{SphereShape(0.5f)});
            (void)sphere.setBodyType(Core::BodyType::Dynamic);
            (void)sphere.setMass(1.0f);
            (void)sphere.setMaterial(0.0f, 0.0f); // Inelastic
            (void)sphere.setCCD(useCCD);
            (void)sphere.setLinearVelocity(Vector3(v0, 0, 0));

            Body box = sim.createBody(Pose(Vector3(0, 0, 0)),
                                      CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
            (void)box.setBodyType(Core::BodyType::Dynamic);
            (void)box.setMass(1.0f);
            (void)box.setMaterial(0.0f, 0.0f);
            (void)box.setCCD(useCCD);
            (void)box.setLinearVelocity(Vector3(0, 0, 0));

            int steps = static_cast<int>(std::ceil((2.5f / v0) * 60.0f)) + 15;
            if (steps < 15) steps = 15;
            if (steps > 120) steps = 120;

            for (int s = 0; s < steps; ++s) {
                sim.step(1.0f / 60.0f);
            }

            float boxVx = box.getLinearVelocity().value().x;
            float expected = v0 * 0.5f;
            float ratio = boxVx / v0;
            std::cout << "v0=" << v0 << " m/s | CCD=" << (useCCD ? "ON " : "OFF")
                      << " -> kutu Vx: " << boxVx << " (beklenen ~" << expected
                      << ", oran: " << ratio << ")" << std::endl;

            CHECK_MSG(boxVx >= expected * 0.92f && boxVx <= expected * 1.08f,
                      ("v0=" + std::to_string(v0) + " CCD=" + (useCCD ? "ON" : "OFF") + " kutu hizi v0/2 (+-%8) olmali").c_str());
        }
    }
}

// 3. Faz 4.4: Restitution Damping ve Apeks Ayrışım Testi (e=0.8, H=5.0m)
void testRestitutionDampingBreakdown() {
    std::cout << "\n--- FAZ 4.4 RESTITUTION DAMPING AYRISIM TESTI (e=0.8, H=5.0m) ---" << std::endl;

    auto measureApex = [](float damping) -> float {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));

        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)),
                                     CollisionShape{BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);
        (void)ground.setMaterial(0.0f, 0.8f);

        Body ball = sim.createBody(Pose(Vector3(0, 5.5f, 0)),
                                   CollisionShape{SphereShape(0.5f)});
        (void)ball.setBodyType(Core::BodyType::Dynamic);
        (void)ball.setMass(1.0f);
        (void)ball.setMaterial(0.0f, 0.8f);
        (void)ball.setCCD(false);
        (void)ball.setDamping(damping, 0.0f);

        float peak1 = 0.0f;
        bool wasMovingDown = true;
        for (int step = 0; step < 200; ++step) {
            sim.step(1.0f / 60.0f);
            Vector3 pos = ball.getPosition().value();
            Vector3 vel = ball.getLinearVelocity().value();

            if (vel.y > 0.05f) {
                wasMovingDown = false;
            } else if (!wasMovingDown && vel.y <= 0.0f) {
                peak1 = pos.y - 0.5f;
                break;
            }
        }
        return peak1;
    };

    float apexDamped = measureApex(0.05f); // Varsayılan linearDamping
    float apexUndamped = measureApex(0.0f); // Sıfır sönümleme

    std::cout << "  e=0.8, H=5.0m | Damping=0.05 (Varsayilan): " << apexDamped << " m (Beklenen ~3.11 m)\n";
    std::cout << "  e=0.8, H=5.0m | Damping=0.00 (Sifir Sonumleme): " << apexUndamped << " m (Beklenen ~3.29 m, Analitik 3.20 m)\n";

    // 1. Varsayılan sönümleme 3.10 m baseline civarında olmalıdır
    CHECK_MSG(apexDamped >= 3.05f && apexDamped <= 3.17f,
              "Varsayilan damping=0.05 ile apex 3.05 - 3.17 m arasinda olmali");

    // 2. Sıfır sönümleme analitik 3.20m değerine yakın olmalı ve sönümlüden kesinlikle yüksek olmalıdır
    CHECK_MSG(apexUndamped >= 3.20f && apexUndamped <= 3.35f,
              "Sifir damping ile apex analitik 3.20 m civarinda (3.20 - 3.35) olmali");
    CHECK_MSG(apexUndamped > apexDamped + 0.15f,
              "Sifir damping apeksi, sonumlu apexten en az 0.15 m daha yuksek olmali");
}

// 4. Faz 4.4: Restitution Hız Eşiği Testi (RestitutionVelocityThreshold = 1.0 m/s)
void testRestitutionVelocityThreshold() {
    std::cout << "\n--- FAZ 4.4 RESTITUTION HIZ ESIGI TESTI (Threshold = 1.0 m/s) ---" << std::endl;

    auto testDropVelocityThreshold = [](float dropHeight, float restitution) -> float {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));

        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)),
                                     CollisionShape{BoxShape(Vector3(50.0f, 0.5f, 50.0f))});
        (void)ground.setBodyType(Core::BodyType::Static);
        (void)ground.setMaterial(0.0f, restitution);

        Body ball = sim.createBody(Pose(Vector3(0, dropHeight + 0.5f, 0)),
                                   CollisionShape{SphereShape(0.5f)});
        (void)ball.setBodyType(Core::BodyType::Dynamic);
        (void)ball.setMass(1.0f);
        (void)ball.setMaterial(0.0f, restitution);
        (void)ball.setDamping(0.0f, 0.0f);

        float maxRebound = 0.0f;
        bool contacted = false;
        for (int step = 0; step < 90; ++step) {
            sim.step(1.0f / 60.0f);
            Vector3 pos = ball.getPosition().value();
            Vector3 vel = ball.getLinearVelocity().value();

            if (!contacted && pos.y <= 0.51f) {
                contacted = true;
            }
            if (contacted && vel.y > 0.01f) {
                float rebound = pos.y - 0.5f;
                if (rebound > maxRebound) maxRebound = rebound;
            }
        }
        return maxRebound;
    };

    // H = 0.03m -> v = sqrt(2 * 9.81 * 0.03) = 0.767 m/s < 1.0 m/s (Eşiğin altında, sekmemeli)
    float lowDropRebound = testDropVelocityThreshold(0.03f, 0.8f);
    std::cout << "  H=0.03m (v~0.77 m/s < 1.0 m/s) -> Rebound: " << lowDropRebound << " m\n";
    CHECK_MSG(lowDropRebound < 0.005f, "Esik altindaki carpismada sekme olmamalidir (rebound ~0)");

    // H = 0.30m -> v = sqrt(2 * 9.81 * 0.30) = 2.426 m/s > 1.0 m/s (Eşiğin üstünde, sekmelidir)
    float highDropRebound = testDropVelocityThreshold(0.30f, 0.8f);
    std::cout << "  H=0.30m (v~2.43 m/s > 1.0 m/s) -> Rebound: " << highDropRebound << " m\n";
    CHECK_MSG(highDropRebound > 0.10f, "Esik ustundeki carpismada sekme gerceklesmelidir");
}

int main() {
    std::cout << "=== N2 RESTITUTION VE CCD REPRO TEST SUITE ===" << std::endl;
    testRestitutionDrops();
    testMomentumTransfer();
    testRestitutionDampingBreakdown();
    testRestitutionVelocityThreshold();
    return reportAndExit();
}

