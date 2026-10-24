/*
 * Baryon - A custom physics engine
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file SubsteppingTest.cpp
 * @brief Adım içi alt adımlama (Substepping / Soft-step) testleri (Faz 4.1).
 * @details Çok katlı kuleler ve uzun eklem zincirlerinde daha az iterasyonla
 *          yüksek kararlılık sağlayan alt adımlama modunu doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief Alt adımlama ayar API'lerinin doğrulanması.
 */
void testSubsteppingSettingsAPI() {
    std::cout << "[SubsteppingTest] Ayar API Testi...\n";
    Simulator sim;

    CHECK(!sim.isSubsteppingEnabled());
    CHECK(sim.getSubstepCount() == 1);

    sim.setSubstepping(true, 4);
    CHECK(sim.isSubsteppingEnabled());
    CHECK(sim.getSubstepCount() == 4);

    sim.setSubstepping(false);
    CHECK(!sim.isSubsteppingEnabled());

    std::cout << "  -> Ayar API testi basarili.\n";
}

/**
 * @brief 20 Katlı Kule Stabilitesi Testi (Faz 4.1).
 * @details 20 adet kutu üst üste dizilir. Substepping (4 alt adım) aktifken
 *          düşük çözücü iterasyonlarında (velocityIterations = 6, positionIterations = 2)
 *          bile kule devrilmeden dik ve kararlı durur.
 */
void test20StoryTowerWithSubstepping() {
    std::cout << "[SubsteppingTest] 20 Katli Kule Kararlilik Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // Düşük çözücü iterasyonu + 4 alt adım
    auto settings = sim.getSettings();
    settings.velocityIterations = 6;
    settings.positionIterations = 2;
    settings.enableSubstepping = true;
    settings.substepCount = 4;
    sim.setSettings(settings);

    // Statik Zemin
    Pose groundPose(Vector3(0.0f, -0.5f, 0.0f));
    Body ground = sim.createBody(groundPose, CollisionShape{BoxShape(Vector3(15.0f, 0.5f, 15.0f))});
    (void)ground.setBodyType(Core::BodyType::Static);

    const int storyCount = 20;
    const float boxHalf = 0.5f;
    std::vector<Body> towerBoxes;
    towerBoxes.reserve(storyCount);

    for (int i = 0; i < storyCount; ++i) {
        float y = boxHalf + static_cast<float>(i) * (boxHalf * 2.0f);
        Pose boxPose(Vector3(0.0f, y, 0.0f));
        Body box = sim.createBody(boxPose, CollisionShape{BoxShape(Vector3(boxHalf, boxHalf, boxHalf))});
        (void)box.setBodyType(Core::BodyType::Dynamic);
        (void)box.setMass(1.0f);
        towerBoxes.push_back(box);
    }

    const float dt = 1.0f / 60.0f;
    // 180 adım (3 saniye) simüle et
    for (int step = 0; step < 180; ++step) {
        sim.step(dt);
    }

    // Kulenin tepesindeki kutu devrilmemiş ve dikeyde kalmış olmalı
    Vector3 topPos = towerBoxes.back().getPosition().value();
    float expectedTopY = boxHalf + static_cast<float>(storyCount - 1) * (boxHalf * 2.0f);

    std::cout << "  -> 20. kat Y pozisyonu: " << topPos.y << " m (Beklenen: ~" << expectedTopY << " m)\n";
    CHECK(topPos.y > expectedTopY - 0.5f);
    CHECK(std::abs(topPos.x) < 0.25f);
    CHECK(std::abs(topPos.z) < 0.25f);

    // Tüm katların yatay kayması kontrol edilir
    for (int i = 0; i < storyCount; ++i) {
        Vector3 p = towerBoxes[static_cast<size_t>(i)].getPosition().value();
        CHECK(std::abs(p.x) < 0.35f);
        CHECK(std::abs(p.z) < 0.35f);
    }

    std::cout << "  -> 20 Katli Kule testi basarili.\n";
}

/**
 * @brief 30 Halkalı Uzun Zincir Stabilitesi Testi (Faz 4.1).
 * @details 30 adet mesafe kısıtıyla birbirine bağlı gövde, ucunda ağır yük taşırken
 *          substepping sayesinde aşırı sarkma veya patlama yapmadan salınır ve durur.
 */
void test30LinkChainStability() {
    std::cout << "[SubsteppingTest] 30 Halkali Zincir Kararlilik Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, -9.81f, 0.0f));

    // 4 alt adım aktif
    sim.setSubstepping(true, 4);

    const int linkCount = 30;
    const float segDist = 0.4f;

    Pose rootPose(Vector3(0.0f, 20.0f, 0.0f));
    Body prevBody = sim.createBody(rootPose, CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)prevBody.setBodyType(Core::BodyType::Static);

    std::vector<Body> links;
    links.reserve(linkCount);

    for (int i = 1; i <= linkCount; ++i) {
        Pose linkPose(Vector3(0.0f, 20.0f - static_cast<float>(i) * segDist, 0.0f));
        Body link = sim.createBody(linkPose, CollisionShape{SphereShape(0.15f)});
        (void)link.setBodyType(Core::BodyType::Dynamic);
        // En alttaki uç yükü daha ağır (5 kg), diğerleri 0.5 kg
        (void)link.setMass(i == linkCount ? 5.0f : 0.5f);

        sim.createDistanceConstraint(prevBody, link, segDist);
        links.push_back(link);
        prevBody = link;
    }

    const float dt = 1.0f / 60.0f;
    for (int step = 0; step < 240; ++step) {
        sim.step(dt);
    }

    // Toplam beklenen uzunluk: 30 * 0.4 = 12.0 m
    Vector3 rootPos = Vector3(0.0f, 20.0f, 0.0f);
    Vector3 tipPos = links.back().getPosition().value();
    float totalDist = (rootPos - tipPos).length();
    float expectedTotal = static_cast<float>(linkCount) * segDist;
    float chainDrift = std::abs(totalDist - expectedTotal);

    std::cout << "  -> 30 halkali zincir toplam uzunlugu: " << totalDist 
              << " m | Sapma: " << chainDrift << " m\n";

    // 30 halka boyunca toplam birikimli sapma 0.05 m'nin altında olmalı
    CHECK(chainDrift < 0.05f);
    CHECK(std::abs(tipPos.x) < 0.1f);
    CHECK(std::abs(tipPos.z) < 0.1f);

    std::cout << "  -> 30 Halkali Zincir testi basarili.\n";
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "Baryon Fizik Motoru - Alt Adimlama (Substepping) Testleri (Faz 4.1)\n";
    std::cout << "========================================\n";

    testSubsteppingSettingsAPI();
    test20StoryTowerWithSubstepping();
    test30LinkChainStability();

    std::cout << "Tum Alt Adimlama (Faz 4.1) testleri basariyla gecti!\n";
    return 0;
}
