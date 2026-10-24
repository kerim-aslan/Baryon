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
 * @file SpeculativeContactsTest.cpp
 * @brief Spekülatif Kontak (Speculative Contacts) testleri (Faz 4.2).
 * @details TOI root-finding gerektirmeyen, negatif penetrasyon ve hız önyargısı (bias)
 *          ile çalışan öngörülü temas mekanizmasının tünelleme önleme performansını doğrular.
 */

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

namespace {

/**
 * @brief Spekülatif kontak yapılandırma API'lerinin doğrulanması.
 */
void testSpeculativeContactsSettingsAPI() {
    std::cout << "[SpeculativeContactsTest] Ayar API Testi...\n";
    Simulator sim;

    CHECK(!sim.isSpeculativeContactsEnabled());
    sim.setSpeculativeContacts(true, 20.0f);
    CHECK(sim.isSpeculativeContactsEnabled());
    CHECK(std::abs(sim.getSettings().maxSpeculativeDistance - 20.0f) < 1e-4f);

    sim.setSpeculativeContacts(false);
    CHECK(!sim.isSpeculativeContactsEnabled());

    // Gövde düzeyinde API testi
    Pose p0(Vector3(0.0f, 0.0f, 0.0f));
    auto body = sim.createBody(p0, CollisionShape{SphereShape(0.2f)});
    (void)body.setMass(1.0f);

    auto hasSpec = body.hasSpeculativeContacts();
    CHECK(hasSpec.has_value() && !hasSpec.value());

    auto res = body.setSpeculativeContacts(true);
    CHECK(res.has_value());

    hasSpec = body.hasSpeculativeContacts();
    CHECK(hasSpec.has_value() && hasSpec.value());

    std::cout << "  -> Ayar API testi basarili.\n";
}

/**
 * @brief Mermi vs İnce Duvar: Spekülatif kontak ile tünellemenin önlenmesi.
 * @details 500 m/s hızla giden küre mermi, CCD kapalıyken 0.1m kalınlığındaki
 *          ince duvarı delip geçmeden duvar yüzeyinde durmalıdır.
 */
void testBulletTunnelingPrevented() {
    std::cout << "[SpeculativeContactsTest] Mermi vs Ince Duvar (Spekulatif Kontak ACIK)...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f)); // Yerçekimsiz saf balistik ortam

    // 0.1m kalınlığında ince duvar (x = 10.0m, kalınlık yarıçapı = 0.05m -> Ön yüzey x = 9.95m)
    Pose wallPose(Vector3(10.0f, 0.0f, 0.0f));
    Body wall = sim.createBody(wallPose, CollisionShape{BoxShape(Vector3(0.05f, 5.0f, 5.0f))});
    (void)wall.setBodyType(Core::BodyType::Static);

    // 500 m/s hızında mermi (yarıçap = 0.1m, başlangıç x = 0.0m)
    Pose bulletPose(Vector3(0.0f, 0.0f, 0.0f));
    Body bullet = sim.createBody(bulletPose, CollisionShape{SphereShape(0.1f)});
    (void)bullet.setMass(1.0f);
    (void)bullet.setLinearVelocity(Vector3(500.0f, 0.0f, 0.0f));

    // CCD'yi devre dışı bırak, yalnızca Spekülatif Kontak aç
    bullet.setCCD(false);
    bullet.setSpeculativeContacts(true);

    const float dt = 1.0f / 60.0f;
    // 5 adım simüle et (5 * 1/60s = 0.083s -> Serbest uçuşta 41.6 metre yol alırdı)
    for (int step = 0; step < 5; ++step) {
        sim.step(dt);
        Vector3 pos = bullet.getPosition().value();
        // Mermi asla duvarın arka tarafına (x > 10.05m) tünellememeli!
        CHECK(pos.x <= 10.0f);
    }

    Vector3 finalPos = bullet.getPosition().value();
    std::cout << "  -> Mermi durma konumu x: " << finalPos.x << " m (Duvar on yuzu: 9.95m)\n";
    // Merminin yüzeyde durduğunu doğrula (Küre yarıçapı 0.1m olduğundan merkez ~9.85m civarında olmalıdır)
    CHECK(finalPos.x >= 9.70f && finalPos.x <= 10.0f);
    std::cout << "  -> Mermi tunelleme onleme testi basarili.\n";
}

/**
 * @brief Karşı-Test: Spekülatif kontak KAPALIYKEN merminin duvardan tünellemesi.
 * @details Aynı 500 m/s mermi CCD ve Spekülatif Kontak kapalıyken duvardan tamamen geçer (x > 15m).
 */
void testTunnelingOccursWithoutSpeculativeContacts() {
    std::cout << "[SpeculativeContactsTest] Karsi-Test: Spekulatif Kontak KAPALI (Tunelleme Dogrulamasi)...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose wallPose(Vector3(10.0f, 0.0f, 0.0f));
    Body wall = sim.createBody(wallPose, CollisionShape{BoxShape(Vector3(0.05f, 5.0f, 5.0f))});
    (void)wall.setBodyType(Core::BodyType::Static);

    Pose bulletPose(Vector3(0.0f, 0.0f, 0.0f));
    Body bullet = sim.createBody(bulletPose, CollisionShape{SphereShape(0.1f)});
    (void)bullet.setMass(1.0f);
    (void)bullet.setLinearVelocity(Vector3(500.0f, 0.0f, 0.0f));

    bullet.setCCD(false);
    bullet.setSpeculativeContacts(false);

    const float dt = 1.0f / 60.0f;
    // 2 adım simüle et (2 * 500 * 1/60 = 16.67m -> Duvarı delip geçer)
    sim.step(dt);
    sim.step(dt);

    Vector3 pos = bullet.getPosition().value();
    std::cout << "  -> Spekulatif kontak yokken mermi konumu x: " << pos.x << " m\n";
    CHECK(pos.x > 10.5f); // Duvarı tamamen aşmış olmalıdır
    std::cout << "  -> Karsi-test basarili (Tunelleme dogrulandi).\n";
}

/**
 * @brief Genel (World) Spekülatif Kontak Yapılandırması Testi.
 * @details sim.setSpeculativeContacts(true) çağrıldığında tek tek gövdelere ayar yapmadan
 *          hızlı cisimlerin duvara çarpması kontrol edilir.
 */
void testGlobalSpeculativeSetting() {
    std::cout << "[SpeculativeContactsTest] Genel Dunya Ayari Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));
    sim.setSpeculativeContacts(true); // Dünya genelinde aktif

    Pose wallPose(Vector3(8.0f, 0.0f, 0.0f));
    Body wall = sim.createBody(wallPose, CollisionShape{BoxShape(Vector3(0.1f, 5.0f, 5.0f))});
    (void)wall.setBodyType(Core::BodyType::Static);

    Pose bulletPose(Vector3(0.0f, 0.0f, 0.0f));
    Body bullet = sim.createBody(bulletPose, CollisionShape{SphereShape(0.15f)});
    (void)bullet.setMass(1.0f);
    (void)bullet.setLinearVelocity(Vector3(400.0f, 0.0f, 0.0f));

    bullet.setCCD(false);

    const float dt = 1.0f / 60.0f;
    for (int step = 0; step < 4; ++step) {
        sim.step(dt);
    }

    Vector3 finalPos = bullet.getPosition().value();
    std::cout << "  -> Genel ayar mermi konumu x: " << finalPos.x << " m\n";
    CHECK(finalPos.x <= 8.0f);
    CHECK(finalPos.x >= 7.5f);
    std::cout << "  -> Genel dunya ayari testi basarili.\n";
}

/**
 * @brief Snapshot ve Geri Yükleme Determinizm Testi.
 */
void testSnapshotDeterminism() {
    std::cout << "[SpeculativeContactsTest] Snapshot Determinizm Testi...\n";
    Simulator sim;
    sim.setAccelerationField(Vector3(0.0f, 0.0f, 0.0f));

    Pose wallPose(Vector3(10.0f, 0.0f, 0.0f));
    Body wall = sim.createBody(wallPose, CollisionShape{BoxShape(Vector3(0.1f, 5.0f, 5.0f))});
    (void)wall.setBodyType(Core::BodyType::Static);

    Pose bulletPose(Vector3(0.0f, 0.0f, 0.0f));
    Body bullet = sim.createBody(bulletPose, CollisionShape{SphereShape(0.1f)});
    (void)bullet.setMass(1.0f);
    (void)bullet.setLinearVelocity(Vector3(300.0f, 0.0f, 0.0f));
    bullet.setCCD(false);
    bullet.setSpeculativeContacts(true);

    const float dt = 1.0f / 60.0f;
    sim.step(dt); // 1 adım ilerlet

    auto snapshot = sim.saveSnapshot();

    // 5 adım ilerlet
    for (int i = 0; i < 5; ++i) sim.step(dt);
    Vector3 run1Pos = bullet.getPosition().value();

    // Snapshot geri yükle ve tekrar 5 adım ilerlet
    sim.restoreSnapshot(snapshot);
    for (int i = 0; i < 5; ++i) sim.step(dt);
    Vector3 run2Pos = bullet.getPosition().value();

    CHECK(std::abs(run1Pos.x - run2Pos.x) < 1e-5f);
    CHECK(std::abs(run1Pos.y - run2Pos.y) < 1e-5f);
    CHECK(std::abs(run1Pos.z - run2Pos.z) < 1e-5f);
    std::cout << "  -> Snapshot determinizm testi basarili.\n";
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "   Spekulatif Kontak Testleri (Faz 4.2) \n";
    std::cout << "========================================\n";

    testSpeculativeContactsSettingsAPI();
    testBulletTunnelingPrevented();
    testTunnelingOccursWithoutSpeculativeContacts();
    testGlobalSpeculativeSetting();
    testSnapshotDeterminism();

    std::cout << "\n[TUM SPEKULATIF KONTAK TESTLERI BASARIYLA TAMAMLANDI!]\n";
    return 0;
}
