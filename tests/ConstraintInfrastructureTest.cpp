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

#include "TestCommon.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/DistanceConstraint.hpp>
#include <Baryon/collision/RevoluteConstraint.hpp>
#include <iostream>
#include <cmath>

using namespace Baryon;
using namespace Baryon::collision;

// Test 1: Ölü Varlık (Dead Entity) Eklem Temizliği
void testDeadEntityCleanup() {
    std::cout << "--- 1. Test: Olu Varlik (Dead Entity) Eklem Temizligi ---" << std::endl;
    Simulator sim;

    // Senaryo A: B cismi yok edildiğinde A üzerindeki DistanceConstraint temizlenmeli
    {
        Body a = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.5f)});
        Body b = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{SphereShape(0.5f)});
        (void)a.setBodyType(Core::BodyType::Static);
        (void)b.setBodyType(Core::BodyType::Dynamic);

        sim.createDistanceConstraint(a, b, 2.0f);
        CHECK_MSG(sim.getRegistry().hasComponent<DistanceConstraint>(a.getEntity()),
                  "A cismi baslangicta DistanceConstraint bilesenine sahip olmali");

        // B cismini yok et
        sim.destroyBody(b);

        CHECK_MSG(!sim.getRegistry().isAlive(b.getEntity()), "B cismi yok edilmis olmali");
        CHECK_MSG(!sim.getRegistry().hasComponent<DistanceConstraint>(a.getEntity()),
                  "B yok edildiginde A uzerindeki eklem otomatik temizlenmeli");
        CHECK_MSG(sim.getRegistry().isAlive(a.getEntity()), "A cismi hala yasiyor olmali");
    }

    // Senaryo B: A cismi yok edildiğinde B canlı kalmalı ve eklem kalmamalı
    {
        Body a = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.5f)});
        Body b = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{SphereShape(0.5f)});
        sim.createDistanceConstraint(a, b, 2.0f);

        sim.destroyBody(a);

        CHECK_MSG(!sim.getRegistry().isAlive(a.getEntity()), "A cismi yok edilmis olmali");
        CHECK_MSG(sim.getRegistry().isAlive(b.getEntity()), "B cismi yasiyor olmali");
        CHECK_MSG(!sim.getRegistry().hasComponent<DistanceConstraint>(b.getEntity()),
                  "B uzerinde asili eklem bileseni kalmamali");
    }

    // Senaryo C: RevoluteConstraint için temizlik kontrolü
    {
        Body a = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        Body b = sim.createBody(Pose(Vector3(1, 5, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
        sim.createRevoluteConstraint(a, b, Vector3(0.5f, 0, 0), Vector3(-0.5f, 0, 0),
                                     Vector3(0, 1, 0), Vector3(0, 1, 0));

        CHECK_MSG(sim.getRegistry().hasComponent<RevoluteConstraint>(a.getEntity()),
                  "A cismi RevoluteConstraint bilesenine sahip olmali");

        sim.destroyBody(b);

        CHECK_MSG(!sim.getRegistry().hasComponent<RevoluteConstraint>(a.getEntity()),
                  "B silindiginde A uzerindeki RevoluteConstraint temizlenmeli");
    }

    // Senaryo D: Geri dönüştürülen slot ile yeni varlık eklendiğinde çökme / hayalet bağ olmamalı
    {
        Body c1 = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.5f)});
        Body c2 = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{SphereShape(0.5f)});
        sim.createDistanceConstraint(c1, c2, 2.0f);

        sim.destroyBody(c2);

        // Yeni bir cisim oluştur (aynı slot ID'sini alabilir)
        Body c3 = sim.createBody(Pose(Vector3(50, 50, 50)), CollisionShape{SphereShape(0.5f)});
        (void)c3.setBodyType(Core::BodyType::Dynamic);

        // 60 adım simüle et, c1 ve c3 arasında hiçbir hayalet kuvvet olmamalı
        for (int i = 0; i < 60; ++i) {
            sim.step(1.0f / 60.0f);
        }

        Vector3 pC3 = sim.getRegistry().getComponent<Pose>(c3.getEntity()).position;
        CHECK_MSG(!std::isnan(pC3.x) && !std::isnan(pC3.y) && !std::isnan(pC3.z),
                  "Recycled slot sonrasi pozisyon NaN olmamali");
    }
    std::cout << "  -> PASSED: Olu varlik eklem temizligi basarili." << std::endl;
}

// Test 2: Ada Entegrasyonu ve Uyku/Uyanma Kuralları
void testIslandSleepWake() {
    std::cout << "--- 2. Test: Ada Entegrasyonu ve Uyku/Uyanma Kurallari ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));
    sim.setSleepThresholds(0.05f, 0.05f, 0.2f);

    // Zemin
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape{BoxShape(Vector3(20, 0.5f, 20))});
    (void)ground.setBodyType(Core::BodyType::Static);

    // İki dinamik kutu, birbirine mesafe eklemiyle bağlı
    Body b1 = sim.createBody(Pose(Vector3(-1.0f, 0.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    Body b2 = sim.createBody(Pose(Vector3(1.0f, 0.5f, 0)), CollisionShape{BoxShape(Vector3(0.5f, 0.5f, 0.5f))});
    (void)b1.setBodyType(Core::BodyType::Dynamic);
    (void)b2.setBodyType(Core::BodyType::Dynamic);
    (void)b1.setMass(1.0f);
    (void)b2.setMass(1.0f);

    sim.createDistanceConstraint(b1, b2, 2.0f);

    // Cisimlerin dinlenip uykuya geçmesi için simüle et
    for (int i = 0; i < 180; ++i) {
        sim.step(1.0f / 60.0f);
    }

    bool b1Sleep = sim.getRegistry().getComponent<Core::BodyState>(b1.getEntity()).isSleeping;
    bool b2Sleep = sim.getRegistry().getComponent<Core::BodyState>(b2.getEntity()).isSleeping;

    CHECK_MSG(b1Sleep, "b1 cismi uykuya gecmis olmali");
    CHECK_MSG(b2Sleep, "b2 cismi uykuya gecmis olmali");

    // Sadece b1'e bir darbe (impulse) uygula
    (void)b1.applyLinearImpulse(Vector3(5.0f, 0, 0));

    // 1 adım ilerlet
    sim.step(1.0f / 60.0f);

    // Ada kuralı: Bağlı cisimlerden biri uyanırsa adadaki tüm cisimler uyanmalıdır!
    bool b1Awake = !sim.getRegistry().getComponent<Core::BodyState>(b1.getEntity()).isSleeping;
    bool b2Awake = !sim.getRegistry().getComponent<Core::BodyState>(b2.getEntity()).isSleeping;

    CHECK_MSG(b1Awake, "b1 darbe sonrasi uyanmali");
    CHECK_MSG(b2Awake, "b2 eklem baglantisi uzerinden ada dalgasiyla uyanmali");

    std::cout << "  -> PASSED: Eklem baglantili ada uyku/uyanma yayilimi dogrulandi." << std::endl;
}

// Test 3: Snapshot ve Geri Yükleme Determinizmi
void testSnapshotDeterminism() {
    std::cout << "--- 3. Test: Eklem Iceren Sahnede Snapshot Determinizmi ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body anchor = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{BoxShape(Vector3(0.2f, 0.2f, 0.2f))});
    (void)anchor.setBodyType(Core::BodyType::Static);

    Body bob = sim.createBody(Pose(Vector3(2.0f, 10.0f, 0)), CollisionShape{SphereShape(0.3f)});
    (void)bob.setBodyType(Core::BodyType::Dynamic);
    (void)bob.setMass(2.0f);

    sim.createDistanceConstraint(anchor, bob, 2.0f);

    // 60 adım ilerletip serbest salınım başlat
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
    }

    // Snapshot kaydet
    Core::Snapshot snap = sim.saveSnapshot();

    // 60 adım daha ilerlet ve yörüngeyi kaydet (Koşu 1)
    std::vector<Vector3> trajectory1;
    trajectory1.reserve(60);
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        trajectory1.push_back(sim.getRegistry().getComponent<Pose>(bob.getEntity()).position);
    }

    // Snapshot'ı geri yükle
    sim.restoreSnapshot(snap);

    // Tekrar 60 adım ilerlet (Koşu 2)
    std::vector<Vector3> trajectory2;
    trajectory2.reserve(60);
    for (int i = 0; i < 60; ++i) {
        sim.step(1.0f / 60.0f);
        trajectory2.push_back(sim.getRegistry().getComponent<Pose>(bob.getEntity()).position);
    }

    // Birebir eşleşmeyi kontrol et
    for (size_t i = 0; i < 60; ++i) {
        CHECK_MSG(trajectory1[i].x == trajectory2[i].x &&
                  trajectory1[i].y == trajectory2[i].y &&
                  trajectory1[i].z == trajectory2[i].z,
                  "Snapshot sonrasi trajektori birebir deterministik olmali");
    }

    std::cout << "  -> PASSED: Snapshot ve geri yukleme 60 adim boyunca bit-for-bit eslesti." << std::endl;
}

// Test 4: Dejenere ve Sınır Durum Girdileri
void testDegenerateInputs() {
    std::cout << "--- 4. Test: Dejenere ve Sinir Durum Girdileri ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Sıfır mesafeli eklem
    {
        Body a = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{SphereShape(0.2f)});
        Body b = sim.createBody(Pose(Vector3(0, 5, 0)), CollisionShape{SphereShape(0.2f)});
        sim.createDistanceConstraint(a, b, 0.0f);
        for (int i = 0; i < 10; ++i) {
            sim.step(1.0f / 60.0f);
        }
        Vector3 pa = sim.getRegistry().getComponent<Pose>(a.getEntity()).position;
        CHECK_MSG(!std::isnan(pa.x) && !std::isnan(pa.y), "Sifir mesafeli eklem NaN uretmemeli");
    }

    // Çok büyük mesafeli eklem (1e6)
    {
        Body a = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape{SphereShape(0.2f)});
        Body b = sim.createBody(Pose(Vector3(10, 0, 0)), CollisionShape{SphereShape(0.2f)});
        sim.createDistanceConstraint(a, b, 1e6f);
        for (int i = 0; i < 10; ++i) {
            sim.step(1.0f / 60.0f);
        }
        Vector3 pa = sim.getRegistry().getComponent<Pose>(a.getEntity()).position;
        CHECK_MSG(!std::isnan(pa.x) && !std::isnan(pa.y), "Asiri buyuk mesafeli eklem NaN uretmemeli");
    }

    // isEnabled bayrağı kontrolü
    {
        Body anchor = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape{SphereShape(0.2f)});
        (void)anchor.setBodyType(Core::BodyType::Static);
        Body bob = sim.createBody(Pose(Vector3(0, 8, 0)), CollisionShape{SphereShape(0.2f)});
        (void)bob.setBodyType(Core::BodyType::Dynamic);
        (void)bob.setMass(1.0f);

        sim.createDistanceConstraint(anchor, bob, 2.0f);
        auto& dc = sim.getRegistry().getComponent<DistanceConstraint>(anchor.getEntity());
        dc.isEnabled = false; // Devre dışı bırak

        for (int i = 0; i < 60; ++i) {
            sim.step(1.0f / 60.0f);
        }

        Vector3 pBob = sim.getRegistry().getComponent<Pose>(bob.getEntity()).position;
        // Serbest düşüşte olmalı (y < 8.0 - 2.0)
        CHECK_MSG(pBob.y < 5.0f, "Devre disi birakilan eklem serbest dususu engellememeli");
    }

    std::cout << "  -> PASSED: Dejenere ve sinir durum girdileri guvenli." << std::endl;
}

int main() {
    std::cout << "=== BARYON FAZ 3.1: GENEL KISIT ALTYAPISI TESTLERI ===\n" << std::endl;

    testDeadEntityCleanup();
    testIslandSleepWake();
    testSnapshotDeterminism();
    testDegenerateInputs();

    return reportAndExit();
}
