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
#include "harness/Metrics.hpp"
#include <Baryon/Simulator.hpp>
#include <Baryon/collision/CollisionShape.hpp>
#include <iostream>
#include <cmath>
#include <vector>

using namespace Baryon;
using namespace Baryon::collision;

// 1. İki özdeş kutudan "dambıl": birleşik kütle = toplam, kütle merkezi analitik değerde (±1e-4), atalet tensörü analitik formülle ±%0.1
void testDumbbellInertiaAndCoM() {
    std::cout << "\n--- 1. Compound Dumbbell Inertia & CoM Test ---" << std::endl;
    Simulator sim;

    // İki 1x1x1 birim küp (half-extents 0.5), X ekseninde -2.0 ve +2.0 konumlarında
    BoxShape box1(Vector3(0.5f, 0.5f, 0.5f));
    BoxShape box2(Vector3(0.5f, 0.5f, 0.5f));

    CompoundShape dumbbell;
    dumbbell.children.push_back(CompoundChild(CollisionShape(box1), Pose(Vector3(-2.0f, 0.0f, 0.0f))));
    dumbbell.children.push_back(CompoundChild(CollisionShape(box2), Pose(Vector3(2.0f, 0.0f, 0.0f))));

    Vector3 com = dumbbell.computeCenterOfMass();
    std::cout << "Dumbbell CoM: (" << com.x << ", " << com.y << ", " << com.z << ")" << std::endl;
    CHECK_MSG(std::abs(com.x) < 1e-4f && std::abs(com.y) < 1e-4f && std::abs(com.z) < 1e-4f,
              "Dumbbell kutle merkezi (0,0,0) olmali (hata < 1e-4)");

    // Toplam kütle 2.0 kg (her kutu 1.0 kg)
    float totalMass = 2.0f;
    Matrix3x3 I = dumbbell.computeInertiaTensor(totalMass);

    // Her kutunun kendi merkezindeki ataleti: I_box = (1/6) * m = 1/6
    // Paralel eksen: d = 2.0 -> m * d^2 = 1.0 * 4.0 = 4.0
    // I_xx = 2 * (1/6) = 1/3 ~= 0.33333f
    // I_yy = 2 * (1/6 + 4.0) = 2 * (25/6) = 25/3 ~= 8.33333f
    // I_zz = 2 * (1/6 + 4.0) = 25/3 ~= 8.33333f
    float expectedIxx = 1.0f / 3.0f;
    float expectedIyy = 25.0f / 3.0f;
    float expectedIzz = 25.0f / 3.0f;

    std::cout << "Dumbbell Ixx: " << I[0].x << " (Expected: " << expectedIxx << ")" << std::endl;
    std::cout << "Dumbbell Iyy: " << I[1].y << " (Expected: " << expectedIyy << ")" << std::endl;
    std::cout << "Dumbbell Izz: " << I[2].z << " (Expected: " << expectedIzz << ")" << std::endl;

    float errX = std::abs(I[0].x - expectedIxx) / expectedIxx;
    float errY = std::abs(I[1].y - expectedIyy) / expectedIyy;
    float errZ = std::abs(I[2].z - expectedIzz) / expectedIzz;

    CHECK_MSG(errX < 0.001f, "Ixx analitik formul ile +-0.1% icinde olmali");
    CHECK_MSG(errY < 0.001f, "Iyy analitik formul ile +-0.1% icinde olmali");
    CHECK_MSG(errZ < 0.001f, "Izz analitik formul ile +-0.1% icinde olmali");

    // Simulator'a ekleyip MassProps doğrulama
    Body body = sim.createBody(Pose(Vector3(0, 10, 0)), CollisionShape(dumbbell));
    (void)body.setBodyType(Core::BodyType::Dynamic);
    (void)body.setMass(totalMass);

    Matrix3x3 bodyI = body.getLocalInertiaTensor().value();
    CHECK_MSG(std::abs(bodyI[0].x - expectedIxx) / expectedIxx < 0.001f, "Body LocalInertiaTensor Ixx eslesmeli");
    CHECK_MSG(std::abs(bodyI[1].y - expectedIyy) / expectedIyy < 0.001f, "Body LocalInertiaTensor Iyy eslesmeli");
    CHECK_MSG(std::abs(bodyI[2].z - expectedIzz) / expectedIzz < 0.001f, "Body LocalInertiaTensor Izz eslesmeli");
}

// 2. L şekli (iki kutu), zeminde 5 sn: dinlenme yüksekliği analitik değerle ±0.02 m, uyuyabiliyor
void testLShapedCompoundResting() {
    std::cout << "\n--- 2. L-Shaped Compound Resting & Sleep Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    // Zemin: y = 0 üst yüzeyi
    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape(BoxShape(Vector3(50, 0.5f, 50))));
    (void)ground.setBodyType(Core::BodyType::Static);

    // L şekli: Yatay taban kutusu (2x0.5x1 -> half 1.0, 0.25, 0.5) merkez (0, 0.25, 0)
    // Dikey bacak kutusu (0.5x1.5x1 -> half 0.25, 0.75, 0.5) merkez (0.75, 1.25, 0)
    CompoundShape lShape;
    lShape.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(1.0f, 0.25f, 0.5f))), Pose(Vector3(0.0f, 0.25f, 0.0f))));
    lShape.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(0.25f, 0.75f, 0.5f))), Pose(Vector3(0.75f, 1.25f, 0.0f))));

    // Cisim y=2.0'den bırakılır
    Body body = sim.createBody(Pose(Vector3(0, 2.0f, 0)), CollisionShape(lShape));
    (void)body.setBodyType(Core::BodyType::Dynamic);

    // 5 saniye = 300 adım
    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Vector3 finalPos = body.getPosition().value();
    bool isSleeping = body.isSleeping().value();
    std::cout << "L-Shape Final Pos: (" << finalPos.x << ", " << finalPos.y << ", " << finalPos.z 
              << ") | Sleeping: " << (isSleeping ? "YES" : "NO") << std::endl;

    // Taban kutusu altı y=0'da dinlenmeli. Yerel taban kutusu y=0'dan başladığı için
    // Cismin orijini y ≈ 0 civarında olmalı (dinlenme yüksekliği analitik ±0.02m)
    CHECK_MSG(std::abs(finalPos.y - 0.0f) <= 0.02f, "L-sekli dinlenme yuksekligi analitik degerle +-0.02 m icinde olmali");
    CHECK_MSG(isSleeping, "L-sekli 5 saniye sonra uykuya gecebilmeli");
}

// 3. T şekli yüksekten düşürülünce stabil bir yönelimde durur, enerji artmaz
void testTShapedDropStability() {
    std::cout << "\n--- 3. T-Shaped Drop & Energy Stability Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape(BoxShape(Vector3(50, 0.5f, 50))));
    (void)ground.setBodyType(Core::BodyType::Static);

    // T şekli: Üst yatay çubuk + alt dikey bacak
    CompoundShape tShape;
    tShape.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(1.5f, 0.25f, 0.5f))), Pose(Vector3(0.0f, 1.5f, 0.0f))));
    tShape.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(0.25f, 0.75f, 0.5f))), Pose(Vector3(0.0f, 0.75f, 0.0f))));

    Body tBody = sim.createBody(Pose(Vector3(0, 5.0f, 0)), CollisionShape(tShape));
    (void)tBody.setBodyType(Core::BodyType::Dynamic);

    float initialEnergy = harness::computeTotalEnergy(sim);
    float maxEnergyObserved = initialEnergy;

    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
        float currentE = harness::computeTotalEnergy(sim);
        if (currentE > maxEnergyObserved) {
            maxEnergyObserved = currentE;
        }
    }

    std::cout << "Initial Energy: " << initialEnergy << " J | Max Energy Observed: " << maxEnergyObserved << " J" << std::endl;
    // Enerji artmamalı (sayısal dalgalanma payı < %2)
    CHECK_MSG(maxEnergyObserved <= initialEnergy * 1.02f, "T-sekli carpisma sirasinda yapay enerji uretmemeli");

    Vector3 finalPos = tBody.getPosition().value();
    CHECK_MSG(!std::isnan(finalPos.x) && !std::isnan(finalPos.y) && !std::isnan(finalPos.z), "T-sekli NaN pozisyon uretmemeli");
}

// 4. Compound küre-küre çarpışması, tek küre ile aynı sonucu verir (çocuk=tek küre, offset 0)
void testCompoundSphereMatchesSingleSphere() {
    std::cout << "\n--- 4. Compound Sphere Equivalence Test ---" << std::endl;

    // Koşu 1: Tekil küre
    Vector3 finalPosSingle;
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));
        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape(BoxShape(Vector3(20, 0.5f, 20))));
        (void)ground.setBodyType(Core::BodyType::Static);

        Body ball = sim.createBody(Pose(Vector3(0, 5.0f, 0)), CollisionShape(SphereShape(0.5f)));
        (void)ball.setBodyType(Core::BodyType::Dynamic);

        for (int i = 0; i < 180; ++i) {
            sim.step(1.0f / 60.0f);
        }
        finalPosSingle = ball.getPosition().value();
    }

    // Koşu 2: Compound tek küre (offset 0)
    Vector3 finalPosCompound;
    {
        Simulator sim;
        sim.setAccelerationField(Vector3(0, -9.81f, 0));
        Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape(BoxShape(Vector3(20, 0.5f, 20))));
        (void)ground.setBodyType(Core::BodyType::Static);

        CompoundShape compSphere;
        compSphere.children.push_back(CompoundChild(CollisionShape(SphereShape(0.5f)), Pose(Vector3(0, 0, 0))));

        Body ball = sim.createBody(Pose(Vector3(0, 5.0f, 0)), CollisionShape(compSphere));
        (void)ball.setBodyType(Core::BodyType::Dynamic);

        for (int i = 0; i < 180; ++i) {
            sim.step(1.0f / 60.0f);
        }
        finalPosCompound = ball.getPosition().value();
    }

    std::cout << "Single Sphere Pos: " << finalPosSingle.y << " | Compound Sphere Pos: " << finalPosCompound.y << std::endl;
    CHECK_MSG(std::abs(finalPosSingle.y - finalPosCompound.y) < 1e-3f,
              "Compound tek kure tekil kure ile ayni sonucu vermeli");
}

// 5. Raycast compound'un uzak çocuğuna çarpar ve doğru distance döner
void testRaycastCompoundFarChild() {
    std::cout << "\n--- 5. Compound Raycast Far Child Test ---" << std::endl;
    Simulator sim;

    // Compound: Yakın çocuk (0, 0, 0)'da küre R=0.5, Uzak çocuk (0, 0, 10)'da kutu half=0.5
    CompoundShape comp;
    comp.children.push_back(CompoundChild(CollisionShape(SphereShape(0.5f)), Pose(Vector3(0, 0, 0))));
    comp.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(0.5f, 0.5f, 0.5f))), Pose(Vector3(0, 0, 10.0f))));

    Body body = sim.createBody(Pose(Vector3(0, 0, 0)), CollisionShape(comp));
    (void)body.setBodyType(Core::BodyType::Static);

    // Işın (0, 0, 20)'den (0, 0, -1) yönünde fırlatılır -> ilk olarak uzak çocuğun ön yüzüne (z = 10.5) çarpar
    // Işın başlangıcı 20, hedef yüzey 10.5 -> mesafe = 9.5m
    RaycastHit hit = sim.raycast(Vector3(0, 0, 20.0f), Vector3(0, 0, -1.0f), 50.0f);

    CHECK_MSG(hit.hasHit, "Raycast compound nesnesine carpmali");
    std::cout << "Raycast Hit Distance: " << hit.distance << " m (Expected: 9.5 m)" << std::endl;
    CHECK_MSG(std::abs(hit.distance - 9.5f) < 0.01f, "Raycast compound uzak cocuguna dogru mesafede carpmali");
}

// 6. Snapshot al -> 300 adım -> geri yükle -> 300 adım: hash'ler aynı
void testSnapshotDeterminismCompound() {
    std::cout << "\n--- 6. Compound Snapshot Determinism Test ---" << std::endl;
    Simulator sim;
    sim.setAccelerationField(Vector3(0, -9.81f, 0));

    Body ground = sim.createBody(Pose(Vector3(0, -0.5f, 0)), CollisionShape(BoxShape(Vector3(30, 0.5f, 30))));
    (void)ground.setBodyType(Core::BodyType::Static);

    CompoundShape comp;
    comp.children.push_back(CompoundChild(CollisionShape(BoxShape(Vector3(0.5f, 0.5f, 0.5f))), Pose(Vector3(-1, 0, 0))));
    comp.children.push_back(CompoundChild(CollisionShape(SphereShape(0.4f)), Pose(Vector3(1, 0, 0))));

    Body body = sim.createBody(Pose(Vector3(0, 3.0f, 0)), CollisionShape(comp));
    (void)body.setBodyType(Core::BodyType::Dynamic);

    for (int i = 0; i < 30; ++i) {
        sim.step(1.0f / 60.0f);
    }

    Core::Snapshot snap = sim.saveSnapshot();

    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }
    uint64_t hash1 = harness::computeStateHash(sim);

    sim.restoreSnapshot(snap);

    for (int i = 0; i < 300; ++i) {
        sim.step(1.0f / 60.0f);
    }
    uint64_t hash2 = harness::computeStateHash(sim);

    std::cout << "Hash 1: " << harness::hashToHexString(hash1) << " | Hash 2: " << harness::hashToHexString(hash2) << std::endl;
    CHECK_MSG(hash1 == hash2, "Compound nesnelerde snapshot sonrasi durum hashleri birebir ayni olmali");
}

// 7. Dejenere: boş çocuk listesi -> PhysicsError::InvalidValue veya güvenli ele alma, çökme yok
void testDegenerateEmptyChildren() {
    std::cout << "\n--- 7. Degenerate Empty Compound Test ---" << std::endl;
    Simulator sim;

    CompoundShape emptyComp;
    CHECK_MSG(emptyComp.children.empty(), "Bos compound cocuk listesi bos olmali");

    AABB aabb = CollisionShape(emptyComp).computeLocalAABB();
    CHECK_MSG(aabb.minBounds.lengthSquare() < 1e-8f && aabb.maxBounds.lengthSquare() < 1e-8f, "Bos compound AABB sifir olmali");

    Body body = sim.createBody(Pose(), CollisionShape(emptyComp));
    CHECK_MSG(body.isActive(), "Bos compound ile body olusturma cokmemeli");

    // Adımlama sırasında çökmemeli
    sim.step(1.0f / 60.0f);
    CHECK_MSG(true, "Bos compound adimlamada cokmedi");
}

int main() {
    std::cout << "=== BARYON COMPOUND SHAPE TEST SUITE (PHASE 1.1) ===\n" << std::endl;

    testDumbbellInertiaAndCoM();
    testLShapedCompoundResting();
    testTShapedDropStability();
    testCompoundSphereMatchesSingleSphere();
    testRaycastCompoundFarChild();
    testSnapshotDeterminismCompound();
    testDegenerateEmptyChildren();

    return reportAndExit();
}
