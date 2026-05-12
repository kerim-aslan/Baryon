#include <GLFW/glfw3.h>
#include <iostream>
#include <ostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>

#include "mochi/core.hh"
#include "mochi/module/window.hh"
#include "mochi/world/node.hh"
#include "mochi/world/light.hh"
#include "mochi/world/visual.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/rhi/pipeline.hh"
#include "mochi/rhi/shader.hh"
#include "Baryon/Simulator.hpp"
#include "Player.hh"

using namespace mochi;
using namespace Baryon;

// =========================================================================
// MESH / MODEL DOSYA YOLLARI
// =========================================================================
const std::string PATH_MESH_CUBE    = "Asset\\kup.obj";
const std::string PATH_MESH_DİAMOND  = "Asset\\elmas.glb";
const std::string PATH_MESH_PLAYER  = "Asset\\kup.obj";
const std::string PATH_MESH_SPHERE  = "Asset\\top.glb"; 
const std::string PATH_MESH_CAPSULE = "Asset\\top.glb"; 

float g_camera_distance = 15.0f;
void scroll_callback(GLFWwindow* window, double x, double y) {
    g_camera_distance = std::clamp(g_camera_distance - (float)y * 1.5f, 4.0f, 60.0f);
}

int Main() {
    Simulator world;
    world.setAccelerationField(Vector3(0.0f, -18.0f, 0.0f));

    std::vector<ecs::Entity> phys_bodies;
    std::vector<visual*> gfx_list;
    std::vector<vec3<f32>> gfx_scales;
    Player* player = nullptr; 
    ecs::Entity movingTargetEntity{0};
    
    std::function<void(Vector3, Vector3)> shootProjectile;

    core eng([](vk::raii::PhysicalDevices devices) { return devices[0]; }, [&](f32 dt) {
        if (!player) return;
        static bool cb_set = false;
        if (!cb_set) { glfwSetScrollCallback(eng.sub<module::window>().glfw(), scroll_callback); cb_set = true; }

        player->updateInputAndCamera(dt);
        
        world.step(dt);

        for (size_t i = 0; i < phys_bodies.size(); i++) {
            auto& t = world.getRegistry().getComponent<Pose>(phys_bodies[i]);
            gfx_list[i]->setModel(mat4<f32>::model(
                vec3<f32>(t.position.x, t.position.y, t.position.z), 
                quaternion<f32>(t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z), 
                gfx_scales[i]
            ));
        }

        // Hareketli Hedef (Speed Test)
        if (movingTargetEntity.id != 0 && world.getRegistry().hasComponent<Core::Motion>(movingTargetEntity)) {
            auto& targetTrans = world.getRegistry().getComponent<Pose>(movingTargetEntity);
            auto& targetMotion = world.getRegistry().getComponent<Core::Motion>(movingTargetEntity);
            
            // X ekseninde -15 ile +15 arasında hızlıca git-gel
            if (targetTrans.position.x > 15.0f) targetMotion.linearVelocity.x = -15.0f;
            if (targetTrans.position.x < -15.0f) targetMotion.linearVelocity.x = 15.0f;
            
            world.getRegistry().getComponent<Core::BodyState>(movingTargetEntity).isSleeping = false;
            world.getRegistry().getComponent<Core::BodyState>(movingTargetEntity).sleepTimer = 0.0f;
        }

        // --- Atış Mekaniği ---
        if (player->isShooting()) {
            auto& pose = world.getRegistry().getComponent<Pose>(player->getEntity());
            Baryon::Vector3 spawnPos(pose.position.x, pose.position.y + 0.5f, pose.position.z);
            Baryon::Vector3 dir = player->getForwardVector();
            shootProjectile(spawnPos, dir);
        }
       
    });

    node scene(nil, {});
    eng.scene() = &scene;

    camera* cam = new camera(eng, &scene, mat4<f32>(), 90, 0.1, 1000);
    eng.camera() = cam;
    light lig1(eng, &scene, mat4<f32>::model(vec3<f32>(20, 30, 20), quaternion<f32>(1,0,0,0), vec3<f32>(1)), {1, 1, 1}, 800.0f);

    asset::mesh m_cube(eng, PATH_MESH_CUBE);
    asset::mesh m_diamond(eng, PATH_MESH_DİAMOND);
    asset::mesh m_player(eng, PATH_MESH_PLAYER);
    asset::mesh m_sphere(eng, PATH_MESH_SPHERE);
    asset::mesh m_capsule(eng, PATH_MESH_CAPSULE);
    rhi::info<rhi::pipeline> pbr_i({{mochi::vt::make<mochi::mat4<f32>>(), vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment}}, 
        {{&asset::vertex_i, vk::VertexInputRate::eVertex}}, 
        {{&mochi::camera_i, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment}, 
         {&mochi::light_i, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment}});
    
    std::vector<rhi::shaderSlot> shaders;
    shaders.emplace_back(vk::ShaderStageFlagBits::eVertex, rhi::shader(eng, "pkgs_mochi/lib.qaos.mochi/.qcache/pbr.vert.spv", "main"));
    shaders.emplace_back(vk::ShaderStageFlagBits::eFragment, rhi::shader(eng, "pkgs_mochi/lib.qaos.mochi/.qcache/pbr.frag.spv", "main"));
    auto pipe = rhi::pipeline::make(eng, &pbr_i, std::move(shaders));

    auto spawnBox = [&](Vector3 pos, Vector3 ext, Core::BodyType type, float qW=1.0f, float qX=0.0f, float qY=0.0f, float qZ=0.0f, asset::mesh* customMesh = nullptr) {
        Pose t;
        t.position = pos;
        t.orientation.w = qW; t.orientation.x = qX; t.orientation.y = qY; t.orientation.z = qZ;
        auto rb = world.createBody(t, collision::CollisionShape(collision::BoxShape(ext)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        // BoxShape 'half-extents' (yarı-boyut) bekler. 
        // Standart küp objeleri genelde 2x2x2'dir, bu yüzden scale olarak direkt 'ext' vermek boyutları eşitler.
        gfx_scales.push_back(vec3<f32>(ext.x, ext.y, ext.z));
        gfx_list.push_back(visual::make(eng, &scene, mat4<f32>(), customMesh ? customMesh : &m_cube, pipe));
        return rb;
    };

    auto spawnSphere = [&](Vector3 pos, float radius, Core::BodyType type, asset::mesh* customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::SphereShape(radius)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        gfx_scales.push_back(vec3<f32>(radius, radius, radius));
        gfx_list.push_back(visual::make(eng, &scene, mat4<f32>(), customMesh ? customMesh : &m_sphere, pipe));
        return rb;
    };

    auto spawnCapsule = [&](Vector3 pos, float radius, float height, Core::BodyType type, asset::mesh* customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::CapsuleShape(radius, height)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        gfx_scales.push_back(vec3<f32>(radius, (height * 0.5f) + radius, radius));
        gfx_list.push_back(visual::make(eng, &scene, mat4<f32>(), customMesh ? customMesh : &m_capsule, pipe));
        return rb;
    };

    auto spawnConvexHull = [&](Vector3 pos, const std::vector<Vector3>& verts, float visualScale, Core::BodyType type, asset::mesh* customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::ConvexHullShape(verts)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        // Görsel boyutu da fiziksel boyuta eşitlemek için visualScale parametresini kullanıyoruz
        gfx_scales.push_back(vec3<f32>(visualScale, visualScale, visualScale));
        gfx_list.push_back(visual::make(eng, &scene, mat4<f32>(), customMesh ? customMesh : &m_diamond, pipe));
        return rb;
    };

    // =========================================================================
    // STATIC MESH SPAWN YARDIMCISI
    // StaticMesh cisimleri hiç hareket etmez; grafik listesine eklenmez.
    // =========================================================================
    auto spawnStaticMesh = [&](const std::vector<Vector3>& verts,
                               const std::vector<uint32_t>& idx,
                               Vector3 pos) {
        auto meshPtr = std::make_shared<collision::StaticMesh>(verts, idx);
        collision::CollisionShape shape{collision::StaticMeshShape(meshPtr)};
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, shape);
        (void)rb.setBodyType(Core::BodyType::Static);
        // StaticMesh cisimleri grafik listesine EKLENMİYOR (hareketsiz).
        return rb;
    };

    shootProjectile = [&](Vector3 pos, Vector3 dir) {
        auto projectile = spawnSphere(pos, 0.4f, Core::BodyType::Dynamic, &m_sphere);
        (void)projectile.setMass(10.0f);
        (void)projectile.setLinearVelocity(dir * 45.0f);
    };

    // ===================== SCENE: PHYSICS TEST SUITE =====================

    // 0. ZEMİN (Static Box)
    spawnBox(Vector3(0, -0.5f, 0), Vector3(50, 0.5f, 50), Core::BodyType::Static);

    // 1. OYUNCU
    player = new Player(eng, world, cam, Vector3(0, 5, 20), &m_player, pipe);

    // ===================== PHYSICS TEST SAHNESI =====================

    // Duvar (Wall of Boxes)
        for (int y = 0; y < 8; ++y) {
        // Her çift satırda yarım kutu boyu (0.5f) kaydırma yap
        float xOffset = (y % 2 == 0) ? 0.0f : 0.5f; 
        
        for (int x = -5; x <= 5; ++x) {
            Vector3 pos(
                (float)x * 1.05f + xOffset, 
                0.5f + (float)y * 1.05f, 
                -5.0f
            );
            spawnBox(pos, Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Dynamic);
        }
    }

    // Rampalar ve Şekil Testleri (Kapsül ve Küre)
    // Rampa 1 - Kapsül testi
    auto ramp1 = spawnBox(Vector3(10.0f, 1.5f, -5.0f), Vector3(2.0f, 0.2f, 4.0f), Core::BodyType::Static);
    auto& r1Trans = world.getRegistry().getComponent<Pose>(ramp1.getEntity());
    r1Trans.orientation = Quaternion(0.9238f, 0.3826f, 0.0f, 0.0f); // X ekseni etrafında eğik
    
    // Rampadan aşağı yuvarlanan kapsül
    auto testCapsule = spawnCapsule(Vector3(10.0f, 5.0f, -4.0f), 0.5f, 1.0f, Core::BodyType::Dynamic);
    (void)testCapsule.setMass(2.0f);

    // Rampa 2 - Küre testi
    auto ramp2 = spawnBox(Vector3(16.0f, 1.5f, -5.0f), Vector3(2.0f, 0.2f, 4.0f), Core::BodyType::Static);
    auto& r2Trans = world.getRegistry().getComponent<Pose>(ramp2.getEntity());
    r2Trans.orientation = Quaternion(0.9238f, 0.3826f, 0.0f, 0.0f); // X ekseni etrafında eğik

    // Rampadan aşağı yuvarlanan küre
    auto testSphere = spawnSphere(Vector3(16.0f, 5.0f, -4.0f), 0.8f, Core::BodyType::Dynamic);
    (void)testSphere.setMass(2.0f);

    // ===================== SARKAÇ (PENDULUM) TESTLERİ =====================

    // 1. Menteşe (Revolute) Kısıtlaması ile Sarkaç
    auto revSupport = spawnBox(Vector3(-10.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto revBob = spawnSphere(Vector3(-6.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)revBob.setMass(5.0f);
    
    world.createRevoluteConstraint(
        revSupport, revBob, 
        Vector3(0.0f, 0.0f, 0.0f),  // Destek noktasının yerel merkezi
        Vector3(-4.0f, 0.0f, 0.0f), // Kürenin destekten 4 birim uzakta olduğu yerel nokta
        Vector3(0.0f, 0.0f, 1.0f),  // Z ekseni etrafında dönüş (destek)
        Vector3(0.0f, 0.0f, 1.0f)   // Z ekseni etrafında dönüş (küre)
    );

    // 2. Mesafe (Distance) Kısıtlaması ile Sarkaç
    auto distSupport = spawnBox(Vector3(-15.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto distBob = spawnSphere(Vector3(-11.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)distBob.setMass(5.0f);
    
    world.createDistanceConstraint(distSupport, distBob, 4.0f);

    // ===================== CONVEX HULL TESTİ (BÜYÜTÜLMÜŞ VE TERS ÇEVRİLMİŞ ELMAS) =====================
    std::vector<Vector3> diamondVerts;
    float elmasScale = 3.0f;    // Boyutu 3 katına çıkarıyoruz
    float tableRadius = 0.4f * elmasScale; 
    float girdleRadius = 1.0f * elmasScale;
    
    // Geniş tarafın yerde durması için Y eksenini ters çevirdik
    float tableY = 0.6f * elmasScale;   // Üst kısım (geniş ve düz yüzey)
    float girdleY = 0.0f * elmasScale;  // Orta kısım
    float culetY = -1.2f * elmasScale;  // Alt kısım (ince sivri uç)

    // 1. Alt Düzlük (Table - Artık yerde olacak)
    for (int i = 0; i < 8; ++i) {
        float angle = (float)i * (3.141592f / 4.0f);
        diamondVerts.push_back(Vector3(std::cos(angle) * tableRadius, tableY, std::sin(angle) * tableRadius));
    }
    // 2. Orta Kuşak (Girdle)
    for (int i = 0; i < 8; ++i) {
        float angle = (float)i * (3.141592f / 4.0f);
        diamondVerts.push_back(Vector3(std::cos(angle) * girdleRadius, girdleY, std::sin(angle) * girdleRadius));
    }
    // 3. Üst Sivri Uç (Culet - Havada olacak)
    diamondVerts.push_back(Vector3(0, culetY, 0));

    for (int i = 0; i < 3; ++i) {
        // Hem fiziksel noktaları hem de görsel mesh scale'ini 'elmasScale' ile güncelledik
        auto diamond = spawnConvexHull(Vector3(10.0f, 10.0f + (float)i * 6.0f, -15.0f), diamondVerts, elmasScale, Core::BodyType::Dynamic);
        (void)diamond.setMass(10.0f); // Boyut arttığı için kütleyi de artırdık
    }

    // Hareketli hedef (Kinematik)
    auto movingTarget = spawnBox(Vector3(0.0f, 3.0f, -15.0f), Vector3(1.0f, 1.0f, 1.0f), Core::BodyType::Dynamic);
    movingTargetEntity = movingTarget.getEntity();
    (void)movingTarget.setLinearVelocity(Vector3(15.0f, 0.0f, 0.0f));

    // --- 1) EĞİMLİ RAMPA (iki üçgen = bir quad) ---
    // Zemin seviyesinden (y=0) yaklaşık 4 birim yüksekliğe çıkan 8x8'lik bir rampa.
    // Pozisyon: (30, 0, -10) olarak yerleştirildi.
    {
        std::vector<Vector3> rampVerts = {
            Vector3(-4.0f,  0.0f, -4.0f),   // 0 – sol arka alt
            Vector3( 4.0f,  0.0f, -4.0f),   // 1 – sağ arka alt
            Vector3( 4.0f,  4.0f,  4.0f),   // 2 – sağ ön üst
            Vector3(-4.0f,  4.0f,  4.0f),   // 3 – sol ön üst
        };
        std::vector<uint32_t> rampIdx = {
            0, 1, 2,   // üçgen A
            0, 2, 3    // üçgen B
        };
        spawnStaticMesh(rampVerts, rampIdx, Vector3(30.0f, 0.0f, -10.0f));

        // Rampa üzerine düşen küre
        auto ball = spawnSphere(Vector3(30.0f, 8.0f, -13.0f), 0.7f, Core::BodyType::Dynamic);
        (void)ball.setMass(3.0f);

        // Rampa üzerine düşen kutu
        auto box = spawnBox(Vector3(30.0f, 10.0f, -12.0f), Vector3(0.6f, 0.6f, 0.6f), Core::BodyType::Dynamic);
        (void)box.setMass(3.0f);
    }

    eng.run(); 

    delete player;
    return 0;
}
int main() { try { return Main(); } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return -1; } }