#include <GLFW/glfw3.h>
#include <iostream>
#include <ostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>

#include "Baryon/ecs/EntityManager.hpp"
#include "mochi/core.hh"
#include "mochi/module/display.hh"
#include "mochi/asset/mesh.hh"

// Yeni ECS modülleri
#include "mochi/ecs/camera.hh"
#include "mochi/ecs/transform.hh"
#include "mochi/ecs/point_light.hh"
#include "mochi/ecs/mesh.hh"
#include "mochi/vfs/vfs_res.hh"
#include "mochi/vfs/vfs_file.hh"

#include "Baryon/Simulator.hpp"
#include "Player.hh"

using namespace mochi;
using namespace Baryon;

// =========================================================================
// MESH / MODEL DOSYA YOLLARI
// =========================================================================
const std::string PATH_MESH_CUBE    = "Asset/kup.obj";
const std::string PATH_MESH_DİAMOND = "Asset/elmas.glb";
const std::string PATH_MESH_PLAYER  = "Asset/kup.obj";
const std::string PATH_MESH_SPHERE  = "Asset/top.glb"; 
const std::string PATH_MESH_CAPSULE = "Asset/top.glb"; 

float g_camera_distance = 15.0f;
void scroll_callback(GLFWwindow* window, double x, double y) {
    g_camera_distance = std::clamp(g_camera_distance - (float)y * 1.5f, 4.0f, 60.0f);
}

int Main() {
    auto vfs_res = vfs::__res::get();
    auto vfs_file = vfs::__file::get();

    Simulator world;
    world.setAccelerationField(Vector3(0.0f, -18.0f, 0.0f));

    std::vector<Baryon::ecs::Entity> phys_bodies;
    std::vector<entt::entity> gfx_list; 
    std::vector<vec3<f32>> gfx_scales;
    Player* player = nullptr; 
    Baryon::ecs::Entity movingTargetEntity{0};
    
    std::function<void(Vector3, Vector3)> shootProjectile;

    core eng([](vk::raii::PhysicalDevices devices) { return devices[0]; }, [&](f32 dt) {

        if (!player) return;

        if (dt > 0.1f) dt = 0.1f;
        
        static bool cb_set = false;
        if (!cb_set) { 
            glfwSetScrollCallback(eng.sub<mochi::module::display>().glfw(), scroll_callback); 
            cb_set = true; 
        }

        // --- 2) GÖRÜNTÜ GELMEME ÇÖZÜMÜ: Projeksiyon Matrisinin Güncellenmesi ---
        // ECS üzerinden kamerayı bulup pencere boyutlarına göre perspektifini ayarlıyoruz.
        auto& disp = eng.sub<mochi::module::display>();
        if (disp.width() > 0 && disp.height() > 0) {
            auto cameras = eng.registry().view<mochi::ecs::Camera>();
            for (auto entity : cameras) {
                auto& cam = cameras.get<mochi::ecs::Camera>(entity);
                float aspect = (float)disp.width() / (float)disp.height();
                cam.proj = mochi::mat4<f32>::perspective(cam.fov, aspect, cam.near, cam.far);
            }
        }

        player->updateInputAndCamera(dt);
        
        world.step(dt);

        // Fizik motorundan gelen pozisyonları ECS bileşenlerine aktar
        for (size_t i = 0; i < phys_bodies.size(); i++) {
            auto& t = world.getRegistry().getComponent<Pose>(phys_bodies[i]);
            auto& transform = eng.registry().get<mochi::ecs::Transform>(gfx_list[i]);
            
            transform.model = mat4<f32>::model(
                vec3<f32>(t.position.x, t.position.y, t.position.z), 
                quaternion<f32>(t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z), 
                gfx_scales[i]
            );
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

    // 1. KAMERA OLUŞTURMA 
    entt::entity camera_ent = eng.registry().create();
    eng.registry().emplace<mochi::ecs::Transform>(camera_ent);
    auto& cam_comp = eng.registry().emplace<mochi::ecs::Camera>(camera_ent);
    cam_comp.fov = 90.0f;
    cam_comp.near = 0.1f;
    cam_comp.far = 1000.0f;

    // 2. IŞIK OLUŞTURMA
    entt::entity light_ent = eng.registry().create();
    auto& lig_trans = eng.registry().emplace<mochi::ecs::Transform>(light_ent);
    lig_trans.model = mat4<f32>::model(vec3<f32>(20, 30, 20), quaternion<f32>(1,0,0,0), vec3<f32>(1));
    auto& lig_comp = eng.registry().emplace<mochi::ecs::PointLight>(light_ent);
    lig_comp.color = {1, 1, 1};
    lig_comp.intensity = 800.0f;

    // 3. MODELLERİN YÜKLENMESİ 
    auto m_cube    = asset::mesh::make(eng, PATH_MESH_CUBE);
    auto m_diamond = asset::mesh::make(eng, PATH_MESH_DİAMOND);
    auto m_player  = asset::mesh::make(eng, PATH_MESH_PLAYER);
    auto m_sphere  = asset::mesh::make(eng, PATH_MESH_SPHERE);
    auto m_capsule = asset::mesh::make(eng, PATH_MESH_CAPSULE);

    auto spawnBox = [&](Vector3 pos, Vector3 ext, Core::BodyType type, float qW=1.0f, float qX=0.0f, float qY=0.0f, float qZ=0.0f, sptr<asset::mesh> customMesh = nullptr) {
        Pose t;
        t.position = pos;
        t.orientation.w = qW; t.orientation.x = qX; t.orientation.y = qY; t.orientation.z = qZ;
        auto rb = world.createBody(t, collision::CollisionShape(collision::BoxShape(ext)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        
        gfx_scales.push_back(vec3<f32>(ext.x, ext.y, ext.z));
        
        // Render edilebilir objeyi ECS'ye ekle
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, customMesh ? customMesh : m_cube);
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnSphere = [&](Vector3 pos, float radius, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::SphereShape(radius)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(radius, radius, radius));

        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, customMesh ? customMesh : m_sphere);
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnCapsule = [&](Vector3 pos, float radius, float height, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::CapsuleShape(radius, height)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(radius, (height * 0.5f) + radius, radius));
        
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, customMesh ? customMesh : m_capsule);
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnConvexHull = [&](Vector3 pos, const std::vector<Vector3>& verts, float visualScale, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::ConvexHullShape(verts)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(visualScale, visualScale, visualScale));
        
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, customMesh ? customMesh : m_diamond);
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnStaticMesh = [&](const std::vector<Vector3>& verts,
                               const std::vector<uint32_t>& idx,
                               Vector3 pos) {
        auto meshPtr = std::make_shared<collision::StaticMesh>(verts, idx);
        collision::CollisionShape shape{collision::StaticMeshShape(meshPtr)};
        Pose t;
        t.position = pos;
        auto rb = world.createBody(t, shape);
        (void)rb.setBodyType(Core::BodyType::Static);
        return rb;
    };

    shootProjectile = [&](Vector3 pos, Vector3 dir) {
        auto projectile = spawnSphere(pos, 0.4f, Core::BodyType::Dynamic, m_sphere);
        (void)projectile.setMass(10.0f);
        (void)projectile.setLinearVelocity(dir * 45.0f);
    };

    // ===================== SCENE: PHYSICS TEST SUITE =====================

    // 0. ZEMİN (Static Box)
    spawnBox(Vector3(0, -0.5f, 0), Vector3(50, 0.5f, 50), Core::BodyType::Static);

    // 1. OYUNCU 
    player = new Player(eng, world, &cam_comp, Vector3(0, 5, 20), m_player);

    // Duvar (Wall of Boxes)
    for (int y = 0; y < 8; ++y) {
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
    auto ramp1 = spawnBox(Vector3(10.0f, 1.5f, -5.0f), Vector3(2.0f, 0.2f, 4.0f), Core::BodyType::Static);
    auto& r1Trans = world.getRegistry().getComponent<Pose>(ramp1.getEntity());
    r1Trans.orientation = Quaternion(0.9238f, 0.3826f, 0.0f, 0.0f); 
    
    auto testCapsule = spawnCapsule(Vector3(10.0f, 5.0f, -4.0f), 0.5f, 1.0f, Core::BodyType::Dynamic);
    (void)testCapsule.setMass(2.0f);

    auto ramp2 = spawnBox(Vector3(16.0f, 1.5f, -5.0f), Vector3(2.0f, 0.2f, 4.0f), Core::BodyType::Static);
    auto& r2Trans = world.getRegistry().getComponent<Pose>(ramp2.getEntity());
    r2Trans.orientation = Quaternion(0.9238f, 0.3826f, 0.0f, 0.0f); 

    auto testSphere = spawnSphere(Vector3(16.0f, 5.0f, -4.0f), 0.8f, Core::BodyType::Dynamic);
    (void)testSphere.setMass(2.0f);

    // SARKAÇ (PENDULUM) TESTLERİ
    auto revSupport = spawnBox(Vector3(-10.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto revBob = spawnSphere(Vector3(-6.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)revBob.setMass(5.0f);
    
    world.createRevoluteConstraint(
        revSupport, revBob, 
        Vector3(0.0f, 0.0f, 0.0f), 
        Vector3(-4.0f, 0.0f, 0.0f), 
        Vector3(0.0f, 0.0f, 1.0f),  
        Vector3(0.0f, 0.0f, 1.0f)   
    );

    auto distSupport = spawnBox(Vector3(-15.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto distBob = spawnSphere(Vector3(-11.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)distBob.setMass(5.0f);
    
    world.createDistanceConstraint(distSupport, distBob, 4.0f);

    // CONVEX HULL TESTİ
    std::vector<Vector3> diamondVerts;
    float elmasScale = 3.0f;    
    float tableRadius = 0.4f * elmasScale; 
    float girdleRadius = 1.0f * elmasScale;
    float tableY = 0.6f * elmasScale;   
    float girdleY = 0.0f * elmasScale;  
    float culetY = -1.2f * elmasScale;  

    for (int i = 0; i < 8; ++i) {
        float angle = (float)i * (3.141592f / 4.0f);
        diamondVerts.push_back(Vector3(std::cos(angle) * tableRadius, tableY, std::sin(angle) * tableRadius));
    }
    for (int i = 0; i < 8; ++i) {
        float angle = (float)i * (3.141592f / 4.0f);
        diamondVerts.push_back(Vector3(std::cos(angle) * girdleRadius, girdleY, std::sin(angle) * girdleRadius));
    }
    diamondVerts.push_back(Vector3(0, culetY, 0));

    for (int i = 0; i < 3; ++i) {
        auto diamond = spawnConvexHull(Vector3(10.0f, 10.0f + (float)i * 6.0f, -15.0f), diamondVerts, elmasScale, Core::BodyType::Dynamic);
        (void)diamond.setMass(10.0f); 
    }

    auto movingTarget = spawnBox(Vector3(0.0f, 3.0f, -15.0f), Vector3(1.0f, 1.0f, 1.0f), Core::BodyType::Dynamic);
    movingTargetEntity = movingTarget.getEntity();
    (void)movingTarget.setLinearVelocity(Vector3(15.0f, 0.0f, 0.0f));

    {
        std::vector<Vector3> rampVerts = {
            Vector3(-4.0f,  0.0f, -4.0f),  
            Vector3( 4.0f,  0.0f, -4.0f),  
            Vector3( 4.0f,  4.0f,  4.0f),  
            Vector3(-4.0f,  4.0f,  4.0f),  
        };
        std::vector<uint32_t> rampIdx = {
            0, 1, 2,   
            0, 2, 3    
        };
        spawnStaticMesh(rampVerts, rampIdx, Vector3(30.0f, 0.0f, -10.0f));

        auto ball = spawnSphere(Vector3(30.0f, 8.0f, -13.0f), 0.7f, Core::BodyType::Dynamic);
        (void)ball.setMass(3.0f);

        auto box = spawnBox(Vector3(30.0f, 10.0f, -12.0f), Vector3(0.6f, 0.6f, 0.6f), Core::BodyType::Dynamic);
        (void)box.setMass(3.0f);
    }

    eng.run(); 

    delete player;
    return 0;
}

int main() { 
    try { 
        return Main(); 
    } catch (const std::exception &e) { 
        std::cerr << e.what() << std::endl; 
        return -1; 
    } 
}
