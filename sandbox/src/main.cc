#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Baryon Bileşenleri ve Matematik Kütüphaneleri
#include "Baryon/ecs/EntityManager.hpp"
#include "Baryon/Simulator.hpp"
#include "Baryon/math/Vector3.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/math/Quaternion.hpp"
#include "Baryon/Core/CoreComponents.hpp"
#include "Baryon/collision/CollisionShape.hpp"

// Mochi Bileşenleri
#include "mochi/core.hh"
#include "mochi/module/display.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/ecs/camera.hh"
#include "mochi/ecs/transform.hh"
#include "mochi/ecs/point_light.hh"
#include "mochi/ecs/mesh.hh"
#include "mochi/vfs/vfs_embed.hh"
#include "mochi/vfs/vfs_file.hh"

#include "Player.hh"
#include "DebugDraw.hpp"
#include "PhysicsLogger.hpp" // GÖREV 1, 4, 5: Logger

using namespace mochi;
using namespace Baryon;

float g_camera_distance = 25.0f;
bool g_debug_draw = false;
void scroll_callback(GLFWwindow* window, double x, double y) {
    g_camera_distance = std::clamp(g_camera_distance - (float)y * 2.0f, 2.0f, 100.0f);
}

int Main() {
    auto vfs_file = vfs::__file::get();
    auto vfs_embed = vfs::__embed::get();

    Simulator world;
    world.setAccelerationField(Vector3(0.0f, -15.0f, 0.0f));
    world.setFixedTimeStep(1.0f / 60.0f);
    std::vector<Baryon::ecs::Entity> phys_bodies;
    std::vector<entt::entity> gfx_list; 
    std::vector<vec3<f32>> gfx_scales;
    std::vector<sptr<asset::mesh>> gfx_solid_meshes;
    std::vector<sptr<asset::mesh>> gfx_wire_meshes;
    Player* player = nullptr; 
    Baryon::ecs::Entity movingTargetEntity{0};
    Baryon::ecs::Entity bulletEntity{0};
    
    std::function<void(Vector3, Vector3)> shootProjectile;

    core eng([](vk::raii::PhysicalDevices devices) { return devices[0]; }, [&](f32 dt) {

        if (!player) return;

        if (dt > 0.1f) dt = 0.1f;
        
        static bool cb_set = false;
        if (!cb_set) { 
            glfwSetScrollCallback(eng.sub<mochi::module::display>().glfw(), scroll_callback); 
            cb_set = true; 
        }

        auto& disp = eng.sub<mochi::module::display>();
        if (disp.width() > 0 && disp.height() > 0) {
            auto cameras = eng.registry().view<mochi::ecs::Camera>();
            for (auto entity : cameras) {
                auto& cam = eng.registry().get<mochi::ecs::Camera>(entity);
                float aspect = (float)disp.width() / (float)disp.height();
                cam.proj = mochi::mat4<f32>::perspective(cam.fov, aspect, cam.near, cam.far);
            }
        }

        player->updateInputAndCamera(dt);
        
        world.step(dt);

        auto win = eng.sub<module::display>().glfw();
        static bool f1_pressed_last = false;
        bool f1_now = (glfwGetKey(win, GLFW_KEY_F1) == GLFW_PRESS);
        if (f1_now && !f1_pressed_last) {
            g_debug_draw = !g_debug_draw;
            std::cout << "Debug Draw: " << (g_debug_draw ? "ON" : "OFF") << "\n";
            for (size_t i = 0; i < gfx_list.size(); i++) {
                eng.registry().replace<mochi::ecs::Mesh>(gfx_list[i], mochi::ecs::Mesh{g_debug_draw ? gfx_wire_meshes[i] : gfx_solid_meshes[i], entt::null});
            }
        }
        f1_pressed_last = f1_now;

        for (size_t i = 0; i < phys_bodies.size(); i++) {
            auto& t = world.getRegistry().getComponent<Pose>(phys_bodies[i]);
            auto& transform = eng.registry().get<mochi::ecs::Transform>(gfx_list[i]);
            
            transform.model = mat4<f32>::model(
                vec3<f32>(t.position.x, t.position.y, t.position.z), 
                quaternion<f32>(t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z), 
                gfx_scales[i]
            );
        }

        if (bulletEntity.id != 0) {
            static float bulletTimer = 0.0f;
            bulletTimer += dt;
            if (bulletTimer > 2.0f) {
                bulletTimer = 0.0f;
                
                Baryon::Body bulletBody(bulletEntity, world.getRegistry());
                (void)bulletBody.setPosition(Vector3(20, 5, -20)); 
                (void)bulletBody.setLinearVelocity(Vector3(0, 0, 0)); 
                (void)bulletBody.applyLinearImpulse(Vector3(-200.0f, 0, 0)); 

                // GÖREV 5: Velocity Tunneling Kontrolü (Log)
                if (world.getRegistry().hasComponent<Core::Motion>(bulletEntity)) {
                    auto& motion = world.getRegistry().getComponent<Core::Motion>(bulletEntity);
                    PhysicsLogger::checkVelocityTunneling("HighSpeedBullet", motion.linearVelocity, dt, 0.1f);
                }
            }
        }

        // Atış Mekaniği
        if (player->isShooting()) {
            auto& pose = world.getRegistry().getComponent<Pose>(player->getEntity());
            Baryon::Vector3 spawnPos(pose.position.x, pose.position.y + 0.5f, pose.position.z);
            Baryon::Vector3 dir = player->getForwardVector();
            shootProjectile(spawnPos, dir);
        }
    });

    entt::entity camera_ent = eng.registry().create();
    eng.registry().emplace<mochi::ecs::Transform>(camera_ent);
    auto& cam_comp = eng.registry().emplace<mochi::ecs::Camera>(camera_ent);
    cam_comp.fov = 90.0f;
    cam_comp.near = 0.1f;
    cam_comp.far = 1000.0f;

    entt::entity light_ent = eng.registry().create();
    auto& lig_trans = eng.registry().emplace<mochi::ecs::Transform>(light_ent);
    lig_trans.model = mat4<f32>::model(vec3<f32>(20, 30, 20), quaternion<f32>(1,0,0,0), vec3<f32>(1));
    auto& lig_comp = eng.registry().emplace<mochi::ecs::PointLight>(light_ent);
    lig_comp.color = {1, 1, 1};
    lig_comp.intensity = 800.0f;

        
    auto m_cube    = asset::mesh::make(eng, "file://Asset/kup.obj"_vfs_map->span(), ".obj");
    auto m_diamond = asset::mesh::make(eng, "file://Asset/elmas.glb"_vfs_map->span(), ".glb");
    auto m_player  = asset::mesh::make(eng, "file://Asset/kup.obj"_vfs_map->span(), ".obj");
    auto m_sphere  = asset::mesh::make(eng, "file://Asset/top.glb"_vfs_map->span(), ".glb");
    auto m_capsule = asset::mesh::make(eng, "file://Asset/top.glb"_vfs_map->span(), ".glb");
    
    auto m_wire_box = DebugDraw::createWireframeBox(eng, Baryon::Vector3(1,1,1));
    auto m_wire_sphere = DebugDraw::createWireframeSphere(eng, 1.0f);

    auto spawnBox = [&](Vector3 pos, Vector3 ext, Core::BodyType type, float qW=1.0f, float qX=0.0f, float qY=0.0f, float qZ=0.0f, sptr<asset::mesh> customMesh = nullptr) {
        Pose t; t.position = pos;
        t.orientation.w = qW; t.orientation.x = qX; t.orientation.y = qY; t.orientation.z = qZ;
        auto rb = world.createBody(t, collision::CollisionShape(collision::BoxShape(ext)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 
        
        // GÖREV 1: Fizik-Grafik Boyut Doğrulama ve AABB Log
        Baryon::Vector3 minB(-ext.x, -ext.y, -ext.z);
        Baryon::Vector3 maxB(ext.x, ext.y, ext.z);
        PhysicsLogger::logAABB("Box", minB, maxB);
        PhysicsLogger::validateScale("Box", ext, Vector3(ext.x, ext.y, ext.z)); 

        gfx_scales.push_back(vec3<f32>(ext.x, ext.y, ext.z));
        auto smesh = customMesh ? customMesh : m_cube;
        gfx_solid_meshes.push_back(smesh);
        gfx_wire_meshes.push_back(m_wire_box);
        
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, mochi::ecs::Mesh{smesh, entt::null});
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnSphere = [&](Vector3 pos, float radius, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t; t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::SphereShape(radius)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(radius, radius, radius));
        auto smesh = customMesh ? customMesh : m_sphere;
        gfx_solid_meshes.push_back(smesh);
        gfx_wire_meshes.push_back(m_wire_sphere);

        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, mochi::ecs::Mesh{smesh, entt::null});
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnCapsule = [&](Vector3 pos, float radius, float height, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t; t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::CapsuleShape(radius, height)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(radius, (height * 0.5f) + radius, radius));
        auto smesh = customMesh ? customMesh : m_capsule;
        gfx_solid_meshes.push_back(smesh);
        gfx_wire_meshes.push_back(m_wire_box);
        
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, mochi::ecs::Mesh{smesh, entt::null});
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnConvexHull = [&](Vector3 pos, const std::vector<Vector3>& verts, float visualScale, Core::BodyType type, sptr<asset::mesh> customMesh = nullptr) {
        Pose t; t.position = pos;
        auto rb = world.createBody(t, collision::CollisionShape(collision::ConvexHullShape(verts)));
        (void)rb.setBodyType(type);
        phys_bodies.push_back(rb.getEntity()); 

        gfx_scales.push_back(vec3<f32>(visualScale, visualScale, visualScale));
        auto smesh = customMesh ? customMesh : m_diamond;
        gfx_solid_meshes.push_back(smesh);
        std::vector<Vector3> unitVerts;
        for (const auto& v : verts) unitVerts.push_back(Vector3(v.x/visualScale, v.y/visualScale, v.z/visualScale));
        gfx_wire_meshes.push_back(DebugDraw::createWireframeConvexHull(eng, unitVerts));
        
        entt::entity visual_ent = eng.registry().create();
        eng.registry().emplace<mochi::ecs::Transform>(visual_ent);
        eng.registry().emplace<mochi::ecs::Mesh>(visual_ent, mochi::ecs::Mesh{smesh, entt::null});
        gfx_list.push_back(visual_ent);

        return rb;
    };

    auto spawnStaticMesh = [&](const std::vector<Vector3>& verts, const std::vector<uint32_t>& idx, Vector3 pos) {
        auto meshPtr = std::make_shared<collision::StaticMesh>(verts, idx);
        collision::CollisionShape shape{collision::StaticMeshShape(meshPtr)};
        Pose t; t.position = pos;
        auto rb = world.createBody(t, shape);
        (void)rb.setBodyType(Core::BodyType::Static);
        return rb;
    };

    shootProjectile = [&](Vector3 pos, Vector3 dir) {
        auto projectile = spawnSphere(pos, 0.4f, Core::BodyType::Dynamic, m_sphere);
        (void)projectile.setMass(10.0f);
        (void)projectile.setLinearVelocity(dir * 45.0f);
    };

    // 3. TUNNELING TEST 
    spawnBox(Vector3(-10, 5, -20), Vector3(0.1f, 10.0f, 10.0f), Core::BodyType::Static, 1.0f, 0.0f, 0.0f, 0.0f, m_cube);
    auto bullet = spawnSphere(Vector3(20, 5, -20), 0.3f, Core::BodyType::Dynamic);
    bulletEntity = bullet.getEntity();
    (void)bullet.setMass(1.0f);
    (void)bullet.setCCD(true); 
    (void)bullet.applyLinearImpulse(Vector3(-5.0f, 0.0f, 0.0f));


    // 0. ZEMİN (Static Box)
    spawnBox(Vector3(0, -0.5f, 0), Vector3(50, 0.5f, 50), Core::BodyType::Static);

    // 1. OYUNCU 
    player = new Player(eng, world, &cam_comp, Vector3(0, 5, 20), m_player);

    for (int y = 0; y < 8; ++y) {
        float xOffset = (y % 2 == 0) ? 0.0f : 0.5f; 
        for (int x = -5; x <= 5; ++x) {
            Vector3 pos((float)x * 1.05f + xOffset, 0.5f + (float)y * 1.05f, -5.0f);
            spawnBox(pos, Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Dynamic);
        }
    }

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

    auto revSupport = spawnBox(Vector3(-10.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto revBob = spawnSphere(Vector3(-6.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)revBob.setMass(5.0f);
    
    world.createRevoluteConstraint(
        revSupport, revBob, 
        Vector3(0.0f, 0.0f, 0.0f), Vector3(-4.0f, 0.0f, 0.0f), 
        Vector3(0.0f, 0.0f, 1.0f), Vector3(0.0f, 0.0f, 1.0f)   
    );

    auto distSupport = spawnBox(Vector3(-15.0f, 8.0f, -5.0f), Vector3(0.5f, 0.5f, 0.5f), Core::BodyType::Static);
    auto distBob = spawnSphere(Vector3(-11.0f, 8.0f, -5.0f), 0.8f, Core::BodyType::Dynamic); 
    (void)distBob.setMass(5.0f);
    
    world.createDistanceConstraint(distSupport, distBob, 4.0f);

    std::vector<Vector3> diamondVerts;
    float elmasScale = 3.0f;    
    float tableRadius = 0.4f * elmasScale, girdleRadius = 1.0f * elmasScale;
    float tableY = 0.6f * elmasScale, girdleY = 0.0f * elmasScale, culetY = -1.2f * elmasScale;  

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

    // {
    //     std::vector<Vector3> rampVerts = {
    //         Vector3(-4.0f,  0.0f, -4.0f), Vector3( 4.0f,  0.0f, -4.0f),  
    //         Vector3( 4.0f,  4.0f,  4.0f), Vector3(-4.0f,  4.0f,  4.0f),  
    //     };
    //     std::vector<uint32_t> rampIdx = { 0, 1, 2, 0, 2, 3 };
    //     spawnStaticMesh(rampVerts, rampIdx, Vector3(30.0f, 0.0f, -10.0f));

    //     auto ball = spawnSphere(Vector3(30.0f, 8.0f, -13.0f), 0.7f, Core::BodyType::Dynamic);
    //     (void)ball.setMass(3.0f);

    //     auto box = spawnBox(Vector3(30.0f, 10.0f, -12.0f), Vector3(0.6f, 0.6f, 0.6f), Core::BodyType::Dynamic);
    //     (void)box.setMass(3.0f);
    // }

    eng.run(); 

    delete player;

    // Free resources explicitly before the engine (and its VMA allocator) gets destroyed
    eng.registry().clear();
    gfx_list.clear();
    gfx_solid_meshes.clear();
    gfx_wire_meshes.clear();

    return 0;
}

int main() { 
    try { return Main(); } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return -1; } 
}