#pragma once

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>

#include "mochi/core.hh"
#include "mochi/module/display.hh" // window.hh -> display.hh olarak güncellendi
#include "mochi/ecs/camera.hh"     // Yeni ECS Kamera sistemi
#include "mochi/ecs/transform.hh"  // Yeni ECS Transform sistemi
#include "mochi/ecs/mesh.hh"       // Yeni ECS Mesh sistemi
#include "Baryon/Simulator.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"

extern float g_camera_distance;

class Player {
public:
    // Pipeline parametresi kaldırıldı, mesh sptr (paylaşımlı işaretçi) yapıldı ve camera ECS component'i oldu
    Player(mochi::core& engine, Baryon::Simulator& physicsWorld, mochi::ecs::Camera* cam,
           const Baryon::Vector3& startPos, sptr<mochi::asset::mesh> playerMesh)
        : m_engine(engine), m_world(physicsWorld), m_camera(cam), m_body(Baryon::ecs::Entity{0}, physicsWorld.getRegistry()) 
    {
        Baryon::Pose startTransform;
        startTransform.position = startPos;
        startTransform.orientation = Baryon::Quaternion::identity();
        
        Baryon::collision::BoxShape karakterSekli(Baryon::Vector3(0.5f, 0.5f, 0.5f)); 
        m_body = m_world.createBody(startTransform, Baryon::collision::CollisionShape(karakterSekli));
        
        // --- KİNEMATİK AYARLAR ---
        (void)m_body.setBodyType(Baryon::Core::BodyType::Kinematic); 
        
        auto& massProps = m_world.getRegistry().getComponent<Baryon::Core::MassProps>(m_body.getEntity());
        massProps.inverseMass = 0.0f; // Kinematik: Sonsuz kütle
        massProps.inverseInertiaTensor = Baryon::Matrix3x3(0,0,0, 0,0,0, 0,0,0);

        // --- Yeni ECS Sisteminde Görsel Obje Oluşturma ---
        m_visualEntity = m_engine.registry().create();
        auto& transform = m_engine.registry().emplace<mochi::ecs::Transform>(m_visualEntity);
        transform.model = mochi::mat4<f32>::model(
            mochi::vec3<f32>(startPos.x, startPos.y, startPos.z), 
            mochi::quaternion<f32>(1.0f, 0.0f, 0.0f, 0.0f), 
            mochi::vec3<f32>(0.5f) // Görsel ölçek
        );
        m_engine.registry().emplace<mochi::ecs::Mesh>(m_visualEntity, playerMesh);
    }

    Baryon::ecs::Entity getEntity() const { return m_body.getEntity(); }

    Baryon::Vector3 getForwardVector() const {
        const float PI = 3.1415926535f;
        float radYaw = m_yaw * (PI / 180.0f);
        return Baryon::Vector3(-cos(radYaw), 0, -sin(radYaw));
    }

    bool isShooting() {
        // window -> display
        auto win = m_engine.sub<mochi::module::display>().glfw();
        bool left_click = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        
        if (left_click && !m_shootPressedLast) {
            m_shootPressedLast = true;
            return true;
        }
        if (!left_click) {
            m_shootPressedLast = false;
        }
        return false;
    }

    void updateInputAndCamera(f32 dt) {
        // window -> display
        auto win = m_engine.sub<mochi::module::display>().glfw();
        const float PI = 3.1415926535f;

        // --- Mouse Kontrolü ---
        bool esc_now = (glfwGetKey(win, GLFW_KEY_ESCAPE) == GLFW_PRESS);
        if (esc_now && !m_escPressedLast) {
            m_cursorLocked = !m_cursorLocked;
            glfwSetInputMode(win, GLFW_CURSOR, m_cursorLocked ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            m_firstMouse = true; 
        }
        m_escPressedLast = esc_now;

        if (m_cursorLocked) {
            double xpos, ypos;
            glfwGetCursorPos(win, &xpos, &ypos);
            if (m_firstMouse) { m_lastX = xpos; m_lastY = ypos; m_firstMouse = false; }
            m_yaw += (float)(xpos - m_lastX) * 0.15f;
            m_pitch += (float)(m_lastY - ypos) * 0.15f;
            m_pitch = std::clamp(m_pitch, -85.0f, 85.0f);
            m_lastX = xpos; m_lastY = ypos;
        }

        // --- Kinematik Hareket ---
        auto& motion = m_world.getRegistry().getComponent<Baryon::Core::Motion>(m_body.getEntity());
        auto& pose = m_world.getRegistry().getComponent<Baryon::Pose>(m_body.getEntity());
        
        Baryon::Vector3 moveDir(0, 0, 0);
        float radYaw = m_yaw * (PI / 180.0f);
        Baryon::Vector3 forward(-cos(radYaw), 0, -sin(radYaw));
        Baryon::Vector3 right(sin(radYaw), 0, -cos(radYaw));

        if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) moveDir = moveDir + forward;
        if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) moveDir = moveDir - forward;
        if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) moveDir = moveDir - right;
        if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) moveDir = moveDir + right;

        float len = sqrt(moveDir.x * moveDir.x + moveDir.z * moveDir.z);
        if (len > 0.1f && m_cursorLocked) {
            float speed = 10.0f;
            motion.linearVelocity.x = (moveDir.x / len) * speed;
            motion.linearVelocity.z = (moveDir.z / len) * speed;
        } else {
            motion.linearVelocity.x = 0;
            motion.linearVelocity.z = 0;
        }

        // Yerçekimi Simülasyonu
        bool isGrounded = (pose.position.y <= 0.51f);
        if (!isGrounded) {
            motion.linearVelocity.y += -20.0f * dt; 
        } else {
            if (motion.linearVelocity.y < 0) motion.linearVelocity.y = 0;
            pose.position.y = 0.5f; 
        }

        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS && m_cursorLocked && isGrounded) {
            motion.linearVelocity.y = 8.5f;
        }

        // --- KINEMATIC CHARACTER CONTROLLER (KCC) ---
        auto contacts = m_world.getContactsForEntity(m_body.getEntity());
        for (const auto& manifold : contacts) {
            if (manifold.isTriggerPair || manifold.penetration <= 0.0f) continue;
            
            bool isA = (manifold.entityA.id == m_body.getEntity().id);
            Baryon::Vector3 normal = manifold.normal;
            
            auto otherEntity = isA ? manifold.entityB : manifold.entityA;
            if (m_world.getRegistry().hasComponent<Baryon::Core::BodyState>(otherEntity)) {
                auto type = m_world.getRegistry().getComponent<Baryon::Core::BodyState>(otherEntity).type;
                if (type == Baryon::Core::BodyType::Dynamic) continue;
                
                auto& otherPose = m_world.getRegistry().getComponent<Baryon::Pose>(otherEntity);
                Baryon::Vector3 toPlayer = pose.position - otherPose.position;
                if (normal.dot(toPlayer) < 0.0f) {
                    normal = normal * -1.0f; 
                }
            }

            // 1. Pozisyon Düzeltme (Depenetration)
            pose.position = pose.position + (normal * (manifold.penetration + 0.005f));
            
            // 2. Hız Kaydırma (Velocity Sliding)
            float velAlongNormal = motion.linearVelocity.dot(normal);
            if (velAlongNormal < 0.0f) {
                motion.linearVelocity = motion.linearVelocity - (normal * velAlongNormal);
            }
        }

        // --- Kamera ve Görsel Güncelleme ---
        mochi::vec3<f32> offset;
        offset.X = g_camera_distance * cos(radYaw) * cos(m_pitch * PI / 180.0f);
        offset.Y = g_camera_distance * sin(m_pitch * PI / 180.0f);
        offset.Z = g_camera_distance * sin(radYaw) * cos(m_pitch * PI / 180.0f);
        
        if (m_camera) {
            // Eskiden setModel(lookAt.inverse()) yapılıyordu. Yeni mimaride view matrisine 
            // direkt lookAt matrisini atamak yeterlidir.
            m_camera->view = mochi::mat4<f32>::lookAt(
                mochi::vec3<f32>(pose.position.x + offset.X, pose.position.y + offset.Y + 2.0f, pose.position.z + offset.Z), 
                mochi::vec3<f32>(pose.position.x, pose.position.y + 1.2f, pose.position.z), 
                mochi::vec3<f32>(0, 1, 0)
            );
        }

        // Yeni sistemde objenin ECS verisini registry üzerinden çekiyoruz
        if (m_engine.registry().valid(m_visualEntity)) {
            auto& transform = m_engine.registry().get<mochi::ecs::Transform>(m_visualEntity);
            float angle = -radYaw - (PI / 2.0f);
            mochi::quaternion<f32> qRotation(cos(angle / 2.0f), 0, sin(angle / 2.0f), 0);
            
            transform.model = mochi::mat4<f32>::model(
                mochi::vec3<f32>(pose.position.x, pose.position.y, pose.position.z), 
                qRotation, 
                mochi::vec3<f32>(1.0f)
            );
        }
        
        auto& state = m_world.getRegistry().getComponent<Baryon::Core::BodyState>(m_body.getEntity());
        state.isSleeping = false;
        state.sleepTimer = 0.0f;
    }

private:
    mochi::core& m_engine;
    Baryon::Simulator& m_world;
    Baryon::Body m_body;
    
    entt::entity m_visualEntity{entt::null}; // Eski `visual*` yerine ECS Entity ID'si
    mochi::ecs::Camera* m_camera;            // Yeni ECS Kamera Bileşeni referansı

    float m_yaw = +90.0f, m_pitch = +25.0f;
    double m_lastX = 640.0, m_lastY = 360.0;
    bool m_firstMouse = true, m_cursorLocked = true, m_escPressedLast = false;
    bool m_shootPressedLast = false;
};
