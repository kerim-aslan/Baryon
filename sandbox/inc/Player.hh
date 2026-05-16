#pragma once

#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>

#include "mochi/core.hh"
#include "mochi/module/display.hh"
#include "mochi/ecs/camera.hh"
#include "mochi/ecs/transform.hh"
#include "mochi/ecs/mesh.hh"
#include "Baryon/Simulator.hpp"
#include "Baryon/math/Pose.hpp"
#include "Baryon/Core/CoreComponents.hpp"

extern float g_camera_distance;

class Player {
public:
    Player(mochi::core& engine, Baryon::Simulator& physicsWorld, mochi::ecs::Camera* cam,
           const Baryon::Vector3& startPos, sptr<mochi::asset::mesh> playerMesh)
        : m_engine(engine), m_world(physicsWorld), m_camera(cam), m_body(Baryon::ecs::Entity{0}, physicsWorld.getRegistry()) 
    {
        Baryon::Pose startTransform;
        startTransform.position = startPos;
        startTransform.orientation = Baryon::Quaternion::identity();
        
        Baryon::collision::BoxShape characterShape(Baryon::Vector3(0.5f, 0.9f, 0.5f)); 
        m_body = m_world.createBody(startTransform, Baryon::collision::CollisionShape(characterShape));
        
        auto res = m_body.setBodyType(Baryon::Core::BodyType::Kinematic); 
        (void)res;
        
        auto& massProps = m_world.getRegistry().getComponent<Baryon::Core::MassProps>(m_body.getEntity());
        massProps.inverseMass = 0.0f;
        massProps.inverseInertiaTensor = Baryon::Matrix3x3(0,0,0, 0,0,0, 0,0,0);

        m_visualEntity = m_engine.registry().create();
        auto& transform = m_engine.registry().emplace<mochi::ecs::Transform>(m_visualEntity);
        transform.model = mochi::mat4<f32>::model(
            mochi::vec3<f32>(startPos.x, startPos.y, startPos.z), 
            mochi::quaternion<f32>(1.0f, 0.0f, 0.0f, 0.0f), 
            mochi::vec3<f32>(0.5f)
        );
        m_engine.registry().emplace<mochi::ecs::Mesh>(m_visualEntity, mochi::ecs::Mesh{playerMesh, entt::null});
    }

    Baryon::ecs::Entity getEntity() const { return m_body.getEntity(); }

    Baryon::Vector3 getForwardVector() const {
        const float PI = 3.1415926535f;
        float radYaw = m_yaw * (PI / 180.0f);
        return Baryon::Vector3(-std::cos(radYaw), 0, -std::sin(radYaw));
    }

    bool isShooting() {
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
        auto win = m_engine.sub<mochi::module::display>().glfw();
        const float PI = 3.1415926535f;

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

        auto& motion = m_world.getRegistry().getComponent<Baryon::Core::Motion>(m_body.getEntity());
        auto& pose = m_world.getRegistry().getComponent<Baryon::Pose>(m_body.getEntity());
        
        Baryon::Vector3 moveDir(0, 0, 0);
        float radYaw = m_yaw * (PI / 180.0f);
        Baryon::Vector3 forward(-std::cos(radYaw), 0, -std::sin(radYaw));
        Baryon::Vector3 right(std::sin(radYaw), 0, -std::cos(radYaw));

        if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) moveDir = moveDir + forward;
        if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) moveDir = moveDir - forward;
        if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) moveDir = moveDir - right;
        if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) moveDir = moveDir + right;

        float len = std::sqrt(moveDir.x * moveDir.x + moveDir.z * moveDir.z);
        float targetSpeed = 12.0f;
        
        if (len > 0.1f && m_cursorLocked) {
            motion.linearVelocity.x = (moveDir.x / len) * targetSpeed;
            motion.linearVelocity.z = (moveDir.z / len) * targetSpeed;
        } else {
            motion.linearVelocity.x = 0;
            motion.linearVelocity.z = 0;
        }

        bool isGrounded = (pose.position.y <= 0.91f);
        if (!isGrounded) {
            motion.linearVelocity.y += -25.0f * dt; 
        } else {
            if (motion.linearVelocity.y < 0) motion.linearVelocity.y = 0;
            pose.position.y = 0.9f; 
        }

        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS && m_cursorLocked && isGrounded) {
            motion.linearVelocity.y = 10.0f;
        }

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
                if (normal.dot(toPlayer) < 0.0f) normal = normal * -1.0f; 

                pose.position = pose.position + (normal * (manifold.penetration + 0.001f));
                
                float velAlongNormal = motion.linearVelocity.dot(normal);
                if (velAlongNormal < 0.0f) {
                    motion.linearVelocity = motion.linearVelocity - (normal * velAlongNormal);
                }
            }
        }

        mochi::vec3<f32> offset;
        offset.X = g_camera_distance * std::cos(radYaw) * std::cos(m_pitch * PI / 180.0f);
        offset.Y = g_camera_distance * std::sin(m_pitch * PI / 180.0f);
        offset.Z = g_camera_distance * std::sin(radYaw) * std::cos(m_pitch * PI / 180.0f);
        
        if (m_camera) {
            m_camera->view = mochi::mat4<f32>::lookAt(
                mochi::vec3<f32>(pose.position.x + offset.X, pose.position.y + offset.Y + 2.0f, pose.position.z + offset.Z), 
                mochi::vec3<f32>(pose.position.x, pose.position.y + 1.2f, pose.position.z), 
                mochi::vec3<f32>(0, 1, 0)
            );
        }

        if (m_engine.registry().valid(m_visualEntity)) {
            auto& transform = m_engine.registry().get<mochi::ecs::Transform>(m_visualEntity);
            float angle = -radYaw - (PI / 2.0f);
            mochi::quaternion<f32> qRotation(std::cos(angle / 2.0f), 0, std::sin(angle / 2.0f), 0);
            
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
    
    entt::entity m_visualEntity{entt::null};
    mochi::ecs::Camera* m_camera;

    float m_yaw = +90.0f, m_pitch = +25.0f;
    double m_lastX = 640.0, m_lastY = 360.0;
    bool m_firstMouse = true, m_cursorLocked = true, m_escPressedLast = false;
    bool m_shootPressedLast = false;
};