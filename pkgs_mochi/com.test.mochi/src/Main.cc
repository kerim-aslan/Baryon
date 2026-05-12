/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include <GLFW/glfw3.h>
#include <stdexcept>
#include <iostream>
#include "mochi/core.hh"
#include "mochi/module/window.hh"
#include "mochi/world/node.hh"
#include "mochi/world/camera.hh"
#include "mochi/world/light.hh"
#include "mochi/world/visual.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/rhi/pipeline.hh"
#include "mochi/rhi/shader.hh"
#include "vulkan/vulkan.hpp"


using namespace mochi;


  
int Main()
{
  mochi::vec3<f32> cam_pos{0,0,4};
  mochi::vec3<f32> cam_front{-1};
  cam_front = cam_front.normalize();
  mochi::vec3<f32> cam_up{0,1,0};
  f32 cam_speed = 10;
  f32 yaw = -135;
  f32 pitch = -35; 
  f64 last_x = 400, last_y = 300;
  bool first_mouse = true;
  f32 mouse_sensitivity = 0.1;

  camera *cam{};
  

  core eng(
    [](vk::raii::PhysicalDevices devices) -> vk::raii::PhysicalDevice
    {
      if (devices.empty())
        throw std::runtime_error("Vulkan destekli bir grafik birimi bulunamadı.");

      return devices[0];
    },
    [&](f32 dt)
    {
      if (!cam) return;

      auto win = eng.sub<module::window>().glfw();


      auto is_key_pressed = [win](int key) { return glfwGetKey(win, key) == GLFW_PRESS; };
      auto disable_cursor = [win]() { glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED); };
      auto enable_cursor = [win]() { glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_NORMAL); };
      auto get_cursor_pos = [win](f64 &x, f64 &y) { glfwGetCursorPos(win, &x, &y); };


      static bool cursor_locked = false;
      static bool esc_pressed_last_frame = false;
      
      bool esc_pressed = is_key_pressed(GLFW_KEY_ESCAPE);
      
      
      if (esc_pressed && !esc_pressed_last_frame) {
        cursor_locked = !cursor_locked;
        
        if (cursor_locked) {
          disable_cursor();
          first_mouse = true;
        } else {
          enable_cursor();
        }
      }
      esc_pressed_last_frame = esc_pressed;

    
      double xpos, ypos;
      get_cursor_pos(xpos, ypos);

      if (first_mouse) {
        last_x = xpos;
        last_y = ypos;
        first_mouse = false;
      }

      float xoffset = xpos - last_x;
      float yoffset = last_y - ypos; 
      last_x = xpos;
      last_y = ypos;

      
      if (cursor_locked) {
        xoffset *= mouse_sensitivity;
        yoffset *= mouse_sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;
        
        mochi::vec3<f32> front;
        front.X = std::cos(yaw * (M_PI / 180.0f)) * std::cos(pitch * (M_PI / 180.0f));
        front.Y = std::sin(pitch * (M_PI / 180.0f));
        front.Z = std::sin(yaw * (M_PI / 180.0f)) * std::cos(pitch * (M_PI / 180.0f));
        cam_front = front.normalize();
      }

      
      mochi::vec3<f32> cam_right = cam_front.cross(cam_up).normalize();

      bool moved = false;
      
      
      if (is_key_pressed(GLFW_KEY_W)) { cam_pos += cam_front * (cam_speed * dt); moved = true; }
      if (is_key_pressed(GLFW_KEY_S)) { cam_pos -= cam_front * (cam_speed * dt); moved = true; }
      if (is_key_pressed(GLFW_KEY_A)) { cam_pos -= cam_right * (cam_speed * dt); moved = true; }
      if (is_key_pressed(GLFW_KEY_D)) { cam_pos += cam_right * (cam_speed * dt); moved = true; }
      
      if (is_key_pressed(GLFW_KEY_SPACE)) { cam_pos += cam_up * (cam_speed * dt); moved = true; }
      if (is_key_pressed(GLFW_KEY_LEFT_SHIFT)) { cam_pos -= cam_up * (cam_speed * dt); moved = true; }

      if (moved || cursor_locked) {
        auto view_matrix = mochi::mat4<f32>::lookAt(cam_pos, cam_pos + cam_front, cam_up);
        cam->setModel(view_matrix.inverse());
      }
    }
  );




  node scene(nil, {});
  
  cam = new camera(eng, &scene,
    mochi::mat4<f32>::lookAt(cam_pos, cam_pos + cam_front, cam_up).inverse(),
    90, 0.1, 1000
  );

  light lig1(eng, &scene,
    mat4<f32>::model({4,0,0}, quaternion<f32>{}, {1}),
    {1,0,0},
    10
  );

  light lig2(eng, &scene,
    mat4<f32>::model({0,4,0}, quaternion<f32>{}, {1}),
    {0,1,0},
    10
  );

  light lig3(eng, &scene,
    mat4<f32>::model({-4,0,0}, quaternion<f32>{}, {1}),
    {0,0,1},
    10
  );

  light lig4(eng, &scene,
    mat4<f32>::model({0,-4,0}, quaternion<f32>{}, {1}),
    {1,1,0},
    10
  );


  asset::mesh m3d(eng, "C:\\Users\\Kerim Aslan\\Downloads\\Leon.obj");


  rhi::info<rhi::pipeline> pbr_i(
    {
      {mochi::vt::make<mochi::mat4<f32>>(), vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment},
    },
    {
      {&asset::vertex_i, vk::VertexInputRate::eVertex},
    },
    {
      {&mochi::camera_i, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment},
      {&mochi::light_i, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment},
    }
  );


  std::vector<rhi::shaderSlot> shaders;
  shaders.push_back({vk::ShaderStageFlagBits::eVertex, rhi::shader(eng, "pkgs/lib.qaos.mochi/.qcache/pbr.vert.spv", "main")});
  shaders.push_back({vk::ShaderStageFlagBits::eFragment, rhi::shader(eng, "pkgs/lib.qaos.mochi/.qcache/pbr.frag.spv", "main")});
  
  auto pipe = rhi::pipeline::make(eng, &pbr_i, std::move(shaders));

  auto obj = visual::make(eng,
    &scene,
    mat4<f32>::model({}, quaternion<f32>(), {1}),
    &m3d,
    pipe
  );


  eng.scene() = &scene;
  eng.camera() = cam;
  eng.run();

  return 0;
}




int main()
{
  try { return Main(); }
  
  catch (const vk::SystemError &e) {
    std::cerr << "Vulkan Hatasi: " << e.what() << std::endl;
    return -1;
  }
  catch (const std::exception &e) {
    std::cerr << "Standart Hata: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
