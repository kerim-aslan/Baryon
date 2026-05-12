/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "Basis.hh"
#include "mochi/core.hh"
#include "mochi/asset/mesh.hh"
#include "mochi/module/device.hh"
#include "mochi/module/memory.hh"
#include "mochi/module/swapchain.hh"
#include "mochi/rhi/pipeline.hh"
#include "mochi/rhi/buffer.hh"
#include <chrono>
#include <vulkan/vulkan_raii.hpp>



namespace mochi
{

  core::core(
    std::function<vk::raii::PhysicalDevice (vk::raii::PhysicalDevices)> GpuPicker,
    std::function<void (f32 dt)> Idle
  )
    : m_bridge("Mochi Test", {1,0,0,0})
    , m_window(m_bridge, "Mochi Test", 800, 600)
    , m_device(GpuPicker(m_bridge.physicalDevices()))
    , m_swapchain(m_device, m_window)
    , m_renderer(m_device, m_window, m_swapchain)
    , m_memory(m_device, m_renderer)

    , m_idle(Idle)
  {}

  core::~core()
  {
    // Close GPU
    m_device.vdevice().waitIdle();
  }




  fun core::run() -> void
  {
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (m_window.proc_events()) {
      auto current_time = std::chrono::high_resolution_clock::now();
      float dt = std::chrono::duration<float, std::chrono::seconds::period>(current_time-last_time).count();
      last_time = current_time;
        
      // İleride buraya eklenecekler:
      // update_input();
      // update_physics();
      // update_camera();

      m_idle(dt);

      draw();
    }
  }


  fun core::draw() -> void
  {
    auto &cmd = m_renderer.begin_frame();
    m_renderer.begin_swapchain_rendering(cmd, {0.1f, 0.1f, 0.1f, 1.0f});

    // 2. Dinamik Viewport ve Scissor (Karede 1 kez yapılması yeterlidir, döngüye girmez!)
    auto extent = m_swapchain.extent(); 
    cmd.setViewport(0, {vk::Viewport(0.0f, 0.0f, (float)extent.width, (float)extent.height, 0.0f, 1.0f)});
    cmd.setScissor(0, {vk::Rect2D({0, 0}, extent)});

    // --- BİRDEN FAZLA NESNEYİ ÇİZME DÖNGÜSÜ ---
    rhi::pipeline *last_pipeline{};
    rhi::buffer   *last_buffer{};


    // --- YENİ: DESCRIPTOR SET (UBO) BAĞLAMA ---
    for (const auto &obj: sub<module::memory>().list<visual>())
    {
      // 1. Pipeline Bağla
      if (obj->getPipeline() != last_pipeline) {
        cmd.bindPipeline(vk::PipelineBindPoint::eGraphics, *obj->getPipeline()->get());
        last_pipeline = obj->getPipeline();
      }

      // 2. Vertex ve Instance Buffer'ları Bağla
      if (obj->getMesh()->data() != last_buffer) {
        cmd.bindVertexBuffers(0, {*obj->getMesh()->data()->get()}, {0}); 
        last_buffer = obj->getMesh()->data();
      }

      cmd.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics,
        *obj->getPipeline()->layout(),
        0,                       // firstSet (0'dan başlıyor)
        {*obj->getDescSets()[0], },  // Bizim hazırladığımız paket
        {}                       // dynamicOffsets (boş)
      );

      auto model_mat = obj->getModel();
      cmd.pushConstants<mochi::mat4<f32>>(
        *last_pipeline->layout(),
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
        0,                                // Offset
        model_mat                         // Verinin kendisi
      );

      // 3. Draw Komutu
      cmd.draw(obj->getMesh()->data()->size() / asset::vertex_i.stride(), 1, 0, 0);
    }

    // 3. Tuvali Kapat ve Ekrana Gönder
    m_renderer.end_swapchain_rendering(cmd);
    m_renderer.end_frame(cmd);
  }
  
}
