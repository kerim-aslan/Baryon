/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/module/renderer.hh"
#include "Basis.hh"
#include "mochi/module/device.hh"
#include "mochi/module/window.hh"
#include <vulkan/vulkan_raii.hpp>

#define ef else if



namespace mochi::module
{

  renderer::renderer(device &device, window &window, swapchain &swapchain)
    : m_device(device)
    , m_window(window)
    , m_swapchain(swapchain)
    , m_cmd_pool(nil)
  {
    // Command Pool
    vk::CommandPoolCreateInfo pool_info(
      vk::CommandPoolCreateFlagBits::eResetCommandBuffer, 
      0
    );
    m_cmd_pool = vk::raii::CommandPool(m_device.vdevice(), pool_info);

    
    // Command Buffer
    vk::CommandBufferAllocateInfo alloc_info(*m_cmd_pool, vk::CommandBufferLevel::ePrimary, MAX_FRAMES_IN_FLIGHT);
    m_cmd_buffers = vk::raii::CommandBuffers(m_device.vdevice(), alloc_info);



    // Synchronize
    vk::SemaphoreCreateInfo sem_info{};
    vk::FenceCreateInfo fence_info(vk::FenceCreateFlagBits::eSignaled);

    for (u32 i{}; i < MAX_FRAMES_IN_FLIGHT; i++) {
      m_image_available_sems.push_back(vk::raii::Semaphore(m_device.vdevice(), sem_info));
      m_in_flight_fences.push_back(vk::raii::Fence(m_device.vdevice(), fence_info));
    }

    for (u32 i{}; i < m_swapchain.image_count(); i++) {
      m_render_finished_sems.push_back(vk::raii::Semaphore(m_device.vdevice(), sem_info));
    }
  }

  fun renderer::begin_swapchain_rendering(vk::raii::CommandBuffer &cmd, const std::array<float, 4> &clear_color) -> void
  {
    vk::ImageMemoryBarrier color_barrier(
      {}, vk::AccessFlagBits::eColorAttachmentWrite,
      vk::ImageLayout::eUndefined, vk::ImageLayout::eColorAttachmentOptimal,
      VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
      m_swapchain.images()[m_image_index], 
      {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
    );

    vk::ImageMemoryBarrier depth_barrier(
      vk::AccessFlagBits::eNone, vk::AccessFlagBits::eDepthStencilAttachmentWrite | vk::AccessFlagBits::eDepthStencilAttachmentRead,
      vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthAttachmentOptimal,
      VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
      *m_swapchain.depth_image(), 
      {vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1}
    );

    cmd.pipelineBarrier(
      vk::PipelineStageFlagBits::eTopOfPipe, 
      vk::PipelineStageFlagBits::eColorAttachmentOutput | vk::PipelineStageFlagBits::eEarlyFragmentTests,
      {}, {}, {}, {color_barrier, depth_barrier}
    );

    // 2. Renk ve Derinlik Temizleme (Clear) Değerleri
    vk::ClearValue clear_color_val(clear_color);
    vk::ClearValue clear_depth_val;
    clear_depth_val.depthStencil = vk::ClearDepthStencilValue(1.0f, 0); // 1.0 = En uzak

    // 3. Renk Eklentisi (Mevcut olan)
    vk::RenderingAttachmentInfo color_attachment(
      *m_swapchain.image_views()[m_image_index], 
      vk::ImageLayout::eColorAttachmentOptimal,
      vk::ResolveModeFlagBits::eNone, nil, vk::ImageLayout::eUndefined,
      vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore, clear_color_val
    );

    // 4. Derinlik Eklentisi (YENİ)
    vk::RenderingAttachmentInfo depth_attachment(
      *m_swapchain.depth_view(),
      vk::ImageLayout::eDepthAttachmentOptimal,
      vk::ResolveModeFlagBits::eNone, nil, vk::ImageLayout::eUndefined,
      vk::AttachmentLoadOp::eClear, vk::AttachmentStoreOp::eStore, clear_depth_val
    );
    
    // 5. Dynamic Rendering Başlat
    vk::RenderingInfo render_info(
      {}, {{0, 0}, m_swapchain.extent()}, 
      1, 0, 1, &color_attachment, &depth_attachment, nil
    );
    cmd.beginRendering(render_info);
  }

  fun renderer::end_swapchain_rendering(vk::raii::CommandBuffer &cmd) -> void
  {
    // 1. Dynamic Rendering Bitir
    cmd.endRendering();

    // 2. Resmi Ekrana Hazırla (ColorAttachment -> PresentSrc)
    vk::ImageMemoryBarrier img_barrier(
      vk::AccessFlagBits::eColorAttachmentWrite, {},
      vk::ImageLayout::eColorAttachmentOptimal, vk::ImageLayout::ePresentSrcKHR,
      VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
      m_swapchain.images()[m_image_index], 
      {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}
    );
    
    cmd.pipelineBarrier(
      vk::PipelineStageFlagBits::eColorAttachmentOutput, vk::PipelineStageFlagBits::eBottomOfPipe,
      {}, {}, {}, {img_barrier}
    );
  }



  fun renderer::begin_frame() -> vk::raii::CommandBuffer&
  {
    vk::Result err = m_device.vdevice().waitForFences({*m_in_flight_fences[m_current_frame]}, VK_TRUE, UINT64_MAX);

    if (err != vk::Result::eSuccess)
      throw std::runtime_error("GPU beklenirken hata oluştu veya cihaz kaybedildi!");
    

    // Request a new image from Swapchain
    auto [result, img_idx] = m_swapchain.get().acquireNextImage(
      UINT64_MAX, 
      *m_image_available_sems[m_current_frame], 
      nil
    );
    m_image_index = img_idx;


    // Close the fence
    m_device.vdevice().resetFences({*m_in_flight_fences[m_current_frame]});


    // Reset the command prompt and start typing
    vk::raii::CommandBuffer& cmd = m_cmd_buffers[m_current_frame];
    cmd.reset();
    cmd.begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    return cmd;
  }

  fun renderer::end_frame(vk::raii::CommandBuffer &cmd) -> void
  {
    cmd.end();

    // GPU Submit
    vk::PipelineStageFlags wait_stage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    
    vk::SubmitInfo submit_info(
      1, &*m_image_available_sems[m_current_frame], &wait_stage,
      1, &*cmd,
      1, &*m_render_finished_sems[m_image_index]
    );

    m_device.graphics_q().best().submit(submit_info, *m_in_flight_fences[m_current_frame]);


    // Screen Present
    vk::PresentInfoKHR present_info(
      1, &*m_render_finished_sems[m_image_index],
      1, &*m_swapchain.get(),
      &m_image_index
    );


    // Send Output
    // *** C++ İSTİSNASINI (EXCEPTION) YAKALAMAK İÇİN TRY-CATCH EKLENDİ ***
    try 
    {
      auto err = m_device.graphics_q().best().presentKHR(present_info);

      if (err == vk::Result::eErrorOutOfDateKHR || err == vk::Result::eSuboptimalKHR || m_window.resized())
      {
        m_swapchain.recreate();
      }
      ef (err != vk::Result::eSuccess)
        throw std::runtime_error("Ekrana goruntu basilirken kritik hata olustu!");
    }
    catch (const vk::OutOfDateKHRError &) // Bazı sürümlerde doğrudan bu exception fırlatılır
    {
      m_swapchain.recreate();
    }
    catch (const vk::SystemError &e) // Genel sistem/Vulkan hatası fırlatılırsa
    {
      if (e.code() == vk::Result::eErrorOutOfDateKHR || e.code() == vk::Result::eSuboptimalKHR) {
        m_swapchain.recreate();
      } else {
        throw e; // Gerçekten kritik başka bir hataysa fırlatmaya devam et
      }
    }

    // Next Frame
    m_current_frame = (m_current_frame + 1) % MAX_FRAMES_IN_FLIGHT;
  }

}
