/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/module/device.hh"
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  device::device(vk::raii::PhysicalDevice phys_dev)
    : vk_phys_dev(phys_dev), vk_device(nil)
  {
    // Find families
    auto props = phys_dev.getQueueFamilyProperties();
    
    std::vector<std::vector<f32>> all_priorities(props.size());
    std::vector<vk::DeviceQueueCreateInfo> queue_infos;
    queue_infos.reserve(props.size());

    for (u32 i{}; i < props.size(); i++) {
      u32 count = props[i].queueCount;

      all_priorities[i].resize(count, 1);

      queue_infos.push_back({
        {}, i, count, all_priorities[i].data()
      });
    }



    // Extensions & Features & pNext
    std::vector<const char*> extensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };

    vk::PhysicalDeviceFeatures features;
    features.samplerAnisotropy = VK_TRUE;
    //features.sampleRateShading = VK_TRUE;
    
    vk::PhysicalDeviceDynamicRenderingFeatures dynamicRenderingFeature;
    dynamicRenderingFeature.dynamicRendering = VK_TRUE;

    

    // Create Device
    vk::DeviceCreateInfo dev_info({}, queue_infos, {}, extensions, &features, &dynamicRenderingFeature);
    vk_device = vk::raii::Device(phys_dev, dev_info);



    // Open Queue
    for (u32 i{}; i < props.size(); i++)
    {
      auto flags = props[i].queueFlags;
      u32 count = props[i].queueCount;

      for (u32 q_idx{}; q_idx < count; q_idx++)
      {
        // GRAPHICS
        if (flags & vk::QueueFlagBits::eGraphics)
          graphics_q().m_primary.push_back(vk::raii::Queue(vk_device, i, q_idx));
        
        // COMPUTE
        if (flags & vk::QueueFlagBits::eCompute) {
          if (!(flags & vk::QueueFlagBits::eGraphics))
            compute_q().m_primary.push_back(vk::raii::Queue(vk_device, i, q_idx));
          else
            compute_q().m_secondary.push_back(vk::raii::Queue(vk_device, i, q_idx));
        }

        // TRANSFER
        if (flags & vk::QueueFlagBits::eTransfer) {
          if (!(flags & vk::QueueFlagBits::eGraphics) && !(flags & vk::QueueFlagBits::eCompute))
            transfer_q().m_primary.push_back(vk::raii::Queue(vk_device, i, q_idx));
          else
            transfer_q().m_secondary.push_back(vk::raii::Queue(vk_device, i, q_idx));
        }
      }
    }


  }

}
