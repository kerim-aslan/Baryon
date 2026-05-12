/*
  This file is part of QAOS

  This file is licensed under the GNU General Public License version 3 (GPL3).

  You should have received a copy of the GNU General Public License
  along with QAOS. If not, see <https://www.gnu.org/licenses/>.

  Copyright (c) 2025-2026 by Kadir Aydın.
*/


#include "mochi/module/window.hh"
#include "Basis.hh"
#include "mochi/module/bridge.hh"
#include <GLFW/glfw3.h>
#include <string>
#include <vulkan/vulkan_raii.hpp>



namespace mochi::module
{

  fun __attribute__((constructor())) window_init() { glfwInit(); }
  fun __attribute__((destructor()))  window_fini() { glfwTerminate(); }



  window::window(bridge &bridge, std::string_view title, int width, int height)
    : vk_surface(nil)
  {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    m_window = glfwCreateWindow(
      width, height,
      std::string(title).c_str(),
      nil, nil
    );

    glfwGetFramebufferSize(m_window, &m_width, &m_height);

    glfwSetWindowUserPointer(m_window, this);

    glfwSetFramebufferSizeCallback(m_window, framebuffer_resize_callback);

    VkSurfaceKHR rawSurface;
    if (glfwCreateWindowSurface(*bridge.inst(), m_window, nil, &rawSurface) != VK_SUCCESS)
      throw std::runtime_error("Yüzey oluşturulamadı!");
    
    vk_surface = vk::raii::SurfaceKHR(bridge.inst(), rawSurface);
  }



  void window::framebuffer_resize_callback(GLFWwindow* win, int width, int height)
  {
    // GLFW penceresinin içine gömdüğümüz "this" pointer'ını geri çıkartıyoruz
    auto app = reinterpret_cast<window*>(glfwGetWindowUserPointer(win));
    
    // Artık sınıfın private üyelerine erişebiliriz!
    app->m_resized = true;
    app->m_width = width;
    app->m_height = height;
  }

  fun window::proc_events() -> bool
  {
    glfwPollEvents();

    // Eğer pencere minimize edildiyse (boyutlar 0 olduysa), uygulama alta alınmış demektir.
    // Vulkan 0 piksele çizim yapamaz, bu yüzden pencere tekrar açılana kadar motoru uyutuyoruz.
    if (m_width == 0 || m_height == 0) {
        glfwWaitEvents(); // CPU'yu yormamak için aktif bekleme (Sleep) yapar
        return !glfwWindowShouldClose(m_window);
    }

    return !glfwWindowShouldClose(m_window);
  }

}
