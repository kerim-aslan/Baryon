# Baryon - A custom physics engine
# Copyright (C) 2026 Kerim Aslan
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.20)
project(QAOS_GameEngine_Workspace)

# C++ Standardını 23 yapıyoruz (Baryon modern C++23 özellikleri kullanıyor)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CMake'in MSYS2 Clang ile uyumsuz olan C++20 Module taramasını devre dışı bırakıyoruz.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
# Sanitizer
if(NOT WIN32)
  add_compile_options($<$<CONFIG:Debug>:-fsanitize=address,leak,undefined>)
  add_link_options($<$<CONFIG:Debug>:-fsanitize=address,leak,undefined>)
endif()

# 1. Baryon Core (Saf C++23 fizik motoru - Sıfır dış bağımlılık)
add_subdirectory(baryon)

# 2. İsteğe bağlı Godot GDExtension eklentisi (Klasör varsa ve açıksa derlenir)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gdextension")
    option(BARYON_BUILD_GDEXTENSION "Build Godot GDExtension bindings" OFF)
    if(BARYON_BUILD_GDEXTENSION)
        include(FetchContent)
        FetchContent_Declare(
            godot-cpp
            GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
            GIT_TAG godot-4.2.2-stable
        )
        FetchContent_MakeAvailable(godot-cpp)
        add_subdirectory(gdextension)
    endif()
endif()

# 3. Otomatik Dogrulama Testleri
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/TunnelingTest.cpp")
    add_executable(TunnelingTest tests/TunnelingTest.cpp)
    target_link_libraries(TunnelingTest PRIVATE Baryon::BaryonStatic)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/FeatureVerificationTest.cpp")
    add_executable(FeatureVerificationTest tests/FeatureVerificationTest.cpp)
    target_link_libraries(FeatureVerificationTest PRIVATE Baryon::BaryonStatic)
endif()

# 4. Baryon Görsel Teşhis ve Test Aracı (Baryon Visual Debugger & Testbed)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/debugger/CMakeLists.txt")
    option(BARYON_BUILD_DEBUGGER "Build Baryon Visual Debugger and Diagnostic Tool" ON)
    if(BARYON_BUILD_DEBUGGER)
        add_subdirectory(debugger tools/debugger)
    endif()
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/debugger/CMakeLists.txt")
    option(BARYON_BUILD_DEBUGGER "Build Baryon Visual Debugger and Diagnostic Tool" ON)
    if(BARYON_BUILD_DEBUGGER)
        add_subdirectory(tools/debugger)
    endif()
endif()
