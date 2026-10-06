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
