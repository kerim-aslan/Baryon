cmake_minimum_required(VERSION 3.20)
project(QAOS_GameEngine_Workspace)

# C++ Standardını 23 yapıyoruz (RE Engine modern C++23 özellikleri kullanıyor)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Mochi
add_subdirectory(pkgs_mochi/lib.qaos.basis)
add_subdirectory(pkgs_mochi/ext.cgltf)
add_subdirectory(pkgs_mochi/lib.qaos.mochi)

# 2. RE Engine
add_subdirectory(baryon)

# 3. Sandbox
add_subdirectory(sandbox)