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

project(Baryon VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME Baryon)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp")

# Shared Library
add_library(${TARGET_NAME}_shared SHARED ${SOURCES})
target_include_directories(${TARGET_NAME}_shared PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(${TARGET_NAME}_shared PROPERTIES OUTPUT_NAME "${TARGET_NAME}")

# Static Library
add_library(${TARGET_NAME}_static STATIC ${SOURCES})
target_include_directories(${TARGET_NAME}_static PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(${TARGET_NAME}_static PROPERTIES OUTPUT_NAME "${TARGET_NAME}")

if(MSVC)
    target_compile_options(${TARGET_NAME}_shared PRIVATE /W4)
    target_compile_options(${TARGET_NAME}_static PRIVATE /W4)
else()
    target_compile_options(${TARGET_NAME}_shared PRIVATE -Wall -Wextra -Wshadow -Wconversion)
    target_compile_options(${TARGET_NAME}_static PRIVATE -Wall -Wextra -Wshadow -Wconversion)
endif()

# --- Modularity & Extension Options ---
option(BARYON_ENABLE_SIMD "Baryon SIMD kontak cozucu destegi" OFF)
if(BARYON_ENABLE_SIMD)
    target_compile_definitions(${TARGET_NAME}_shared PUBLIC BARYON_ENABLE_SIMD)
    target_compile_definitions(${TARGET_NAME}_static PUBLIC BARYON_ENABLE_SIMD)
    if(MSVC)
        target_compile_options(${TARGET_NAME}_shared PRIVATE /arch:AVX2)
        target_compile_options(${TARGET_NAME}_static PRIVATE /arch:AVX2)
    else()
        target_compile_options(${TARGET_NAME}_shared PRIVATE -mavx2 -mfma)
        target_compile_options(${TARGET_NAME}_static PRIVATE -mavx2 -mfma)
    endif()
endif()

option(BARYON_ENABLE_MULTITHREADING "Baryon cok is parcacikli ada cozucu destegi (std::jthread ThreadPool)" ON)
if(BARYON_ENABLE_MULTITHREADING)
    target_compile_definitions(${TARGET_NAME}_shared PUBLIC BARYON_ENABLE_MULTITHREADING)
    target_compile_definitions(${TARGET_NAME}_static PUBLIC BARYON_ENABLE_MULTITHREADING)
endif()

option(BARYON_ENABLE_DEBUG_TOOLS "Baryon hata ayiklama ve log yoneticisi destegi" ON)
if(BARYON_ENABLE_DEBUG_TOOLS)
    target_compile_definitions(${TARGET_NAME}_shared PUBLIC BARYON_ENABLE_DEBUG_TOOLS)
    target_compile_definitions(${TARGET_NAME}_static PUBLIC BARYON_ENABLE_DEBUG_TOOLS)
endif()

# Modern CMake Alias (Allows target_link_libraries(MyProject PRIVATE Baryon::Baryon))
add_library(Baryon::Baryon ALIAS ${TARGET_NAME}_static)
add_library(Baryon::BaryonStatic ALIAS ${TARGET_NAME}_static)
add_library(Baryon::BaryonShared ALIAS ${TARGET_NAME}_shared)

