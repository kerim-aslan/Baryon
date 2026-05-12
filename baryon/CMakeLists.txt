cmake_minimum_required(VERSION 3.20)

project(Baryon VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME Baryon)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp")

# Shared Library
add_library(${TARGET_NAME}_shared SHARED ${SOURCES})
target_include_directories(${TARGET_NAME}_shared PUBLIC include)
set_target_properties(${TARGET_NAME}_shared PROPERTIES OUTPUT_NAME "${TARGET_NAME}")

# Static Library
add_library(${TARGET_NAME}_static STATIC ${SOURCES})
target_include_directories(${TARGET_NAME}_static PUBLIC include)
set_target_properties(${TARGET_NAME}_static PROPERTIES OUTPUT_NAME "${TARGET_NAME}")

