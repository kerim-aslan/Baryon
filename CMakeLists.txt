cmake_minimum_required(VERSION 3.20)
project(QAOS_GameEngine_Workspace)

# C++ Standardını 23 yapıyoruz (Baryon modern C++23 özellikleri kullanıyor)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sanitizer
if(NOT WIN32)
  add_compile_options($<$<CONFIG:Debug>:-fsanitize=address,leak,undefined>)
  add_link_options($<$<CONFIG:Debug>:-fsanitize=address,leak,undefined>)
endif()

# Thirdparty
file(GLOB childs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty" "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/*")
foreach(child ${childs})
  if(
    IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/${child}" AND
    EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/${child}/CMakeLists.txt"
  )

    message(STATUS "Depency Project Added: ${child}")
    
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/${child}")
      
  endif()
endforeach()


# 2. Baryon
add_subdirectory(baryon)

# 3. Sandbox
add_subdirectory(sandbox)
