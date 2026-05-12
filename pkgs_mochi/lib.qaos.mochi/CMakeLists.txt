set(TARGET_NAME qaos_mochi)


file(GLOB_RECURSE SOURCES "src/*.cc")
file(GLOB_RECURSE HEADERS "inc/*.hh")


find_package(Vulkan REQUIRED)

find_package(glfw3 CONFIG QUIET)
if(glfw3_FOUND)
  set(GLFW_TARGET glfw)
  if(TARGET glfw3::glfw)
    set(GLFW_TARGET glfw3::glfw)
  endif()
else()
  find_package(GLFW REQUIRED)
  set(GLFW_TARGET glfw)
endif()

add_library(glfw_ INTERFACE)
target_link_libraries(glfw_ INTERFACE ${GLFW_TARGET})


# Shader Compile

find_program(GLSLC glslc REQUIRED)

file(GLOB_RECURSE SHADERS "res/*.vert" "res/*.frag")

set(SPV_FILES "")

foreach(SHADER IN LISTS SHADERS)
  get_filename_component(FILE_NAME ${SHADER} NAME)
  
  set(SPV_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.qcache/${FILE_NAME}.spv")
  
  add_custom_command(
    OUTPUT ${SPV_FILE}
    COMMAND ${GLSLC} ${SHADER} -o ${SPV_FILE}
    DEPENDS ${SHADER}
    COMMENT "Vulkan Shader Compiling: ${FILE_NAME}"
  )
  
  list(APPEND SPV_FILES ${SPV_FILE})
endforeach()

# Shader Compile


add_library(${TARGET_NAME}_shared SHARED ${SOURCES} ${SPV_FILES})
add_library(${TARGET_NAME}_static STATIC ${SOURCES} ${SPV_FILES})


target_include_directories(${TARGET_NAME}_shared PUBLIC inc)

target_link_libraries(${TARGET_NAME}_shared PUBLIC
  qaos_basis

  glfw_
  Vulkan::Vulkan
  ext_cgltf
)

set_target_properties(${TARGET_NAME}_shared PROPERTIES 
  OUTPUT_NAME "${TARGET_NAME}"
)



target_include_directories(${TARGET_NAME}_static PUBLIC inc)

target_link_libraries(${TARGET_NAME}_static PUBLIC
  qaos_basis

  glfw_
  Vulkan::Vulkan
  ext_cgltf
)

set_target_properties(${TARGET_NAME}_static PROPERTIES 
  OUTPUT_NAME "${TARGET_NAME}"
)

