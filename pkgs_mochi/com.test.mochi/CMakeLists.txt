set(TARGET_NAME com_test_mochi)



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



file(GLOB_RECURSE SOURCES "src/*.cc")

add_executable(${TARGET_NAME} ${SOURCES} ${RES_OBJECTS} ${SPV_FILES})

if(NOT WIN32)
  set(SUFX "shared")
else()
  set(SUFX "static")
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
  qaos_mochi_${SUFX}
)


target_link_options(${TARGET_NAME} PUBLIC -rdynamic)
