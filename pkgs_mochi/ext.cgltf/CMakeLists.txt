set(TARGET_NAME ext_cgltf)

include(FetchContent)

FetchContent_Declare(
  cgltf_fetch
  GIT_REPOSITORY https://github.com/jkuhlmann/cgltf.git
  GIT_TAG        v1.15
)

FetchContent_MakeAvailable(cgltf_fetch)

set(CGLTF_IMPL_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/cgltf_impl.c")
file(WRITE ${CGLTF_IMPL_SOURCE} "#define CGLTF_IMPLEMENTATION\n#include \"cgltf.h\"\n")


add_library(${TARGET_NAME} STATIC ${CGLTF_IMPL_SOURCE})


target_include_directories(${TARGET_NAME} PUBLIC "${cgltf_fetch_SOURCE_DIR}")
