cmake_minimum_required(VERSION 3.20)
project(SandboxApp)

file(GLOB_RECURSE SRC_FILES "src/*.cc" "src/*.cpp")
file(GLOB_RECURSE INC_FILES "inc/*.hh" "inc/*.hpp")

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INC_FILES})

# Dizinleri CMAKE_CURRENT_SOURCE_DIR ile mutlak yol olarak belirtiyoruz
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc 
    ${CMAKE_CURRENT_SOURCE_DIR}/../baryon/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    qaos_mochi_shared
    Baryon_shared
)

target_link_options(${PROJECT_NAME} PUBLIC -rdynamic)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:qaos_mochi_shared> $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Baryon_shared> $<TARGET_FILE_DIR:${PROJECT_NAME}>
)
