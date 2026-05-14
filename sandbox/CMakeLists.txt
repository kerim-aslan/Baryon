cmake_minimum_required(VERSION 3.20)
project(SandboxApp)

file(GLOB_RECURSE SRC_FILES "src/*.cc" "src/*.cpp")
file(GLOB_RECURSE INC_FILES "inc/*.hh" "inc/*.hpp")

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INC_FILES})

# *** BAŞLIK DOSYASI YOLLARINI AÇIKÇA BELİRTİYORUZ ***
target_include_directories(${PROJECT_NAME} PRIVATE 
    inc 
    ../baryon/include
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    qaos_mochi_shared
    Baryon_shared
)

target_link_options(${PROJECT_NAME} PUBLIC -rdynamic)
