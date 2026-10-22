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

# Derleyici Uyarı Bayrakları (Compiler Warning Flags - E5)
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wshadow -Wconversion)
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

# 3. Otomatik Dogrulama Testleri
enable_testing()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/TunnelingTest.cpp")
    add_executable(TunnelingTest tests/TunnelingTest.cpp)
    target_link_libraries(TunnelingTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME TunnelingTest COMMAND TunnelingTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/FeatureVerificationTest.cpp")
    add_executable(FeatureVerificationTest tests/FeatureVerificationTest.cpp)
    target_link_libraries(FeatureVerificationTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME FeatureVerificationTest COMMAND FeatureVerificationTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/SleepTest.cpp")
    add_executable(SleepTest tests/SleepTest.cpp)
    target_link_libraries(SleepTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME SleepTest COMMAND SleepTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/RestitutionTest.cpp")
    add_executable(RestitutionTest tests/RestitutionTest.cpp)
    target_link_libraries(RestitutionTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME RestitutionTest COMMAND RestitutionTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/RegressionTests.cpp")
    add_executable(RegressionTests tests/RegressionTests.cpp)
    target_link_libraries(RegressionTests PRIVATE Baryon::BaryonStatic)
    add_test(NAME RegressionTests COMMAND RegressionTests)
    set_tests_properties(RegressionTests PROPERTIES LABELS "perf")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/ReadmeCodeTest.cpp")
    add_executable(ReadmeCodeTest tests/ReadmeCodeTest.cpp)
    target_link_libraries(ReadmeCodeTest PRIVATE Baryon::BaryonStatic)
    target_compile_definitions(ReadmeCodeTest PRIVATE 
        CXX_COMPILER="${CMAKE_CXX_COMPILER}"
        BARYON_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    add_test(NAME ReadmeCodeTest COMMAND ReadmeCodeTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/InertiaTest.cpp")
    add_executable(InertiaTest tests/InertiaTest.cpp)
    target_link_libraries(InertiaTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME InertiaTest COMMAND InertiaTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/MissingRegressionTest.cpp")
    add_executable(MissingRegressionTest tests/MissingRegressionTest.cpp)
    target_link_libraries(MissingRegressionTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME MissingRegressionTest COMMAND MissingRegressionTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/FuzzStressTest.cpp")
    add_executable(FuzzStressTest tests/FuzzStressTest.cpp)
    target_link_libraries(FuzzStressTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME FuzzStressTest COMMAND FuzzStressTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/RestingHeightTest.cpp")
    add_executable(RestingHeightTest tests/RestingHeightTest.cpp)
    target_link_libraries(RestingHeightTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME RestingHeightTest COMMAND RestingHeightTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/BroadphaseAllocTest.cpp")
    add_executable(BroadphaseAllocTest tests/BroadphaseAllocTest.cpp)
    target_link_libraries(BroadphaseAllocTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME BroadphaseAllocTest COMMAND BroadphaseAllocTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/DegenerateNarrowphaseTest.cpp")
    add_executable(DegenerateNarrowphaseTest tests/DegenerateNarrowphaseTest.cpp)
    target_link_libraries(DegenerateNarrowphaseTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME DegenerateNarrowphaseTest COMMAND DegenerateNarrowphaseTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/ComponentRegistrationTest.cpp")
    add_executable(ComponentRegistrationTest tests/ComponentRegistrationTest.cpp)
    target_link_libraries(ComponentRegistrationTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME ComponentRegistrationTest COMMAND ComponentRegistrationTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/DebugManagerTest.cpp")
    add_executable(DebugManagerTest tests/DebugManagerTest.cpp)
    target_link_libraries(DebugManagerTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME DebugManagerTest COMMAND DebugManagerTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/MutationAndDenseMeshTest.cpp")
    add_executable(MutationAndDenseMeshTest tests/MutationAndDenseMeshTest.cpp)
    target_link_libraries(MutationAndDenseMeshTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME MutationAndDenseMeshTest COMMAND MutationAndDenseMeshTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/QualityChecksTest.cpp")
    add_executable(QualityChecksTest tests/QualityChecksTest.cpp)
    target_link_libraries(QualityChecksTest PRIVATE Baryon::BaryonStatic)
    target_compile_definitions(QualityChecksTest PRIVATE SOURCE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME QualityChecksTest COMMAND QualityChecksTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/NarrowPhaseRefactorTest.cpp")
    add_executable(NarrowPhaseRefactorTest tests/NarrowPhaseRefactorTest.cpp)
    target_link_libraries(NarrowPhaseRefactorTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME NarrowPhaseRefactorTest COMMAND NarrowPhaseRefactorTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/EntityCapacityTest.cpp")
    add_executable(EntityCapacityTest tests/EntityCapacityTest.cpp)
    target_link_libraries(EntityCapacityTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME EntityCapacityTest COMMAND EntityCapacityTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/TowerStabilityTest.cpp")
    add_executable(TowerStabilityTest tests/TowerStabilityTest.cpp)
    target_link_libraries(TowerStabilityTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME TowerStabilityTest COMMAND TowerStabilityTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CcdBounceAirTest.cpp")
    add_executable(CcdBounceAirTest tests/CcdBounceAirTest.cpp)
    target_link_libraries(CcdBounceAirTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME CcdBounceAirTest COMMAND CcdBounceAirTest)
endif()

# Compound Shape Test (Faz 1.1)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CompoundShapeTest.cpp")
    add_executable(CompoundShapeTest tests/CompoundShapeTest.cpp)
    target_link_libraries(CompoundShapeTest PRIVATE BaryonHarnessLib Baryon::BaryonStatic)
    add_test(NAME CompoundShapeTest COMMAND CompoundShapeTest)
endif()

# Overlap Query Test (Faz 1.2)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/OverlapQueryTest.cpp")
    add_executable(OverlapQueryTest tests/OverlapQueryTest.cpp)
    target_link_libraries(OverlapQueryTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME OverlapQueryTest COMMAND OverlapQueryTest)
endif()

# Sweep Query Test (Faz 1.3)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/SweepQueryTest.cpp")
    add_executable(SweepQueryTest tests/SweepQueryTest.cpp)
    target_link_libraries(SweepQueryTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME SweepQueryTest COMMAND SweepQueryTest)
endif()

# Raycast All Test (Faz 1.4)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/RaycastAllTest.cpp")
    add_executable(RaycastAllTest tests/RaycastAllTest.cpp)
    target_link_libraries(RaycastAllTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME RaycastAllTest COMMAND RaycastAllTest)
endif()

# Cylinder Shape Test (Faz 2.1)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CylinderShapeTest.cpp")
    add_executable(CylinderShapeTest tests/CylinderShapeTest.cpp)
    target_link_libraries(CylinderShapeTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME CylinderShapeTest COMMAND CylinderShapeTest)
endif()

# Cone Shape Test (Faz 2.2)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/ConeShapeTest.cpp")
    add_executable(ConeShapeTest tests/ConeShapeTest.cpp)
    target_link_libraries(ConeShapeTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME ConeShapeTest COMMAND ConeShapeTest)
endif()

# Heightfield Shape Test (Faz 2.3)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/HeightfieldShapeTest.cpp")
    add_executable(HeightfieldShapeTest tests/HeightfieldShapeTest.cpp)
    target_link_libraries(HeightfieldShapeTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME HeightfieldShapeTest COMMAND HeightfieldShapeTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/ConstraintInfrastructureTest.cpp")
    add_executable(ConstraintInfrastructureTest tests/ConstraintInfrastructureTest.cpp)
    target_link_libraries(ConstraintInfrastructureTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME ConstraintInfrastructureTest COMMAND ConstraintInfrastructureTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/FixedConstraintTest.cpp")
    add_executable(FixedConstraintTest tests/FixedConstraintTest.cpp)
    target_link_libraries(FixedConstraintTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME FixedConstraintTest COMMAND FixedConstraintTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/BallSocketConstraintTest.cpp")
    add_executable(BallSocketConstraintTest tests/BallSocketConstraintTest.cpp)
    target_link_libraries(BallSocketConstraintTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME BallSocketConstraintTest COMMAND BallSocketConstraintTest)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/PrismaticConstraintTest.cpp")
    add_executable(PrismaticConstraintTest tests/PrismaticConstraintTest.cpp)
    target_link_libraries(PrismaticConstraintTest PRIVATE Baryon::BaryonStatic)
    add_test(NAME PrismaticConstraintTest COMMAND PrismaticConstraintTest)
endif()



# Headless Physics Harness Library & Executable (Faz 0.1)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/harness/HarnessRunner.cpp")
    add_library(BaryonHarnessLib STATIC tests/harness/HarnessRunner.cpp)
    target_include_directories(BaryonHarnessLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(BaryonHarnessLib PUBLIC Baryon::BaryonStatic)

    add_executable(BaryonHarness tests/harness/main.cpp)
    target_link_libraries(BaryonHarness PRIVATE BaryonHarnessLib)
endif()

# Baseline Regression Test (Faz 0.2)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/BaselineRegressionTest.cpp")
    add_executable(BaselineRegressionTest tests/BaselineRegressionTest.cpp)
    target_link_libraries(BaselineRegressionTest PRIVATE BaryonHarnessLib Baryon::BaryonStatic)
    target_compile_definitions(BaselineRegressionTest PRIVATE 
        BASELINE_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/baselines/baseline_v1.json"
    )
    add_test(NAME BaselineRegressionTest COMMAND BaselineRegressionTest)
endif()


# 4. Baryon Görsel Teşhis ve Test Aracı (Baryon Visual Debugger & Testbed)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/debugger/CMakeLists.txt")
    option(BARYON_BUILD_DEBUGGER "Build Baryon Visual Debugger and Diagnostic Tool" ON)
    if(BARYON_BUILD_DEBUGGER)
        add_subdirectory(debugger tools/debugger)
    endif()
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/debugger/CMakeLists.txt")
    option(BARYON_BUILD_DEBUGGER "Build Baryon Visual Debugger and Diagnostic Tool" ON)
    if(BARYON_BUILD_DEBUGGER)
        add_subdirectory(tools/debugger)
    endif()
endif()
