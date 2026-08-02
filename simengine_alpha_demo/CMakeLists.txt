cmake_minimum_required(VERSION 3.20)
project(simengine_math LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Math is header-only; expose it as an INTERFACE target so every future
# subsystem (Physics, Flight Dynamics, Rendering, ...) links against the
# same definitions with zero risk of divergence.
add_library(simengine_math INTERFACE)
target_include_directories(simengine_math INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(simengine_math INTERFACE cxx_std_20)

add_library(simengine_core INTERFACE)
target_include_directories(simengine_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(simengine_core INTERFACE cxx_std_20)

find_package(Threads REQUIRED)
target_link_libraries(simengine_core INTERFACE Threads::Threads)

option(SIMENGINE_BUILD_TESTS "Build simengine unit tests" ON)
option(SIMENGINE_BUILD_APPS "Build simengine demo apps" ON)

# --- Flight subsystem: physics + ECS glue systems + aircraft data. This
# is a compiled (non-header-only) library, unlike simengine_math/core,
# because rigid_body_6dof's derivative math is intentionally kept out of
# headers (see rigid_body_6dof.hpp's top comment) so it isn't
# recompiled by every translation unit that merely touches an aircraft
# entity. ---
add_library(simengine_flight STATIC
    src/physics/rigid_body_6dof.cpp
    src/physics/landing_gear.cpp
    src/aircraft/aircraft_factory.cpp
    src/systems/flight_dynamics_system.cpp
    src/systems/landing_gear_system.cpp
    src/systems/engine_system.cpp
    src/systems/animation_system.cpp
    src/systems/input_system.cpp
    src/io/xml.cpp
    src/io/jsbsim_import.cpp
)
target_include_directories(simengine_flight PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(simengine_flight PUBLIC simengine_math simengine_core)
target_compile_features(simengine_flight PUBLIC cxx_std_20)

if(SIMENGINE_BUILD_APPS)
    add_executable(alpha_demo apps/alpha_demo/main.cpp)
    target_link_libraries(alpha_demo PRIVATE simengine_flight simengine_core simengine_math)

    add_executable(fgfs_import tools/fgfs_import/main.cpp)
    target_link_libraries(fgfs_import PRIVATE simengine_flight simengine_core simengine_math)

    add_executable(ac3d_to_obj tools/ac3d_to_obj/main.cpp)
    target_compile_features(ac3d_to_obj PRIVATE cxx_std_20)
endif()

# --- Render subsystem: procedural mesh generation + GLES3 draw layer.
# Header-only public API, compiled .cpp implementation (mesh generation
# and GL calls are non-trivial enough to keep out of headers, same
# rationale as simengine_flight above). Deliberately written against
# GLES3 (not desktop GL) so this exact code is what runs on Android once
# a NativeActivity/EGL entry point replaces apps/renderer_demo's
# GLFW-based one — see that file's header comment. ---
find_package(OpenGL COMPONENTS EGL)
find_path(GLES3_INCLUDE_DIR GLES3/gl3.h)
find_library(GLES_V2_LIBRARY NAMES GLESv2)
find_library(EGL_LIBRARY NAMES EGL)

option(SIMENGINE_BUILD_RENDERER "Build the GLES3 renderer + windowed Alpha demo (requires GLES3/EGL + GLFW dev packages)" ON)

if(SIMENGINE_BUILD_RENDERER AND GLES3_INCLUDE_DIR AND GLES_V2_LIBRARY AND EGL_LIBRARY)
    add_library(simengine_render STATIC
        src/render/mesh.cpp
        src/render/mesh_a320.cpp
        src/render/obj_loader.cpp
        src/render/gl_renderer.cpp
        src/render/aircraft_rig.cpp
    )
    target_include_directories(simengine_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GLES3_INCLUDE_DIR})
    target_link_libraries(simengine_render PUBLIC simengine_math simengine_core ${GLES_V2_LIBRARY} ${EGL_LIBRARY})
    target_compile_features(simengine_render PUBLIC cxx_std_20)

    find_package(glfw3 QUIET)
    if(SIMENGINE_BUILD_APPS AND glfw3_FOUND)
        add_executable(renderer_demo apps/renderer_demo/main.cpp)
        target_link_libraries(renderer_demo PRIVATE
            simengine_flight simengine_render simengine_core simengine_math glfw)
        target_compile_definitions(renderer_demo PRIVATE GLFW_INCLUDE_ES3)
    else()
        message(STATUS "simengine: glfw3 not found (or apps disabled) — renderer_demo (windowed Alpha) skipped; simengine_render library still builds for a future Android entry point.")
    endif()
else()
    message(STATUS "simengine: GLES3/EGL dev headers not found — SIMENGINE_BUILD_RENDERER disabled; physics/ECS/animation targets still build.")
endif()

if(SIMENGINE_BUILD_TESTS)
    enable_testing()

    add_executable(test_math tests/test_math.cpp)
    target_link_libraries(test_math PRIVATE simengine_math)
    add_test(NAME math_unit_tests COMMAND test_math)

    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE simengine_math simengine_core)
    add_test(NAME core_unit_tests COMMAND test_core)

    add_executable(test_flight_dynamics tests/test_flight_dynamics.cpp)
    target_link_libraries(test_flight_dynamics PRIVATE simengine_flight simengine_core simengine_math)
    add_test(NAME flight_dynamics_unit_tests COMMAND test_flight_dynamics)

    add_executable(test_jsbsim_import tests/test_jsbsim_import.cpp)
    target_link_libraries(test_jsbsim_import PRIVATE simengine_flight simengine_core simengine_math)
    add_test(NAME jsbsim_import_unit_tests COMMAND test_jsbsim_import ${CMAKE_CURRENT_SOURCE_DIR}/assets/fgfs_source)

    # mesh_a320.cpp/obj_loader.cpp are pure geometry data (no GL calls),
    # so this test builds and runs even when SIMENGINE_BUILD_RENDERER is
    # off (no GLES3/EGL dev headers available) — compiled directly here
    # rather than depending on the GL-gated simengine_render target.
    add_executable(test_a320_mesh
        tests/test_a320_mesh.cpp
        src/render/mesh_a320.cpp
        src/render/obj_loader.cpp
    )
    add_test(NAME a320_mesh_unit_tests COMMAND test_a320_mesh ${CMAKE_CURRENT_SOURCE_DIR}/assets/converted_models)
endif()
