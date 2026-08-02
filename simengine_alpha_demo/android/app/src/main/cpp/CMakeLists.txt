# android/app/src/main/cpp/CMakeLists.txt
#
# This does NOT duplicate simengine's source lists. It pulls in the
# project's own root CMakeLists.txt (the exact same one used for the
# desktop build validated in docs/ROADMAP.md) via add_subdirectory, with
# tests/desktop-apps turned off — so simengine_flight and simengine_render
# are, byte-for-byte, the same libraries already verified on desktop
# (RK4/ISA/aero/gear physics, and the GLES3 renderer verified against
# llvmpipe — see docs/ROADMAP.md's "renderer" section). This file only
# adds what's genuinely Android-specific: the NativeActivity entry point
# and its EGL/input glue.
#
# ANDROID_NDK / ANDROID_ABI / ANDROID_PLATFORM are set by Gradle's
# externalNativeBuild via the NDK's CMake toolchain file (see
# app/build.gradle.kts) — this file assumes it's always invoked that way,
# never configured standalone.

cmake_minimum_required(VERSION 3.20)
project(simengine_android)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pull in the real engine build. SIMENGINE_ROOT is passed in from Gradle
# (see app/build.gradle.kts's externalNativeBuild.cmake.arguments) so
# this file works whether the Android project lives inside the main repo
# (the normal case: <repo>/android/) or is relocated.
if(NOT DEFINED SIMENGINE_ROOT)
    set(SIMENGINE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)
endif()
get_filename_component(SIMENGINE_ROOT "${SIMENGINE_ROOT}" ABSOLUTE)

set(SIMENGINE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SIMENGINE_BUILD_APPS OFF CACHE BOOL "" FORCE)   # desktop alpha_demo/renderer_demo need GLFW; not built here
set(SIMENGINE_BUILD_RENDERER ON CACHE BOOL "" FORCE) # NDK sysroot ships GLES3/EGL, so this succeeds on Android

add_subdirectory(${SIMENGINE_ROOT} simengine_core_build)

if(NOT TARGET simengine_render)
    message(FATAL_ERROR
        "simengine_render was not configured — check that the NDK sysroot "
        "exposes GLES3/gl3.h, libGLESv2, and libEGL (it always should; if "
        "this fires, something is wrong with the toolchain file, not the "
        "engine code).")
endif()

# --- android_native_app_glue: the standard NDK NativeActivity bootstrap
# (event loop, input queue, window lifecycle callbacks). Shipped inside
# every NDK install, not vendored here. ---
if(NOT ANDROID_NDK)
    message(FATAL_ERROR "ANDROID_NDK not set — this project must be built through Gradle's externalNativeBuild, not CMake standalone.")
endif()
set(NATIVE_APP_GLUE_DIR ${ANDROID_NDK}/sources/android/native_app_glue)

add_library(simengine_android SHARED
    android_main.cpp
    ${NATIVE_APP_GLUE_DIR}/android_native_app_glue.c
)

target_include_directories(simengine_android PRIVATE ${NATIVE_APP_GLUE_DIR})

# Required so the linker doesn't strip ANativeActivity_onCreate, the
# symbol android.app.NativeActivity looks up via the "android.app.lib_name"
# meta-data in AndroidManifest.xml.
target_link_options(simengine_android PRIVATE "-u" "ANativeActivity_onCreate")

target_link_libraries(simengine_android PRIVATE
    simengine_flight
    simengine_render
    simengine_core
    simengine_math
    android
    log
    EGL
    GLESv3
)
