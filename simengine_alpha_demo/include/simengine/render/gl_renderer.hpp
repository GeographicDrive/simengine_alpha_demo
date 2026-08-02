// simengine/render/gl_renderer.hpp — Rendering subsystem.
//
// The first functional 3D renderer for the engine. Deliberately written
// against the OpenGL ES 3.0 API only (no desktop-only extensions, no
// fixed-function calls) so this exact translation unit is what will run
// on Android once a native-activity/EGL window layer is added — GLFW is
// used only by apps/renderer_demo for desktop *windowing/input*, and is
// requested to hand back a real GLES3 context (see main.cpp), not a
// desktop-GL context, specifically so nothing here is written against
// something Android can't run.
//
// Design:
//  - One VAO/VBO/EBO triple per MeshPart, uploaded once at load time
//    (buildPlaceholderAircraft()/buildBaseplate() output is static
//    geometry — nothing here streams per-frame vertex data).
//  - A single flat-lit (Lambert + ambient) shader for the whole Alpha —
//    correct scope for "prove the pipeline draws the right shapes with
//    the right transforms", not a PBR pass; swapping the shader later
//    doesn't touch anything else in this file's public API.
//  - No global GL state assumptions beyond what init() sets up; safe to
//    be the only renderer instantiated by an app (this Alpha's case).

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "../math/matrix4.hpp"
#include "mesh.hpp"

namespace simengine::render {

using simengine::math::Matrix4f;

// One GL-side buffer set for a MeshPart, plus the part's authored pivot/
// attach data carried through unchanged (AircraftRig needs it every
// frame to build the per-part model matrix).
struct GpuMeshPart {
    unsigned int vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;
    Vector3f pivot{0, 0, 0};
    Vector3f attachBody{0, 0, 0};
};

struct GpuMesh {
    std::unordered_map<std::string, GpuMeshPart> parts;
};

struct DrawItem {
    const GpuMeshPart* part = nullptr;
    Matrix4f model;             // full world transform for this part
    float r = 0.75f, g = 0.75f, b = 0.78f; // flat base color
};

struct DirectionalLight {
    Vector3f directionWorld{0.4f, 0.3f, -0.85f}; // pointing FROM the light
    float intensity = 1.0f;
};

class GLRenderer {
public:
    // Must be called once with a current GL context bound (desktop:
    // after glfwMakeContextCurrent; Android: after eglMakeCurrent).
    // Returns false (with a message on stderr) on shader compile/link
    // failure so the caller can fail fast instead of drawing garbage.
    bool init();
    void shutdown();

    // Uploads every part of `mesh` as GPU buffers; the returned GpuMesh
    // owns the GL handles and must outlive any frame that draws from it.
    // Safe to call multiple times for different meshes (aircraft,
    // baseplate, future scenery) — each call is independent.
    GpuMesh upload(const StaticMesh& mesh) const;
    void destroy(GpuMesh& mesh) const;

    void resize(int widthPx, int heightPx) noexcept;

    void beginFrame(float skyR, float skyG, float skyB);
    void draw(const std::vector<DrawItem>& items,
              const Matrix4f& view, const Matrix4f& proj,
              const DirectionalLight& light);
    void endFrame() {} // present/swap is the windowing layer's job (main.cpp)

    int viewportWidth() const noexcept { return width_; }
    int viewportHeight() const noexcept { return height_; }

private:
    unsigned int program_ = 0;
    int uModel_ = -1, uView_ = -1, uProj_ = -1, uColor_ = -1, uLightDir_ = -1, uLightIntensity_ = -1;
    int width_ = 1, height_ = 1;
};

// Standard OpenGL (NDC z in [-1,1], no Y-flip) right-handed perspective
// projection — deliberately separate from Matrix4::perspectiveVulkan(),
// which uses Vulkan clip-space conventions unsuitable for GL/GLES
// directly (this module targets GLES3, not Vulkan).
Matrix4f perspectiveGL(float fovYRadians, float aspect, float zNear, float zFar) noexcept;

} // namespace simengine::render
