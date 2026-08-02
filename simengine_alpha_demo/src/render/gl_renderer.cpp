// simengine/render/gl_renderer.cpp — see gl_renderer.hpp for design notes.

#include "simengine/render/gl_renderer.hpp"

#include <GLES3/gl3.h>

#include <cmath>
#include <cstdio>

namespace simengine::render {

namespace {

// Minimal GLES 3.0 shader: single directional light (Lambert) + flat
// ambient floor, per-part flat color via a uniform (not a texture — the
// Alpha has no material/texture pipeline yet, see docs/ROADMAP.md).
const char* kVertexShaderSrc = R"GLSL(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vNormalWorld;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vNormalWorld = mat3(uModel) * aNormal;
    gl_Position = uProj * uView * worldPos;
}
)GLSL";

const char* kFragmentShaderSrc = R"GLSL(#version 300 es
precision mediump float;

in vec3 vNormalWorld;
out vec4 fragColor;

uniform vec3 uColor;
uniform vec3 uLightDir;
uniform float uLightIntensity;

void main() {
    vec3 n = normalize(vNormalWorld);
    float ndl = max(dot(n, -normalize(uLightDir)), 0.0);
    float ambient = 0.35;
    float lit = ambient + (1.0 - ambient) * ndl * uLightIntensity;
    fragColor = vec4(uColor * lit, 1.0);
}
)GLSL";

unsigned int compileShader(unsigned int type, const char* src) {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    int ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[GLRenderer] shader compile failed: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

bool GLRenderer::init() {
    unsigned int vs = compileShader(GL_VERTEX_SHADER, kVertexShaderSrc);
    unsigned int fs = compileShader(GL_FRAGMENT_SHADER, kFragmentShaderSrc);
    if (!vs || !fs) return false;

    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);
    int linked = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &linked);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!linked) {
        char log[1024];
        glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[GLRenderer] program link failed: %s\n", log);
        return false;
    }

    uModel_ = glGetUniformLocation(program_, "uModel");
    uView_ = glGetUniformLocation(program_, "uView");
    uProj_ = glGetUniformLocation(program_, "uProj");
    uColor_ = glGetUniformLocation(program_, "uColor");
    uLightDir_ = glGetUniformLocation(program_, "uLightDir");
    uLightIntensity_ = glGetUniformLocation(program_, "uLightIntensity");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    return true;
}

void GLRenderer::shutdown() {
    if (program_) { glDeleteProgram(program_); program_ = 0; }
}

GpuMesh GLRenderer::upload(const StaticMesh& mesh) const {
    GpuMesh gpu;
    for (const auto& part : mesh.parts) {
        GpuMeshPart gp;
        gp.pivot = part.pivot;
        gp.attachBody = part.attachBody;
        gp.indexCount = static_cast<int>(part.indices.size());

        glGenVertexArrays(1, &gp.vao);
        glBindVertexArray(gp.vao);

        glGenBuffers(1, &gp.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, gp.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<long>(part.vertices.size() * sizeof(Vertex)),
                     part.vertices.data(), GL_STATIC_DRAW);

        glGenBuffers(1, &gp.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gp.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<long>(part.indices.size() * sizeof(uint32_t)),
                     part.indices.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, px)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, nx)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, u)));

        glBindVertexArray(0);
        gpu.parts.emplace(part.name, gp);
    }
    return gpu;
}

void GLRenderer::destroy(GpuMesh& mesh) const {
    for (auto& [name, part] : mesh.parts) {
        glDeleteBuffers(1, &part.vbo);
        glDeleteBuffers(1, &part.ebo);
        glDeleteVertexArrays(1, &part.vao);
    }
    mesh.parts.clear();
}

void GLRenderer::resize(int w, int h) noexcept {
    width_ = w > 0 ? w : 1;
    height_ = h > 0 ? h : 1;
    glViewport(0, 0, width_, height_);
}

void GLRenderer::beginFrame(float skyR, float skyG, float skyB) {
    glClearColor(skyR, skyG, skyB, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void GLRenderer::draw(const std::vector<DrawItem>& items,
                       const Matrix4f& view, const Matrix4f& proj,
                       const DirectionalLight& light) {
    glUseProgram(program_);
    glUniformMatrix4fv(uView_, 1, GL_FALSE, view.data());
    glUniformMatrix4fv(uProj_, 1, GL_FALSE, proj.data());
    glUniform3f(uLightDir_, light.directionWorld.x, light.directionWorld.y, light.directionWorld.z);
    glUniform1f(uLightIntensity_, light.intensity);

    for (const auto& item : items) {
        if (!item.part || item.part->indexCount == 0) continue;
        glUniformMatrix4fv(uModel_, 1, GL_FALSE, item.model.data());
        glUniform3f(uColor_, item.r, item.g, item.b);
        glBindVertexArray(item.part->vao);
        glDrawElements(GL_TRIANGLES, item.part->indexCount, GL_UNSIGNED_INT, nullptr);
    }
    glBindVertexArray(0);
}

Matrix4f perspectiveGL(float fovY, float aspect, float zNear, float zFar) noexcept {
    Matrix4f r;
    for (auto& col : r.m) col.fill(0.0f);
    const float f = 1.0f / std::tan(fovY * 0.5f);
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = -1.0f;
    r.m[3][2] = (2.0f * zFar * zNear) / (zNear - zFar);
    return r;
}

} // namespace simengine::render
