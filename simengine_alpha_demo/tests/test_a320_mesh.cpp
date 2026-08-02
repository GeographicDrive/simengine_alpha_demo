// tests/test_a320_mesh.cpp — checks that buildA320Aircraft() (see
// render/mesh_a320.cpp, and assets/A320_MESH_NOTES.md for the offset/
// axis derivation) produces every part AircraftRig expects, with no
// NaN vertices or degenerate triangles, and overall dimensions that
// land close to the real A320's (37.57 m fuselage length, 34.1 m
// wingspan) — a meaningful check on the offset math even without an
// actual GPU render pass. Run from build/: `./test_a320_mesh`, or pass
// an explicit path to assets/converted_models/ as argv[1].

#include "simengine/render/mesh_a320.hpp"

#include <cmath>
#include <cstdio>
#include <string>

int main(int argc, char** argv) {
    using namespace simengine::render;
    const std::string assetDir = argc >= 2 ? argv[1] : "../assets/converted_models";

    StaticMesh mesh;
    try {
        mesh = meshgen::buildA320Aircraft(assetDir);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "buildA320Aircraft threw: %s\n", e.what());
        return 1;
    }

    static const char* kExpectedParts[] = {
        "fuselage", "wing_L", "wing_R", "aileron_L", "aileron_R", "flap_L", "flap_R",
        "slat_L", "slat_R", "spoiler_L", "spoiler_R", "stabilizer", "elevator_L", "elevator_R",
        "fin", "rudder", "gear_strut_nose", "gear_wheel_nose", "gear_strut_L_main",
        "gear_strut_R_main", "gear_wheel_L_main", "gear_wheel_R_main", "engine_nacelle_L",
        "engine_nacelle_R", "reverser_L", "reverser_R", "engine_fan_L", "engine_fan_R",
    };

    int failures = 0;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    size_t totalVerts = 0, totalTris = 0, nanCount = 0, degenCount = 0;

    for (const char* name : kExpectedParts) {
        const MeshPart* part = mesh.find(name);
        if (!part || part->vertices.empty()) {
            std::fprintf(stderr, "FAIL: missing/empty part \"%s\"\n", name);
            failures++;
            continue;
        }
        totalVerts += part->vertices.size();
        totalTris += part->indices.size() / 3;
        for (auto& v : part->vertices) {
            if (std::isnan(v.px) || std::isnan(v.py) || std::isnan(v.pz)) nanCount++;
            minx = std::min(minx, v.px); maxx = std::max(maxx, v.px);
            miny = std::min(miny, v.py); maxy = std::max(maxy, v.py);
        }
        for (size_t i = 0; i + 2 < part->indices.size(); i += 3) {
            if (part->indices[i] == part->indices[i + 1] ||
                part->indices[i + 1] == part->indices[i + 2] ||
                part->indices[i] == part->indices[i + 2]) {
                degenCount++;
            }
        }
    }

    const float length = maxx - minx;
    const float span = maxy - miny;
    std::printf("parts=%zu (of %zu expected) totalVerts=%zu totalTris=%zu\n",
                mesh.parts.size(), sizeof(kExpectedParts) / sizeof(kExpectedParts[0]),
                totalVerts, totalTris);
    std::printf("fuselage-length=%.2fm (real A320 ~37.57m)  wingspan=%.2fm (real A320 ~34.1m)\n",
                length, span);
    std::printf("NaN vertices=%zu  degenerate triangles=%zu\n", nanCount, degenCount);

    if (nanCount > 0) { std::fprintf(stderr, "FAIL: NaN vertices present\n"); failures++; }
    if (degenCount > 0) { std::fprintf(stderr, "FAIL: degenerate triangles present\n"); failures++; }
    if (std::fabs(length - 37.57f) > 2.0f) { std::fprintf(stderr, "FAIL: fuselage length out of tolerance\n"); failures++; }
    if (std::fabs(span - 34.1f) > 2.0f) { std::fprintf(stderr, "FAIL: wingspan out of tolerance\n"); failures++; }

    if (failures == 0) std::printf("PASS\n");
    return failures == 0 ? 0 : 1;
}
