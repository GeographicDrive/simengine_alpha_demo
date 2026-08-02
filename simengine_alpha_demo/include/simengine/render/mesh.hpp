// simengine/render/mesh.hpp — Rendering subsystem.
//
// Plain-data mesh type (positions/normals/uvs + indices, float32, single
// draw call per part) plus procedural builders for the Alpha's placeholder
// content:
//   - a low-poly "generic narrowbody" aircraft, built from named PARTS so
//     each part can be independently transformed per-frame by
//     AircraftRig (control surfaces hinge, gear translates/rotates,
//     wheels spin, engine fans spin) — this is intentionally NOT a
//     single rigid mesh; every animated part in animation_system.hpp's
//     channel list has a corresponding named part here.
//   - an infinite-looking grey baseplate (a large flat grid quad), the
//     only "world" this Alpha ships with.
//
// This is placeholder art, not final aircraft geometry — the box-and-
// wedge construction here exists purely to prove the rig/animation/
// render pipeline end-to-end. Swapping in real modeled/skinned meshes
// (via tools/ac3d_to_obj or a future glTF importer) later only touches
// this file's builders and AircraftRig's part-name table, nothing else
// in the engine.

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "../math/vector3.hpp"

namespace simengine::render {

using simengine::math::Vector3f;

struct Vertex {
    float px = 0, py = 0, pz = 0;
    float nx = 0, ny = 0, nz = 1;
    float u = 0, v = 0;
};

// One drawable part: its own vertex/index buffer, uploaded as its own
// VAO by GLRenderer. Kept as separate parts (rather than one mesh with
// sub-ranges) because each part gets an independent per-frame transform
// from AircraftRig — separate GPU objects is the simplest correct model
// for a part count this small (dozens, not thousands).
struct MeshPart {
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    // Pivot point in the part's own local modeling space, i.e. the point
    // that AircraftRig's hinge rotation is applied about (e.g. an
    // aileron's hinge line). Vertices are authored relative to this
    // implicitly — AircraftRig applies: worldTransform = attachTransform
    // * translate(pivot) * hingeRotation * translate(-pivot).
    Vector3f pivot{0, 0, 0};
    // Where this part attaches in the aircraft's own body frame (X
    // forward, Y right, Z down — matching RigidBody6DOFState's body
    // axes, so no conversion is needed between physics and rendering
    // orientation) when at its neutral/unanimated position.
    Vector3f attachBody{0, 0, 0};
};

struct StaticMesh {
    std::vector<MeshPart> parts;

    const MeshPart* find(const std::string& name) const noexcept {
        for (auto& p : parts) if (p.name == name) return &p;
        return nullptr;
    }
};

namespace meshgen {

// Axis-aligned box, centered at `center`, given half-extents, in the
// mesh part's local space. Shared helper used by every placeholder part
// below.
void appendBox(MeshPart& part, Vector3f center, Vector3f halfExtent);

// Tapered box (frustum-ish wedge) — used for the fuselage nose/tail cap
// and wingtip taper so the placeholder doesn't read as pure Lego bricks.
void appendWedge(MeshPart& part, Vector3f center, Vector3f halfExtentBase,
                  Vector3f halfExtentTip, float taperAxis /*0=x,1=y,2=z*/);

// Builds the full placeholder aircraft: fuselage, wings, tail, and every
// animatable part named to match AnimationComponent's channel list
// (see systems/animation_system.hpp). Scale is a rough single-aisle
// narrowbody (~38m long) so it reads correctly against the camera's
// default follow distance; pass a different scale for other airframes
// later.
StaticMesh buildPlaceholderAircraft(float lengthMeters = 38.0f);

// Large flat grid on the body XY plane (Z=0), used both as the visible
// grey baseplate and as a distance/motion reference (grid lines) so
// camera motion and aircraft speed are actually perceptible with no
// other scenery in the scene.
StaticMesh buildBaseplate(float halfSizeMeters = 20000.0f, float gridStepMeters = 50.0f);

} // namespace meshgen

} // namespace simengine::render
