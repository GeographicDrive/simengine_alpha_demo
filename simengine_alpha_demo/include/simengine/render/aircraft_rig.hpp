// simengine/render/aircraft_rig.hpp — Rendering subsystem.
//
// The binding layer between AnimationComponent's named channels
// (produced every tick by systems::AnimationSystem — see
// systems/animation_system.hpp for the full channel list/semantics) and
// per-MeshPart model matrices. This is the ONLY place in the engine that
// knows what a channel value means in *geometric* terms (how many
// radians "surface.aileron_left = 1.0" is, which local axis a hinge
// rotates about, etc.) — physics/animation_system stay unit-agnostic
// (normalized [-1,1]/[0,1]) exactly so this mapping can change (e.g. a
// different aircraft's flap travels 40 degrees instead of 30) without
// touching any other subsystem.
//
// One AircraftRig instance is bound to one entity's aircraft GpuMesh; the
// renderer_demo app owns one per spawned aircraft.

#pragma once

#include <vector>

#include "../aircraft/components.hpp"
#include "../core/ecs.hpp"
#include "gl_renderer.hpp"

namespace simengine::render {

using simengine::math::Matrix4f;
using simengine::math::Vector3f;

struct AircraftRigConfig {
    // Max mechanical travel per surface, matched to the placeholder
    // geometry's proportions (see meshgen::buildPlaceholderAircraft).
    // Override per real aircraft type later; these are generic
    // transport-category numbers.
    float aileronMaxRad = 0.35f;
    float elevatorMaxRad = 0.35f;
    float rudderMaxRad = 0.35f;
    float flapMaxRad = 0.60f;      // ~34 degrees, full flap
    float slatMaxRad = 0.35f;      // slats droop down+forward
    float spoilerMaxRad = 1.0f;    // ~57 degrees, near-vertical when deployed
    float speedbrakeMaxRad = 0.9f;
    float trimMaxRad = 0.20f;
    float gearStowTranslate = -0.55f; // fraction of strut length retracted up into the body
    float nwsMaxRad = 0.6f;
};

// Builds the list of world-space DrawItems for one aircraft entity this
// frame, reading AnimationComponent + the entity's RigidBody6DOFComponent
// (for the aircraft's own world placement) from the ECS. `aircraftOrigin`
// is where body-frame (0,0,0) sits in render-world space (render-world:
// X forward, Y right, Z UP — i.e. body-NED with Z negated once here, so
// every other render-side computation can just assume "up is up").
void appendAircraftDrawItems(
    std::vector<DrawItem>& out,
    const GpuMesh& gpu,
    const simengine::aircraft::AnimationComponent& anim,
    const Matrix4f& aircraftWorldTransform,
    const AircraftRigConfig& cfg = {});

// Converts a RigidBody6DOFState (body-axis NED: X fwd, Y right, Z down)
// into a render-world transform (X fwd, Y right, Z up), i.e. only a
// single-axis (Z) sign flip on both position and the rotation's effect —
// kept as one explicit, well-documented function so this is the one
// place the NED/render handedness conversion happens, not scattered
// inline math throughout the render/camera code.
Matrix4f nedStateToRenderWorld(const simengine::math::Vector3<double>& positionNED,
                                const simengine::math::Quaternion<double>& attitude,
                                const simengine::math::Vector3<double>& renderOriginNED);

} // namespace simengine::render
