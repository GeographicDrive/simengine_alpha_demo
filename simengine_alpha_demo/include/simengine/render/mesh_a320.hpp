// simengine/render/mesh_a320.hpp — assembles the real A320 airframe
// (converted from the A320-family.zip AC3D meshes via tools/ac3d_to_obj)
// into this engine's StaticMesh/MeshPart representation, replacing
// mesh.cpp's procedural placeholder for aircraft::spawnFromJSBSim().
//
// See assets/A320_MESH_NOTES.md for how the body-frame offsets baked
// into each part were derived from the aircraft's FlightGear model XML
// (Aircraft/A320-family/XMLs/A320.xml) and cross-checked against the
// JSBSim gear positions already used by io/jsbsim_import.cpp.
#pragma once

#include <string>

#include "simengine/render/mesh.hpp"

namespace simengine::render::meshgen {

// assetDir: directory containing the converted .obj files (this repo's
// assets/converted_models/). Throws std::runtime_error (propagated from
// objio::loadObjGroups) if a required file is missing.
//
// Not every AircraftRig part name has a real mesh yet — see the
// "known gaps" list in A320_MESH_NOTES.md (speedbrake and both
// engine_fan_* currently fall back silently to "no geometry", which
// AircraftRig already handles by skipping missing parts).
StaticMesh buildA320Aircraft(const std::string& assetDir);

} // namespace simengine::render::meshgen
