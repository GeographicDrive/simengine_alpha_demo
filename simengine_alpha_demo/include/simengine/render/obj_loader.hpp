// simengine/render/obj_loader.hpp — loads Wavefront .obj files (as
// produced by tools/ac3d_to_obj from the A320-family.zip's AC3D meshes)
// into this engine's MeshPart representation, one MeshPart per OBJ `g`
// group (which ac3d_to_obj emits per source AC3D OBJECT name, e.g.
// "AileronL", "FlapR1" — see that tool's header comment).
//
// Coordinate conversion: the source AC3D files use FlightGear/JSBSim's
// structural convention (X aft-positive, Z up-positive, Y right-positive
// — the same convention io/jsbsim_import.cpp already converts from for
// the flight-dynamics data). This loader applies the same axis
// conversion to every vertex/normal it reads: (x, y, z) -> (-x, y, -z),
// giving this engine's forward-positive/right-positive/down-positive
// body frame. Only the axis directions are converted here; the caller
// is responsible for adding whatever body-frame translation places a
// given part correctly (see assets/A320_MESH_NOTES.md for how
// render/mesh_a320.cpp derives those offsets from the aircraft's own
// FlightGear model-composition XML).
#pragma once

#include <string>
#include <unordered_map>

#include "simengine/render/mesh.hpp"

namespace simengine::render::objio {

// Parses an OBJ file (v/vn/f only — sufficient for ac3d_to_obj's output;
// f lines are fan-triangulated if they have more than 3 vertex refs).
// Returns one entry per `g <name>` group encountered; if the same group
// name appears more than once in the file, its faces are accumulated
// into a single MeshPart. Texcoords are dropped (not used by this
// engine's GLES3 shader path yet). Throws std::runtime_error if the
// file can't be opened.
//
// wingSurfaceAxes: the A320-family.zip's wing/tail-surface files
// (a320.wings.ac, a320.hstab.ac, a320.vstab.ac, a320.winglets.ac) turn
// out to use a different local axis layout than the fuselage/gear/
// engine files — confirmed empirically by checking named group extents
// (e.g. AileronL/AileronR mirror across local Z, not local Y) rather
// than assumed. For those four files pass true, which applies
// (x, y, z) -> (-x, -z, -y) [chord-aft, up, left-positive-span ->
// forward, right-positive, down]. All other files pass false, which
// applies the fuselage/JSBSim-structural convention
// (x, y, z) -> (-x, y, -z). See assets/A320_MESH_NOTES.md.
std::unordered_map<std::string, MeshPart> loadObjGroups(const std::string& path, bool wingSurfaceAxes = false);

} // namespace simengine::render::objio
