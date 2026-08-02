// simengine/aircraft/aircraft_factory.hpp — Aircraft subsystem.
//
// Builds a fully-populated aircraft entity (all components this Alpha's
// systems need) from plain numeric parameters. This is the seam where a
// real data file (JSON/XML) loader belongs later — see
// assets/aircraft/generic_narrowbody.json for the parameter set this
// function currently hardcodes as defaults, and docs/ROADMAP.md for the
// note on why real A320-type-certificate data isn't wired in for this
// Alpha (the uploaded A320-family.zip is FlightGear-format data/assets,
// not usable as-is by this engine's data model).
//
// Numbers below are representative of a generic twin-jet narrowbody
// (A320/737 class) gathered from public aerodynamic-textbook magnitudes
// (Stevens & Lewis-style coefficients), NOT a certified type's real
// aerodynamic database — sufficient for an Alpha tech-demo baseplate,
// not for anything resembling a real aircraft's handling qualities.

#pragma once

#include "../core/ecs.hpp"
#include "components.hpp"
#include "../io/jsbsim_import.hpp"

namespace simengine::aircraft {

struct SpawnParams {
    simengine::math::Vector3<double> positionNED{0.0, 0.0, -2.0}; // start 2m AGL
    double headingRad = 0.0;
};

// Creates one player-flyable, generic narrowbody aircraft entity with
// RigidBody6DOFComponent, AircraftComponent, FlightControlsComponent,
// LandingGearComponent (nose + 2 main legs), EngineComponent (2 engines),
// and AnimationComponent, all wired consistently with each other (e.g.
// gear leg attach points are plausible relative to the mass properties'
// implied fuselage size).
core::Entity spawnGenericNarrowbody(core::World& world, const SpawnParams& params);

// Builds an aircraft entity from real JSBSim-sourced data (see
// io/jsbsim_import.hpp) — mass, inertia, wing geometry, gear leg
// geometry/spring/damper/friction, and static thrust are the actual
// numbers from the source file; aerodynamic derivatives are the
// importer's linearized approximation of JSBSim's table-driven model
// (see that header's caveats). Falls back to nose+2-main defaults if
// `data.gearLegs` is empty (a malformed/partial import shouldn't crash
// the spawn, just fly worse).
core::Entity spawnFromJSBSim(core::World& world, const io::JSBSimAircraftData& data,
                              const SpawnParams& params);

} // namespace simengine::aircraft
