// simengine/aircraft/components.hpp — Aircraft/ECS glue subsystem.
//
// Plain-data ECS components for an aircraft entity. These are what
// FlightDynamicsSystem, LandingGearSystem, EngineSystem, AnimationSystem
// and InputSystem read/write each tick via World::storage<T>(). No
// behavior lives here — components stay pure data so JobSystem::parallelFor
// can iterate ComponentStorage<T>::dense() safely across threads.

#pragma once

#include <array>
#include <string>
#include <vector>

#include "../math/vector3.hpp"
#include "../physics/rigid_body_6dof.hpp"
#include "../physics/landing_gear.hpp"

namespace simengine::aircraft {

using simengine::math::Vector3;
using simengine::physics::RigidBody6DOFState;
using simengine::physics::FlightControls;
using simengine::physics::MassProperties;
using simengine::physics::AeroCoefficients;
using simengine::physics::PropulsionModel;
using simengine::physics::GearLegConfig;
using simengine::physics::GearLegState;

// --- Core rigid body component: wraps the physics module's state so it
// slots directly into World::storage<RigidBody6DOFComponent>(). ---
struct RigidBody6DOFComponent {
    RigidBody6DOFState state;
};

// --- Static aircraft definition (loaded once from a data file, shared
// conceptually per aircraft type; stored per-entity here for simplicity
// so FlightDynamicsSystem needs only one component lookup per entity —
// a future optimization can move this to a shared/interned resource if
// many identical aircraft are ever spawned at once). ---
struct AircraftComponent {
    std::string typeName;              // e.g. "generic-narrowbody"
    MassProperties mass;
    AeroCoefficients aero;
    PropulsionModel propulsion; // primary/reference engine model
    int engineCount = 2;
};

// --- Raw control inputs, written by InputSystem from touch UI events,
// read by FlightDynamicsSystem/EngineSystem/LandingGearSystem. Kept
// separate from FlightControls (the physics module's own struct) since
// this component carries a few extra discrete/ground-only controls the
// physics core doesn't need to know about (parking brake, tiller). ---
struct FlightControlsComponent {
    FlightControls controls;   // aero surfaces + throttle + gear/flap/brake/reverse flags
    double flapHandlePosition = 0.0; // [0,1] commanded, smoothed toward by system
    double slatHandlePosition = 0.0; // [0,1] derived from an auto-slat schedule
                                      // (see EngineSystem/FlightDynamicsSystem
                                      // wiring) but kept as its own channel so
                                      // it is independently inspectable/testable
    double speedbrakeHandle = 0.0;   // [0,1] dedicated speedbrake surface
    double spoilerHandle = 0.0;      // [0,1] ground/flight spoiler lever
    double trimCommand = 0.0;        // [-1,1] rate command into deltaThsDeg
    double tillerCommand = 0.0;      // [-1,1] nose wheel steering (ground, low speed)
    double parkingBrake = 0.0;       // [0,1] (treated as boolean by systems, kept analog for UI)
    bool pauseRequested = false;
};

// --- One landing gear leg. An aircraft has several (nose + main gear
// legs, or more for multi-bogey types); modeled as a fixed-size array on
// the LandingGearComponent below rather than N separate WheelComponents,
// since gear legs on one aircraft are always processed together and
// never queried independently across entities. ---
struct WheelComponent {
    GearLegConfig config;
    GearLegState state;
    std::string animationNodeName; // e.g. "wheel_L_main_1"
};

struct LandingGearComponent {
    std::vector<WheelComponent> legs;   // index 0 = nose, rest = main gear
    double gearPosition = 1.0;          // [0,1] 1 = down/locked, 0 = up/stowed, for animation
    double gearTargetPosition = 1.0;
    double gearTransitionRate = 0.25;   // fraction of travel per second
    bool weightOnWheels = false;        // derived, any leg on ground
};

// --- Per-engine runtime state, one AircraftComponent may drive several
// via EngineComponent::engines. ---
struct EngineState {
    double n1Percent = 0.0;       // fan speed, for spool animation + sound hook
    double n2Percent = 0.0;
    double commandedThrottle = 0.0;
    double thrustNewtons = 0.0;
    bool reverserDeployed = false;
    double reverserPosition = 0.0; // [0,1] for animation
    bool running = true;
};

struct EngineComponent {
    std::vector<EngineState> engines;
    double spoolRateUp = 0.25;   // fraction N1 per second toward commanded
    double spoolRateDown = 0.18;
};

// --- Drives the AnimationSystem: named animation channels in [0,1] (or
// signed for symmetric surfaces), decoupled from physics entirely —
// FlightDynamicsSystem/LandingGearSystem/EngineSystem only ever write
// into this component; the renderer-facing AnimationSystem only ever
// reads it and pushes values into whatever node/bone/morph binding the
// rendering backend uses (not implemented in this Alpha — see
// docs/ROADMAP.md). ---
struct AnimationComponent {
    struct Channel {
        std::string name;
        double value = 0.0; // semantics documented per-channel in animation_system.hpp
    };
    std::vector<Channel> channels;

    // O(n) linear lookup is fine here: channel counts are small (dozens,
    // not thousands) and this runs once per entity per tick, not in an
    // inner loop.
    double* find(const std::string& name) noexcept {
        for (auto& c : channels) if (c.name == name) return &c.value;
        return nullptr;
    }
    double& getOrAdd(const std::string& name) {
        if (auto* v = find(name)) return *v;
        channels.push_back({name, 0.0});
        return channels.back().value;
    }
};

} // namespace simengine::aircraft
