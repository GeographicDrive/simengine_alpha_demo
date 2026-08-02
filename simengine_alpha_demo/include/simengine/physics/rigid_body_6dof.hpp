// simengine/physics/rigid_body_6dof.hpp — Physics / Flight Dynamics subsystem.
//
// Full 6-DOF body-axis rigid body model with coupled (asymmetric) inertia,
// integrated with classic RK4. Ported from the flight model proven out in
// GeoDrive's `AdvancedFlightDynamics` (Stevens & Lewis "Aircraft Control
// and Simulation" formulation: gamma-coefficient inertia coupling,
// quaternion attitude, gravity via DCM).
//
// This header declares data types and the public API only. The physics
// implementation lives in rigid_body_6dof.cpp — kept out of the header so
// that:
//   - every translation unit that touches an aircraft entity (ECS systems,
//     the animation-binding module, tests) doesn't have to recompile the
//     derivative math on every touch; this function is large and will only
//     grow (ground-effect, icing, structural flex hooks, etc.).
//   - the derivative/integration internals can be unit-tested and later
//     optimized (SIMD/batched variants) without changing any caller.
//
// Design notes (unchanged from the original header-only draft):
//  - State does NOT store Euler angles anywhere — attitude is the unit
//    Quaternion<double> from math/quaternion.hpp, consistent with the
//    engine-wide invariant (see ARCHITECTURE.md). eulerZYX() is a
//    read-only derived view for instrumentation/animation binding only.
//  - Position is stored in local double-precision NED meters, relative to
//    whatever origin the caller rebased against (see math/geodetic.hpp).
//    This module does not know about ECEF/lat-lon at all; that conversion
//    is the caller's job.
//  - AeroCoefficients / MassProperties / PropulsionModel are plain data —
//    intended to be loaded from a per-aircraft data file (JSON or
//    similar), not hardcoded, so new aircraft don't require an engine
//    rebuild.
//  - calculateDerivatives() is a pure function: no hidden state, safe to
//    call from multiple RK4 stages and from multiple threads for
//    different entities concurrently (no shared mutable state anywhere in
//    this module).
//  - One RigidBody6DOFState per entity, iterated via JobSystem::parallelFor
//    over all entities holding this component — the model itself has no
//    knowledge of the ECS; a thin FlightDynamicsSystem (not in this file)
//    is what does that iteration and controls-component lookup.

#pragma once

#include "../math/vector3.hpp"
#include "../math/quaternion.hpp"

namespace simengine::physics {

using simengine::math::Vector3;
using simengine::math::Quaternion;

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

struct RigidBody6DOFState {
    double u = 0.0, v = 0.0, w = 0.0;        // body-axis velocity, m/s
    double p = 0.0, q = 0.0, r = 0.0;        // body rates, rad/s
    Quaternion<double> attitude = Quaternion<double>::identity();
    Vector3<double> positionNED{0.0, 0.0, 0.0}; // meters, +z down

    void normalizeAttitude() noexcept { attitude.normalize(); }

    // Derived, for display/instrumentation/animation binding only — never
    // fed back into the integrator.
    double trueAirspeed() const noexcept;
    double angleOfAttack() const noexcept;
    double sideslip() const noexcept;
    Vector3<double> eulerZYX() const noexcept;
    double altitudeMeters() const noexcept { return -positionNED.z; }
};

// A state derivative has the exact same shape as the state itself (the
// integrator advances every field by its derivative), so we reuse the
// struct rather than defining a parallel one.
using RigidBody6DOFDerivative = RigidBody6DOFState;

// ---------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------

struct FlightControls {
    double deltaElevator = 0.0;   // rad
    double deltaAileron = 0.0;    // rad
    double deltaRudder = 0.0;     // rad
    double deltaThsDeg = 0.0;     // trimmable horizontal stabilizer, degrees
    double throttle = 0.0;        // [0,1]
    bool gearDown = false;

    // Continuous high-lift / drag devices (Alpha: each modeled as its own
    // aerodynamic contributor rather than ganged booleans). All [0,1],
    // 0 = stowed/retracted, 1 = fully deployed.
    double flapPosition = 0.0;      // trailing-edge flaps
    double slatPosition = 0.0;      // leading-edge slats (independent of flap)
    double spoilerPosition = 0.0;   // ground/flight spoilers (lift dump + drag)
    double speedbrakePosition = 0.0; // dedicated speedbrake surface (drag only)

    double brakeCommand = 0.0;      // [0,1] wheel brakes (kept so the
                                     // physics core's drag/decel picture is
                                     // self-consistent; ground reaction
                                     // itself is computed by landing_gear.*)
    bool brakeActive = false;       // convenience flag: brakeCommand > 0
    double nwsCommand = 0.0;        // [-1,1] nose wheel steering tiller/pedals
    bool reverseActive = false;

    // Backward-compat accessor: some older call sites still checked a
    // simple "flaps down" boolean. Kept as a thin helper over
    // flapPosition so both styles interoperate during migration.
    bool flapsDown() const noexcept { return flapPosition > 0.01; }
};

// ---------------------------------------------------------------------
// Aircraft data (per-aircraft, data-driven — not hardcoded per type)
// ---------------------------------------------------------------------

struct MassProperties {
    double mass = 0.0;   // kg
    double S = 0.0;      // wing area, m^2
    double b = 0.0;      // wingspan, m
    double c = 0.0;      // mean aerodynamic chord, m
    double AR = 0.0;     // aspect ratio (b^2 / S)
    double Ix = 0.0, Iy = 0.0, Iz = 0.0, Ixz = 0.0; // kg*m^2

    // Precomputed gamma coefficients (Stevens & Lewis) for the coupled
    // rotational equations of motion. Computed once from the inertia
    // tensor above — call finalize() after populating Ix/Iy/Iz/Ixz.
    struct Gammas {
        double G1 = 0, G2 = 0, G3 = 0, G4 = 0, G5 = 0, G6 = 0, G7 = 0, G8 = 0;
    } gamma;

    // Must be called once after Ix/Iy/Iz/Ixz are populated (and again if
    // any of them ever change, e.g. fuel burn moving CG — not modeled
    // here, but the hook point is this function).
    void finalize() noexcept;
};

struct AeroCoefficients {
    double CL0 = 0, CLa = 0, CLq = 0, CLde = 0;
    double CD0 = 0, e = 0.78, Mcrit = 1.0, Kc = 0;
    double alphaStall = 0.25;       // rad
    double CD_gear = 0, CD_brake = 0;
    double CLflap = 0, alphaStallFlap = 0, CD_flap = 0;
    double alphaStallSlat = 0.0, CD_slat = 0.0;
    double CLspoiler = 0.0, CD_spoiler = 0.0;
    double CD_speedbrake = 0.0;
    // Flight-envelope (load-factor) protection limits. Leave
    // hasLoadFactorProtection = false for aircraft with no envelope
    // protection (e.g. a light GA aircraft in direct law).
    bool hasLoadFactorProtection = false;
    double nMaxClean = 2.5, nMinClean = -1.0;
    double nMaxFlap = 2.0, nMinFlap = 0.0;
    double CYb = 0, CYp = 0, CYdr = 0;
    double Clb = 0, Clp = 0, Clda = 0;
    double Cm0 = 0, Cma = 0, Cmq = 0, Cmde = 0, Cmdt = 0;
    double Cnb = 0, Cnp = 0, Cnr = 0, Cndr = 0;
};

enum class PropulsionType { Jet, Piston };

struct PropulsionModel {
    PropulsionType type = PropulsionType::Jet;

    // Jet
    double T_max = 0.0; // N, static max thrust

    // Piston / propeller
    double P_max = 0.0;          // W
    double RPM = 2700.0;
    double propDiameter = 0.0;   // m
    double etaMax = 0.85;
    double advanceRatioDesign = 0.6;
    double yOffset = 0.0;        // P-factor moment arm
    double alphaRef = 0.15;      // rad, P-factor reference AoA
    double IProp = 0.0;          // propeller polar moment of inertia
};

struct AtmosphereSample {
    double temperatureK = 0.0;
    double pressurePa = 0.0;
    double densityKgM3 = 0.0;
    double speedOfSoundMs = 0.0;
};

// ISA atmosphere model (troposphere + isothermal lower stratosphere —
// sufficient range for anything this engine flies at).
AtmosphereSample isaAtmosphere(double altitudeMeters) noexcept;

// ---------------------------------------------------------------------
// Derivatives / Integration — the public entry points for a
// FlightDynamicsSystem to call once per fixed tick, per entity.
// ---------------------------------------------------------------------

// Pure function: computes the state derivative at `s` under `controls`.
// No hidden state; safe to call concurrently for different entities, and
// safe to call multiple times per tick (RK4 needs 4 evaluations).
RigidBody6DOFDerivative calculateDerivatives(
    const RigidBody6DOFState& s,
    const FlightControls& controls,
    const MassProperties& mass,
    const AeroCoefficients& aero,
    const PropulsionModel& prop) noexcept;

// Classic 4-stage RK4, one fixed step, in place. Intended to be called
// once per tick from the fixed-timestep system that owns this entity's
// RigidBody6DOFState. Renormalizes the attitude quaternion at the end.
void integrateRK4(
    RigidBody6DOFState& state,
    double dt,
    const FlightControls& controls,
    const MassProperties& mass,
    const AeroCoefficients& aero,
    const PropulsionModel& prop) noexcept;

} // namespace simengine::physics
