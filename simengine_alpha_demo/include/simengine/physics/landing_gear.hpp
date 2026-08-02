// simengine/physics/landing_gear.hpp — Physics subsystem.
//
// Per-wheel landing gear model: strut suspension (spring/damper), tire
// compression, ground contact force, rolling/skidding friction, brakes,
// and nose-wheel steering. Designed to be driven once per fixed tick per
// gear leg, in body axes, then the resulting force/moment is summed into
// the rigid body's external force accumulator by FlightDynamicsSystem
// (this module has no knowledge of RigidBody6DOFState — it is a pure
// per-wheel force generator, kept that way so it is unit-testable and
// reusable for taildraggers, tricycle gear, or multi-bogey gear without
// change).
//
// Ground contact here assumes a flat, infinite plane at NED z = 0 (the
// Alpha "baseplate"). Terrain height sampling is a single hook
// (groundHeightNED) to swap in later without touching the rest of the
// model.

#pragma once

#include "../math/vector3.hpp"

namespace simengine::physics {

using simengine::math::Vector3;

// Static per-leg configuration (data-driven, per aircraft).
struct GearLegConfig {
    Vector3<double> attachBody{0.0, 0.0, 0.0}; // strut top, body axes, m from CG
    double strutLength = 1.0;        // fully-extended strut length, m
    double travel = 0.3;             // max compression travel, m
    double springConstant = 350000.0; // N/m
    double damperConstant = 25000.0;  // N/(m/s)
    double staticFriction = 0.8;
    double dynamicFriction = 0.6;
    double rollingResistance = 0.02;
    double wheelRadius = 0.5;        // m
    bool steerable = false;          // nose wheel
    double maxSteerAngleRad = 0.0;
    bool hasBrake = false;
};

// Per-tick mutable state for one gear leg.
struct GearLegState {
    double compression = 0.0;        // [0, travel], m
    double compressionRate = 0.0;    // m/s
    bool onGround = false;
    double wheelSpinRad = 0.0;       // accumulated rotation, for animation
    double wheelSpinRateRadS = 0.0;
    double steerAngleRad = 0.0;      // current commanded/applied steer
};

struct GearLegInput {
    bool gearDown = true;
    double steerCommand = 0.0;   // [-1,1], from rudder pedals / tiller
    double brakeCommand = 0.0;   // [0,1]
    bool parkingBrake = false;
};

// Force/moment produced by one gear leg this tick, in body axes, applied
// at the CG (moment already includes the r x F term from attachBody).
struct GearLegForce {
    Vector3<double> forceBody{0.0, 0.0, 0.0};
    Vector3<double> momentBody{0.0, 0.0, 0.0};
};

// Evaluates one gear leg for one fixed tick: updates `state` in place
// (compression, spin, steer angle) and returns the resulting force. Pure
// with respect to everything except `state`; safe to call for N legs in
// parallel as long as each leg owns its own GearLegState.
//
//  bodyVelocity      - u,v,w of the CG, body axes, m/s
//  bodyRates         - p,q,r, rad/s
//  heightAboveGround - vertical distance from wheel-neutral contact point
//                       to ground along body -z, m (>=0 means airborne
//                       at full extension; caller supplies this from the
//                       rigid body's position/attitude + groundHeightNED)
//  gravityBody       - gravity vector rotated into body axes, m/s^2
//  mass              - aircraft mass, kg (for weight-on-wheels friction)
GearLegForce evaluateGearLeg(
    GearLegState& state,
    const GearLegConfig& cfg,
    const GearLegInput& input,
    const Vector3<double>& bodyVelocity,
    const Vector3<double>& bodyRates,
    double heightAboveGround,
    double dt,
    double mass) noexcept;

} // namespace simengine::physics
