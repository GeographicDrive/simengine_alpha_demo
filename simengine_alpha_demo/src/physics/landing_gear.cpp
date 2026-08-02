// simengine/physics/landing_gear.cpp — see landing_gear.hpp for design notes.

#include "simengine/physics/landing_gear.hpp"
#include <algorithm>
#include <cmath>

namespace simengine::physics {

namespace {
constexpr double kEps = 1e-6;
}

GearLegForce evaluateGearLeg(
    GearLegState& state,
    const GearLegConfig& cfg,
    const GearLegInput& input,
    const Vector3<double>& bodyVelocity,
    const Vector3<double>& bodyRates,
    double heightAboveGround,
    double dt,
    double mass) noexcept {

    GearLegForce out;

    if (!input.gearDown) {
        state.compression = 0.0;
        state.compressionRate = 0.0;
        state.onGround = false;
        // Wheel keeps no spin memory retracted; animation system reads
        // gearDown separately to drive the stow animation.
        return out;
    }

    // heightAboveGround < 0 means the fully-extended wheel contact point
    // is below the ground plane by that amount -> that's how much the
    // strut must compress to keep the wheel on the surface.
    const double desiredCompression = std::clamp(-heightAboveGround, 0.0, cfg.travel);
    const double prevCompression = state.compression;

    state.onGround = desiredCompression > kEps;

    if (!state.onGround) {
        // Airborne: strut relaxes back toward zero compression under its
        // own spring, rather than snapping (avoids gear-oscillation pops
        // on liftoff).
        const double relax = std::min(prevCompression, 5.0 * cfg.travel * dt);
        state.compression = std::max(0.0, prevCompression - relax);
        state.compressionRate = dt > kEps ? (state.compression - prevCompression) / dt : 0.0;
        // Wheel spin decays with rolling friction, no ground torque.
        state.wheelSpinRateRadS *= std::max(0.0, 1.0 - 2.0 * dt);
        state.wheelSpinRad += state.wheelSpinRateRadS * dt;
        return out;
    }

    state.compression = desiredCompression;
    state.compressionRate = dt > kEps ? (state.compression - prevCompression) / dt : 0.0;

    // --- Normal (vertical, body -z) force: spring + damper, strut acts
    // along body Z (assumes near-vertical struts, adequate for this
    // model; a fully general strut-axis projection is a straightforward
    // extension point if canted gear legs are ever needed). ---
    double normalForce = cfg.springConstant * state.compression
                        + cfg.damperConstant * state.compressionRate;
    normalForce = std::max(0.0, normalForce); // gear can't pull the aircraft down

    // Weight-on-wheels approximation for friction available at this leg.
    const double weightOnWheel = normalForce;

    // --- Ground-relative velocity at the contact point (accounts for
    // body rotation rates via v_contact = v_cg + omega x r). ---
    const Vector3<double> r = cfg.attachBody + Vector3<double>{0.0, 0.0, cfg.strutLength - state.compression};
    const Vector3<double> omegaCrossR{
        bodyRates.y * r.z - bodyRates.z * r.y,
        bodyRates.z * r.x - bodyRates.x * r.z,
        bodyRates.x * r.y - bodyRates.y * r.x
    };
    const Vector3<double> contactVel = bodyVelocity + omegaCrossR;

    // --- Steering: nose wheel rotates the rolling axis in the XY plane. ---
    if (cfg.steerable) {
        const double targetSteer = std::clamp(input.steerCommand, -1.0, 1.0) * cfg.maxSteerAngleRad;
        const double steerRate = 3.0; // rad/s actuator rate, generic
        const double maxDelta = steerRate * dt;
        const double delta = std::clamp(targetSteer - state.steerAngleRad, -maxDelta, maxDelta);
        state.steerAngleRad += delta;
    } else {
        state.steerAngleRad = 0.0;
    }

    const double cs = std::cos(state.steerAngleRad);
    const double sn = std::sin(state.steerAngleRad);
    // Rolling axis (forward for this wheel) and lateral axis in body XY.
    const Vector3<double> rollAxis{cs, sn, 0.0};
    const Vector3<double> lateralAxis{-sn, cs, 0.0};

    const double vRoll = contactVel.x * rollAxis.x + contactVel.y * rollAxis.y;
    const double vLateral = contactVel.x * lateralAxis.x + contactVel.y * lateralAxis.y;

    // Wheel spin tracks rolling speed unless braked/skidding (simple
    // first-order model: good enough for taxi/rollout behavior + the
    // wheel-spin animation feed).
    const double freeSpinRate = cfg.wheelRadius > kEps ? vRoll / cfg.wheelRadius : 0.0;
    const double brakeAuthority = cfg.hasBrake
        ? std::clamp(std::max(input.brakeCommand, input.parkingBrake ? 1.0 : 0.0), 0.0, 1.0)
        : 0.0;
    const double spinBlend = std::clamp(8.0 * dt, 0.0, 1.0) * (1.0 - 0.9 * brakeAuthority);
    state.wheelSpinRateRadS += (freeSpinRate - state.wheelSpinRateRadS) * spinBlend;
    state.wheelSpinRad += state.wheelSpinRateRadS * dt;

    // Longitudinal (rolling resistance + braking) friction opposes vRoll.
    const double muLong = cfg.rollingResistance + brakeAuthority * (cfg.dynamicFriction - cfg.rollingResistance);
    const double longFrictionMag = std::min(muLong * weightOnWheel, std::abs(vRoll) * mass / std::max(dt, 1e-3));
    const double longForceMag = -std::copysign(longFrictionMag, vRoll);

    // Lateral (cornering/skid) friction opposes side-slip at the contact
    // patch — this is what gives nose-wheel steering and crosswind
    // weathervaning authority during rollout.
    const double muLat = cfg.dynamicFriction;
    const double latFrictionMag = std::min(muLat * weightOnWheel, std::abs(vLateral) * mass / std::max(dt, 1e-3));
    const double latForceMag = -std::copysign(latFrictionMag, vLateral);

    Vector3<double> frictionForce = rollAxis * longForceMag + lateralAxis * latForceMag;

    out.forceBody = Vector3<double>{frictionForce.x, frictionForce.y, -normalForce};
    // Moment about the CG from a force applied at r (body axes):
    // M = r x F
    const Vector3<double> F = out.forceBody;
    out.momentBody = Vector3<double>{
        r.y * F.z - r.z * F.y,
        r.z * F.x - r.x * F.z,
        r.x * F.y - r.y * F.x
    };

    return out;
}

} // namespace simengine::physics
