// simengine/physics/rigid_body_6dof.cpp — Physics / Flight Dynamics subsystem.
//
// Implementation of the 6-DOF body-axis rigid body flight model declared in
// rigid_body_6dof.hpp. Ported from GeoDrive's `AdvancedFlightDynamics`
// (Stevens & Lewis "Aircraft Control and Simulation" formulation). No term
// present in the original model has been dropped or simplified; every
// coefficient, correction, and protection layer below has a direct
// counterpart in the source model. See the header for the design
// rationale (why this is split out of the header, state/units
// conventions, threading contract).

#include "simengine/physics/rigid_body_6dof.hpp"

#include <algorithm>
#include <cmath>

namespace simengine::physics {

// =======================================================================
// RigidBody6DOFState — derived read-only views
// =======================================================================

double RigidBody6DOFState::trueAirspeed() const noexcept {
    return std::max(0.01, std::sqrt(u * u + v * v + w * w));
}

double RigidBody6DOFState::angleOfAttack() const noexcept {
    return std::atan2(w, u);
}

double RigidBody6DOFState::sideslip() const noexcept {
    const double vt = trueAirspeed();
    return std::asin(std::clamp(v / std::max(vt, 0.1), -1.0, 1.0));
}

Vector3<double> RigidBody6DOFState::eulerZYX() const noexcept {
    return attitude.toEulerZYX();
}

// =======================================================================
// MassProperties — inertia-coupling gamma coefficients
// =======================================================================
//
// Standard Stevens & Lewis reduction of the full (Ix, Iy, Iz, Ixz) inertia
// tensor into eight scalar coefficients that appear throughout the
// rotational equations of motion below. Computed once per aircraft
// (or whenever mass properties genuinely change, e.g. a future fuel-burn/
// CG-shift model), never per integration step.

void MassProperties::finalize() noexcept {
    const double Gamma = Ix * Iz - Ixz * Ixz;
    gamma.G1 = (Ixz * (Ix + Iy - Iz)) / Gamma;
    gamma.G2 = (Iz * (Iz - Iy) + Ixz * Ixz) / Gamma;
    gamma.G3 = Iz / Gamma;
    gamma.G4 = Ixz / Gamma;
    gamma.G5 = (Iz - Ix) / Iy;
    gamma.G6 = Ixz / Iy;
    gamma.G7 = (Ix * (Ix - Iy) + Ixz * Ixz) / Gamma;
    gamma.G8 = Ix / Gamma;
}

// =======================================================================
// Atmosphere — International Standard Atmosphere (ISA)
// =======================================================================
//
// Two-layer model: linear-lapse troposphere below 11 km, isothermal lower
// stratosphere above. Covers the full operating envelope of any aircraft
// this engine is expected to simulate (subsonic/transonic airliners and
// GA aircraft; nothing here flies above the tropopause+stratosphere
// boundary modeled). Density is recovered from the ideal gas law once
// temperature and pressure are known; speed of sound follows directly
// from temperature for a calorically perfect gas (gamma = 1.4 for air).

AtmosphereSample isaAtmosphere(double altitudeMeters) noexcept {
    const double h = std::max(0.0, altitudeMeters);
    constexpr double kGravity = 9.80665;      // m/s^2, standard gravity
    constexpr double kGasConstant = 287.05;   // J/(kg*K), specific gas constant for dry air
    constexpr double kSeaLevelTempK = 288.15;
    constexpr double kSeaLevelPressurePa = 101325.0;
    constexpr double kTropopauseAltM = 11000.0;
    constexpr double kLapseRate = 0.0065;     // K/m
    constexpr double kStratosphereTempK = 216.65;
    constexpr double kTropopausePressurePa = 22632.0;
    constexpr double kSpecificHeatRatio = 1.4;

    double temperatureK, pressurePa;
    if (h < kTropopauseAltM) {
        temperatureK = kSeaLevelTempK - kLapseRate * h;
        pressurePa = kSeaLevelPressurePa * std::pow(temperatureK / kSeaLevelTempK, 5.2561);
    } else {
        temperatureK = kStratosphereTempK;
        pressurePa = kTropopausePressurePa *
            std::exp(-kGravity * (h - kTropopauseAltM) / (kGasConstant * temperatureK));
    }

    const double densityKgM3 = pressurePa / (kGasConstant * temperatureK);
    const double speedOfSoundMs = std::sqrt(kSpecificHeatRatio * kGasConstant * temperatureK);

    return AtmosphereSample{temperatureK, pressurePa, densityKgM3, speedOfSoundMs};
}

// =======================================================================
// calculateDerivatives — the core physics
// =======================================================================
//
// Computes d(state)/dt at the given state and controls. Organized in the
// same order as the source model:
//   1. Airspeed / angle of attack / sideslip from body-axis velocity
//   2. Atmosphere sample at current altitude, dynamic pressure, Mach
//   3. Lift coefficient build-up: linear term, flap augmentation,
//      post-stall falloff, Prandtl-Glauert compressibility correction
//   4. Flight-envelope (load-factor) protection clamp on lift coefficient
//   5. Drag coefficient build-up: parasite, induced, wave, gear, brake,
//      flap
//   6. Body-axis force/moment coefficients (CX/CZ from CL/CD rotated
//      through alpha; CY/Cl/Cm/Cn from stability derivatives)
//   7. Propulsion: jet or piston/propeller model, including P-factor and
//      gyroscopic precession moments for the propeller case
//   8. Gravity, resolved into body axes via the quaternion-derived DCM
//   9. Translational equations of motion (body-axis Newton's second law
//      with rotational coupling terms)
//  10. Rotational equations of motion (Euler's equations with full
//      asymmetric-inertia coupling via the gamma coefficients)
//  11. Quaternion kinematic equation (attitude rate from body rates)
//  12. Position kinematics (body-axis velocity rotated into NED via DCM)

RigidBody6DOFDerivative calculateDerivatives(
    const RigidBody6DOFState& s,
    const FlightControls& controls,
    const MassProperties& mass,
    const AeroCoefficients& aero,
    const PropulsionModel& prop) noexcept
{
    constexpr double kGravity = 9.80665; // m/s^2

    // --- 1. Airspeed, angle of attack, sideslip ---
    const double trueAirspeed = std::max(0.01, std::sqrt(s.u * s.u + s.v * s.v + s.w * s.w));
    const double alpha = std::atan2(s.w, s.u);
    const double beta = std::asin(std::clamp(s.v / std::max(trueAirspeed, 0.1), -1.0, 1.0));

    // --- 2. Atmosphere / dynamic pressure / Mach ---
    const AtmosphereSample atm = isaAtmosphere(-s.positionNED.z); // NED: -z = altitude ASL
    const double dynamicPressure = 0.5 * atm.densityKgM3 * trueAirspeed * trueAirspeed;
    const double machNumber = trueAirspeed / atm.speedOfSoundMs;

    // --- 3. Lift coefficient build-up ---
    // Flaps: high-lift-device model. Extending flaps adds camber (CL0
    // boost), extends the usable angle-of-attack range before the stall
    // break (alphaStall boost), and costs parasite drag (CD_flap).
    const double flap = std::clamp(controls.flapPosition, 0.0, 1.0);
    const double slat = std::clamp(controls.slatPosition, 0.0, 1.0);
    const double spoiler = std::clamp(controls.spoilerPosition, 0.0, 1.0);
    const double speedbrake = std::clamp(controls.speedbrakePosition, 0.0, 1.0);

    const double flapCL0Boost   = flap * aero.CLflap;
    const double flapAlphaBoost = flap * aero.alphaStallFlap;
    const double flapDragDelta  = flap * aero.CD_flap;

    // Slats extend the usable AoA range independently of flaps (a slats-
    // only, flaps-up config is a real approach/departure state, e.g.
    // slats-out for extra stall margin without the flap drag penalty).
    const double slatAlphaBoost = slat * aero.alphaStallSlat;
    const double slatDragDelta  = slat * aero.CD_slat;

    // Spoilers dump lift (roll-in-flight and ground lift-dump on
    // rollout) while also adding drag; kept strictly separate from the
    // dedicated speedbrake surface below even though both are "drag
    // devices" — many aircraft deploy them independently/asymmetrically.
    const double spoilerCLDelta = -spoiler * std::abs(aero.CLspoiler);
    const double spoilerDragDelta = spoiler * aero.CD_spoiler;
    const double speedbrakeDragDelta = speedbrake * aero.CD_speedbrake;

    const double liftCoeffLinear =
        (aero.CL0 + flapCL0Boost)
        + aero.CLa * alpha
        + aero.CLq * (s.q * mass.c / (2.0 * trueAirspeed))
        + aero.CLde * controls.deltaElevator
        + spoilerCLDelta;

    // Post-stall lift falloff, so the aircraft retains a recognizable
    // stall break instead of the linear model extrapolating lift forever
    // past the physical stall angle. Flaps and slats both extend the
    // usable range, independently and additively (matches real high-lift
    // system behavior: full flaps+slats gives the largest margin).
    const double alphaMagnitude = std::abs(alpha);
    const double effectiveStallAlpha = aero.alphaStall + flapAlphaBoost + slatAlphaBoost;
    double liftCoeffBase = liftCoeffLinear;
    double stallDragDelta = 0.0;
    if (alphaMagnitude > effectiveStallAlpha) {
        const double overshoot = alphaMagnitude - effectiveStallAlpha;
        const double stallFalloffFactor = std::max(0.15, 1.0 - overshoot * 3.5);
        liftCoeffBase = liftCoeffLinear * stallFalloffFactor;
        stallDragDelta = overshoot * 1.8;
    }

    // Prandtl-Glauert subsonic compressibility correction. Above M0.99 the
    // 1/sqrt(1-M^2) singularity is avoided by falling back to a bounded
    // (2x) multiplier rather than blowing up — consistent with the source
    // model, which never simulates true supersonic flight.
    double liftCoeffCompressible = (machNumber < 0.99)
        ? liftCoeffBase / std::sqrt(1.0 - machNumber * machNumber)
        : liftCoeffBase * 2.0;

    // --- 4. Flight-envelope (load-factor) protection ---
    // L = 0.5 * rho * V^2 * S * CL <= n_max * m * g
    // Directly clamps the lift coefficient so the airframe cannot be
    // commanded past its structural g-limit, mirroring fly-by-wire
    // envelope protection (e.g. Airbus Normal Law). Flaps-down limits are
    // tighter, matching reduced structural margin in landing
    // configuration.
    if (aero.hasLoadFactorProtection) {
        const bool flapsDown = flap > 0.01;
        const double loadFactorMax = flapsDown ? aero.nMaxFlap : aero.nMaxClean;
        const double loadFactorMin = flapsDown ? aero.nMinFlap : aero.nMinClean;
        const double dynamicPressureTimesArea = std::max(1.0, dynamicPressure * mass.S);
        const double liftCoeffLimitPositive = (loadFactorMax * mass.mass * kGravity) / dynamicPressureTimesArea;
        const double liftCoeffLimitNegative = (loadFactorMin * mass.mass * kGravity) / dynamicPressureTimesArea;
        liftCoeffCompressible = std::clamp(liftCoeffCompressible, liftCoeffLimitNegative, liftCoeffLimitPositive);
    }

    // --- 5. Drag coefficient build-up ---
    const double inducedDragCoeff = (liftCoeffCompressible * liftCoeffCompressible) / (M_PI * mass.AR * aero.e);
    const double waveDragCoeff = (machNumber > aero.Mcrit)
        ? aero.Kc * (machNumber - aero.Mcrit) * (machNumber - aero.Mcrit)
        : 0.0;
    const double gearDragCoeff  = controls.gearDown ? aero.CD_gear : 0.0;
    const bool brakesOn = controls.brakeActive || controls.brakeCommand > 0.01;
    const double brakeDragCoeff = brakesOn
        ? aero.CD_brake * std::clamp(std::max(controls.brakeCommand, controls.brakeActive ? 1.0 : 0.0), 0.0, 1.0)
        : 0.0;
    const double dragCoeffTotal =
        aero.CD0 + inducedDragCoeff + waveDragCoeff + stallDragDelta
        + gearDragCoeff + brakeDragCoeff + flapDragDelta
        + slatDragDelta + spoilerDragDelta + speedbrakeDragDelta;

    // --- 6. Body-axis aerodynamic force/moment coefficients ---
    // Lift/drag act along the wind axes; rotate through alpha into body
    // axes (CX forward, CZ down in the body frame).
    const double bodyForceCoeffX = -dragCoeffTotal * std::cos(alpha) + liftCoeffCompressible * std::sin(alpha);
    const double bodyForceCoeffZ = -dragCoeffTotal * std::sin(alpha) - liftCoeffCompressible * std::cos(alpha);

    const double sideForceCoeff =
        aero.CYb * beta
        + aero.CYp * (s.p * mass.b / (2.0 * trueAirspeed))
        + aero.CYdr * controls.deltaRudder;

    const double rollMomentCoeff =
        aero.Clb * beta
        + aero.Clp * (s.p * mass.b / (2.0 * trueAirspeed))
        + aero.Clda * controls.deltaAileron;

    // Pitching moment: aerodynamic terms plus THS (Trimmable Horizontal
    // Stabilizer) trim contribution, folded in the same way as the
    // elevator term (both scale with dynamic pressure via q_inf*S*c
    // applied below) so trim authority correctly weakens at low airspeed
    // exactly like elevator authority does — no separate speed-independent
    // trim term.
    const double pitchMomentCoeff =
        aero.Cm0
        + aero.Cma * alpha
        + aero.Cmq * (s.q * mass.c / (2.0 * trueAirspeed))
        + aero.Cmde * controls.deltaElevator
        + aero.Cmdt * controls.deltaThsDeg;

    const double yawMomentCoeff =
        aero.Cnb * beta
        + aero.Cnp * (s.p * mass.b / (2.0 * trueAirspeed))
        + aero.Cnr * (s.r * mass.b / (2.0 * trueAirspeed))
        + aero.Cndr * controls.deltaRudder;

    const double aeroForceX = dynamicPressure * mass.S * bodyForceCoeffX;
    const double aeroForceY = dynamicPressure * mass.S * sideForceCoeff;
    const double aeroForceZ = dynamicPressure * mass.S * bodyForceCoeffZ;
    const double aeroRollMoment  = dynamicPressure * mass.S * mass.b * rollMomentCoeff;
    const double aeroPitchMoment = dynamicPressure * mass.S * mass.c * pitchMomentCoeff;
    const double aeroYawMoment   = dynamicPressure * mass.S * mass.b * yawMomentCoeff;

    // --- 7. Propulsion ---
    // Two independent models selected per-aircraft via prop.type:
    //   Jet     — near-constant static thrust that falls off with air
    //             density (altitude) and a mild Mach-based ram-drag lapse
    //             at cruise; no 1/airspeed singularity.
    //   Piston  — propeller disc: thrust from shaft power via an
    //             advance-ratio efficiency curve, plus P-factor
    //             (asymmetric disc loading at high alpha) and gyroscopic
    //             precession moments from the spinning propeller mass.
    double thrust = 0.0;
    double pFactorYawMoment = 0.0;
    double gyroscopicRollMoment = 0.0;
    double gyroscopicPitchMoment = 0.0;

    if (prop.type == PropulsionType::Jet) {
        const double densityRatio = atm.densityKgM3 / 1.225; // relative to sea-level ISA
        const double ramDragSpeedLapse = std::max(0.55, 1.0 - 0.12 * machNumber);
        thrust = prop.T_max * controls.throttle * std::pow(densityRatio, 0.7) * ramDragSpeedLapse;
        if (controls.reverseActive) {
            thrust = -thrust * 0.45; // reverse thrust is derated relative to forward
        }
    } else {
        const double propRevsPerSecond = prop.RPM / 60.0;
        const double advanceRatio = trueAirspeed / (propRevsPerSecond * prop.propDiameter);
        const double advanceRatioDelta = (advanceRatio - prop.advanceRatioDesign) / prop.advanceRatioDesign;
        const double propEfficiency = prop.etaMax * (1.0 - advanceRatioDelta * advanceRatioDelta);
        thrust = std::max(0.0, (prop.P_max * controls.throttle * propEfficiency) / std::max(trueAirspeed, 10.0));
        if (controls.reverseActive) {
            thrust = -thrust * 0.5;
        }

        // P-factor: asymmetric propeller disc loading at non-zero angle of
        // attack produces a yawing moment proportional to thrust and alpha.
        pFactorYawMoment = thrust * prop.yOffset * (alpha / prop.alphaRef);

        // Gyroscopic precession of the spinning propeller mass couples
        // pitch rate into roll moment and roll rate into pitch moment.
        const double propAngularMomentum = prop.IProp * (prop.RPM * M_PI / 30.0);
        gyroscopicRollMoment  =  propAngularMomentum * s.q;
        gyroscopicPitchMoment = -propAngularMomentum * s.p;
    }

    const double totalForceX = aeroForceX + thrust;
    const double totalForceY = aeroForceY;
    const double totalForceZ = aeroForceZ;
    const double totalRollMoment  = aeroRollMoment + gyroscopicRollMoment;
    const double totalPitchMoment = aeroPitchMoment + gyroscopicPitchMoment;
    const double totalYawMoment   = aeroYawMoment + pFactorYawMoment;

    // --- 8. Gravity, resolved into body axes via the quaternion DCM ---
    // Only the third row of the body-from-NED direction cosine matrix is
    // needed (gravity acts purely in NED +z); derived directly from the
    // quaternion components, avoiding any Euler-angle conversion (and the
    // gimbal-lock risk that would come with it).
    const auto& q = s.attitude; // (w,x,y,z) == (e0,e1,e2,e3)
    const double dcmRow3X = 2.0 * (q.x * q.z - q.w * q.y);
    const double dcmRow3Y = 2.0 * (q.y * q.z + q.w * q.x);
    const double dcmRow3Z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    const double gravityBodyX = kGravity * dcmRow3X;
    const double gravityBodyY = kGravity * dcmRow3Y;
    const double gravityBodyZ = kGravity * dcmRow3Z;

    // --- 9. Translational equations of motion (body axes) ---
    // Newton's second law in a rotating (body) frame: acceleration =
    // (specific force) - (omega x velocity), expanded component-wise.
    RigidBody6DOFDerivative derivative;
    derivative.u = s.r * s.v - s.q * s.w + gravityBodyX + totalForceX / mass.mass;
    derivative.v = s.p * s.w - s.r * s.u + gravityBodyY + totalForceY / mass.mass;
    derivative.w = s.q * s.u - s.p * s.v + gravityBodyZ + totalForceZ / mass.mass;

    // --- 10. Rotational equations of motion (full asymmetric-inertia
    // coupling, Stevens & Lewis gamma-coefficient form) ---
    const auto& gamma = mass.gamma;
    derivative.p = gamma.G1 * s.p * s.q - gamma.G2 * s.q * s.r
                 + gamma.G3 * totalRollMoment + gamma.G4 * totalYawMoment;
    derivative.q = gamma.G5 * s.p * s.r - gamma.G6 * (s.p * s.p - s.r * s.r)
                 + totalPitchMoment / mass.Iy;
    derivative.r = gamma.G7 * s.p * s.q + gamma.G1 * s.q * s.r
                 + gamma.G4 * totalRollMoment + gamma.G8 * totalYawMoment;

    // --- 11. Quaternion kinematic equation ---
    // dq/dt = 0.5 * q (x) [0, p, q, r] (pure-quaternion body-rate product).
    derivative.attitude.w = -0.5 * (q.x * s.p + q.y * s.q + q.z * s.r);
    derivative.attitude.x =  0.5 * (q.w * s.p + q.y * s.r - q.z * s.q);
    derivative.attitude.y =  0.5 * (q.w * s.q + q.z * s.p - q.x * s.r);
    derivative.attitude.z =  0.5 * (q.w * s.r + q.x * s.q - q.y * s.p);

    // --- 12. Position kinematics (body velocity rotated into NED) ---
    // Full direction cosine matrix, body-to-NED, from the quaternion.
    const double dcmRow1X = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
    const double dcmRow1Y = 2.0 * (q.x * q.y - q.w * q.z);
    const double dcmRow1Z = 2.0 * (q.x * q.z + q.w * q.y);
    const double dcmRow2X = 2.0 * (q.x * q.y + q.w * q.z);
    const double dcmRow2Y = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
    const double dcmRow2Z = 2.0 * (q.y * q.z - q.w * q.x);
    // dcmRow3{X,Y,Z} already computed above for gravity.

    derivative.positionNED.x = dcmRow1X * s.u + dcmRow1Y * s.v + dcmRow1Z * s.w;
    derivative.positionNED.y = dcmRow2X * s.u + dcmRow2Y * s.v + dcmRow2Z * s.w;
    derivative.positionNED.z = dcmRow3X * s.u + dcmRow3Y * s.v + dcmRow3Z * s.w;

    return derivative;
}

// =======================================================================
// Integration — classic 4th-order Runge-Kutta
// =======================================================================

namespace {

// Returns `base` with every field advanced by `derivative * scale`. Used
// to build the intermediate RK4 stage states (s + k1*dt/2, s + k2*dt/2,
// s + k3*dt). Kept as an anonymous-namespace helper (internal linkage) —
// not part of the public API, so it can be freely changed without
// touching any caller outside this translation unit.
RigidBody6DOFState advanceBy(const RigidBody6DOFState& base,
                              const RigidBody6DOFDerivative& derivative,
                              double scale) noexcept {
    RigidBody6DOFState result = base;
    result.u += derivative.u * scale;
    result.v += derivative.v * scale;
    result.w += derivative.w * scale;
    result.p += derivative.p * scale;
    result.q += derivative.q * scale;
    result.r += derivative.r * scale;
    result.attitude.w += derivative.attitude.w * scale;
    result.attitude.x += derivative.attitude.x * scale;
    result.attitude.y += derivative.attitude.y * scale;
    result.attitude.z += derivative.attitude.z * scale;
    result.positionNED.x += derivative.positionNED.x * scale;
    result.positionNED.y += derivative.positionNED.y * scale;
    result.positionNED.z += derivative.positionNED.z * scale;
    return result;
}

// Accumulates the weighted RK4 blend (k1 + 2*k2 + 2*k3 + k4) * dt/6 into
// `state` in place, field by field.
void accumulateRK4Blend(RigidBody6DOFState& state,
                         const RigidBody6DOFDerivative& k1,
                         const RigidBody6DOFDerivative& k2,
                         const RigidBody6DOFDerivative& k3,
                         const RigidBody6DOFDerivative& k4,
                         double dt) noexcept {
    const double sixthDt = dt / 6.0;
    state.u += sixthDt * (k1.u + 2.0 * k2.u + 2.0 * k3.u + k4.u);
    state.v += sixthDt * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    state.w += sixthDt * (k1.w + 2.0 * k2.w + 2.0 * k3.w + k4.w);
    state.p += sixthDt * (k1.p + 2.0 * k2.p + 2.0 * k3.p + k4.p);
    state.q += sixthDt * (k1.q + 2.0 * k2.q + 2.0 * k3.q + k4.q);
    state.r += sixthDt * (k1.r + 2.0 * k2.r + 2.0 * k3.r + k4.r);
    state.attitude.w += sixthDt * (k1.attitude.w + 2.0 * k2.attitude.w + 2.0 * k3.attitude.w + k4.attitude.w);
    state.attitude.x += sixthDt * (k1.attitude.x + 2.0 * k2.attitude.x + 2.0 * k3.attitude.x + k4.attitude.x);
    state.attitude.y += sixthDt * (k1.attitude.y + 2.0 * k2.attitude.y + 2.0 * k3.attitude.y + k4.attitude.y);
    state.attitude.z += sixthDt * (k1.attitude.z + 2.0 * k2.attitude.z + 2.0 * k3.attitude.z + k4.attitude.z);
    state.positionNED.x += sixthDt * (k1.positionNED.x + 2.0 * k2.positionNED.x + 2.0 * k3.positionNED.x + k4.positionNED.x);
    state.positionNED.y += sixthDt * (k1.positionNED.y + 2.0 * k2.positionNED.y + 2.0 * k3.positionNED.y + k4.positionNED.y);
    state.positionNED.z += sixthDt * (k1.positionNED.z + 2.0 * k2.positionNED.z + 2.0 * k3.positionNED.z + k4.positionNED.z);
}

} // namespace

void integrateRK4(
    RigidBody6DOFState& state,
    double dt,
    const FlightControls& controls,
    const MassProperties& mass,
    const AeroCoefficients& aero,
    const PropulsionModel& prop) noexcept
{
    // Stage 1: derivative at the start of the interval.
    const RigidBody6DOFDerivative k1 = calculateDerivatives(state, controls, mass, aero, prop);

    // Stage 2: derivative at the midpoint, using k1 to get there.
    const RigidBody6DOFState stage2State = advanceBy(state, k1, dt * 0.5);
    const RigidBody6DOFDerivative k2 = calculateDerivatives(stage2State, controls, mass, aero, prop);

    // Stage 3: derivative at the midpoint again, refined using k2.
    const RigidBody6DOFState stage3State = advanceBy(state, k2, dt * 0.5);
    const RigidBody6DOFDerivative k3 = calculateDerivatives(stage3State, controls, mass, aero, prop);

    // Stage 4: derivative at the end of the interval, using k3 to get there.
    const RigidBody6DOFState stage4State = advanceBy(state, k3, dt);
    const RigidBody6DOFDerivative k4 = calculateDerivatives(stage4State, controls, mass, aero, prop);

    // Weighted blend of the four stage derivatives (Simpson's-rule-like
    // weighting: 1,2,2,1) advances the actual state by one fixed step.
    accumulateRK4Blend(state, k1, k2, k3, k4, dt);

    // The integration above operates on the quaternion components as four
    // independent scalars, which does not preserve unit length exactly;
    // renormalizing here is the standard, required correction (not an
    // optional cleanup step) before the state is used for anything else.
    state.normalizeAttitude();
}

} // namespace simengine::physics
