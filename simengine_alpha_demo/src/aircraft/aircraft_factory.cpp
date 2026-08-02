#include "simengine/aircraft/aircraft_factory.hpp"
#include <algorithm>

namespace simengine::aircraft {

using simengine::math::Vector3;
using simengine::math::Quaternion;

core::Entity spawnGenericNarrowbody(core::World& world, const SpawnParams& params) {
    core::Entity e = world.createEntity();

    // --- Rigid body ---
    RigidBody6DOFComponent body;
    body.state.positionNED = params.positionNED;
    body.state.attitude = Quaternion<double>::fromEulerZYX(params.headingRad, 0.0, 0.0);
    world.addComponent<RigidBody6DOFComponent>(e, body);

    // --- Aircraft definition (generic narrowbody-class magnitudes) ---
    AircraftComponent ac;
    ac.typeName = "generic-narrowbody";
    ac.engineCount = 2;

    ac.mass.mass = 64000.0;   // kg, mid-range operating weight
    ac.mass.S = 122.6;        // m^2
    ac.mass.b = 34.1;         // m
    ac.mass.c = 4.29;         // m
    ac.mass.AR = (ac.mass.b * ac.mass.b) / ac.mass.S;
    ac.mass.Ix = 2.4e6;  ac.mass.Iy = 3.4e6;  ac.mass.Iz = 5.6e6;  ac.mass.Ixz = 6.0e4;
    ac.mass.finalize();

    ac.aero.CL0 = 0.20; ac.aero.CLa = 5.5; ac.aero.CLq = 8.0; ac.aero.CLde = 0.6;
    ac.aero.CD0 = 0.022; ac.aero.e = 0.80; ac.aero.Mcrit = 0.78; ac.aero.Kc = 0.10;
    ac.aero.alphaStall = 0.28;
    ac.aero.CD_gear = 0.018; ac.aero.CD_brake = 0.0;
    ac.aero.CLflap = 0.6; ac.aero.alphaStallFlap = 0.22; ac.aero.CD_flap = 0.02;
    ac.aero.alphaStallSlat = 0.09; ac.aero.CD_slat = 0.004;
    ac.aero.CLspoiler = 0.35; ac.aero.CD_spoiler = 0.045;
    ac.aero.CD_speedbrake = 0.028;
    ac.aero.hasLoadFactorProtection = true;
    ac.aero.nMaxClean = 2.5; ac.aero.nMinClean = -1.0;
    ac.aero.nMaxFlap = 2.0;  ac.aero.nMinFlap = 0.0;
    ac.aero.CYb = -0.8; ac.aero.CYp = 0.0; ac.aero.CYdr = 0.15;
    ac.aero.Clb = -0.09; ac.aero.Clp = -0.4; ac.aero.Clda = 0.12;
    ac.aero.Cm0 = 0.02; ac.aero.Cma = -1.0; ac.aero.Cmq = -18.0; ac.aero.Cmde = -1.3; ac.aero.Cmdt = -0.6;
    ac.aero.Cnb = 0.12; ac.aero.Cnp = -0.02; ac.aero.Cnr = -0.2; ac.aero.Cndr = -0.09;

    ac.propulsion.type = simengine::physics::PropulsionType::Jet;
    ac.propulsion.T_max = 2.0 * 120000.0; // N, two engines, ~120kN each static

    world.addComponent<AircraftComponent>(e, ac);

    // --- Controls ---
    // Gear starts down: this Alpha's scenarios all begin on or just
    // above the baseplate, not mid-flight, so a retracted-gear default
    // (physics::FlightControls::gearDown's engine-wide default, correct
    // for an in-flight spawn) would be the wrong choice here.
    FlightControlsComponent controlsComp;
    controlsComp.controls.gearDown = true;
    world.addComponent<FlightControlsComponent>(e, controlsComp);

    // --- Landing gear: nose + 2 main legs, plausible geometry relative
    // to a ~38m fuselage. ---
    LandingGearComponent gear;
    {
        WheelComponent nose;
        nose.config.attachBody = Vector3<double>{15.0, 0.0, 1.2};
        nose.config.strutLength = 1.8;
        nose.config.travel = 0.35;
        nose.config.springConstant = 300000.0;
        nose.config.damperConstant = 20000.0;
        nose.config.wheelRadius = 0.45;
        nose.config.steerable = true;
        nose.config.maxSteerAngleRad = 1.1; // ~65 deg, typical tiller range
        nose.config.hasBrake = false;
        nose.animationNodeName = "nose";
        gear.legs.push_back(nose);

        WheelComponent mainL;
        mainL.config.attachBody = Vector3<double>{-1.0, -3.0, 1.6};
        mainL.config.strutLength = 2.0;
        mainL.config.travel = 0.45;
        mainL.config.springConstant = 900000.0;
        mainL.config.damperConstant = 60000.0;
        mainL.config.wheelRadius = 0.55;
        mainL.config.hasBrake = true;
        mainL.animationNodeName = "L_main";
        gear.legs.push_back(mainL);

        WheelComponent mainR = mainL;
        mainR.config.attachBody.y = 3.0;
        mainR.animationNodeName = "R_main";
        gear.legs.push_back(mainR);
    }
    world.addComponent<LandingGearComponent>(e, gear);

    // --- Engines ---
    EngineComponent eng;
    eng.engines.resize(2);
    world.addComponent<EngineComponent>(e, eng);

    // --- Animation channels (pre-seeded so AnimationSystem's getOrAdd
    // is a pure lookup in the steady-state case; not required for
    // correctness, just avoids first-tick vector growth). ---
    AnimationComponent anim;
    world.addComponent<AnimationComponent>(e, anim);

    return e;
}

core::Entity spawnFromJSBSim(core::World& world, const io::JSBSimAircraftData& data,
                              const SpawnParams& params) {
    core::Entity e = world.createEntity();

    RigidBody6DOFComponent body;
    body.state.positionNED = params.positionNED;
    body.state.attitude = Quaternion<double>::fromEulerZYX(params.headingRad, 0.0, 0.0);
    world.addComponent<RigidBody6DOFComponent>(e, body);

    AircraftComponent ac;
    ac.typeName = data.name.empty() ? "jsbsim-import" : data.name;
    ac.mass = data.mass;
    ac.aero = data.aero;
    ac.propulsion = data.propulsion;
    ac.engineCount = data.engineCount;
    world.addComponent<AircraftComponent>(e, ac);

    FlightControlsComponent controlsComp;
    controlsComp.controls.gearDown = true; // see spawnGenericNarrowbody's note
    world.addComponent<FlightControlsComponent>(e, controlsComp);

    LandingGearComponent gear;
    if (!data.gearLegs.empty()) {
        for (auto& imported : data.gearLegs) {
            WheelComponent wheel;
            wheel.config = imported.config;
            wheel.animationNodeName = imported.name;
            gear.legs.push_back(std::move(wheel));
        }
    } else {
        // Malformed/partial import with no usable BOGEY contacts —
        // don't spawn a gearless aircraft; fall back to the same
        // generic nose+2-main geometry the default aircraft uses so it
        // can still taxi, even though the geometry won't match the
        // imported airframe.
        WheelComponent nose;
        nose.config.attachBody = Vector3<double>{15.0, 0.0, 0.0};
        nose.config.strutLength = 3.0; nose.config.travel = 0.35;
        nose.config.springConstant = 300000.0; nose.config.damperConstant = 20000.0;
        nose.config.wheelRadius = 0.45; nose.config.steerable = true;
        nose.config.maxSteerAngleRad = 1.1;
        nose.animationNodeName = "nose_fallback";
        gear.legs.push_back(nose);
        WheelComponent mainL = nose;
        mainL.config.attachBody = Vector3<double>{-1.0, -3.0, 0.0};
        mainL.config.strutLength = 4.0; mainL.config.travel = 0.45;
        mainL.config.springConstant = 900000.0; mainL.config.damperConstant = 60000.0;
        mainL.config.wheelRadius = 0.55; mainL.config.steerable = false; mainL.config.hasBrake = true;
        mainL.animationNodeName = "L_main_fallback";
        gear.legs.push_back(mainL);
        WheelComponent mainR = mainL;
        mainR.config.attachBody.y = 3.0;
        mainR.animationNodeName = "R_main_fallback";
        gear.legs.push_back(mainR);
    }
    world.addComponent<LandingGearComponent>(e, gear);

    EngineComponent eng;
    eng.engines.resize(static_cast<std::size_t>(std::max(1, data.engineCount)));
    world.addComponent<EngineComponent>(e, eng);

    world.addComponent<AnimationComponent>(e, AnimationComponent{});

    return e;
}

} // namespace simengine::aircraft
