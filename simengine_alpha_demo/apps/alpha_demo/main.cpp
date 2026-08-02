// apps/alpha_demo/main.cpp — Alpha Technical Demo entry point.
//
// This is a HEADLESS console demo: it wires every ECS system together
// (Input -> FlightDynamics -> LandingGear -> Engine -> Animation) and
// runs a fixed-timestep loop over an infinite grey baseplate at z=0,
// printing telemetry each second. There is no renderer, windowing, or
// touch input plumbing in this Alpha — see docs/ROADMAP.md. Its purpose
// is to prove the physics/ECS/animation pipeline described in
// ARCHITECTURE.md actually integrates and runs, and to give
// mobile_ui/index.html's control layout something concrete to target
// once a real render target exists.
//
// Scenario scripted here: aircraft starts 2m AGL with idle throttle,
// settles onto the baseplate under gravity+gear (proving suspension
// compression/weight-on-wheels), then a scripted throttle-up + elevator
// pull demonstrates a takeoff roll and rotation.

#include <cstdio>

#include "simengine/core/ecs.hpp"
#include "simengine/core/job_system.hpp"
#include "simengine/aircraft/aircraft_factory.hpp"
#include "simengine/aircraft/a320_variants.hpp"
#include "simengine/io/jsbsim_import.hpp"
#include "simengine/systems/flight_dynamics_system.hpp"
#include "simengine/systems/landing_gear_system.hpp"
#include "simengine/systems/engine_system.hpp"
#include "simengine/systems/animation_system.hpp"
#include "simengine/systems/input_system.hpp"
#include "simengine/camera/third_person_camera.hpp"

using namespace simengine;

int main(int argc, char** argv) {
    core::World world;
    core::JobSystem jobs(4);

    aircraft::SpawnParams spawn;
    spawn.positionNED = {0.0, 0.0, -2.0}; // 2m AGL

    // If run as `alpha_demo --a320 <path-to-A320-211.xml> <path-to-cfm56_5a1.xml>`,
    // spawn the real JSBSim-imported A320 instead of the generic
    // narrowbody — proves the import path (io/jsbsim_import.hpp)
    // produces something that actually flies in this engine, not just
    // something that prints plausible JSON.
    // `--a320 <fdm.xml> <engine.xml>`: raw-path mode, any JSBSim file pair.
    // `--variant <A318-111|A319-111|A319-131|A320-111|A320-211|A320-231|
    //  A321-211|A321-231> [variantsDir]`: looks the name up in
    // aircraft::kA320Variants (assets/A320_MESH_NOTES.md has the full
    // list) and loads it from assets/fgfs_source/variants/ by default —
    // all eight were verified with tools/fgfs_import (zero warnings
    // each) before being added to the registry.
    core::Entity plane;
    bool usingImport = false;
    if (argc >= 4 && std::string(argv[1]) == "--a320") {
        io::JSBSimAircraftData data = io::importJSBSimAircraft(argv[2], argv[3]);
        std::printf("Loaded JSBSim aircraft '%s' (%zu gear legs, %zu import warnings)\n",
                     data.name.c_str(), data.gearLegs.size(), data.warnings.size());
        plane = aircraft::spawnFromJSBSim(world, data, spawn);
        usingImport = true;
    } else if (argc >= 3 && std::string(argv[1]) == "--variant") {
        const std::string wanted = argv[2];
        const std::string variantsDir = argc >= 4 ? argv[3] : "../assets/fgfs_source/variants";
        const aircraft::A320Variant* found = nullptr;
        for (auto& v : aircraft::kA320Variants) {
            if (v.id == wanted) { found = &v; break; }
        }
        if (!found) {
            std::fprintf(stderr, "Unknown --variant '%s'. Known: ", wanted.c_str());
            for (auto& v : aircraft::kA320Variants) std::fprintf(stderr, "%s ", std::string(v.id).c_str());
            std::fprintf(stderr, "\n");
            return 1;
        }
        io::JSBSimAircraftData data = io::importJSBSimAircraft(
            variantsDir + "/" + std::string(found->fdmFile), variantsDir + "/" + std::string(found->engineFile));
        std::printf("Loaded variant '%s' (%s): %zu gear legs, %zu import warnings\n",
                     std::string(found->displayName).c_str(), data.name.c_str(), data.gearLegs.size(), data.warnings.size());
        plane = aircraft::spawnFromJSBSim(world, data, spawn);
        usingImport = true;
    } else {
        plane = aircraft::spawnGenericNarrowbody(world, spawn);
    }

    systems::InputSystem inputSystem;
    systems::FlightDynamicsSystem flightDynamics;
    systems::LandingGearSystem landingGear;
    systems::EngineSystem engineSystem;
    systems::AnimationSystem animationSystem;
    camera::ThirdPersonCamera chaseCam;

    constexpr double dt = 1.0 / 60.0;
    constexpr int totalTicks = 60 * 20; // 20 simulated seconds

    std::printf("== simengine Alpha Technical Demo ==\n");
    std::printf("Baseplate scenario: idle settle -> scripted takeoff roll\n\n");

    for (int tick = 0; tick < totalTicks; ++tick) {
        const double t = tick * dt;

        systems::InputSnapshot input;
        if (t < 2.0) {
            input.throttle = 0.0; // let it settle onto gear first
        } else if (t < 15.0) {
            input.throttle = 0.95; // takeoff roll
            if (t > 10.0) input.stickPitch = 0.6; // rotate
        } else {
            input.throttle = 0.95;
        }

        inputSystem.apply(world, plane, input, dt);
        flightDynamics.update(world, jobs, dt);
        landingGear.update(world, jobs, dt);
        engineSystem.update(world, jobs, dt);
        animationSystem.update(world, jobs, dt);

        auto* body = world.getComponent<aircraft::RigidBody6DOFComponent>(plane);
        chaseCam.update(body->state.positionNED, body->state.attitude, dt);

        if (tick % 60 == 0) {
            auto* gear = world.getComponent<aircraft::LandingGearComponent>(plane);
            auto* eng = world.getComponent<aircraft::EngineComponent>(plane);
            std::printf(
                "t=%5.1fs  alt=%6.2fm  TAS=%6.2fm/s  pitch/roll=%+.2f/%+.2f  WOW=%s  N1=%5.1f%%  camPos=(%.1f,%.1f,%.1f)\n",
                t,
                body->state.altitudeMeters(),
                body->state.trueAirspeed(),
                body->state.eulerZYX().y, body->state.eulerZYX().z,
                gear->weightOnWheels ? "yes" : "no",
                eng->engines.empty() ? 0.0 : eng->engines[0].n1Percent,
                chaseCam.position().x, chaseCam.position().y, chaseCam.position().z);
        }
    }

    std::printf("\n%s demo loop completed %d ticks (%.1fs simulated) without error.\n",
                usingImport ? "JSBSim-imported A320" : "Generic narrowbody",
                totalTicks, totalTicks * dt);
    return 0;
}
