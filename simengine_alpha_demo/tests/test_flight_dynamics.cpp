// tests/test_flight_dynamics.cpp — sanity tests for the ECS flight
// pipeline (physics -> gear -> engine -> animation). Not exhaustive
// coverage of every coefficient; these check the properties that would
// most obviously break if the ECS wiring were wrong (e.g. forces not
// applied, gear never settling, animation channels never written).

#include <cassert>
#include <cmath>
#include <cstdio>

#include "simengine/core/ecs.hpp"
#include "simengine/core/job_system.hpp"
#include "simengine/aircraft/aircraft_factory.hpp"
#include "simengine/systems/flight_dynamics_system.hpp"
#include "simengine/systems/landing_gear_system.hpp"
#include "simengine/systems/engine_system.hpp"
#include "simengine/systems/animation_system.hpp"
#include "simengine/systems/input_system.hpp"

using namespace simengine;

namespace {

int g_failures = 0;

void expect(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAILED: %s\n", msg);
        ++g_failures;
    } else {
        std::printf("ok: %s\n", msg);
    }
}

} // namespace

int main() {
    core::JobSystem jobs(2);

    // --- Test 1: an aircraft dropped just above the baseplate settles
    // onto its gear (weight-on-wheels becomes true, vertical velocity
    // damps toward ~0) rather than falling through or bouncing forever. ---
    {
        core::World world;
        aircraft::SpawnParams sp;
        sp.positionNED = {0.0, 0.0, -3.0}; // 3m AGL
        core::Entity plane = aircraft::spawnGenericNarrowbody(world, sp);

        systems::InputSystem inputSystem;
        systems::FlightDynamicsSystem flight;
        systems::LandingGearSystem gearSys;
        systems::EngineSystem engineSys;

        const double dt = 1.0 / 60.0;
        bool sawWeightOnWheels = false;
        for (int i = 0; i < 60 * 8; ++i) {
            systems::InputSnapshot input; // idle throttle, no controls
            inputSystem.apply(world, plane, input, dt);
            flight.update(world, jobs, dt);
            gearSys.update(world, jobs, dt);
            engineSys.update(world, jobs, dt);

            auto* gear = world.getComponent<aircraft::LandingGearComponent>(plane);
            if (gear->weightOnWheels) sawWeightOnWheels = true;
        }

        auto* gear = world.getComponent<aircraft::LandingGearComponent>(plane);
        auto* body = world.getComponent<aircraft::RigidBody6DOFComponent>(plane);
        expect(sawWeightOnWheels, "dropped aircraft eventually achieves weight-on-wheels");
        expect(gear->weightOnWheels, "aircraft is settled (on ground) after 8s idle on baseplate");
        expect(std::abs(body->state.w) < 2.0, "vertical body-axis velocity damps out once settled");
        for (auto& wheel : gear->legs) {
            expect(wheel.state.compression > 0.0 && wheel.state.compression <= wheel.config.travel,
                   "gear leg compression is within [0, travel] once settled");
        }
    }

    // --- Test 2: full throttle on the ground with no back-pressure
    // produces forward acceleration (thrust > drag+friction at low
    // speed), proving FlightDynamicsSystem's thrust path and
    // LandingGearSystem's rolling friction are both actually wired in. ---
    {
        core::World world;
        aircraft::SpawnParams sp;
        sp.positionNED = {0.0, 0.0, -2.0};
        core::Entity plane = aircraft::spawnGenericNarrowbody(world, sp);

        systems::InputSystem inputSystem;
        systems::FlightDynamicsSystem flight;
        systems::LandingGearSystem gearSys;
        systems::EngineSystem engineSys;

        const double dt = 1.0 / 60.0;
        // Let it settle first.
        for (int i = 0; i < 120; ++i) {
            systems::InputSnapshot idle;
            inputSystem.apply(world, plane, idle, dt);
            flight.update(world, jobs, dt);
            gearSys.update(world, jobs, dt);
            engineSys.update(world, jobs, dt);
        }
        for (int i = 0; i < 300; ++i) {
            systems::InputSnapshot full;
            full.throttle = 1.0;
            inputSystem.apply(world, plane, full, dt);
            flight.update(world, jobs, dt);
            gearSys.update(world, jobs, dt);
            engineSys.update(world, jobs, dt);
        }

        auto* body = world.getComponent<aircraft::RigidBody6DOFComponent>(plane);
        auto* eng = world.getComponent<aircraft::EngineComponent>(plane);
        expect(body->state.u > 1.0, "full throttle produces meaningful forward speed within 5s");
        expect(eng->engines[0].n1Percent > 80.0, "engine N1 spools up toward max under full throttle command");
    }

    // --- Test 3: animation channels reflect control input (decoupling
    // sanity check: AnimationSystem must actually read
    // FlightControlsComponent). ---
    {
        core::World world;
        core::Entity plane = aircraft::spawnGenericNarrowbody(world, {});

        systems::InputSystem inputSystem;
        systems::AnimationSystem animSys;
        const double dt = 1.0 / 60.0;

        systems::InputSnapshot input;
        input.stickPitch = 1.0; // full nose-up command
        for (int i = 0; i < 30; ++i) {
            inputSystem.apply(world, plane, input, dt);
        }
        animSys.update(world, jobs, dt);

        auto* anim = world.getComponent<aircraft::AnimationComponent>(plane);
        double* elevator = anim->find("surface.elevator");
        expect(elevator != nullptr, "surface.elevator animation channel exists after AnimationSystem runs");
        expect(elevator != nullptr && *elevator > 0.1,
               "surface.elevator animation channel reflects sustained full nose-up stick input");
    }

    if (g_failures == 0) {
        std::printf("\nAll flight_dynamics sanity tests passed.\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
    return 1;
}
