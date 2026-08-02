// tests/test_jsbsim_import.cpp — sanity tests for the JSBSim/FlightGear
// compatibility importer, run against the real A320-211.xml /
// cfm56_5a1.xml files bundled under assets/fgfs_source/. These are NOT
// synthetic fixtures — if the source files move or the importer's
// parsing breaks, this test is what catches it.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "simengine/io/jsbsim_import.hpp"
#include "simengine/core/ecs.hpp"
#include "simengine/core/job_system.hpp"
#include "simengine/aircraft/aircraft_factory.hpp"
#include "simengine/systems/flight_dynamics_system.hpp"
#include "simengine/systems/landing_gear_system.hpp"
#include "simengine/systems/input_system.hpp"

using namespace simengine;

namespace {
int g_failures = 0;
void expect(bool cond, const char* msg) {
    if (!cond) { std::fprintf(stderr, "FAILED: %s\n", msg); ++g_failures; }
    else { std::printf("ok: %s\n", msg); }
}
}

int main(int argc, char** argv) {
    // Allow the test's working directory assumption to be overridden,
    // since ctest and a manual `./test_jsbsim_import` run from
    // different cwds.
    std::string base = argc >= 2 ? argv[1] : "../assets/fgfs_source";
    std::string fdmPath = base + "/A320-211.xml";
    std::string enginePath = base + "/cfm56_5a1.xml";

    io::JSBSimAircraftData data = io::importJSBSimAircraft(fdmPath, enginePath);

    expect(!data.name.empty(), "importer reads the aircraft name from fdm_config name attribute");
    expect(data.warnings.empty(), "no import warnings against the real A320-211 + cfm56_5a1 files");

    // Real, known A320-family magnitudes (with generous tolerance —
    // this is checking "did we parse the right numbers", not validating
    // aerodynamic fidelity).
    expect(data.mass.S > 100.0 && data.mass.S < 140.0, "imported wing area is in a plausible A320 range (100-140 m^2)");
    expect(data.mass.b > 30.0 && data.mass.b < 38.0, "imported wingspan is in a plausible A320 range (30-38 m)");
    expect(data.mass.mass > 40000.0 && data.mass.mass < 80000.0, "imported mass is in a plausible A320 operating-weight range (40-80 t)");
    expect(data.propulsion.T_max > 80000.0 && data.propulsion.T_max < 150000.0,
           "imported per-engine static thrust is in a plausible CFM56-5A1 range (80-150 kN)");

    expect(data.gearLegs.size() == 3, "importer finds exactly 3 BOGEY contacts (nose + 2 main) in A320-211.xml");
    bool foundSteerable = false;
    for (auto& g : data.gearLegs) if (g.config.steerable) foundSteerable = true;
    expect(foundSteerable, "at least one imported gear leg (the nose gear) is steerable");

    // End-to-end: the imported aircraft should actually settle onto the
    // baseplate under this engine's physics, same as the sanity check
    // for the generic aircraft — proves the import produces something
    // the engine can actually simulate, not just plausible-looking JSON.
    {
        core::World world;
        core::JobSystem jobs(2);
        aircraft::SpawnParams sp;
        sp.positionNED = {0.0, 0.0, -4.0};
        core::Entity plane = aircraft::spawnFromJSBSim(world, data, sp);

        systems::InputSystem inputSystem;
        systems::FlightDynamicsSystem flight;
        systems::LandingGearSystem gearSys;
        const double dt = 1.0 / 60.0;
        for (int i = 0; i < 60 * 10; ++i) {
            systems::InputSnapshot input;
            inputSystem.apply(world, plane, input, dt);
            flight.update(world, jobs, dt);
            gearSys.update(world, jobs, dt);
        }
        auto* gear = world.getComponent<aircraft::LandingGearComponent>(plane);
        expect(gear->weightOnWheels, "JSBSim-imported A320 settles onto the baseplate under this engine's gear physics");
    }

    if (g_failures == 0) {
        std::printf("\nAll jsbsim_import sanity tests passed.\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
    return 1;
}
