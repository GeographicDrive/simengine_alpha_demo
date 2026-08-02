// simengine/systems/flight_dynamics_system.hpp — ECS/Physics glue.
//
// Advances every entity carrying {RigidBody6DOFComponent, AircraftComponent,
// FlightControlsComponent} by one fixed tick using the pure
// simengine::physics module (RK4, ISA atmosphere, aero/thrust). Iterates
// via JobSystem::parallelFor so N aircraft integrate concurrently — the
// physics module is documented pure/thread-safe per entity, so this is
// safe as long as each entity's own component storage slot is only
// touched by its own task (guaranteed by the [begin,end) partition).

#pragma once

#include "../core/ecs.hpp"
#include "../core/job_system.hpp"

namespace simengine::systems {

class FlightDynamicsSystem {
public:
    // dt: fixed simulation timestep in seconds (e.g. 1.0/60.0). Call once
    // per fixed tick from the sim loop, before LandingGearSystem (which
    // reads the post-RK4 state to compute ground contact corrections).
    void update(core::World& world, core::JobSystem& jobs, double dt);
};

} // namespace simengine::systems
