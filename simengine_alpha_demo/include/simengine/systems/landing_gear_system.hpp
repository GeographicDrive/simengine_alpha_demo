// simengine/systems/landing_gear_system.hpp — ECS/Physics glue.
//
// For every entity with {RigidBody6DOFComponent, AircraftComponent,
// LandingGearComponent, FlightControlsComponent}: evaluates each gear leg
// against the ground plane (baseplate, NED z=0 for this Alpha — see
// physics/landing_gear.hpp for the terrain hook), sums the resulting
// forces/moments, and applies them to the rigid body as a velocity
// correction for this tick.
//
// Sequencing: run this AFTER FlightDynamicsSystem::update() each tick.
// Ground contact is deliberately handled as a post-integration
// correction rather than folded into the RK4 derivative — gear
// spring/damper forces are numerically stiff (high spring constant, small
// compression) and coupling them into the same RK4 stages as the aero
// model would force a much smaller aero timestep than the airframe
// needs. This split (soft aero dynamics via RK4, stiff contact via
// semi-implicit correction) is the same strategy most vehicle physics
// engines use for tire/suspension contact.
//
// Also updates gear-position (up/down transition) and weight-on-wheels
// for the AnimationSystem/EngineSystem to read.

#pragma once

#include "../core/ecs.hpp"
#include "../core/job_system.hpp"

namespace simengine::systems {

class LandingGearSystem {
public:
    void update(core::World& world, core::JobSystem& jobs, double dt);
};

} // namespace simengine::systems
