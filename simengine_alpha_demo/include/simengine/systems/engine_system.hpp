// simengine/systems/engine_system.hpp — ECS/Propulsion glue.
//
// Drives per-engine N1/N2 spool dynamics and reverser position toward
// the commanded throttle/reverse-lever state, for entities with
// {EngineComponent, FlightControlsComponent}. This is deliberately
// separate from the thrust actually fed into the rigid-body derivative
// (which physics::calculateDerivatives computes internally from
// FlightControls::throttle + PropulsionModel — see rigid_body_6dof.cpp):
// EngineSystem's job is instrumentation/animation/sound fidelity (spool-up
// lag, N1 gauge, fan-blade rotation rate, reverser transition), not the
// force itself. Keeping these decoupled means a future multi-engine
// asymmetric-thrust model can extend the force side without touching
// this system, and vice versa.

#pragma once

#include "../core/ecs.hpp"
#include "../core/job_system.hpp"

namespace simengine::systems {

class EngineSystem {
public:
    void update(core::World& world, core::JobSystem& jobs, double dt);
};

} // namespace simengine::systems
