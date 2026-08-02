#include "simengine/systems/flight_dynamics_system.hpp"
#include "simengine/aircraft/components.hpp"
#include "simengine/physics/rigid_body_6dof.hpp"

namespace simengine::systems {

using namespace simengine::aircraft;
using namespace simengine::physics;

void FlightDynamicsSystem::update(core::World& world, core::JobSystem& jobs, double dt) {
    // Warm up storages so parallelFor below never triggers a lazy
    // World::storage<T>() insertion (which mutates the type->holder map
    // and is not itself thread-safe) from a worker thread.
    auto& bodies = world.storage<RigidBody6DOFComponent>();
    world.storage<AircraftComponent>();
    world.storage<FlightControlsComponent>();

    auto& entities = bodies.entities();
    auto& dense = bodies.dense();

    jobs.parallelFor(dense.size(), /*minGrainSize=*/8,
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                core::Entity e = entities[i];
                auto* aircraft = world.getComponent<AircraftComponent>(e);
                auto* controlsComp = world.getComponent<FlightControlsComponent>(e);
                if (!aircraft || !controlsComp) continue; // not a flyable entity

                RigidBody6DOFComponent& body = dense[i];
                integrateRK4(body.state, dt, controlsComp->controls,
                             aircraft->mass, aircraft->aero, aircraft->propulsion);
            }
        });
}

} // namespace simengine::systems
