#include "simengine/systems/landing_gear_system.hpp"
#include "simengine/aircraft/components.hpp"
#include "simengine/physics/landing_gear.hpp"

#include <algorithm>
#include <cmath>

namespace simengine::systems {

using namespace simengine::aircraft;
using namespace simengine::physics;

void LandingGearSystem::update(core::World& world, core::JobSystem& jobs, double dt) {
    auto& gearStorage = world.storage<LandingGearComponent>();
    world.storage<RigidBody6DOFComponent>();
    world.storage<AircraftComponent>();
    world.storage<FlightControlsComponent>();

    auto& entities = gearStorage.entities();
    auto& dense = gearStorage.dense();

    jobs.parallelFor(dense.size(), /*minGrainSize=*/4,
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                core::Entity e = entities[i];
                auto* body = world.getComponent<RigidBody6DOFComponent>(e);
                auto* aircraft = world.getComponent<AircraftComponent>(e);
                auto* controlsComp = world.getComponent<FlightControlsComponent>(e);
                if (!body || !aircraft || !controlsComp) continue;

                LandingGearComponent& gear = dense[i];
                RigidBody6DOFState& state = body->state;
                const FlightControls& controls = controlsComp->controls;

                // --- Gear up/down transition (drives the retract/extend
                // animation and gates contact evaluation). ---
                gear.gearTargetPosition = controls.gearDown ? 1.0 : 0.0;
                const double maxDelta = gear.gearTransitionRate * dt;
                const double posDelta = std::clamp(
                    gear.gearTargetPosition - gear.gearPosition, -maxDelta, maxDelta);
                gear.gearPosition = std::clamp(gear.gearPosition + posDelta, 0.0, 1.0);

                Vector3<double> forceSumBody{0.0, 0.0, 0.0};
                Vector3<double> momentSumBody{0.0, 0.0, 0.0};
                bool anyOnGround = false;

                const Vector3<double> bodyVel{state.u, state.v, state.w};
                const Vector3<double> bodyRates{state.p, state.q, state.r};

                for (auto& wheel : gear.legs) {
                    // Contact point in body axes at full/current extension.
                    const Vector3<double> offsetBody = wheel.config.attachBody +
                        Vector3<double>{0.0, 0.0, wheel.config.strutLength - wheel.state.compression};
                    const Vector3<double> offsetWorld = state.attitude.rotate(offsetBody);
                    const double contactZ = state.positionNED.z + offsetWorld.z; // NED, +down
                    const double heightAboveGround = -contactZ; // baseplate at z=0

                    GearLegInput input;
                    input.gearDown = controls.gearDown;
                    input.brakeCommand = std::clamp(
                        std::max(controls.brakeCommand, controlsComp->parkingBrake), 0.0, 1.0);
                    input.parkingBrake = controlsComp->parkingBrake > 0.5;
                    input.steerCommand = wheel.config.steerable ? controls.nwsCommand
                                                                 : controls.deltaRudder;

                    GearLegForce f = evaluateGearLeg(
                        wheel.state, wheel.config, input,
                        bodyVel, bodyRates, heightAboveGround, dt, aircraft->mass.mass);

                    forceSumBody += f.forceBody;
                    momentSumBody += f.momentBody;
                    anyOnGround = anyOnGround || wheel.state.onGround;
                }

                gear.weightOnWheels = anyOnGround;

                if (forceSumBody.lengthSquared() < 1e-9 && momentSumBody.lengthSquared() < 1e-9) {
                    continue; // fully airborne, nothing to correct
                }

                // Semi-implicit velocity correction (see header for why
                // this is applied post-integration rather than inside
                // RK4). Angular correction uses the diagonal inertia
                // terms only (ignores Ixz product-of-inertia coupling
                // for this contact correction — the coupling matters for
                // in-flight dynamics, which RK4 already handles via the
                // full gamma-coefficient equations; on the ground the
                // dominant term is the direct diagonal response).
                const double mass = std::max(1.0, aircraft->mass.mass);
                state.u += (forceSumBody.x / mass) * dt;
                state.v += (forceSumBody.y / mass) * dt;
                state.w += (forceSumBody.z / mass) * dt;

                const double Ix = std::max(1.0, aircraft->mass.Ix);
                const double Iy = std::max(1.0, aircraft->mass.Iy);
                const double Iz = std::max(1.0, aircraft->mass.Iz);
                state.p += (momentSumBody.x / Ix) * dt;
                state.q += (momentSumBody.y / Iy) * dt;
                state.r += (momentSumBody.z / Iz) * dt;
            }
        });
}

} // namespace simengine::systems
