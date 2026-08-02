#include "simengine/systems/engine_system.hpp"
#include "simengine/aircraft/components.hpp"

#include <algorithm>
#include <cmath>

namespace simengine::systems {

using namespace simengine::aircraft;

namespace {
constexpr double kIdleN1 = 20.0;
constexpr double kMaxN1 = 100.0;
constexpr double kReverserRate = 0.5; // fraction per second
}

void EngineSystem::update(core::World& world, core::JobSystem& jobs, double dt) {
    auto& engineStorage = world.storage<EngineComponent>();
    world.storage<FlightControlsComponent>();

    auto& entities = engineStorage.entities();
    auto& dense = engineStorage.dense();

    jobs.parallelFor(dense.size(), /*minGrainSize=*/4,
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                core::Entity e = entities[i];
                auto* controlsComp = world.getComponent<FlightControlsComponent>(e);
                if (!controlsComp) continue;

                EngineComponent& eng = dense[i];
                const FlightControls& controls = controlsComp->controls;
                const double throttleN1Target = kIdleN1 +
                    std::clamp(controls.throttle, 0.0, 1.0) * (kMaxN1 - kIdleN1);

                for (auto& e2 : eng.engines) {
                    e2.commandedThrottle = controls.throttle;

                    const double target = e2.running ? throttleN1Target : 0.0;
                    const double rate = (target > e2.n1Percent ? eng.spoolRateUp : eng.spoolRateDown)
                        * (kMaxN1 - kIdleN1);
                    const double maxDelta = rate * dt;
                    const double delta = std::clamp(target - e2.n1Percent, -maxDelta, maxDelta);
                    e2.n1Percent = std::clamp(e2.n1Percent + delta, 0.0, kMaxN1);
                    // N2 tracks N1 with a fixed offset/ratio for a generic
                    // twin-spool turbofan feel — real per-engine maps are
                    // a data-file concern, not hardcoded here.
                    e2.n2Percent = std::clamp(e2.n1Percent * 1.05 + 5.0, 0.0, 105.0);

                    e2.reverserDeployed = controls.reverseActive;
                    const double reverserTarget = e2.reverserDeployed ? 1.0 : 0.0;
                    const double maxRevDelta = kReverserRate * dt;
                    e2.reverserPosition = std::clamp(
                        e2.reverserPosition + std::clamp(reverserTarget - e2.reverserPosition,
                                                          -maxRevDelta, maxRevDelta),
                        0.0, 1.0);

                    // Thrust readback for instrumentation only (actual
                    // force is computed inside the physics module).
                    e2.thrustNewtons = (e2.n1Percent / kMaxN1) * (e2.reverserDeployed ? -0.45 : 1.0);
                }
            }
        });
}

} // namespace simengine::systems
