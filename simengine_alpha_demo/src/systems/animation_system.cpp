#include "simengine/systems/animation_system.hpp"
#include "simengine/aircraft/components.hpp"

#include <cmath>

namespace simengine::systems {

using namespace simengine::aircraft;

namespace {
// Fan rotation rate: purely presentational (radians/sec at 100% N1),
// tuned to "looks right" rather than a real fan RPM map — swap for a
// per-engine data value when real engine data is wired in.
constexpr double kFanRadPerSecAt100N1 = 120.0;
}

void AnimationSystem::update(core::World& world, core::JobSystem& jobs, double dt) {
    auto& animStorage = world.storage<AnimationComponent>();
    world.storage<FlightControlsComponent>();
    world.storage<LandingGearComponent>();
    world.storage<EngineComponent>();

    auto& entities = animStorage.entities();
    auto& dense = animStorage.dense();

    jobs.parallelFor(dense.size(), /*minGrainSize=*/4,
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                core::Entity e = entities[i];
                AnimationComponent& anim = dense[i];

                if (auto* c = world.getComponent<FlightControlsComponent>(e)) {
                    const auto& controls = c->controls;
                    anim.getOrAdd("surface.aileron_left") = -controls.deltaAileron;
                    anim.getOrAdd("surface.aileron_right") = controls.deltaAileron;
                    anim.getOrAdd("surface.elevator") = controls.deltaElevator;
                    anim.getOrAdd("surface.rudder") = controls.deltaRudder;
                    anim.getOrAdd("surface.flap") = c->flapHandlePosition;
                    anim.getOrAdd("surface.slat") = c->slatHandlePosition; // auto-scheduled, independent channel
                    anim.getOrAdd("surface.speedbrake") = c->speedbrakeHandle;
                    anim.getOrAdd("surface.spoiler") = c->spoilerHandle;   // independent lever, own channel
                    anim.getOrAdd("trim.elevator") = controls.deltaThsDeg / 15.0; // normalize by a generic +-15deg range
                    anim.getOrAdd("gear.nws_command") = c->tillerCommand;
                }

                if (auto* gear = world.getComponent<LandingGearComponent>(e)) {
                    anim.getOrAdd("gear.position") = gear->gearPosition;
                    for (auto& wheel : gear->legs) {
                        const std::string base = "gear.wheel_" + wheel.animationNodeName;
                        anim.getOrAdd(base + ".compression") =
                            wheel.config.travel > 1e-6 ? wheel.state.compression / wheel.config.travel : 0.0;
                        anim.getOrAdd(base + ".spin") = wheel.state.wheelSpinRad;
                        anim.getOrAdd(base + ".steer") = wheel.state.steerAngleRad;
                    }
                }

                if (auto* eng = world.getComponent<EngineComponent>(e)) {
                    for (std::size_t idx = 0; idx < eng->engines.size(); ++idx) {
                        auto& es = eng->engines[idx];
                        const std::string prefix = "engine." + std::to_string(idx) + ".";
                        anim.getOrAdd(prefix + "n1") = es.n1Percent;
                        double& fanRot = anim.getOrAdd(prefix + "fan_rotation");
                        fanRot += (es.n1Percent / 100.0) * kFanRadPerSecAt100N1 * dt;
                        if (fanRot > 1e6) fanRot = std::fmod(fanRot, 2.0 * M_PI); // periodic rebase, avoids unbounded growth
                        anim.getOrAdd(prefix + "reverser") = es.reverserPosition;
                    }
                }
            }
        });
}

} // namespace simengine::systems
