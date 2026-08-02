// simengine/core/scheduler.hpp — Core Engine subsystem.
//
// Fixed-timestep accumulator scheduler. This is the mechanism that
// decouples simulation rate from render rate (a hard requirement across
// the brief: Flight Dynamics, Physics, "simulation rate independent from
// rendering"). Standard "fix your timestep" accumulator pattern (Gaffer/
// Fiedler-style), with:
//   - A hard cap on catch-up steps per frame, so a debugger pause or a
//     slow frame doesn't cause a "spiral of death" where the sim tries
//     to simulate an unbounded backlog of steps.
//   - Deterministic step ordering: systems run in registration order,
//     every fixed step, with no dependence on wall-clock jitter — a
//     prerequisite for the Replay System and any deterministic netcode.
//   - An interpolation alpha exposed for the Rendering module to blend
//     between the previous and current fixed states.

#pragma once

#include <chrono>
#include <functional>
#include <vector>
#include <algorithm>

namespace simengine::core {

using SimSystemFn = std::function<void(double fixedDeltaSeconds)>;

class FixedTimestepScheduler {
public:
    explicit FixedTimestepScheduler(double fixedHz = 120.0, int maxStepsPerFrame = 8)
        : fixedDelta_(1.0 / fixedHz)
        , maxStepsPerFrame_(maxStepsPerFrame)
    {}

    // Systems run every fixed step, in the order registered here.
    void addSystem(SimSystemFn system) { systems_.push_back(std::move(system)); }

    double fixedDeltaSeconds() const noexcept { return fixedDelta_; }
    std::uint64_t stepCount() const noexcept { return stepCount_; }

    // Call once per render frame with the wall-clock delta since the last
    // call. Runs zero or more fixed steps to catch the simulation up.
    // Returns the interpolation alpha in [0,1) for the renderer to blend
    // between the pre-step and post-step state.
    double advance(double frameDeltaSeconds) {
        // Clamp pathological frame gaps (e.g. resuming from a breakpoint
        // or window-drag stall) instead of trying to simulate minutes of
        // backlog in one call.
        constexpr double kMaxFrameDelta = 0.25;
        frameDeltaSeconds = std::min(frameDeltaSeconds, kMaxFrameDelta);

        accumulator_ += frameDeltaSeconds;

        int steps = 0;
        while (accumulator_ >= fixedDelta_ && steps < maxStepsPerFrame_) {
            for (auto& system : systems_) system(fixedDelta_);
            accumulator_ -= fixedDelta_;
            ++stepCount_;
            ++steps;
        }

        // If we hit the step cap, drop the remaining backlog rather than
        // letting the accumulator grow unbounded (spiral-of-death guard).
        if (steps == maxStepsPerFrame_ && accumulator_ >= fixedDelta_) {
            accumulator_ = 0.0;
        }

        return accumulator_ / fixedDelta_;
    }

    void reset() { accumulator_ = 0.0; stepCount_ = 0; }

private:
    double fixedDelta_;
    int maxStepsPerFrame_;
    double accumulator_ = 0.0;
    std::uint64_t stepCount_ = 0;
    std::vector<SimSystemFn> systems_;
};

// Minimal high-resolution frame clock; wraps steady_clock so callers don't
// each reinvent delta-time measurement with their own drift bugs.
class FrameClock {
public:
    FrameClock() : last_(Clock::now()) {}

    double tick() {
        const auto now = Clock::now();
        const double dt = std::chrono::duration<double>(now - last_).count();
        last_ = now;
        return dt;
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_;
};

} // namespace simengine::core
