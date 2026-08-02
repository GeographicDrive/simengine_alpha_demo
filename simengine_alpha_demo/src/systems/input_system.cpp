#include "simengine/systems/input_system.hpp"
#include "simengine/aircraft/components.hpp"

#include <algorithm>
#include <cmath>

namespace simengine::systems {

using namespace simengine::aircraft;

void InputSystem::apply(core::World& world, core::Entity playerEntity,
                         const InputSnapshot& input, double dt) {
    auto* controlsComp = world.getComponent<FlightControlsComponent>(playerEntity);
    if (!controlsComp) return;
    auto& controls = controlsComp->controls;

    // Direct-law-style mapping: stick position maps straight to surface
    // deflection (rate-limited so a snap touch input doesn't teleport a
    // control surface in one frame — control-surface actuators are not
    // infinitely fast). A future autopilot/fly-by-wire layer can
    // intercept InputSnapshot before this system runs and substitute its
    // own commanded deflections without touching this code.
    const double maxRatePerSec = 3.0; // rad/s actuator rate, generic
    const double maxDelta = maxRatePerSec * dt;

    auto rateLimit = [&](double current, double target) {
        return current + std::clamp(target - current, -maxDelta, maxDelta);
    };

    controls.deltaElevator = rateLimit(controls.deltaElevator,
        std::clamp(input.stickPitch, -1.0, 1.0) * InputSystem::kMaxElevatorRad);
    controls.deltaAileron = rateLimit(controls.deltaAileron,
        std::clamp(input.stickRoll, -1.0, 1.0) * InputSystem::kMaxAileronRad);
    controls.deltaRudder = rateLimit(controls.deltaRudder,
        std::clamp(input.rudderPedal, -1.0, 1.0) * InputSystem::kMaxRudderRad);

    controls.throttle = std::clamp(input.throttle, 0.0, 1.0);
    controlsComp->tillerCommand = std::clamp(input.tiller, -1.0, 1.0);
    controlsComp->parkingBrake = input.toggleParkingBrake ? 1.0 : controlsComp->parkingBrake;

    // Wheel brakes: continuous pedal command, with parking brake latching
    // full application. Both the analog command (used by the ground-
    // reaction model in landing_gear.cpp) and the legacy boolean (used by
    // aerodynamic brake-drag bookkeeping in rigid_body_6dof.cpp) are kept
    // in sync from a single source of truth here.
    controls.brakeCommand = std::clamp(
        std::max(input.brake, controlsComp->parkingBrake), 0.0, 1.0);
    controls.brakeActive = controls.brakeCommand > 0.05;
    controls.nwsCommand = controlsComp->tillerCommand;

    // Trim: integrates while commanded, like a real trim wheel/switch
    // rather than snapping to a position.
    const double trimRatePerSec = 1.0; // deg/s at full THS travel budget
    if (std::abs(input.trimInput) > 0.02) {
        controls.deltaThsDeg = std::clamp(
            controls.deltaThsDeg + input.trimInput * trimRatePerSec * dt, -15.0, 15.0);
    }

    // Flap handle: discrete detents, stepped on toggle edges. The caller
    // is responsible for only setting toggleFlapsUp/Down true on the
    // input transition (button press), not every tick it's held.
    const double detentStep = 1.0 / static_cast<double>(InputSystem::kFlapDetents);
    if (input.toggleFlapsDown) {
        controlsComp->flapHandlePosition = std::clamp(
            controlsComp->flapHandlePosition + detentStep, 0.0, 1.0);
    }
    if (input.toggleFlapsUp) {
        controlsComp->flapHandlePosition = std::clamp(
            controlsComp->flapHandlePosition - detentStep, 0.0, 1.0);
    }
    controls.flapPosition = controlsComp->flapHandlePosition;

    // Auto-slat schedule: on most transport aircraft slats are not a
    // separately-commanded surface — they extend automatically as soon
    // as any flap is selected (first detent triggers full slat travel,
    // giving the extra stall margin immediately) and retract only once
    // flaps are fully up. This is still its own animation/aero channel
    // (independent CD/alpha-stall contribution), just driven by the flap
    // handle rather than a dedicated lever — matches real aircraft
    // systems and keeps the touch UI from needing a slat-specific control.
    const double slatTarget = controlsComp->flapHandlePosition > 0.01 ? 1.0 : 0.0;
    const double slatRatePerSec = 2.0; // fraction/sec, generic slat actuator rate
    controlsComp->slatHandlePosition = std::clamp(
        controlsComp->slatHandlePosition +
            std::clamp(slatTarget - controlsComp->slatHandlePosition, -1.0, 1.0) * slatRatePerSec * dt,
        0.0, 1.0);
    controls.slatPosition = controlsComp->slatHandlePosition;

    if (input.toggleSpeedbrake) {
        controlsComp->speedbrakeHandle = controlsComp->speedbrakeHandle > 0.5 ? 0.0 : 1.0;
    }
    controls.speedbrakePosition = controlsComp->speedbrakeHandle;

    if (input.toggleSpoiler) {
        controlsComp->spoilerHandle = controlsComp->spoilerHandle > 0.5 ? 0.0 : 1.0;
    }
    controls.spoilerPosition = controlsComp->spoilerHandle;

    if (input.toggleGear) {
        controls.gearDown = !controls.gearDown;
    }

    if (input.toggleReverse) {
        controls.reverseActive = !controls.reverseActive;
    }

    if (input.togglePause) {
        controlsComp->pauseRequested = !controlsComp->pauseRequested;
    }
}

} // namespace simengine::systems
