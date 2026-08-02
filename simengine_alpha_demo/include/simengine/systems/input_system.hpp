// simengine/systems/input_system.hpp — ECS/Input glue.
//
// Converts a single frame's raw input snapshot (as would arrive from the
// mobile touch UI — see mobile_ui/) into rate-limited, smoothed changes
// to FlightControlsComponent. Kept as its own system (rather than the UI
// writing FlightControlsComponent directly) so control-surface rate
// limits, flap-handle detents, and trim integration live in one place
// regardless of input source (touch UI today; keyboard/joystick/network
// input later, all producing the same InputSnapshot).
//
// This system targets a single player-controlled entity per call
// (typical for a flight sim's player aircraft); AI/replay-driven
// aircraft would populate FlightControlsComponent through a different
// path entirely (not this system).

#pragma once

#include "../core/ecs.hpp"

namespace simengine::systems {

// Raw input for one tick, in the [-1,1] / [0,1] ranges a touch UI widget
// naturally produces (see mobile_ui/index.html for the producing side).
struct InputSnapshot {
    double stickPitch = 0.0;   // [-1,1], + = nose up command
    double stickRoll = 0.0;    // [-1,1], + = roll right command
    double rudderPedal = 0.0;  // [-1,1]
    double throttle = 0.0;     // [0,1]
    double trimInput = 0.0;    // [-1,1], held while adjusting
    double tiller = 0.0;       // [-1,1], nose wheel steering on ground
    double brake = 0.0;        // [0,1]

    bool toggleGear = false;
    bool toggleFlapsUp = false;   // one flap-detent step up (less flap)
    bool toggleFlapsDown = false; // one flap-detent step down (more flap)
    bool toggleSpeedbrake = false;
    bool toggleSpoiler = false;   // ground/flight spoiler lever, independent
                                   // of the dedicated speedbrake surface
    bool toggleReverse = false;
    bool toggleParkingBrake = false;
    bool togglePause = false;
};

class InputSystem {
public:
    // Max control-surface deflection, radians — matches typical
    // transport-category limits; override per aircraft later via a data
    // file if needed.
    static constexpr double kMaxElevatorRad = 0.35;
    static constexpr double kMaxAileronRad = 0.35;
    static constexpr double kMaxRudderRad = 0.35;
    static constexpr int kFlapDetents = 4;

    void apply(core::World& world, core::Entity playerEntity,
               const InputSnapshot& input, double dt);
};

} // namespace simengine::systems
