// simengine/systems/animation_system.hpp — ECS/Presentation glue.
//
// Reads the physics/control/gear/engine state each tick and writes
// normalized animation channel values into AnimationComponent. This is
// the ONLY system that produces animation values — no other system
// writes directly into a mesh/bone/node, which is what "decoupled from
// physics via an Animation System" means in practice: FlightDynamicsSystem
// / LandingGearSystem / EngineSystem never know a renderer exists, and
// the (not-yet-implemented, see docs/ROADMAP.md) rendering backend only
// ever reads AnimationComponent::channels by name.
//
// Channel naming convention (consumed by the rendering backend's
// node/bone binding table, e.g. loaded from an aircraft's animation
// config): all channels are doubles.
//   surface.aileron_left / aileron_right   [-1,1]
//   surface.elevator                        [-1,1]
//   surface.rudder                          [-1,1]
//   surface.spoiler                         [0,1] independent lever channel
//                                             (per-panel L/R fan-out, if the
//                                             model needs it, is a rendering-
//                                             side binding-table concern)
//   surface.speedbrake                      [0,1] dedicated speedbrake surface,
//                                             independent of surface.spoiler
//   surface.flap                            [0,1] (handle position; per-detent
//                                             flap angle is a rendering-side
//                                             lookup table, kept out of engine core)
//   surface.slat                            [0,1] auto-scheduled from the flap
//                                             handle (own actuator rate/channel,
//                                             not a 1:1 copy of surface.flap)
//   trim.elevator                           [-1,1]
//   gear.position                           [0,1] (0=up,1=down)
//   gear.wheel_<name>.compression           [0,1]
//   gear.wheel_<name>.spin                  radians, unwrapped (renderer mods 2*pi)
//   gear.wheel_<name>.steer                 radians
//   engine.<index>.n1                       [0,100]
//   engine.<index>.fan_rotation              radians, unwrapped
//   engine.<index>.reverser                 [0,1]

#pragma once

#include "../core/ecs.hpp"
#include "../core/job_system.hpp"

namespace simengine::systems {

class AnimationSystem {
public:
    void update(core::World& world, core::JobSystem& jobs, double dt);
};

} // namespace simengine::systems
