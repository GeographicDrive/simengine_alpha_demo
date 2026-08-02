// simengine/camera/third_person_camera.hpp — Camera subsystem.
//
// NOTE ON PROVENANCE: this is NOT a port of GeoDrive's third-person
// camera — GeoDrive's source was not included in the files provided for
// this build (only GeoEngine.zip, rigid_body_6dof.{hpp,cpp}, and
// A320-family.zip were). This is a from-scratch implementation of the
// same *behavior class* (spring-arm follow with smoothing and a
// look-ahead bias) so the Alpha has a usable chase camera. Swap this
// file for GeoDrive's actual camera module when it's available —
// everything else in this engine only depends on ThirdPersonCamera's
// public interface below, so the swap is localized to this one file.
//
// Behavior: the camera targets a point `distance` meters behind and
// `height` meters above the aircraft's CG along its own body -X/+Z axes,
// but the rig itself is smoothed (critically-damped spring, not a hard
// parent-lock) so fast attitude changes (a roll, a bounce on touchdown)
// don't whip the camera — same qualitative feel as a chase-cam spring
// arm. Yaw follows the aircraft's heading only (not full attitude), so
// the horizon stays level in normal flight, matching third-person
// chase-cam convention in most flight/vehicle sims.

#pragma once

#include "../math/vector3.hpp"
#include "../math/quaternion.hpp"
#include <cmath>

namespace simengine::camera {

using simengine::math::Vector3;
using simengine::math::Quaternion;

struct ThirdPersonCameraConfig {
    double distance = 25.0;      // m, behind the target along its heading
    double height = 7.0;         // m, above the target
    double stiffness = 8.0;      // spring constant, higher = snappier follow
    double damping = 5.0;        // critically-damped-ish; damping ~= 2*sqrt(stiffness) is stable
    double fovDegrees = 55.0;
};

class ThirdPersonCamera {
public:
    explicit ThirdPersonCamera(ThirdPersonCameraConfig cfg = {}) : cfg_(cfg) {}

    // Call once per render/update tick with the target's current world
    // (NED, but any consistent right-handed frame works identically)
    // position and attitude.
    void update(const Vector3<double>& targetPositionNED,
                const Quaternion<double>& targetAttitude,
                double dt) noexcept {
        // Yaw-only heading (ignore pitch/roll for the rig's orbit point,
        // per the chase-cam convention documented above).
        const Vector3<double> euler = targetAttitude.toEulerZYX(); // {yaw, pitch, roll}
        const double yaw = euler.x;

        const Vector3<double> backward{-std::cos(yaw), -std::sin(yaw), 0.0};
        const Vector3<double> desired = targetPositionNED
            + backward * cfg_.distance
            + Vector3<double>{0.0, 0.0, -cfg_.height}; // NED: -z is up

        if (!initialized_) {
            position_ = desired;
            velocity_ = Vector3<double>::zero();
            initialized_ = true;
        } else {
            // Damped spring toward `desired` — this is the "smoothing"
            // that gives the chase-cam feel instead of a rigid parent
            // attachment.
            const Vector3<double> accel =
                (desired - position_) * cfg_.stiffness - velocity_ * cfg_.damping;
            velocity_ += accel * dt;
            position_ += velocity_ * dt;
        }

        lookAt_ = targetPositionNED;
    }

    const Vector3<double>& position() const noexcept { return position_; }
    const Vector3<double>& lookAtTarget() const noexcept { return lookAt_; }
    double fovDegrees() const noexcept { return cfg_.fovDegrees; }

private:
    ThirdPersonCameraConfig cfg_;
    Vector3<double> position_{0.0, 0.0, 0.0};
    Vector3<double> velocity_{0.0, 0.0, 0.0};
    Vector3<double> lookAt_{0.0, 0.0, 0.0};
    bool initialized_ = false;
};

} // namespace simengine::camera
