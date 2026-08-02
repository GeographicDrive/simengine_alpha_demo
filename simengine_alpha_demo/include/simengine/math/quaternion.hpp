// simengine/math/quaternion.hpp
// Part of the Mathematics subsystem — Simulation Engine Core.
//
// Design notes:
//  - Orientation is represented exclusively as a unit quaternion, never
//    Euler angles, at the state-storage level. Euler angles are only ever
//    derived transiently for display/instrumentation. This avoids gimbal
//    lock in the flight dynamics integrator (a real risk during high pitch
//    attitudes / spins, which are explicitly in scope for this engine).
//  - Hamilton convention, right-handed, w-last-in-memory-order avoided in
//    favor of w-first (w,x,y,z) to match common aerospace texts (e.g.
//    Stevens & Lewis "Aircraft Control and Simulation").
//  - Renormalization is exposed explicitly (not automatic on every op)
//    because forcing a normalize() after every multiply would hide
//    integrator drift that the Physics module needs to detect and correct
//    with a proper drift-correction step, not a silent patch.

#pragma once

#include "vector3.hpp"
#include <cmath>

namespace simengine::math {

template <typename T>
struct Quaternion {
    T w{T(1)};
    T x{T(0)};
    T y{T(0)};
    T z{T(0)};

    constexpr Quaternion() = default;
    constexpr Quaternion(T w_, T x_, T y_, T z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quaternion identity() noexcept { return Quaternion(T(1), T(0), T(0), T(0)); }

    // Axis MUST be normalized by the caller; we don't silently normalize
    // here because that would mask a bug in the caller producing a
    // degenerate axis (e.g. zero angular rate integrated incorrectly).
    static Quaternion fromAxisAngle(const Vector3<T>& axis, T angleRadians) noexcept {
        const T half = angleRadians * T(0.5);
        const T s = std::sin(half);
        return Quaternion(std::cos(half), axis.x * s, axis.y * s, axis.z * s);
    }

    // Aerospace convention: yaw (Z, heading) -> pitch (Y) -> roll (X),
    // intrinsic, applied in that order (ZYX / 3-2-1 sequence), matching
    // standard body-axis definitions used throughout flight dynamics.
    static Quaternion fromEulerZYX(T yaw, T pitch, T roll) noexcept {
        const T cy = std::cos(yaw * T(0.5)),   sy = std::sin(yaw * T(0.5));
        const T cp = std::cos(pitch * T(0.5)), sp = std::sin(pitch * T(0.5));
        const T cr = std::cos(roll * T(0.5)),  sr = std::sin(roll * T(0.5));

        return Quaternion(
            cr * cp * cy + sr * sp * sy,   // w
            sr * cp * cy - cr * sp * sy,   // x (roll)
            cr * sp * cy + sr * cp * sy,   // y (pitch)
            cr * cp * sy - sr * sp * cy    // z (yaw)
        );
    }

    // Returns (yaw, pitch, roll) in radians. Pitch is clamped at the
    // +-90 degree singularity rather than producing NaN; a caller that
    // needs to fly through vertical (e.g. a spin/loop simulation) must
    // use the quaternion state directly, not this derived representation.
    Vector3<T> toEulerZYX() const noexcept {
        Vector3<T> e;
        // roll (x-axis rotation)
        const T sinr_cosp = T(2) * (w * x + y * z);
        const T cosr_cosp = T(1) - T(2) * (x * x + y * y);
        e.x = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        const T sinp = T(2) * (w * y - z * x);
        if (std::abs(sinp) >= T(1)) {
            e.y = std::copysign(static_cast<T>(M_PI / 2.0), sinp);
        } else {
            e.y = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        const T siny_cosp = T(2) * (w * z + x * y);
        const T cosy_cosp = T(1) - T(2) * (y * y + z * z);
        e.z = std::atan2(siny_cosp, cosy_cosp);
        return e; // (roll, pitch, yaw) stored in (x,y,z)
    }

    constexpr Quaternion operator*(const Quaternion& o) const noexcept {
        return Quaternion(
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        );
    }

    Vector3<T> rotate(const Vector3<T>& v) const noexcept {
        // t = 2 * cross(q.xyz, v)
        const Vector3<T> qv(x, y, z);
        const Vector3<T> t = qv.cross(v) * T(2);
        return v + t * w + qv.cross(t);
    }

    constexpr Quaternion conjugate() const noexcept { return Quaternion(w, -x, -y, -z); }

    T lengthSquared() const noexcept { return w * w + x * x + y * y + z * z; }
    T length() const noexcept { return std::sqrt(lengthSquared()); }

    Quaternion normalized() const noexcept {
        const T len = length();
        if (len < std::numeric_limits<T>::epsilon()) return Quaternion::identity();
        const T inv = T(1) / len;
        return Quaternion(w * inv, x * inv, y * inv, z * inv);
    }

    void normalize() noexcept { *this = normalized(); }

    // Spherical linear interpolation - required for smooth camera/replay
    // playback and for control-surface actuator blending.
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, T t) noexcept {
        Quaternion b2 = b;
        T dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

        // Take the short path around the hypersphere.
        if (dot < T(0)) {
            b2 = Quaternion(-b.w, -b.x, -b.y, -b.z);
            dot = -dot;
        }

        constexpr T kThreshold = T(0.9995);
        if (dot > kThreshold) {
            // Nearly parallel: linear interpolation is numerically stable
            // here and avoids a divide-by-near-zero in sin(theta).
            Quaternion result(
                a.w + t * (b2.w - a.w),
                a.x + t * (b2.x - a.x),
                a.y + t * (b2.y - a.y),
                a.z + t * (b2.z - a.z)
            );
            return result.normalized();
        }

        const T theta0 = std::acos(dot);
        const T theta = theta0 * t;
        const T sinTheta0 = std::sin(theta0);
        const T sinTheta = std::sin(theta);

        const T s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
        const T s1 = sinTheta / sinTheta0;

        return Quaternion(
            s0 * a.w + s1 * b2.w,
            s0 * a.x + s1 * b2.x,
            s0 * a.y + s1 * b2.y,
            s0 * a.z + s1 * b2.z
        );
    }
};

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace simengine::math
