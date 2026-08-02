// simengine/math/vector3.hpp
// Part of the Mathematics subsystem — Simulation Engine Core.
//
// Design notes:
//  - Templated on scalar type so the same code serves both the
//    double-precision "world space" pipeline (planet-scale coordinates)
//    and the float32 "local space" pipeline (rendering, per-object physics)
//    without duplicating logic or risking drift between two hand-written
//    implementations.
//  - No heap allocation, no virtual dispatch, trivially copyable -> safe
//    to place in tightly packed arrays for cache-friendly, SIMD-autovectorizable
//    batch operations later in the Physics module.
//  - constexpr where possible so constants can be evaluated at compile time.

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace simengine::math {

template <typename T>
struct Vector3 {
    static_assert(std::is_floating_point_v<T>, "Vector3 requires a floating point scalar");

    T x{};
    T y{};
    T z{};

    constexpr Vector3() = default;
    constexpr Vector3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    // ---- Element access ----
    constexpr T& operator[](int i) noexcept {
        return i == 0 ? x : (i == 1 ? y : z);
    }
    constexpr const T& operator[](int i) const noexcept {
        return i == 0 ? x : (i == 1 ? y : z);
    }

    // ---- Arithmetic ----
    constexpr Vector3 operator+(const Vector3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vector3 operator-(const Vector3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vector3 operator-() const noexcept { return {-x, -y, -z}; }
    constexpr Vector3 operator*(T s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(T s) const noexcept { return {x / s, y / s, z / s}; }

    constexpr Vector3& operator+=(const Vector3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vector3& operator-=(const Vector3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vector3& operator*=(T s) noexcept { x *= s; y *= s; z *= s; return *this; }
    constexpr Vector3& operator/=(T s) noexcept { x /= s; y /= s; z /= s; return *this; }

    friend constexpr Vector3 operator*(T s, const Vector3& v) noexcept { return v * s; }

    constexpr bool operator==(const Vector3& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
    constexpr bool operator!=(const Vector3& o) const noexcept { return !(*this == o); }

    // ---- Vector algebra ----
    constexpr T dot(const Vector3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }

    constexpr Vector3 cross(const Vector3& o) const noexcept {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    T lengthSquared() const noexcept { return dot(*this); }
    T length() const noexcept { return std::sqrt(lengthSquared()); }

    // Returns a zero vector if length is below epsilon, rather than
    // producing NaN — physically, a zero-length direction is undefined,
    // and silently propagating NaN through a flight-dynamics integrator
    // is exactly the kind of "fake physics" this engine forbids.
    Vector3 normalized(T epsilon = std::numeric_limits<T>::epsilon() * T(10)) const noexcept {
        const T len = length();
        if (len < epsilon) return Vector3{};
        return *this / len;
    }

    void normalize(T epsilon = std::numeric_limits<T>::epsilon() * T(10)) noexcept {
        *this = normalized(epsilon);
    }

    static constexpr Vector3 zero() noexcept { return {T(0), T(0), T(0)}; }
    static constexpr Vector3 unitX() noexcept { return {T(1), T(0), T(0)}; }
    static constexpr Vector3 unitY() noexcept { return {T(0), T(1), T(0)}; }
    static constexpr Vector3 unitZ() noexcept { return {T(0), T(0), T(1)}; }

    // Linear interpolation - used by the Replay System and animation blending.
    static constexpr Vector3 lerp(const Vector3& a, const Vector3& b, T t) noexcept {
        return a + (b - a) * t;
    }

    // Component-wise cast between precisions (e.g. double world-space ->
    // float local-space just before handing data to the renderer).
    template <typename U>
    Vector3<U> cast() const noexcept {
        return Vector3<U>(static_cast<U>(x), static_cast<U>(y), static_cast<U>(z));
    }
};

using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

} // namespace simengine::math
