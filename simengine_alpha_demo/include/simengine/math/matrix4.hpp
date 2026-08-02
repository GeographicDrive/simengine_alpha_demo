// simengine/math/matrix4.hpp
// Part of the Mathematics subsystem — Simulation Engine Core.
//
// Design notes:
//  - Column-major storage to match Vulkan/GLSL conventions directly
//    (the Rendering module will feed this straight into uniform buffers
//    with no transpose step — one less place for a silent bug to hide).
//  - This type is for LOCAL-space transforms (camera, object, projection).
//    World-scale placement uses double-precision origin-rebasing in the
//    World module, not this matrix, to avoid float precision collapse
//    far from the origin (a classic flight-sim bug: jitter far from 0,0,0).

#pragma once

#include "vector3.hpp"
#include "quaternion.hpp"
#include <array>
#include <cmath>

namespace simengine::math {

template <typename T>
struct Matrix4 {
    // m[col][row], column-major.
    std::array<std::array<T, 4>, 4> m{};

    constexpr Matrix4() {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                m[c][r] = (c == r) ? T(1) : T(0);
    }

    static constexpr Matrix4 identity() noexcept { return Matrix4(); }

    static Matrix4 translation(const Vector3<T>& t) noexcept {
        Matrix4 r;
        r.m[3][0] = t.x; r.m[3][1] = t.y; r.m[3][2] = t.z;
        return r;
    }

    static Matrix4 scale(const Vector3<T>& s) noexcept {
        Matrix4 r;
        r.m[0][0] = s.x; r.m[1][1] = s.y; r.m[2][2] = s.z;
        return r;
    }

    static Matrix4 fromQuaternion(const Quaternion<T>& q) noexcept {
        Matrix4 r;
        const T xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const T xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
        const T wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

        r.m[0][0] = T(1) - T(2) * (yy + zz);
        r.m[0][1] = T(2) * (xy + wz);
        r.m[0][2] = T(2) * (xz - wy);

        r.m[1][0] = T(2) * (xy - wz);
        r.m[1][1] = T(1) - T(2) * (xx + zz);
        r.m[1][2] = T(2) * (yz + wx);

        r.m[2][0] = T(2) * (xz + wy);
        r.m[2][1] = T(2) * (yz - wx);
        r.m[2][2] = T(1) - T(2) * (xx + yy);
        return r;
    }

    static Matrix4 trs(const Vector3<T>& t, const Quaternion<T>& q, const Vector3<T>& s) noexcept {
        Matrix4 result = fromQuaternion(q);
        // apply scale to basis columns
        for (int c = 0; c < 3; ++c) {
            const T sc = (c == 0) ? s.x : (c == 1 ? s.y : s.z);
            result.m[c][0] *= sc; result.m[c][1] *= sc; result.m[c][2] *= sc;
        }
        result.m[3][0] = t.x; result.m[3][1] = t.y; result.m[3][2] = t.z;
        return result;
    }

    // Right-handed look-at, matching Vulkan's clip space after the
    // projection below applies its own axis flip.
    static Matrix4 lookAt(const Vector3<T>& eye, const Vector3<T>& target, const Vector3<T>& up) noexcept {
        const Vector3<T> f = (target - eye).normalized();
        const Vector3<T> s = f.cross(up).normalized();
        const Vector3<T> u = s.cross(f);

        Matrix4 r;
        r.m[0][0] = s.x; r.m[1][0] = s.y; r.m[2][0] = s.z;
        r.m[0][1] = u.x; r.m[1][1] = u.y; r.m[2][1] = u.z;
        r.m[0][2] = -f.x; r.m[1][2] = -f.y; r.m[2][2] = -f.z;
        r.m[3][0] = -s.dot(eye);
        r.m[3][1] = -u.dot(eye);
        r.m[3][2] = f.dot(eye);
        return r;
    }

    // Vulkan clip space: depth range [0,1] (not OpenGL's [-1,1]), Y flipped.
    // This is deliberate — feeding an OpenGL-convention projection into
    // Vulkan without correction is a common source of upside-down/incorrect
    // depth-tested renders.
    static Matrix4 perspectiveVulkan(T fovYRadians, T aspect, T zNear, T zFar) noexcept {
        Matrix4 r;
        for (auto& col : r.m) col.fill(T(0));
        const T f = T(1) / std::tan(fovYRadians * T(0.5));
        r.m[0][0] = f / aspect;
        r.m[1][1] = -f; // Vulkan Y-flip
        r.m[2][2] = zFar / (zNear - zFar);
        r.m[2][3] = T(-1);
        r.m[3][2] = (zFar * zNear) / (zNear - zFar);
        return r;
    }

    Matrix4 operator*(const Matrix4& o) const noexcept {
        Matrix4 r;
        for (int c = 0; c < 4; ++c) {
            for (int row = 0; row < 4; ++row) {
                T sum = T(0);
                for (int k = 0; k < 4; ++k) sum += m[k][row] * o.m[c][k];
                r.m[c][row] = sum;
            }
        }
        return r;
    }

    Vector3<T> transformPoint(const Vector3<T>& p) const noexcept {
        const T x = m[0][0]*p.x + m[1][0]*p.y + m[2][0]*p.z + m[3][0];
        const T y = m[0][1]*p.x + m[1][1]*p.y + m[2][1]*p.z + m[3][1];
        const T z = m[0][2]*p.x + m[1][2]*p.y + m[2][2]*p.z + m[3][2];
        const T w = m[0][3]*p.x + m[1][3]*p.y + m[2][3]*p.z + m[3][3];
        if (w != T(0) && w != T(1)) return Vector3<T>(x, y, z) / w;
        return Vector3<T>(x, y, z);
    }

    Vector3<T> transformDirection(const Vector3<T>& d) const noexcept {
        return Vector3<T>(
            m[0][0]*d.x + m[1][0]*d.y + m[2][0]*d.z,
            m[0][1]*d.x + m[1][1]*d.y + m[2][1]*d.z,
            m[0][2]*d.x + m[1][2]*d.y + m[2][2]*d.z
        );
    }

    // General 4x4 inverse via Gauss-Jordan with partial pivoting.
    // Used sparingly (view/projection inversion for picking, not per-vertex),
    // so clarity is prioritized over a hand-unrolled cofactor expansion.
    Matrix4 inverse(bool* outSuccess = nullptr) const noexcept {
        std::array<std::array<T, 8>, 4> a{};
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) a[r][c] = m[c][r];
            a[r][4 + r] = T(1);
        }
        for (int col = 0; col < 4; ++col) {
            int pivot = col;
            T best = std::abs(a[col][col]);
            for (int r = col + 1; r < 4; ++r) {
                const T v = std::abs(a[r][col]);
                if (v > best) { best = v; pivot = r; }
            }
            if (best < std::numeric_limits<T>::epsilon() * T(100)) {
                if (outSuccess) *outSuccess = false;
                return Matrix4::identity();
            }
            std::swap(a[col], a[pivot]);
            const T invPivot = T(1) / a[col][col];
            for (int c = 0; c < 8; ++c) a[col][c] *= invPivot;
            for (int r = 0; r < 4; ++r) {
                if (r == col) continue;
                const T factor = a[r][col];
                for (int c = 0; c < 8; ++c) a[r][c] -= factor * a[col][c];
            }
        }
        Matrix4 result;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                result.m[c][r] = a[r][4 + c];
        if (outSuccess) *outSuccess = true;
        return result;
    }

    const T* data() const noexcept { return &m[0][0]; }
};

using Matrix4f = Matrix4<float>;
using Matrix4d = Matrix4<double>;

} // namespace simengine::math
