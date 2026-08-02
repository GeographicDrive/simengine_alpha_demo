// Unit tests for the Mathematics subsystem.
// No test framework dependency (keeps this subsystem's build graph
// self-contained) — a minimal assert-based harness with clear failure
// output is sufficient at this stage and is itself replaceable later.

#include "simengine/math/vector3.hpp"
#include "simengine/math/quaternion.hpp"
#include "simengine/math/matrix4.hpp"
#include "simengine/math/geodetic.hpp"

#include <cstdio>
#include <cmath>

using namespace simengine::math;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__); \
    } \
} while (0)

static bool nearlyEqual(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

static void test_vector3() {
    std::printf("test_vector3\n");
    Vector3d a(1, 2, 3), b(4, 5, 6);
    CHECK(nearlyEqual((a + b).x, 5.0));
    CHECK(nearlyEqual(a.dot(b), 32.0));
    Vector3d cross = a.cross(b); // (2*6-3*5, 3*4-1*6, 1*5-2*4) = (-3, 6, -3)
    CHECK(nearlyEqual(cross.x, -3.0) && nearlyEqual(cross.y, 6.0) && nearlyEqual(cross.z, -3.0));

    Vector3d unit = Vector3d(3, 0, 4).normalized();
    CHECK(nearlyEqual(unit.length(), 1.0));
    CHECK(nearlyEqual(unit.x, 0.6) && nearlyEqual(unit.y, 0.0) && nearlyEqual(unit.z, 0.8));

    // Degenerate normalize must not produce NaN.
    Vector3d z = Vector3d::zero().normalized();
    CHECK(z.x == 0.0 && z.y == 0.0 && z.z == 0.0);
}

static void test_quaternion() {
    std::printf("test_quaternion\n");
    // 90 degree yaw should rotate +X to +Y (right-handed, Z-up convention).
    Quaterniond q = Quaterniond::fromAxisAngle(Vector3d::unitZ(), M_PI / 2.0);
    Vector3d rotated = q.rotate(Vector3d::unitX());
    CHECK(nearlyEqual(rotated.x, 0.0, 1e-9) && nearlyEqual(rotated.y, 1.0, 1e-9));

    // Roundtrip Euler -> quaternion -> Euler.
    const double yaw = 0.3, pitch = 0.2, roll = 0.1;
    Quaterniond q2 = Quaterniond::fromEulerZYX(yaw, pitch, roll);
    Vector3d euler = q2.toEulerZYX(); // (roll, pitch, yaw)
    CHECK(nearlyEqual(euler.x, roll, 1e-9));
    CHECK(nearlyEqual(euler.y, pitch, 1e-9));
    CHECK(nearlyEqual(euler.z, yaw, 1e-9));

    // Quaternion must stay unit length after composition.
    Quaterniond composed = q * q2;
    CHECK(nearlyEqual(composed.length(), 1.0, 1e-9));

    // Slerp at t=0 and t=1 returns endpoints.
    Quaterniond s0 = Quaterniond::slerp(q, q2, 0.0);
    Quaterniond s1 = Quaterniond::slerp(q, q2, 1.0);
    CHECK(nearlyEqual(s0.w, q.w, 1e-9));
    CHECK(nearlyEqual(s1.w, q2.w, 1e-9));
}

static void test_matrix4() {
    std::printf("test_matrix4\n");
    Matrix4d t = Matrix4d::translation(Vector3d(10, 20, 30));
    Vector3d p = t.transformPoint(Vector3d(1, 1, 1));
    CHECK(nearlyEqual(p.x, 11) && nearlyEqual(p.y, 21) && nearlyEqual(p.z, 31));

    // Inverse of translation composed with itself should return to origin point.
    bool ok = false;
    Matrix4d inv = t.inverse(&ok);
    CHECK(ok);
    Vector3d back = inv.transformPoint(p);
    CHECK(nearlyEqual(back.x, 1) && nearlyEqual(back.y, 1) && nearlyEqual(back.z, 1));

    // Rotation matrix from quaternion should match direct quaternion rotation.
    Quaterniond q = Quaterniond::fromAxisAngle(Vector3d::unitZ(), M_PI / 2.0);
    Matrix4d r = Matrix4d::fromQuaternion(q);
    Vector3d viaMatrix = r.transformDirection(Vector3d::unitX());
    Vector3d viaQuat = q.rotate(Vector3d::unitX());
    CHECK(nearlyEqual(viaMatrix.x, viaQuat.x, 1e-9));
    CHECK(nearlyEqual(viaMatrix.y, viaQuat.y, 1e-9));

    // Singular matrix (all zero) inverse must report failure, not garbage.
    Matrix4d singular;
    for (auto& col : singular.m) col.fill(0.0);
    bool singularOk = true;
    singular.inverse(&singularOk);
    CHECK(!singularOk);
}

static void test_geodetic() {
    std::printf("test_geodetic\n");
    // Known reference: Equator/Prime Meridian at sea level -> ECEF (a, 0, 0)
    GeodeticCoord eq{0.0, 0.0, 0.0};
    Vector3d ecef = geodeticToEcef(eq);
    CHECK(nearlyEqual(ecef.x, WGS84::kSemiMajorAxis, 1e-3));
    CHECK(nearlyEqual(ecef.y, 0.0, 1e-3));
    CHECK(nearlyEqual(ecef.z, 0.0, 1e-3));

    // North pole -> ECEF (0, 0, b)
    GeodeticCoord np{M_PI / 2.0, 0.0, 0.0};
    Vector3d ecefNp = geodeticToEcef(np);
    CHECK(nearlyEqual(ecefNp.z, WGS84::kSemiMinorAxis, 1e-3));

    // Roundtrip test across a spread of lat/lon/alt values.
    const double testLats[] = {0.0, 0.5, -0.5, 1.0, -1.0, 1.4};
    const double testLons[] = {0.0, 1.0, -2.0, 3.0, -3.0};
    const double testAlts[] = {0.0, 1000.0, 11000.0, -50.0};

    for (double lat : testLats) {
        for (double lon : testLons) {
            for (double alt : testAlts) {
                GeodeticCoord g{lat, lon, alt};
                Vector3d e = geodeticToEcef(g);
                GeodeticCoord back = ecefToGeodetic(e, 5);
                CHECK(nearlyEqual(back.latitudeRad, lat, 1e-9));
                CHECK(nearlyEqual(back.longitudeRad, lon, 1e-9));
                CHECK(nearlyEqual(back.altitudeMeters, alt, 1e-4));
            }
        }
    }

    // ENU basis: "up" at the reference point should point away from Earth's center.
    GeodeticCoord ref{0.7, 1.2, 500.0};
    EnuBasis basis = enuBasisAt(ref);
    CHECK(nearlyEqual(basis.up.length(), 1.0, 1e-9));
    CHECK(nearlyEqual(basis.east.length(), 1.0, 1e-9));
    CHECK(nearlyEqual(basis.north.length(), 1.0, 1e-9));
    // East, North, Up must be mutually orthogonal.
    CHECK(nearlyEqual(basis.east.dot(basis.north), 0.0, 1e-9));
    CHECK(nearlyEqual(basis.east.dot(basis.up), 0.0, 1e-9));
    CHECK(nearlyEqual(basis.north.dot(basis.up), 0.0, 1e-9));

    // A point 100m above the reference in ENU should roundtrip through ECEF.
    Vector3d enuPoint(0.0, 0.0, 100.0);
    Vector3d ecefPoint = enuToEcef(enuPoint, basis);
    Vector3d backEnu = ecefToEnu(ecefPoint, basis);
    CHECK(nearlyEqual(backEnu.z, 100.0, 1e-6));
}

int main() {
    test_vector3();
    test_quaternion();
    test_matrix4();
    test_geodetic();

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures > 0) {
        std::printf("RESULT: FAIL (%d failures)\n", g_failures);
        return 1;
    }
    std::printf("RESULT: PASS\n");
    return 0;
}
