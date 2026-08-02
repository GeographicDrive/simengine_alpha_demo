// simengine/math/geodetic.hpp
// Part of the Mathematics subsystem — Simulation Engine Core.
//
// Provides conversions between:
//   - Geodetic (lat, lon, altitude) on the WGS84 reference ellipsoid
//   - ECEF (Earth-Centered, Earth-Fixed) Cartesian, double precision
//
// This is the physically correct model required for planet-scale
// simulation: Earth is an oblate spheroid, not a sphere, and the
// difference (~21 km at the equator vs. poles) is significant for any
// navigation/avionics module (GPS, INS, ILS glideslope geometry) that
// will consume this later. Approximating Earth as a sphere here would
// count as "fake physics" under this project's own rules.
//
// Reference: WGS84 defining parameters (NIMA TR8350.2).

#pragma once

#include "vector3.hpp"
#include <cmath>

namespace simengine::math {

struct WGS84 {
    static constexpr double kSemiMajorAxis = 6378137.0;            // a, meters
    static constexpr double kFlattening = 1.0 / 298.257223563;     // f
    static constexpr double kSemiMinorAxis = kSemiMajorAxis * (1.0 - kFlattening); // b
    // First eccentricity squared: e^2 = f(2-f)
    static constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);
};

struct GeodeticCoord {
    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double altitudeMeters = 0.0; // height above the WGS84 ellipsoid, NOT above sea level (MSL requires a geoid model — separate concern)
};

// Radius of curvature in the prime vertical, N(phi) — the distance from
// the ellipsoid surface up to the polar axis along the surface normal.
inline double primeVerticalRadius(double latitudeRad) noexcept {
    const double sinLat = std::sin(latitudeRad);
    return WGS84::kSemiMajorAxis / std::sqrt(1.0 - WGS84::kEccentricitySquared * sinLat * sinLat);
}

// Geodetic -> ECEF (exact, closed-form; no iteration needed this direction).
inline Vector3d geodeticToEcef(const GeodeticCoord& g) noexcept {
    const double N = primeVerticalRadius(g.latitudeRad);
    const double cosLat = std::cos(g.latitudeRad);
    const double sinLat = std::sin(g.latitudeRad);
    const double cosLon = std::cos(g.longitudeRad);
    const double sinLon = std::sin(g.longitudeRad);

    const double x = (N + g.altitudeMeters) * cosLat * cosLon;
    const double y = (N + g.altitudeMeters) * cosLat * sinLon;
    const double z = (N * (1.0 - WGS84::kEccentricitySquared) + g.altitudeMeters) * sinLat;
    return Vector3d(x, y, z);
}

// ECEF -> Geodetic via Bowring's method: fast-converging closed-form
// iteration, accurate to sub-millimeter within 2-3 iterations. Preferred
// over the naive iterative solution for numerical stability near the poles.
inline GeodeticCoord ecefToGeodetic(const Vector3d& ecef, int iterations = 3) noexcept {
    const double x = ecef.x, y = ecef.y, z = ecef.z;
    const double a = WGS84::kSemiMajorAxis;
    const double b = WGS84::kSemiMinorAxis;
    const double e2 = WGS84::kEccentricitySquared;
    const double ep2 = (a * a - b * b) / (b * b); // second eccentricity squared

    const double p = std::sqrt(x * x + y * y);
    const double lon = std::atan2(y, x);

    if (p < 1e-9) {
        // On the polar axis: longitude undefined, handle degenerate case explicitly.
        GeodeticCoord g;
        g.latitudeRad = z >= 0.0 ? (M_PI / 2.0) : -(M_PI / 2.0);
        g.longitudeRad = 0.0;
        g.altitudeMeters = std::abs(z) - b;
        return g;
    }

    double theta = std::atan2(z * a, p * b);
    double lat = std::atan2(
        z + ep2 * b * std::pow(std::sin(theta), 3),
        p - e2 * a * std::pow(std::cos(theta), 3)
    );

    for (int i = 0; i < iterations; ++i) {
        const double N = primeVerticalRadius(lat);
        const double h = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1.0 - e2 * N / (N + h)));
    }

    const double N = primeVerticalRadius(lat);
    const double alt = p / std::cos(lat) - N;

    return GeodeticCoord{lat, lon, alt};
}

// Local East-North-Up (ENU) basis at a geodetic reference point — this is
// the frame the Flight Dynamics module will actually integrate forces in
// locally, since ECEF axes aren't aligned with "up" anywhere except the
// poles/equator crossings.
struct EnuBasis {
    Vector3d east, north, up;
    Vector3d originEcef;
};

inline EnuBasis enuBasisAt(const GeodeticCoord& refPoint) noexcept {
    const double sinLat = std::sin(refPoint.latitudeRad);
    const double cosLat = std::cos(refPoint.latitudeRad);
    const double sinLon = std::sin(refPoint.longitudeRad);
    const double cosLon = std::cos(refPoint.longitudeRad);

    EnuBasis basis;
    basis.east  = Vector3d(-sinLon, cosLon, 0.0);
    basis.north = Vector3d(-sinLat * cosLon, -sinLat * sinLon, cosLat);
    basis.up    = Vector3d(cosLat * cosLon, cosLat * sinLon, sinLat);
    basis.originEcef = geodeticToEcef(refPoint);
    return basis;
}

// Converts an ECEF point into local ENU meters relative to basis.origin.
// This is the "origin rebasing" mechanism that keeps physics in
// well-conditioned float range even though the world is stored in
// double-precision ECEF — critical for avoiding jitter far from (0,0,0),
// a well-known failure mode in naive planet-scale sims.
inline Vector3d ecefToEnu(const Vector3d& ecef, const EnuBasis& basis) noexcept {
    const Vector3d delta = ecef - basis.originEcef;
    return Vector3d(basis.east.dot(delta), basis.north.dot(delta), basis.up.dot(delta));
}

inline Vector3d enuToEcef(const Vector3d& enu, const EnuBasis& basis) noexcept {
    return basis.originEcef
         + basis.east * enu.x
         + basis.north * enu.y
         + basis.up * enu.z;
}

} // namespace simengine::math
