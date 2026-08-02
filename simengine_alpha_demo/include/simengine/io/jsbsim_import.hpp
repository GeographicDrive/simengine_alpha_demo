// simengine/io/jsbsim_import.hpp — IO / compatibility subsystem.
//
// Reads a JSBSim-format aircraft data file (the format the uploaded
// A320-family.zip's A320-211.xml is written in — JSBSim is the flight
// dynamics model FlightGear's "A320-family" aircraft uses, distinct
// from FlightGear's own YASim FDM) and produces simengine::physics
// MassProperties / AeroCoefficients / PropulsionModel / GearLegConfig
// values populated from REAL numbers in that file: wing geometry, mass
// and inertia, actual per-leg spring/damper/friction/geometry for every
// <contact type="BOGEY"> in <ground_reactions>, and real engine static
// thrust from the referenced <turbine_engine> file.
//
// WHAT THIS DOES NOT DO — read before trusting the output:
//  - JSBSim's <aerodynamics> section defines force/moment coefficients
//    as arbitrary <table>-driven, multi-axis, often nonlinear functions
//    (see A320-211.xml's CLalpha/CDalpha/etc — 2D tables over alpha AND
//    flap position). This importer does NOT reproduce that model; it
//    extracts single representative linear derivatives (a slope through
//    the table near alpha=0/beta=0, or the literal value for functions
//    that are already a plain <value> rather than a <table>) to populate
//    simengine's much simpler static-derivative AeroCoefficients struct.
//    This is a genuine simplification, not a full JSBSim-compatible
//    aerodynamics model — treat imported handling qualities as
//    approximate, not authoritative.
//  - Only <contact type="BOGEY"> entries become gear legs (steerable
//    wheels). <contact type="STRUCTURE"> entries (fuselage scrape
//    points — JSBSim's crash/tail-strike contact points) are read but
//    intentionally not turned into gear legs; this engine doesn't yet
//    have a structural-contact model (documented gap, see ROADMAP.md).
//  - Coordinate convention: JSBSim's structural reference frame here is
//    X positive AFT, Y positive right, Z positive UP, all measured from
//    a fixed datum (not the CG). This importer converts every location
//    to simengine's body-axes convention (X positive FORWARD, Y positive
//    right, Z positive DOWN, relative to the CG) via:
//        bodyX = -(structX - cgX) * inchesToMeters
//        bodyY =  (structY - cgY) * inchesToMeters
//        bodyZ = -(structZ - cgZ) * inchesToMeters
//    This was derived from the sign/magnitude pattern in A320-211.xml
//    (nose gear ends up forward and below CG as expected) and cross-
//    checked against that one file; it has not been validated against
//    a second JSBSim aircraft, so treat it as a documented best-effort
//    convention, not a guaranteed-correct general JSBSim importer.

#pragma once

#include <string>
#include <vector>
#include <optional>

#include "../physics/rigid_body_6dof.hpp"
#include "../physics/landing_gear.hpp"

namespace simengine::io {

struct ImportedGearLeg {
    std::string name;                      // JSBSim contact name, e.g. "NOSE_LG"
    simengine::physics::GearLegConfig config;
};

struct JSBSimAircraftData {
    std::string name;
    simengine::physics::MassProperties mass;
    simengine::physics::AeroCoefficients aero;
    simengine::physics::PropulsionModel propulsion;
    int engineCount = 2;
    std::vector<ImportedGearLeg> gearLegs;

    // True if every field this importer considers load-bearing was
    // found in the source file(s). If false, check `warnings` for what
    // was missing/defaulted — the returned struct is still usable (gaps
    // are filled with the same generic-narrowbody defaults
    // aircraft_factory.cpp uses) but should not be presented as a
    // faithful import.
    bool complete = true;
    std::vector<std::string> warnings;
};

// fdmXmlPath:    path to the JSBSim aircraft file, e.g. A320-211.xml
// engineXmlPath: path to the referenced turbine_engine file, e.g.
//                cfm56_5a1.xml (optional — pass empty to skip and keep
//                a generic default thrust value, with a warning noting
//                that).
JSBSimAircraftData importJSBSimAircraft(const std::string& fdmXmlPath,
                                         const std::string& engineXmlPath);

} // namespace simengine::io
