#include "simengine/io/jsbsim_import.hpp"
#include "simengine/io/xml.hpp"

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>

namespace simengine::io {

using simengine::math::Vector3;
using simengine::physics::MassProperties;
using simengine::physics::AeroCoefficients;
using simengine::physics::PropulsionModel;
using simengine::physics::PropulsionType;
using simengine::physics::GearLegConfig;

namespace {

constexpr double kInToM = 0.0254;
constexpr double kFtToM = 0.3048;
constexpr double kFt2ToM2 = 0.09290304;
constexpr double kLbToKg = 0.45359237;
constexpr double kSlugFt2ToKgM2 = 1.35581795;
constexpr double kLbfPerFtToNPerM = 14.5939;      // spring/damper unit conversion
constexpr double kLbfToN = 4.4482216;
constexpr double kDegToRad = 0.017453292519943295;

std::optional<std::string> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Recursively finds the first <function name="wanted"> anywhere under el.
const XmlElement* findFunctionByName(const XmlElement& el, const std::string& wanted) {
    for (auto& c : el.children) {
        if (c->tag == "function") {
            if (auto* n = c->attribute("name"); n && *n == wanted) return c.get();
        }
        if (auto* found = findFunctionByName(*c, wanted)) return found;
    }
    return nullptr;
}

// Splits JSBSim tableData text into rows of numeric tokens, one vector
// per non-empty line — mirrors the file's own row-per-line layout so we
// don't need to know the column count in advance.
std::vector<std::vector<double>> parseTableRows(const std::string& text) {
    std::vector<std::vector<double>> rows;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream tok(line);
        std::vector<double> row;
        double v;
        while (tok >> v) row.push_back(v);
        if (!row.empty()) rows.push_back(std::move(row));
    }
    return rows;
}

// Extracts a single representative linear derivative from a JSBSim
// <function>: if it's a plain product ending in a literal <value>,
// returns that value directly; if it wraps a <table>, returns the slope
// through the table nearest zero on the first independent variable
// (using the first data column for 2-independentVar tables — see
// jsbsim_import.hpp's header comment on why this is an approximation,
// not a full reproduction of the table).
std::optional<double> extractDerivative(const XmlElement* fn, std::vector<std::string>& warnings, const std::string& fieldName) {
    if (!fn) {
        warnings.push_back(fieldName + ": function not found in source file, using engine default");
        return std::nullopt;
    }
    if (auto* table = fn->find("table")) {
        const auto indepVars = table->childrenOf("independentVar");
        const auto* tableDataEl = table->child("tableData");
        if (!tableDataEl) {
            warnings.push_back(fieldName + ": <table> with no <tableData>, using engine default");
            return std::nullopt;
        }
        auto rows = parseTableRows(tableDataEl->text);
        if (rows.empty()) {
            warnings.push_back(fieldName + ": empty table, using engine default");
            return std::nullopt;
        }

        std::vector<std::pair<double, double>> series; // (x, y) using column 0 of data
        if (indepVars.size() >= 2) {
            // First row is column headers; skip it. Each subsequent row
            // is [x, col0, col1, ...] — take col0 (first data column).
            for (std::size_t i = 1; i < rows.size(); ++i) {
                if (rows[i].size() >= 2) series.emplace_back(rows[i][0], rows[i][1]);
            }
        } else {
            for (auto& r : rows) {
                if (r.size() >= 2) series.emplace_back(r[0], r[1]);
            }
        }
        if (series.size() < 2) {
            warnings.push_back(fieldName + ": table has fewer than 2 usable rows, using engine default");
            return std::nullopt;
        }
        // Find the two points that bracket x=0 most tightly (or the two
        // closest to it if all same sign) and take the finite-difference
        // slope through them as the linear derivative.
        std::sort(series.begin(), series.end());
        std::size_t bestIdx = 0;
        double bestDist = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < series.size(); ++i) {
            const double d = std::abs(series[i].first);
            if (d < bestDist) { bestDist = d; bestIdx = i; }
        }
        std::size_t i0 = bestIdx == 0 ? 0 : bestIdx - 1;
        std::size_t i1 = std::min(series.size() - 1, i0 + 1);
        if (i1 == i0) { i1 = std::min(series.size() - 1, i0 + 1); if (i1 == i0 && i0 > 0) i0 -= 1; }
        const double dx = series[i1].first - series[i0].first;
        if (std::abs(dx) < 1e-9) {
            warnings.push_back(fieldName + ": degenerate table slope, using engine default");
            return std::nullopt;
        }
        return (series[i1].second - series[i0].second) / dx;
    }
    if (auto* v = fn->find("value")) {
        try { return std::stod(v->text); } catch (...) {}
    }
    warnings.push_back(fieldName + ": function has neither <table> nor <value>, using engine default");
    return std::nullopt;
}

} // namespace

JSBSimAircraftData importJSBSimAircraft(const std::string& fdmXmlPath,
                                         const std::string& engineXmlPath) {
    JSBSimAircraftData out;

    auto fdmText = readFile(fdmXmlPath);
    if (!fdmText) {
        out.complete = false;
        out.warnings.push_back("could not read fdm file: " + fdmXmlPath);
        return out;
    }
    auto root = parseXml(*fdmText);
    if (!root) {
        out.complete = false;
        out.warnings.push_back("failed to parse fdm XML: " + fdmXmlPath);
        return out;
    }
    if (auto* n = root->attribute("name")) out.name = *n;

    // --- metrics ---
    if (auto* metrics = root->find("metrics")) {
        if (auto s = metrics->childDouble("wingarea")) out.mass.S = *s * kFt2ToM2;
        if (auto s = metrics->childDouble("wingspan")) out.mass.b = *s * kFtToM;
        if (auto s = metrics->childDouble("chord")) out.mass.c = *s * kFtToM;
        if (out.mass.S > 0.0 && out.mass.b > 0.0) out.mass.AR = (out.mass.b * out.mass.b) / out.mass.S;
    } else {
        out.warnings.push_back("no <metrics> section found");
    }

    // --- mass_balance ---
    double cgX = 0.0, cgY = 0.0, cgZ = 0.0;
    if (auto* mb = root->find("mass_balance")) {
        if (auto s = mb->childDouble("ixx")) out.mass.Ix = *s * kSlugFt2ToKgM2;
        if (auto s = mb->childDouble("iyy")) out.mass.Iy = *s * kSlugFt2ToKgM2;
        if (auto s = mb->childDouble("izz")) out.mass.Iz = *s * kSlugFt2ToKgM2;
        if (auto s = mb->childDouble("ixz")) out.mass.Ixz = *s * kSlugFt2ToKgM2;

        double totalWeightLbs = 0.0;
        if (auto s = mb->childDouble("emptywt")) totalWeightLbs += *s;
        for (auto* pm : mb->childrenOf("pointmass")) {
            if (auto s = pm->childDouble("weight")) totalWeightLbs += *s;
        }
        if (totalWeightLbs > 0.0) out.mass.mass = totalWeightLbs * kLbToKg;
        else out.warnings.push_back("no emptywt/pointmass data, mass left at engine default");

        if (auto* cg = mb->find("location")) {
            if (auto* nameAttr = cg->attribute("name"); nameAttr && *nameAttr == "CG") {
                if (auto x = cg->childDouble("x")) cgX = *x;
                if (auto y = cg->childDouble("y")) cgY = *y;
                if (auto z = cg->childDouble("z")) cgZ = *z;
            }
        }
        // location tag search above only finds the first <location>
        // under mass_balance; explicitly re-scan all locations for one
        // named CG in case AERORP/EYEPOINT/VRP-style siblings precede it.
        for (auto& c : mb->children) {
            if (c->tag == "location") {
                if (auto* nameAttr = c->attribute("name"); nameAttr && *nameAttr == "CG") {
                    if (auto x = c->childDouble("x")) cgX = *x;
                    if (auto y = c->childDouble("y")) cgY = *y;
                    if (auto z = c->childDouble("z")) cgZ = *z;
                }
            }
        }
    } else {
        out.warnings.push_back("no <mass_balance> section found");
    }
    if (out.mass.Ix > 0 && out.mass.Iy > 0 && out.mass.Iz > 0) {
        out.mass.finalize();
    } else {
        out.warnings.push_back("incomplete inertia data, gamma coefficients not finalized");
    }

    // --- ground_reactions: BOGEY contacts become gear legs ---
    if (auto* gr = root->find("ground_reactions")) {
        for (auto* contact : gr->childrenOf("contact")) {
            auto* typeAttr = contact->attribute("type");
            if (!typeAttr || *typeAttr != "BOGEY") continue; // skip STRUCTURE scrape points

            ImportedGearLeg leg;
            leg.name = contact->attribute("name") ? *contact->attribute("name") : "gear";

            double x = cgX, y = cgY, z = cgZ;
            if (auto* loc = contact->child("location")) {
                if (auto v = loc->childDouble("x")) x = *v;
                if (auto v = loc->childDouble("y")) y = *v;
                if (auto v = loc->childDouble("z")) z = *v;
            }
            // JSBSim structural frame (X aft+, Z up+, from a fixed datum)
            // -> simengine body axes (X forward+, Z down+, relative to
            // CG). See header comment for derivation/caveats.
            const double bodyX = -(x - cgX) * kInToM;
            const double bodyY = (y - cgY) * kInToM;
            const double bodyZ = -(z - cgZ) * kInToM;

            // JSBSim's single-point contact model has no separate
            // "strut travel" concept; we place the attach point at CG
            // height and let the full offset be the strut's extended
            // length, with a generic travel fraction (documented
            // approximation — see header).
            leg.config.attachBody = Vector3<double>{bodyX, bodyY, 0.0};
            leg.config.strutLength = std::max(0.3, std::abs(bodyZ));
            leg.config.travel = std::clamp(leg.config.strutLength * 0.18, 0.2, 0.55);

            if (auto v = contact->childDouble("spring_coeff")) leg.config.springConstant = *v * kLbfPerFtToNPerM;
            if (auto v = contact->childDouble("damping_coeff")) leg.config.damperConstant = *v * kLbfPerFtToNPerM;
            if (auto v = contact->childDouble("static_friction")) leg.config.staticFriction = *v;
            if (auto v = contact->childDouble("dynamic_friction")) leg.config.dynamicFriction = *v;
            if (auto v = contact->childDouble("rolling_friction")) leg.config.rollingResistance = *v;

            double maxSteerDeg = 0.0;
            if (auto v = contact->childDouble("max_steer")) maxSteerDeg = *v;
            leg.config.steerable = maxSteerDeg > 0.5;
            leg.config.maxSteerAngleRad = maxSteerDeg * kDegToRad;

            std::string brakeGroup;
            if (auto* bg = contact->child("brake_group")) brakeGroup = bg->text;
            leg.config.hasBrake = !brakeGroup.empty() && brakeGroup != "NONE";

            // Wheel radius isn't in the JSBSim contact block; use a
            // plausible size by role (nose vs main), same magnitudes
            // aircraft_factory.cpp's generic aircraft already uses.
            leg.config.wheelRadius = leg.config.steerable ? 0.45 : 0.55;

            out.gearLegs.push_back(std::move(leg));
        }
        if (out.gearLegs.empty()) out.warnings.push_back("no BOGEY contacts found under <ground_reactions>");
    } else {
        out.warnings.push_back("no <ground_reactions> section found");
    }

    // --- aerodynamics: representative linear derivatives only (see
    // header comment — this is a deliberate simplification of JSBSim's
    // full table-driven model). Pre-seed with generic-narrowbody
    // magnitudes so any field this importer can't find in the source
    // file falls back to a plausible value instead of a hard zero. ---
    out.aero.CL0 = 0.20; out.aero.CLa = 5.5; out.aero.CLq = 8.0; out.aero.CLde = 0.6;
    out.aero.CD0 = 0.022; out.aero.e = 0.80; out.aero.Mcrit = 0.78; out.aero.Kc = 0.10;
    out.aero.alphaStall = 0.28;
    out.aero.CD_gear = 0.018; out.aero.CD_brake = 0.0;
    out.aero.CLflap = 0.6; out.aero.alphaStallFlap = 0.22; out.aero.CD_flap = 0.02;
    out.aero.alphaStallSlat = 0.09; out.aero.CD_slat = 0.004;
    out.aero.CLspoiler = 0.35; out.aero.CD_spoiler = 0.045;
    out.aero.CD_speedbrake = 0.028;
    out.aero.hasLoadFactorProtection = true;
    out.aero.nMaxClean = 2.5; out.aero.nMinClean = -1.0;
    out.aero.nMaxFlap = 2.0; out.aero.nMinFlap = 0.0;
    out.aero.CYb = -0.8; out.aero.CYp = 0.0; out.aero.CYdr = 0.15;
    out.aero.Clb = -0.09; out.aero.Clp = -0.4; out.aero.Clda = 0.12;
    out.aero.Cm0 = 0.02; out.aero.Cma = -1.0; out.aero.Cmq = -18.0; out.aero.Cmde = -1.3; out.aero.Cmdt = -0.6;
    out.aero.Cnb = 0.12; out.aero.Cnp = -0.02; out.aero.Cnr = -0.2; out.aero.Cndr = -0.09;

    if (auto* aero = root->find("aerodynamics")) {
        auto take = [&](const char* jsbName, double& field, double fallback) {
            auto v = extractDerivative(findFunctionByName(*aero, jsbName), out.warnings, jsbName);
            field = v.value_or(fallback);
        };
        take("aero/coefficient/CDo", out.aero.CD0, out.aero.CD0);
        take("aero/coefficient/CLalpha", out.aero.CLa, out.aero.CLa);
        take("aero/coefficient/CLDe", out.aero.CLde, out.aero.CLde);
        take("aero/coefficient/CYb", out.aero.CYb, out.aero.CYb);
        take("aero/coefficient/Clb", out.aero.Clb, out.aero.Clb);
        take("aero/coefficient/Clp", out.aero.Clp, out.aero.Clp);
        take("aero/coefficient/Clda", out.aero.Clda, out.aero.Clda);
        take("aero/coefficient/Cldr", out.aero.CYdr, out.aero.CYdr); // JSBSim has no direct CYdr; Cldr used as nearest proxy, flagged via warnings from `take` if missing
        take("aero/coefficient/Cmo", out.aero.Cm0, out.aero.Cm0);
        take("aero/coefficient/Cmalpha", out.aero.Cma, out.aero.Cma);
        take("aero/coefficient/CmDe", out.aero.Cmde, out.aero.Cmde);
        take("aero/coefficient/Cmq", out.aero.Cmq, out.aero.Cmq);
        take("aero/coefficient/Cnr", out.aero.Cnr, out.aero.Cnr);
        take("aero/coefficient/Cnb", out.aero.Cnb, out.aero.Cnb);
        take("aero/coefficient/Cndr", out.aero.Cndr, out.aero.Cndr);
    } else {
        out.warnings.push_back("no <aerodynamics> section found, aero left at engine defaults");
    }

    // --- propulsion: real static thrust from the referenced engine file ---
    out.propulsion.type = PropulsionType::Jet;
    out.propulsion.T_max = 120000.0; // generic default until/unless overwritten below
    if (!engineXmlPath.empty()) {
        if (auto engText = readFile(engineXmlPath)) {
            if (auto engRoot = parseXml(*engText)) {
                if (auto v = engRoot->childDouble("milthrust")) {
                    out.propulsion.T_max = *v * kLbfToN;
                } else {
                    out.warnings.push_back("engine file had no <milthrust>, thrust left at engine default");
                }
            } else {
                out.warnings.push_back("failed to parse engine XML: " + engineXmlPath);
            }
        } else {
            out.warnings.push_back("could not read engine file: " + engineXmlPath);
        }
    } else {
        out.warnings.push_back("no engine file provided, thrust left at engine default");
    }

    out.complete = out.warnings.empty();
    return out;
}

} // namespace simengine::io
