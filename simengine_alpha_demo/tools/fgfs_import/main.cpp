// tools/fgfs_import/main.cpp — Compatibility tool.
//
// Runs simengine::io::importJSBSimAircraft() against a real JSBSim
// aircraft file and prints the resulting simengine aircraft data as
// JSON (same shape as assets/aircraft/generic_narrowbody.json) plus a
// human-readable warnings list, so it's easy to see exactly what was
// extracted from the source file versus defaulted.

#include <cstdio>
#include "simengine/io/jsbsim_import.hpp"

using namespace simengine;

static void printJson(const io::JSBSimAircraftData& d) {
    std::printf("{\n");
    std::printf("  \"typeName\": \"%s\",\n", d.name.c_str());
    std::printf("  \"mass\": {\n");
    std::printf("    \"mass_kg\": %.1f,\n", d.mass.mass);
    std::printf("    \"S_m2\": %.3f, \"b_m\": %.3f, \"c_m\": %.3f, \"AR\": %.3f,\n",
                 d.mass.S, d.mass.b, d.mass.c, d.mass.AR);
    std::printf("    \"Ix\": %.1f, \"Iy\": %.1f, \"Iz\": %.1f, \"Ixz\": %.1f\n",
                 d.mass.Ix, d.mass.Iy, d.mass.Iz, d.mass.Ixz);
    std::printf("  },\n");
    std::printf("  \"aero\": {\n");
    std::printf("    \"CD0\": %.4f, \"CLa\": %.3f, \"CLde\": %.3f,\n", d.aero.CD0, d.aero.CLa, d.aero.CLde);
    std::printf("    \"Cma\": %.3f, \"Cmde\": %.3f, \"Cmq\": %.3f, \"Cm0\": %.3f,\n",
                 d.aero.Cma, d.aero.Cmde, d.aero.Cmq, d.aero.Cm0);
    std::printf("    \"Clb\": %.4f, \"Clp\": %.3f, \"Clda\": %.3f,\n", d.aero.Clb, d.aero.Clp, d.aero.Clda);
    std::printf("    \"Cnb\": %.4f, \"Cnr\": %.3f, \"Cndr\": %.4f, \"CYb\": %.3f\n",
                 d.aero.Cnb, d.aero.Cnr, d.aero.Cndr, d.aero.CYb);
    std::printf("  },\n");
    std::printf("  \"propulsion\": { \"T_max_per_engine_N\": %.1f, \"engineCount\": %d },\n",
                 d.propulsion.T_max, d.engineCount);
    std::printf("  \"gear\": [\n");
    for (std::size_t i = 0; i < d.gearLegs.size(); ++i) {
        const auto& g = d.gearLegs[i];
        std::printf("    { \"name\": \"%s\", \"attachBody_m\": [%.3f, %.3f, %.3f], \"strutLength_m\": %.3f, "
                    "\"travel_m\": %.3f, \"springConstant_Npm\": %.1f, \"damperConstant_Npmps\": %.1f, "
                    "\"steerable\": %s, \"hasBrake\": %s }%s\n",
                    g.name.c_str(), g.config.attachBody.x, g.config.attachBody.y, g.config.attachBody.z,
                    g.config.strutLength, g.config.travel, g.config.springConstant, g.config.damperConstant,
                    g.config.steerable ? "true" : "false", g.config.hasBrake ? "true" : "false",
                    (i + 1 < d.gearLegs.size()) ? "," : "");
    }
    std::printf("  ]\n");
    std::printf("}\n");
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fgfs_import <fdm.xml> [engine.xml]\n");
        std::fprintf(stderr, "example: fgfs_import assets/fgfs_source/A320-211.xml assets/fgfs_source/cfm56_5a1.xml\n");
        return 1;
    }
    const std::string fdmPath = argv[1];
    const std::string enginePath = argc >= 3 ? argv[2] : "";

    io::JSBSimAircraftData data = io::importJSBSimAircraft(fdmPath, enginePath);

    printJson(data);

    std::fprintf(stderr, "\n--- import warnings (%zu) ---\n", data.warnings.size());
    for (auto& w : data.warnings) std::fprintf(stderr, "  - %s\n", w.c_str());
    if (data.warnings.empty()) std::fprintf(stderr, "  (none — every field was found in the source file)\n");

    return 0;
}
