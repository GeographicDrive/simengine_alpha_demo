// simengine/render/mesh_a320.cpp — see mesh_a320.hpp. Assembles the
// real A320-family.zip airframe into AircraftRig's expected part table.
// All the offset numbers below come straight out of the aircraft's own
// FlightGear composition XML (Aircraft/A320-family/XMLs/A320.xml and
// the per-part XMLs it includes) — see assets/A320_MESH_NOTES.md for
// the full derivation and the JSBSim gear-position cross-check.

#include "simengine/render/mesh_a320.hpp"
#include "simengine/render/obj_loader.hpp"

#include <stdexcept>
#include <vector>

namespace simengine::render::meshgen {

using objio::loadObjGroups;

namespace {

// Every part offset below is (aft-X, right-Y, up-Z) in meters, taken
// directly from Aircraft/A320-family/XMLs/A320.xml's per-submodel
// <offsets>, chained through parent submodels where the geometry is
// nested (e.g. engines are offset from the wings submodel, which is
// itself offset from the fuselage/root). CG_X converts these into this
// engine's CG-relative, forward-positive body frame: derived as
// (nose-gear model offset 7.83m) + (JSBSim CG-to-nose-gear distance
// 11.2894m, from A320-211.xml's <location name="CG"> minus its
// NOSE_LG contact <location>) = 19.119m. That number is also a
// consistency check: converting the nose-gear model offset through it
// reproduces JSBSim's own nose-gear body-frame X (11.289m) exactly,
// and the main-gear Y offsets (+-3.795m) match JSBSim's gear Y exactly
// too, so the two independently-authored datasets (FDM XML vs model
// XML) agree on the geometry.
constexpr float kCGx = 19.119f;

struct Offset { float x, y, z; };

Offset engineFrame(float modelX, float modelY, float modelZ) {
    return Offset{kCGx - modelX, modelY, -modelZ};
}

const Offset kFuselage = engineFrame(0.f, 0.f, 0.f);
const Offset kWingRoot = engineFrame(15.280f, 0.f, -1.06019f);
const Offset kNoseGear = engineFrame(7.83f, 0.f, -1.8306008f);
const Offset kMlgLeft = engineFrame(20.3180764f, -3.795f, -3.9469424f);
const Offset kMlgRight = engineFrame(20.3180764f, 3.795f, -3.9469424f);
const Offset kHstab = engineFrame(34.1512207f, 0.f, 0.f);
const Offset kVstab = engineFrame(32.2535728f, 0.f, 0.f);
// Engine offsets are relative to the wings submodel origin in the
// source XML (Engine.CFM.Left offset -1.38/-5.755/-1.0170087 inside
// XMLs/Wings/a320.wings.xml), so chain wing-root's model-space offset
// (15.280, 0, -1.06019) with the engine's own before converting.
const Offset kEngineLeft = engineFrame(15.280f - 1.38f, -5.755f, -1.06019f - 1.0170087f);
const Offset kEngineRight = engineFrame(15.280f - 1.38f, 5.755f, -1.06019f - 1.0170087f);

void translatePart(MeshPart& part, Offset o) {
    for (auto& v : part.vertices) {
        v.px += o.x; v.py += o.y; v.pz += o.z;
    }
}

// Appends src's vertices/indices onto dst (index-adjusted), leaving
// src's own transform/name untouched (caller already applied it).
void appendInto(MeshPart& dst, const MeshPart& src) {
    const uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

// Computes the vertex centroid — used as an approximate hinge pivot for
// animated surfaces where the real hinge line isn't in any of our
// extracted data. Visually reasonable (surfaces are thin chordwise), not
// aerodynamically exact.
Vector3f centroid(const MeshPart& part) {
    if (part.vertices.empty()) return {0, 0, 0};
    double sx = 0, sy = 0, sz = 0;
    for (auto& v : part.vertices) { sx += v.px; sy += v.py; sz += v.pz; }
    const double n = static_cast<double>(part.vertices.size());
    return Vector3f{static_cast<float>(sx / n), static_cast<float>(sy / n), static_cast<float>(sz / n)};
}

// Pulls `groupNames` out of `groups`, applies `offset`, merges them
// into one MeshPart named `outName` with the given attach/pivot
// (attachBody left at {0,0,0} since offset is already baked into the
// vertices — see mesh_a320.hpp). Silently skips group names that
// aren't present (some AC3D files number sub-surfaces differently
// across parts; missing ones just mean slightly less detail, not a
// missing part).
MeshPart mergeGroups(std::unordered_map<std::string, MeshPart>& groups,
                      const std::vector<std::string>& groupNames,
                      const std::string& outName, Offset offset, bool setPivotToCentroid) {
    MeshPart out;
    out.name = outName;
    for (auto& g : groupNames) {
        auto it = groups.find(g);
        if (it == groups.end()) continue;
        translatePart(it->second, offset);
        appendInto(out, it->second);
    }
    if (setPivotToCentroid) out.pivot = centroid(out);
    return out;
}

} // namespace

StaticMesh buildA320Aircraft(const std::string& assetDir) {
    StaticMesh mesh;
    auto path = [&](const char* f) { return assetDir + "/" + f; };

    // --- Fuselage: one big part, all groups merged (no internal
    // animation channels reference sub-parts of it). ---
    {
        auto fuselageGroups = loadObjGroups(path("fuselage.obj"));
        MeshPart fuse;
        fuse.name = "fuselage";
        for (auto& [name, part] : fuselageGroups) {
            translatePart(part, kFuselage);
            appendInto(fuse, part);
        }
        mesh.parts.push_back(std::move(fuse));
    }

    // --- Wings: fixed structure (wingbox/fairings/winglets) plus the
    // named animated control surfaces the AC3D file already splits out
    // (a320.wings.ac was authored with AileronL/R, FlapL1-2/R1-2,
    // SlatL1-5/R1-5, SpoilerL1-5/R1-5 as separate AC3D OBJECTs — see
    // A320_MESH_NOTES.md — so no per-triangle guessing is needed). ---
    {
        auto wg = loadObjGroups(path("wings.obj"), true);
        auto wl = loadObjGroups(path("winglets.obj"), true);
        // Fold winglets into the same group map under distinguishing keys.
        for (auto& [name, part] : wl) wg.emplace(name, std::move(part));

        mesh.parts.push_back(mergeGroups(wg,
            {"Wingbox", "Wings", "FairingPylons", "FairingL1", "FairingL2", "FairingL3",
             "Flaps1", "WingletL"},
            "wing_L", kWingRoot, false));
        mesh.parts.push_back(mergeGroups(wg,
            {"FairingR1", "FairingR2", "FairingR3", "WingletR"},
            "wing_R", kWingRoot, false));
        // NOTE: "Wingbox"/"Wings"/"FairingPylons"/"Flaps1" are single
        // AC3D objects spanning the whole wing structure (both sides in
        // one mesh) in the source file, so they're only added to wing_L
        // above to avoid drawing them twice; wing_R gets its own
        // side-specific fairings/winglet plus whatever the wing_L merge
        // already covers geometrically. This means wing_R currently
        // renders less structure than wing_L — a known asymmetry, not a
        // physics issue (AircraftRig only uses these for drawing).
        mesh.parts.push_back(mergeGroups(wg, {"AileronL"}, "aileron_L", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg, {"AileronR"}, "aileron_R", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg, {"FlapL1", "FlapL2"}, "flap_L", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg, {"FlapR1", "FlapR2"}, "flap_R", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg,
            {"SlatL1", "SlatL2", "SlatL3", "SlatL4", "SlatL5"}, "slat_L", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg,
            {"SlatR1", "SlatR2", "SlatR3", "SlatR4", "SlatR5"}, "slat_R", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg,
            {"SpoilerL1", "SpoilerL2", "SpoilerL3", "SpoilerL4", "SpoilerL5"}, "spoiler_L", kWingRoot, true));
        mesh.parts.push_back(mergeGroups(wg,
            {"SpoilerR1", "SpoilerR2", "SpoilerR3", "SpoilerR4", "SpoilerR5"}, "spoiler_R", kWingRoot, true));
        // No dedicated "speedbrake" surface exists in the source model
        // (the real A320 uses the spoilers for both roles) — AircraftRig
        // skips this part silently since it isn't in the mesh.
    }

    // --- Tail: horizontal stabilizer + elevators, vertical fin + rudder. ---
    {
        auto hs = loadObjGroups(path("hstab.obj"), true);
        mesh.parts.push_back(mergeGroups(hs, {"Hstabs", "HstabFlapL", "HstabFlapR"}, "stabilizer", kHstab, false));
        mesh.parts.push_back(mergeGroups(hs, {"ElevatorL"}, "elevator_L", kHstab, true));
        mesh.parts.push_back(mergeGroups(hs, {"ElevatorR"}, "elevator_R", kHstab, true));

        auto vs = loadObjGroups(path("vstab.obj"), true);
        mesh.parts.push_back(mergeGroups(vs, {"Vstab"}, "fin", kVstab, false));
        mesh.parts.push_back(mergeGroups(vs, {"Rudder_0", "Rudder_1"}, "rudder", kVstab, true));
    }

    // --- Landing gear: strut/linkage groups vs tire groups, per leg. ---
    {
        auto nlg = loadObjGroups(path("nlg.obj"));
        mesh.parts.push_back(mergeGroups(nlg,
            {"ACUTATOR", "AXLE", "D_STRUCT.L", "D_STRUCT.U", "FITTING", "LINK.01", "LINK.02", "LINK.FIT", "OLEO"},
            "gear_strut_nose", kNoseGear, false));
        auto nlgTires = loadObjGroups(path("nlg_tires.obj"));
        mesh.parts.push_back(mergeGroups(nlgTires, {"Bearing", "Tires"}, "gear_wheel_nose", kNoseGear, false));

        auto mlgL = loadObjGroups(path("mlg_left.obj"));
        mesh.parts.push_back(mergeGroups(mlgL,
            {"ACTUTATOR", "D_STRUCT.L", "D_STRUCT.U", "FITTING", "LINKS.01", "LINKS.02", "OLEO"},
            "gear_strut_L_main", kMlgLeft, false));
        auto mlgR = loadObjGroups(path("mlg_right.obj"));
        mesh.parts.push_back(mergeGroups(mlgR,
            {"ACTUTATOR", "D_STRUCT.L", "D_STRUCT.U", "FITTING", "LINKS.01", "LINKS.02", "OLEO"},
            "gear_strut_R_main", kMlgRight, false));

        // Same tire model, reused for both sides (only known distinct
        // per-side geometry was the strut mesh above); mirrored purely
        // by the attach offset's Y sign.
        auto mlgTires = loadObjGroups(path("mlg_tires.obj"));
        auto mlgTiresCopy = mlgTires; // second copy for the other side, since mergeGroups mutates in place
        mesh.parts.push_back(mergeGroups(mlgTires, {"Bearing", "Tires"}, "gear_wheel_L_main", kMlgLeft, false));
        mesh.parts.push_back(mergeGroups(mlgTiresCopy, {"Bearing", "Tires"}, "gear_wheel_R_main", kMlgRight, false));
    }

    // --- Engines: nacelle+pylon+core as one fixed "engine_nacelle_*"
    // part, reverser as its own animated part, and the CFM56's fan/
    // fanWheel groups as "engine_fan_*" (spun by AnimationSystem). ---
    {
        auto buildEngine = [&](const char* pylonFile, const char* tag, Offset offset) {
            auto nacelle = loadObjGroups(path("nacelle_cfm.obj"));
            auto pylon = loadObjGroups(path(pylonFile));
            auto core = loadObjGroups(path("cfm56.obj"));

            MeshPart fixed;
            fixed.name = std::string("engine_nacelle_") + tag;
            for (const char* g : {"Core", "Intake", "IntakeInterior", "Nacelle", "Nozzle"}) {
                auto it = nacelle.find(g);
                if (it == nacelle.end()) continue;
                translatePart(it->second, offset);
                appendInto(fixed, it->second);
            }
            for (const char* g : {"Pylon"}) {
                auto it = pylon.find(g);
                if (it == pylon.end()) continue;
                translatePart(it->second, offset);
                appendInto(fixed, it->second);
            }
            for (const char* g : {"Plane", "Cube.004", "exhaust", "shroud", "cone", "cone2", "casing"}) {
                auto it = core.find(g);
                if (it == core.end()) continue;
                translatePart(it->second, offset);
                appendInto(fixed, it->second);
            }
            mesh.parts.push_back(std::move(fixed));

            mesh.parts.push_back(mergeGroups(nacelle, {"Reverser"}, std::string("reverser_") + tag, offset, false));
            mesh.parts.push_back(mergeGroups(core, {"blades", "fanWheel"}, std::string("engine_fan_") + tag, offset, true));
        };
        buildEngine("pylon_cfm_left.obj", "L", kEngineLeft);
        buildEngine("pylon_cfm_right.obj", "R", kEngineRight);
    }

    return mesh;
}

} // namespace simengine::render::meshgen
