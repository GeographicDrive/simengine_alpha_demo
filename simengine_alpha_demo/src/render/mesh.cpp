// simengine/render/mesh.cpp — see mesh.hpp for design notes.

#include "simengine/render/mesh.hpp"

#include <array>
#include <cmath>

namespace simengine::render::meshgen {

namespace {

void pushQuad(MeshPart& part, Vector3f a, Vector3f b, Vector3f c, Vector3f d, Vector3f n) {
    const uint32_t base = static_cast<uint32_t>(part.vertices.size());
    for (auto& p : {a, b, c, d}) {
        part.vertices.push_back(Vertex{p.x, p.y, p.z, n.x, n.y, n.z, 0.0f, 0.0f});
    }
    part.indices.insert(part.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
}

} // namespace

void appendBox(MeshPart& part, Vector3f c, Vector3f h) {
    // 8 corners
    const Vector3f p000{c.x - h.x, c.y - h.y, c.z - h.z};
    const Vector3f p100{c.x + h.x, c.y - h.y, c.z - h.z};
    const Vector3f p110{c.x + h.x, c.y + h.y, c.z - h.z};
    const Vector3f p010{c.x - h.x, c.y + h.y, c.z - h.z};
    const Vector3f p001{c.x - h.x, c.y - h.y, c.z + h.z};
    const Vector3f p101{c.x + h.x, c.y - h.y, c.z + h.z};
    const Vector3f p111{c.x + h.x, c.y + h.y, c.z + h.z};
    const Vector3f p011{c.x - h.x, c.y + h.y, c.z + h.z};

    pushQuad(part, p001, p101, p111, p011, Vector3f{0, 0, 1});   // +Z (down, body axes)
    pushQuad(part, p100, p000, p010, p110, Vector3f{0, 0, -1});  // -Z (up)
    pushQuad(part, p101, p100, p110, p111, Vector3f{1, 0, 0});   // +X (nose-ward)
    pushQuad(part, p000, p001, p011, p010, Vector3f{-1, 0, 0});  // -X (tail-ward)
    pushQuad(part, p110, p010, p011, p111, Vector3f{0, 1, 0});   // +Y (right)
    pushQuad(part, p000, p100, p101, p001, Vector3f{0, -1, 0});  // -Y (left)
}

void appendWedge(MeshPart& part, Vector3f center, Vector3f hBase, Vector3f hTip, float taperAxis) {
    // Two rectangular cross-sections (base and tip) offset along
    // taperAxis, connected by side quads — a frustum. taperAxis in
    // {0,1,2} selects which world axis the taper runs along; the two
    // half-extent vectors give the cross-section size at each end (the
    // component along taperAxis in both is used as the +/- offset from
    // `center`).
    auto axisOffset = [&](Vector3f h) -> Vector3f {
        Vector3f o{0, 0, 0};
        if (taperAxis == 0) o.x = h.x; else if (taperAxis == 1) o.y = h.y; else o.z = h.z;
        return o;
    };
    auto crossSection = [&](Vector3f h) -> Vector3f {
        Vector3f cs = h;
        if (taperAxis == 0) cs.x = 0; else if (taperAxis == 1) cs.y = 0; else cs.z = 0;
        return cs;
    };

    const Vector3f baseCenter = Vector3f{center.x - axisOffset(hBase).x, center.y - axisOffset(hBase).y, center.z - axisOffset(hBase).z};
    const Vector3f tipCenter  = Vector3f{center.x + axisOffset(hTip).x,  center.y + axisOffset(hTip).y,  center.z + axisOffset(hTip).z};
    const Vector3f csB = crossSection(hBase);
    const Vector3f csT = crossSection(hTip);

    // Build 4 corners at base and tip (in the plane perpendicular to
    // taperAxis), using the two remaining axes as +/- extents.
    auto corners = [&](Vector3f cCenter, Vector3f cs) {
        std::array<Vector3f, 4> pts;
        if (taperAxis == 0) {
            pts = {Vector3f{cCenter.x, cCenter.y - cs.y, cCenter.z - cs.z},
                   Vector3f{cCenter.x, cCenter.y + cs.y, cCenter.z - cs.z},
                   Vector3f{cCenter.x, cCenter.y + cs.y, cCenter.z + cs.z},
                   Vector3f{cCenter.x, cCenter.y - cs.y, cCenter.z + cs.z}};
        } else if (taperAxis == 1) {
            pts = {Vector3f{cCenter.x - cs.x, cCenter.y, cCenter.z - cs.z},
                   Vector3f{cCenter.x + cs.x, cCenter.y, cCenter.z - cs.z},
                   Vector3f{cCenter.x + cs.x, cCenter.y, cCenter.z + cs.z},
                   Vector3f{cCenter.x - cs.x, cCenter.y, cCenter.z + cs.z}};
        } else {
            pts = {Vector3f{cCenter.x - cs.x, cCenter.y - cs.y, cCenter.z},
                   Vector3f{cCenter.x + cs.x, cCenter.y - cs.y, cCenter.z},
                   Vector3f{cCenter.x + cs.x, cCenter.y + cs.y, cCenter.z},
                   Vector3f{cCenter.x - cs.x, cCenter.y + cs.y, cCenter.z}};
        }
        return pts;
    };

    const auto b = corners(baseCenter, csB);
    const auto t = corners(tipCenter, csT);

    // 4 side quads connecting base ring to tip ring, plus a tip cap
    // (base cap intentionally omitted — wedges are used butted against
    // another part, e.g. fuselage nose against the main fuselage box).
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        const Vector3f edge1 = Vector3f{t[i].x - b[i].x, t[i].y - b[i].y, t[i].z - b[i].z};
        const Vector3f edge2 = Vector3f{b[j].x - b[i].x, b[j].y - b[i].y, b[j].z - b[i].z};
        Vector3f n = edge2.cross(edge1).normalized();
        pushQuad(part, b[i], b[j], t[j], t[i], n);
    }
    Vector3f tipNormal{0, 0, 0};
    if (taperAxis == 0) tipNormal.x = 1; else if (taperAxis == 1) tipNormal.y = 1; else tipNormal.z = 1;
    pushQuad(part, t[0], t[1], t[2], t[3], tipNormal);
}

namespace {

MeshPart makePart(const std::string& name, Vector3f attachBody, Vector3f pivot = {0, 0, 0}) {
    MeshPart p;
    p.name = name;
    p.attachBody = attachBody;
    p.pivot = pivot;
    return p;
}

} // namespace

StaticMesh buildPlaceholderAircraft(float L) {
    StaticMesh mesh;
    // Rough single-aisle proportions, scaled off overall length L.
    const float fuseLen = L * 0.82f;
    const float fuseRadius = L * 0.033f;
    const float wingSpan = L * 0.86f;
    const float wingChord = L * 0.14f;
    const float wingThick = L * 0.012f;
    const float tailSpan = L * 0.30f;
    const float stabChord = L * 0.09f;
    const float finHeight = L * 0.18f;

    // --- Fuselage: central box + nose/tail wedges. All in body axes
    // (X forward/nose, Y right, Z down), attached at the aircraft
    // origin (CG-relative body frame), matching RigidBody6DOFState. ---
    {
        MeshPart p = makePart("fuselage", {0, 0, 0});
        appendBox(p, Vector3f{0, 0, 0}, Vector3f{fuseLen * 0.35f, fuseRadius, fuseRadius});
        appendWedge(p, Vector3f{fuseLen * 0.35f, 0, 0}, Vector3f{0, fuseRadius, fuseRadius},
                    Vector3f{fuseLen * 0.15f, fuseRadius * 0.15f, fuseRadius * 0.15f}, 0);
        appendWedge(p, Vector3f{-fuseLen * 0.35f, 0, 0}, Vector3f{0, fuseRadius, fuseRadius},
                    Vector3f{fuseLen * 0.15f, fuseRadius * 0.35f, fuseRadius * 0.35f}, 0);
        mesh.parts.push_back(std::move(p));
    }

    // --- Wings (fixed, non-animated main box) + separate hinged
    // surfaces (aileron/flap/slat/spoiler) as their own parts so
    // AircraftRig can rotate/translate them independently. ---
    for (float side : {-1.0f, 1.0f}) {
        const std::string tag = side < 0 ? "L" : "R";
        const float rootY = side * fuseRadius * 0.9f;
        const float tipY = side * wingSpan * 0.5f;

        MeshPart wing = makePart("wing_" + tag, {0, rootY, 0});
        appendWedge(wing, Vector3f{0, (tipY - rootY) * 0.5f, 0},
                    Vector3f{wingChord * 0.5f, 0, wingThick * 0.5f},
                    Vector3f{wingChord * 0.28f, 0, wingThick * 0.3f}, 1);
        mesh.parts.push_back(std::move(wing));

        // Aileron: outboard trailing edge, hinges about its leading
        // edge (pivot at local -X edge of the small aileron box).
        {
            const float ay = tipY - side * wingSpan * 0.12f;
            MeshPart p = makePart("aileron_" + tag, {-wingChord * 0.4f, ay, 0},
                                   Vector3f{wingChord * 0.12f, 0, 0});
            appendBox(p, Vector3f{0, 0, 0}, Vector3f{wingChord * 0.12f, wingSpan * 0.08f, wingThick * 0.4f});
            mesh.parts.push_back(std::move(p));
        }
        // Flap: inboard trailing edge, hinges about leading edge too.
        {
            const float fy = rootY + side * wingSpan * 0.18f;
            MeshPart p = makePart("flap_" + tag, {-wingChord * 0.42f, fy, 0},
                                   Vector3f{wingChord * 0.14f, 0, 0});
            appendBox(p, Vector3f{0, 0, 0}, Vector3f{wingChord * 0.14f, wingSpan * 0.14f, wingThick * 0.4f});
            mesh.parts.push_back(std::move(p));
        }
        // Slat: inboard leading edge, hinges about its trailing (aft)
        // edge, translates slightly forward/down when deployed.
        {
            const float sy = rootY + side * wingSpan * 0.16f;
            MeshPart p = makePart("slat_" + tag, {wingChord * 0.46f, sy, 0},
                                   Vector3f{-wingChord * 0.06f, 0, 0});
            appendBox(p, Vector3f{0, 0, 0}, Vector3f{wingChord * 0.06f, wingSpan * 0.16f, wingThick * 0.25f});
            mesh.parts.push_back(std::move(p));
        }
        // Spoiler panel: upper wing surface, hinges up about its
        // forward edge.
        {
            const float spy = rootY + side * wingSpan * 0.2f;
            MeshPart p = makePart("spoiler_" + tag, {wingChord * 0.05f, spy, -wingThick * 0.5f},
                                   Vector3f{-wingChord * 0.08f, 0, 0});
            appendBox(p, Vector3f{0, 0, 0}, Vector3f{wingChord * 0.08f, wingSpan * 0.12f, wingThick * 0.1f});
            mesh.parts.push_back(std::move(p));
        }
    }

    // --- Speedbrake: modeled as a small dorsal panel aft of the wing
    // root, distinct from the wing spoilers (matches the physics
    // model's independent surface.speedbrake channel). ---
    {
        MeshPart p = makePart("speedbrake", {-fuseLen * 0.1f, 0, -fuseRadius},
                               Vector3f{-fuseLen * 0.03f, 0, 0});
        appendBox(p, Vector3f{0, 0, 0}, Vector3f{fuseLen * 0.03f, fuseRadius * 0.6f, fuseRadius * 0.08f});
        mesh.parts.push_back(std::move(p));
    }

    // --- Empennage: horizontal stabilizer + elevators, vertical fin +
    // rudder. ---
    {
        MeshPart p = makePart("stabilizer", {-fuseLen * 0.42f, 0, 0});
        appendWedge(p, Vector3f{0, tailSpan * 0.25f, 0}, Vector3f{stabChord * 0.5f, 0, wingThick * 0.4f},
                    Vector3f{stabChord * 0.25f, 0, wingThick * 0.25f}, 1);
        appendWedge(p, Vector3f{0, -tailSpan * 0.25f, 0}, Vector3f{stabChord * 0.5f, 0, wingThick * 0.4f},
                    Vector3f{stabChord * 0.25f, 0, wingThick * 0.25f}, 1);
        mesh.parts.push_back(std::move(p));
    }
    for (float side : {-1.0f, 1.0f}) {
        const std::string tag = side < 0 ? "L" : "R";
        MeshPart p = makePart("elevator_" + tag, {-fuseLen * 0.45f, side * tailSpan * 0.25f, 0},
                               Vector3f{stabChord * 0.1f, 0, 0});
        appendBox(p, Vector3f{0, 0, 0}, Vector3f{stabChord * 0.15f, tailSpan * 0.22f, wingThick * 0.3f});
        mesh.parts.push_back(std::move(p));
    }
    {
        MeshPart fin = makePart("fin", {-fuseLen * 0.42f, 0, 0});
        appendWedge(fin, Vector3f{0, 0, -finHeight * 0.5f}, Vector3f{stabChord * 0.55f, wingThick * 0.4f, 0},
                    Vector3f{stabChord * 0.22f, wingThick * 0.25f, 0}, 2);
        mesh.parts.push_back(std::move(fin));

        MeshPart rud = makePart("rudder", {-fuseLen * 0.45f, 0, -finHeight * 0.4f},
                                 Vector3f{0, 0, stabChord * 0.1f});
        appendBox(rud, Vector3f{0, 0, 0}, Vector3f{wingThick * 0.3f, stabChord * 0.16f, finHeight * 0.35f});
        mesh.parts.push_back(std::move(rud));
    }

    // --- Engines: underwing nacelles + spinning fan disc (drawn as a
    // thin box that visibly spins about the engine's own X axis). ---
    for (float side : {-1.0f, 1.0f}) {
        const std::string tag = side < 0 ? "L" : "R";
        const float ey = side * wingSpan * 0.28f;
        MeshPart nacelle = makePart("engine_nacelle_" + tag, {fuseLen * 0.05f, ey, fuseRadius * 0.9f});
        appendBox(nacelle, Vector3f{0, 0, 0}, Vector3f{fuseLen * 0.09f, fuseRadius * 0.55f, fuseRadius * 0.55f});
        mesh.parts.push_back(std::move(nacelle));

        MeshPart fan = makePart("engine_fan_" + tag, {fuseLen * 0.13f, ey, fuseRadius * 0.9f});
        appendBox(fan, Vector3f{0, 0, 0}, Vector3f{fuseRadius * 0.06f, fuseRadius * 0.45f, fuseRadius * 0.45f});
        mesh.parts.push_back(std::move(fan));

        MeshPart rev = makePart("reverser_" + tag, {fuseLen * 0.02f, ey, fuseRadius * 0.9f},
                                 Vector3f{-fuseLen * 0.04f, 0, 0});
        appendBox(rev, Vector3f{0, 0, 0}, Vector3f{fuseLen * 0.04f, fuseRadius * 0.5f, fuseRadius * 0.5f});
        mesh.parts.push_back(std::move(rev));
    }

    // --- Landing gear: nose + two main legs, each with a strut part
    // (translates on extend/retract) and a wheel part (spins + steers
    // for the nose leg). Named to match WheelComponent::animationNodeName
    // conventions used by AnimationSystem ("nose", "L_main", "R_main"). ---
    struct GearDef { std::string tag; Vector3f attach; };
    const std::vector<GearDef> gearDefs = {
        {"nose", Vector3f{fuseLen * 0.32f, 0, fuseRadius * 0.85f}},
        {"L_main", Vector3f{-fuseLen * 0.02f, -fuseRadius * 1.6f, fuseRadius * 0.85f}},
        {"R_main", Vector3f{-fuseLen * 0.02f, fuseRadius * 1.6f, fuseRadius * 0.85f}},
    };
    for (auto& g : gearDefs) {
        MeshPart strut = makePart("gear_strut_" + g.tag, g.attach);
        appendBox(strut, Vector3f{0, 0, fuseRadius * 0.45f}, Vector3f{fuseRadius * 0.08f, fuseRadius * 0.08f, fuseRadius * 0.45f});
        mesh.parts.push_back(std::move(strut));

        MeshPart wheel = makePart("gear_wheel_" + g.tag, Vector3f{g.attach.x, g.attach.y, g.attach.z + fuseRadius * 0.9f});
        appendBox(wheel, Vector3f{0, 0, 0}, Vector3f{fuseRadius * 0.22f, fuseRadius * 0.12f, fuseRadius * 0.22f});
        mesh.parts.push_back(std::move(wheel));
    }

    return mesh;
}

StaticMesh buildBaseplate(float half, float step) {
    StaticMesh mesh;
    MeshPart ground = makePart("ground", {0, 0, 0});
    // Single large flat quad (the grey baseplate itself), Z=0 in body/
    // world NED terms means... this part is placed directly in world
    // space by the caller (not attached to the aircraft), so "attachBody"
    // above is unused for this part; z is up in render-local convention
    // handled by the caller's world transform.
    appendBox(ground, Vector3f{0, 0, 0.02f}, Vector3f{half, half, 0.02f});

    MeshPart grid = makePart("grid", {0, 0, 0});
    for (float x = -half; x <= half + 1e-3f; x += step) {
        pushQuad(grid,
                 Vector3f{x - 0.05f, -half, 0.05f}, Vector3f{x + 0.05f, -half, 0.05f},
                 Vector3f{x + 0.05f, half, 0.05f}, Vector3f{x - 0.05f, half, 0.05f},
                 Vector3f{0, 0, 1});
    }
    for (float y = -half; y <= half + 1e-3f; y += step) {
        pushQuad(grid,
                 Vector3f{-half, y - 0.05f, 0.05f}, Vector3f{half, y - 0.05f, 0.05f},
                 Vector3f{half, y + 0.05f, 0.05f}, Vector3f{-half, y + 0.05f, 0.05f},
                 Vector3f{0, 0, 1});
    }
    mesh.parts.push_back(std::move(ground));
    mesh.parts.push_back(std::move(grid));
    return mesh;
}

} // namespace simengine::render::meshgen
