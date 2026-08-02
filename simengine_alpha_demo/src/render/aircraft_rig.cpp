// simengine/render/aircraft_rig.cpp — see aircraft_rig.hpp for design notes.

#include "simengine/render/aircraft_rig.hpp"

#include <cmath>

namespace simengine::render {

using simengine::aircraft::AnimationComponent;
using simengine::math::Matrix4;
using simengine::math::Quaternion;
using simengine::math::Vector3;

namespace {

double channel(const AnimationComponent& anim, const std::string& name) {
    for (auto& c : anim.channels) if (c.name == name) return c.value;
    return 0.0;
}

// Builds: worldTransform(attach) * translate(pivot) * rotateAxis(angle) * translate(-pivot)
// i.e. rotate the part about its authored hinge point, in the part's own
// local space, then place it at its body-frame attach point under the
// aircraft's own world transform.
Matrix4f hingeTransform(const Matrix4f& aircraftWorld, Vector3f attachBody,
                         Vector3f pivot, float angleRad, int axis /*0=x,1=y,2=z*/,
                         Vector3f extraTranslateLocal = {0, 0, 0}) {
    const float c = std::cos(angleRad), s = std::sin(angleRad);
    Matrix4f rot = Matrix4f::identity();
    if (axis == 0) { // roll about local X — ailerons/spoilers hinge line along span
        rot.m[1][1] = c; rot.m[1][2] = s; rot.m[2][1] = -s; rot.m[2][2] = c;
    } else if (axis == 1) { // about local Y — elevators/flaps/slats hinge line along span(Y)
        rot.m[0][0] = c; rot.m[0][2] = -s; rot.m[2][0] = s; rot.m[2][2] = c;
    } else { // about local Z — rudder/NWS
        rot.m[0][0] = c; rot.m[0][1] = s; rot.m[1][0] = -s; rot.m[1][1] = c;
    }

    const Matrix4f toAttach = Matrix4f::translation(Vector3f{attachBody.x, attachBody.y, attachBody.z});
    const Matrix4f toPivot = Matrix4f::translation(pivot);
    const Matrix4f fromPivot = Matrix4f::translation(Vector3f{-pivot.x, -pivot.y, -pivot.z});
    const Matrix4f extra = Matrix4f::translation(extraTranslateLocal);

    return aircraftWorld * toAttach * extra * toPivot * rot * fromPivot;
}

// `pivot` used to be a caller-supplied argument, and every call site
// passed a literal {0,0,0} — silently discarding the mesh part's
// AUTHORED pivot (GpuMesh::Part::pivot, set by meshgen::makePart and
// carried through GLRenderer::upload, see gl_renderer.hpp) and instead
// rotating every hinged surface about its own mesh-space origin
// (usually its geometric center) rather than its real hinge line. For a
// thin trailing-edge box (e.g. the elevator) that made a "pitch"
// rotation show up mostly as the whole box sinking/rising in Z with
// barely any visible tilt — exactly the "instead of pitching, it moves
// in Z" symptom — and any part whose authored pivot sits off to one
// side (engine reverser cowl, gear pieces, ailerons/flaps/slats/
// spoilers/rudder) swept through a visibly circular/orbiting path each
// cycle instead of rotating cleanly in place. Fix: always read the
// part's own authored pivot instead of taking one from the caller.
void addPart(std::vector<DrawItem>& out, const GpuMesh& gpu, const std::string& name,
             const Matrix4f& aircraftWorld, float angleRad, int axis,
             float r, float g, float b, Vector3f extraTranslate = {0, 0, 0}) {
    auto it = gpu.parts.find(name);
    if (it == gpu.parts.end()) return;
    DrawItem item;
    item.part = &it->second;
    item.model = hingeTransform(aircraftWorld, it->second.attachBody, it->second.pivot, angleRad, axis, extraTranslate);
    item.r = r; item.g = g; item.b = b;
    out.push_back(item);
}

} // namespace

void appendAircraftDrawItems(
    std::vector<DrawItem>& out,
    const GpuMesh& gpu,
    const AnimationComponent& anim,
    const Matrix4f& world,
    const AircraftRigConfig& cfg) {

    constexpr float kFuselageGrey[3] = {0.72f, 0.74f, 0.78f};
    constexpr float kSurfaceGrey[3] = {0.55f, 0.57f, 0.62f};
    constexpr float kDarkGrey[3] = {0.25f, 0.25f, 0.28f};
    constexpr float kRedAccent[3] = {0.75f, 0.15f, 0.15f};

    // --- Generic, no-airline A320-style livery. ---
    // This is a flat-shaded, per-part palette (the current pipeline
    // colors a whole MeshPart with one solid RGB — DrawItem has no UV/
    // texture-sampler path yet, see render/gl_renderer.hpp) rather than
    // a real painted texture. It approximates a bare/house livery: white
    // fuselage, natural-metal wing/tailplane, a plain painted tail (no
    // airline logo/titles), dark anti-glare-ish engine nacelles. Wiring
    // an actual UV-mapped livery image would additionally need a
    // texture-coordinate channel on Vertex and a sampler in the GLES
    // shader — worth doing later, out of scope for this color pass.
    constexpr float kLiveryWhite[3] = {0.93f, 0.94f, 0.95f};       // fuselage
    constexpr float kLiveryMetal[3] = {0.62f, 0.64f, 0.67f};       // wings/stabilizer (bare metal look)
    constexpr float kLiveryTailAccent[3] = {0.10f, 0.22f, 0.55f};  // plain blue tail, no logo
    constexpr float kLiveryNacelle[3] = {0.30f, 0.31f, 0.34f};     // engine nacelle

    // Fixed structure (no animation channel — identity hinge).
    addPart(out, gpu, "fuselage", world, 0.0f, 2, kLiveryWhite[0], kLiveryWhite[1], kLiveryWhite[2]);
    for (const char* wingLike : {"wing_L", "wing_R", "stabilizer"}) {
        addPart(out, gpu, wingLike, world, 0.0f, 2, kLiveryMetal[0], kLiveryMetal[1], kLiveryMetal[2]);
    }
    addPart(out, gpu, "fin", world, 0.0f, 2, kLiveryTailAccent[0], kLiveryTailAccent[1], kLiveryTailAccent[2]);
    for (const char* nacelle : {"engine_nacelle_L", "engine_nacelle_R"}) {
        addPart(out, gpu, nacelle, world, 0.0f, 2, kLiveryNacelle[0], kLiveryNacelle[1], kLiveryNacelle[2]);
    }

    // --- Primary flight controls ---
    const float aileron = static_cast<float>(channel(anim, "surface.aileron_left"));
    addPart(out, gpu, "aileron_L", world, aileron * cfg.aileronMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);
    const float aileronR = static_cast<float>(channel(anim, "surface.aileron_right"));
    addPart(out, gpu, "aileron_R", world, aileronR * cfg.aileronMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);

    const float elevator = static_cast<float>(channel(anim, "surface.elevator"));
    addPart(out, gpu, "elevator_L", world, elevator * cfg.elevatorMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);
    addPart(out, gpu, "elevator_R", world, elevator * cfg.elevatorMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);

    const float rudder = static_cast<float>(channel(anim, "surface.rudder"));
    addPart(out, gpu, "rudder", world, rudder * cfg.rudderMaxRad, 2, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);

    // --- High-lift devices (independent channels, see animation_system.hpp) ---
    const float flap = static_cast<float>(channel(anim, "surface.flap"));
    addPart(out, gpu, "flap_L", world, flap * cfg.flapMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);
    addPart(out, gpu, "flap_R", world, flap * cfg.flapMaxRad, 1, kSurfaceGrey[0], kSurfaceGrey[1], kSurfaceGrey[2]);

    const float slat = static_cast<float>(channel(anim, "surface.slat"));
    // Slats droop forward-and-down: rotate about Y plus a small forward
    // translate, both driven by the same channel.
    addPart(out, gpu, "slat_L", world, -slat * cfg.slatMaxRad, 1, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2],
            Vector3f{-slat * 0.4f, 0, 0});
    addPart(out, gpu, "slat_R", world, -slat * cfg.slatMaxRad, 1, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2],
            Vector3f{-slat * 0.4f, 0, 0});

    const float spoiler = static_cast<float>(channel(anim, "surface.spoiler"));
    addPart(out, gpu, "spoiler_L", world, -spoiler * cfg.spoilerMaxRad, 1, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2]);
    addPart(out, gpu, "spoiler_R", world, -spoiler * cfg.spoilerMaxRad, 1, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2]);

    const float speedbrake = static_cast<float>(channel(anim, "surface.speedbrake"));
    addPart(out, gpu, "speedbrake", world, -speedbrake * cfg.speedbrakeMaxRad, 1, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2]);

    // --- Landing gear: strut retract (translate up into the fuselage)
    // and wheel spin/steer. ---
    const float gearPos = static_cast<float>(channel(anim, "gear.position")); // 1=down
    const float gearRetract = (1.0f - gearPos) * cfg.gearStowTranslate;
    for (const char* tag : {"nose", "L_main", "R_main"}) {
        const std::string strutName = std::string("gear_strut_") + tag;
        const std::string wheelName = std::string("gear_wheel_") + tag;
        addPart(out, gpu, strutName, world, 0.0f, 2, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2],
                Vector3f{0, 0, gearRetract});

        float steerRad = 0.0f;
        if (std::string(tag) == "nose") {
            steerRad = static_cast<float>(channel(anim, "gear.nws_command")) * cfg.nwsMaxRad;
        }
        addPart(out, gpu, wheelName, world, steerRad, 2, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2],
                Vector3f{0, 0, gearRetract});
    }

    // --- Engines: fan spin (visualized as a slight yaw wobble proxy
    // since the placeholder fan is a flat disc box — a real fan mesh
    // would spin about its own X axis continuously; the placeholder
    // communicates "spinning" via a fast small oscillation instead of an
    // unbounded rotation, which reads better with a flat box silhouette). ---
    for (const char* tag : {"L", "R"}) {
        const std::string fanChan = std::string("engine.") + (std::string(tag) == "L" ? "0" : "1") + ".fan_rotation";
        const float fanRot = static_cast<float>(channel(anim, fanChan));
        addPart(out, gpu, std::string("engine_fan_") + tag, world, fanRot, 0, kDarkGrey[0], kDarkGrey[1], kDarkGrey[2]);

        const std::string revChan = std::string("engine.") + (std::string(tag) == "L" ? "0" : "1") + ".reverser";
        const float rev = static_cast<float>(channel(anim, revChan));
        addPart(out, gpu, std::string("reverser_") + tag, world, 0.0f, 1, kRedAccent[0], kRedAccent[1], kRedAccent[2],
                Vector3f{-rev * 0.5f, 0, 0});
    }
}

Matrix4f nedStateToRenderWorld(const Vector3<double>& positionNED,
                                const Quaternion<double>& attitude,
                                const Vector3<double>& renderOriginNED) {
    // Position: render-world Z-up from NED Z-down (X/Y unchanged), all
    // relative to renderOriginNED so far-from-origin float precision
    // never enters this conversion (the subtraction happens in double).
    const Vector3<double> rel = positionNED - renderOriginNED;
    const Vector3f posRender{static_cast<float>(rel.x), static_cast<float>(rel.y), static_cast<float>(-rel.z)};

    // Orientation: body axes are X-fwd/Y-right/Z-down; render axes are
    // X-fwd/Y-right/Z-up. The body-to-NED rotation R already maps body
    // vectors into the NED world frame (X-fwd/Y-right/Z-down); to get
    // render-world vectors we just need to reinterpret that NED-world
    // vector in the Z-up frame, i.e. left-multiply by the reflection
    // T = diag(1, 1, -1): v_render = T * (R * v_body). That means only
    // the Z *row* of R needs to be negated — the mesh itself is
    // authored in the same Z-down body convention, so this single flip
    // is what turns it right-side up in the Z-up render frame.
    // (Negating both the Z row and the Z column, i.e. T * R * T, is a
    // different — and wrong — operation: for an identity attitude it
    // cancels back out to the identity matrix, leaving the Z-down mesh
    // rendered as-is and the aircraft upside down, gear pointing up.)
    Matrix4f rot = Matrix4f::fromQuaternion(Quaternion<float>{
        static_cast<float>(attitude.w), static_cast<float>(attitude.x),
        static_cast<float>(attitude.y), static_cast<float>(attitude.z)});
    for (int c = 0; c < 3; ++c) {
        rot.m[c][2] = -rot.m[c][2];
    }

    Matrix4f world = rot;
    world.m[3][0] = posRender.x;
    world.m[3][1] = posRender.y;
    world.m[3][2] = posRender.z;
    return world;
}

} // namespace simengine::render
