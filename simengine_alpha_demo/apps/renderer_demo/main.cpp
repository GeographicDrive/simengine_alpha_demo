// apps/renderer_demo/main.cpp — Alpha Technical Demo, windowed/rendered.
//
// This is the real interactive Alpha: a GLFW window presenting a GLES3
// context, the full ECS pipeline (Input -> FlightDynamics -> LandingGear
// -> Engine -> Animation) running every fixed tick, a third-person
// camera following the aircraft, and GLRenderer drawing the placeholder
// aircraft (every animated part rigged to the real physics state) over
// the grey baseplate.
//
// GLFW/keyboard here is a DESKTOP DEVELOPMENT STAND-IN for the touch UI
// (mobile_ui/index.html) — deliberately funneled through the exact same
// systems::InputSnapshot the touch layer will populate, so wiring the
// real touch events later only means replacing readKeyboardInput()'s
// body, not anything downstream of it. See docs/ROADMAP.md "Android
// port" section for what's still needed beyond that swap (NativeActivity
// entry point, EGL context creation, asset packaging into the APK).
//
// Window/input here is GLFW-specific and is the ONE part of this file
// that will not compile as-is on Android; everything else (GLRenderer,
// the rig, the camera, every ECS system) is platform-neutral and reused
// unchanged by the eventual Android entry point.

#include <cstdio>
#include <vector>

#include <GLFW/glfw3.h>

#include "simengine/core/ecs.hpp"
#include "simengine/core/job_system.hpp"
#include "simengine/aircraft/aircraft_factory.hpp"
#include "simengine/aircraft/a320_variants.hpp"
#include "simengine/io/jsbsim_import.hpp"
#include "simengine/systems/flight_dynamics_system.hpp"
#include "simengine/systems/landing_gear_system.hpp"
#include "simengine/systems/engine_system.hpp"
#include "simengine/systems/animation_system.hpp"
#include "simengine/systems/input_system.hpp"
#include "simengine/camera/third_person_camera.hpp"
#include "simengine/render/gl_renderer.hpp"
#include "simengine/render/aircraft_rig.hpp"
#include "simengine/render/mesh_a320.hpp"

using namespace simengine;

namespace {

// Reads the current GLFW keyboard state into an InputSnapshot using the
// exact same field set the touch UI will populate. Edge-triggered
// toggles (gear/flaps/speedbrake/spoiler/reverse/parking brake) are
// computed here from a simple was-down/is-down comparison, matching
// what a touch button's onPress handler would send.
struct KeyEdgeState {
    bool gear = false, flapsUp = false, flapsDown = false, speedbrake = false,
         spoiler = false, reverse = false, parkingBrake = false, pause = false;
};

systems::InputSnapshot readKeyboardInput(GLFWwindow* win, KeyEdgeState& prev) {
    systems::InputSnapshot in;
    auto down = [&](int key) { return glfwGetKey(win, key) == GLFW_PRESS; };

    in.stickPitch = (down(GLFW_KEY_S) ? 1.0 : 0.0) - (down(GLFW_KEY_W) ? 1.0 : 0.0);
    in.stickRoll = (down(GLFW_KEY_D) ? 1.0 : 0.0) - (down(GLFW_KEY_A) ? 1.0 : 0.0);
    in.rudderPedal = (down(GLFW_KEY_E) ? 1.0 : 0.0) - (down(GLFW_KEY_Q) ? 1.0 : 0.0);
    in.throttle = down(GLFW_KEY_LEFT_SHIFT) ? 1.0 : (down(GLFW_KEY_LEFT_CONTROL) ? 0.0 : -1.0);
    in.tiller = in.rudderPedal;
    in.brake = down(GLFW_KEY_B) ? 1.0 : 0.0;
    in.trimInput = (down(GLFW_KEY_UP) ? 1.0 : 0.0) - (down(GLFW_KEY_DOWN) ? 1.0 : 0.0);

    const bool gearNow = down(GLFW_KEY_G);
    in.toggleGear = gearNow && !prev.gear; prev.gear = gearNow;
    const bool fUpNow = down(GLFW_KEY_LEFT_BRACKET);
    in.toggleFlapsUp = fUpNow && !prev.flapsUp; prev.flapsUp = fUpNow;
    const bool fDownNow = down(GLFW_KEY_RIGHT_BRACKET);
    in.toggleFlapsDown = fDownNow && !prev.flapsDown; prev.flapsDown = fDownNow;
    const bool sbNow = down(GLFW_KEY_X);
    in.toggleSpeedbrake = sbNow && !prev.speedbrake; prev.speedbrake = sbNow;
    const bool spNow = down(GLFW_KEY_C);
    in.toggleSpoiler = spNow && !prev.spoiler; prev.spoiler = spNow;
    const bool revNow = down(GLFW_KEY_R);
    in.toggleReverse = revNow && !prev.reverse; prev.reverse = revNow;
    const bool pbNow = down(GLFW_KEY_P);
    in.toggleParkingBrake = pbNow && !prev.parkingBrake; prev.parkingBrake = pbNow;

    return in;
}

} // namespace

int main(int argc, char** argv) {
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }

    // Explicitly request a GLES 3.0 context (not desktop GL) — this is
    // the one line that keeps this whole rendering stack Android-honest:
    // every GL call in gl_renderer.cpp/aircraft_rig.cpp only ever uses
    // API surface that also exists in real GLES3 on-device.
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "simengine Alpha Technical Demo", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "glfwCreateWindow failed (no GLES3-capable context available)\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    render::GLRenderer renderer;
    if (!renderer.init()) {
        std::fprintf(stderr, "GLRenderer::init failed\n");
        return 1;
    }

    // If run as `renderer_demo --a320 <path-to-A320-211.xml>
    // <path-to-cfm56_5a1.xml> <path-to-converted_models-dir>`, spawn the
    // real A320 (mass/aero/gear from the JSBSim FDM data, same as
    // alpha_demo --a320) and draw it with the real converted airframe
    // mesh instead of the procedural placeholder box-and-wedge model.
    // Falls back to the generic narrowbody + placeholder mesh otherwise
    // (unchanged default behavior).
    render::StaticMesh aircraftMesh;
    core::World world;
    core::JobSystem jobs(4);
    core::Entity plane;
    aircraft::SpawnParams spawn;
    spawn.positionNED = {0.0, 0.0, -2.0};

    if (argc >= 5 && std::string(argv[1]) == "--a320") {
        io::JSBSimAircraftData data = io::importJSBSimAircraft(argv[2], argv[3]);
        plane = aircraft::spawnFromJSBSim(world, data, spawn);
        aircraftMesh = render::meshgen::buildA320Aircraft(argv[4]);
        std::printf("Loaded real A320 FDM + real converted airframe mesh from %s\n", argv[4]);
    } else if (argc >= 3 && std::string(argv[1]) == "--variant") {
        // `renderer_demo --variant <A318-111|...|A321-231> [converted_models-dir] [variantsDir]`
        // — same registry as alpha_demo --variant (see a320_variants.hpp);
        // draws the real converted mesh (A320-211 geometry — see that
        // header's note on why every variant currently shares it) with
        // the picked variant's real mass/CG/engine.
        const std::string wanted = argv[2];
        const std::string meshDir = argc >= 4 ? argv[3] : "../assets/converted_models";
        const std::string variantsDir = argc >= 5 ? argv[4] : "../assets/fgfs_source/variants";
        const aircraft::A320Variant* found = nullptr;
        for (auto& v : aircraft::kA320Variants) {
            if (v.id == wanted) { found = &v; break; }
        }
        if (!found) {
            std::fprintf(stderr, "Unknown --variant '%s'. Known: ", wanted.c_str());
            for (auto& v : aircraft::kA320Variants) std::fprintf(stderr, "%s ", std::string(v.id).c_str());
            std::fprintf(stderr, "\n");
            return 1;
        }
        io::JSBSimAircraftData data = io::importJSBSimAircraft(
            variantsDir + "/" + std::string(found->fdmFile), variantsDir + "/" + std::string(found->engineFile));
        plane = aircraft::spawnFromJSBSim(world, data, spawn);
        aircraftMesh = render::meshgen::buildA320Aircraft(meshDir);
        std::printf("Loaded variant '%s' (%s), real converted airframe mesh from %s\n",
                     std::string(found->displayName).c_str(), data.name.c_str(), meshDir.c_str());
    } else {
        plane = aircraft::spawnGenericNarrowbody(world, spawn);
        aircraftMesh = render::meshgen::buildPlaceholderAircraft(38.0f);
    }
    render::StaticMesh groundMesh = render::meshgen::buildBaseplate();
    render::GpuMesh aircraftGpu = renderer.upload(aircraftMesh);
    render::GpuMesh groundGpu = renderer.upload(groundMesh);

    systems::InputSystem inputSystem;
    systems::FlightDynamicsSystem flightDynamics;
    systems::LandingGearSystem landingGear;
    systems::EngineSystem engineSystem;
    systems::AnimationSystem animationSystem;
    camera::ThirdPersonCamera chaseCam;
    KeyEdgeState keyEdges;

    const math::Vector3<double> renderOrigin{0.0, 0.0, 0.0}; // rebase point; fine for an Alpha-scale scene
    constexpr double dt = 1.0 / 60.0;
    double accumulator = 0.0;
    double lastTime = glfwGetTime();

    std::printf("== simengine Alpha Technical Demo (windowed) ==\n");
    std::printf("Run with `--a320 <A320-211.xml> <cfm56_5a1.xml> <converted_models-dir>` for the\n");
    std::printf("real A320 FDM + real converted airframe mesh; no args = generic placeholder.\n");
    std::printf("Controls (desktop stand-in for touch UI): W/S pitch, A/D roll, Q/E rudder,\n");
    std::printf("Shift=throttle up, Ctrl=idle, B=brake, G=gear, [ / ] flaps, X speedbrake,\n");
    std::printf("C spoiler, R reverse, P parking brake, Up/Down=trim.\n\n");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, GLFW_TRUE);

        const double now = glfwGetTime();
        double frameTime = now - lastTime;
        lastTime = now;
        if (frameTime > 0.25) frameTime = 0.25; // clamp a debugger-pause/hitch spike
        accumulator += frameTime;

        systems::InputSnapshot input = readKeyboardInput(window, keyEdges);

        while (accumulator >= dt) {
            inputSystem.apply(world, plane, input, dt);
            flightDynamics.update(world, jobs, dt);
            landingGear.update(world, jobs, dt);
            engineSystem.update(world, jobs, dt);
            animationSystem.update(world, jobs, dt);
            accumulator -= dt;
        }

        auto* body = world.getComponent<aircraft::RigidBody6DOFComponent>(plane);
        auto* anim = world.getComponent<aircraft::AnimationComponent>(plane);
        chaseCam.update(body->state.positionNED, body->state.attitude, frameTime);

        int fbW = 0, fbH = 0;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        renderer.resize(fbW, fbH);

        const math::Matrix4f aircraftWorld = render::nedStateToRenderWorld(
            body->state.positionNED, body->state.attitude, renderOrigin);

        const math::Vector3<double> camRel = chaseCam.position() - renderOrigin;
        const math::Vector3<double> lookRel = chaseCam.lookAtTarget() - renderOrigin;
        const math::Matrix4f view = math::Matrix4f::lookAt(
            math::Vector3f{static_cast<float>(camRel.x), static_cast<float>(camRel.y), static_cast<float>(-camRel.z)},
            math::Vector3f{static_cast<float>(lookRel.x), static_cast<float>(lookRel.y), static_cast<float>(-lookRel.z)},
            math::Vector3f{0, 0, 1});
        const float aspect = fbH > 0 ? static_cast<float>(fbW) / static_cast<float>(fbH) : 1.0f;
        const math::Matrix4f proj = render::perspectiveGL(
            static_cast<float>(chaseCam.fovDegrees() * 3.14159265358979 / 180.0), aspect, 0.5f, 50000.0f);

        std::vector<render::DrawItem> items;
        if (anim) {
            render::appendAircraftDrawItems(items, aircraftGpu, *anim, aircraftWorld);
        }
        // Baseplate: static at render-world origin, no rotation.
        if (auto it = groundGpu.parts.find("ground"); it != groundGpu.parts.end()) {
            render::DrawItem g; g.part = &it->second; g.model = math::Matrix4f::identity();
            g.r = 0.42f; g.g = 0.42f; g.b = 0.44f; items.push_back(g);
        }
        if (auto it = groundGpu.parts.find("grid"); it != groundGpu.parts.end()) {
            render::DrawItem g; g.part = &it->second; g.model = math::Matrix4f::identity();
            g.r = 0.30f; g.g = 0.30f; g.b = 0.32f; items.push_back(g);
        }

        renderer.beginFrame(0.55f, 0.68f, 0.80f);
        render::DirectionalLight light;
        renderer.draw(items, view, proj, light);
        renderer.endFrame();

        glfwSwapBuffers(window);
    }

    renderer.destroy(aircraftGpu);
    renderer.destroy(groundGpu);
    renderer.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
