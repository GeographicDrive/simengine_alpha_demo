// android/app/src/main/cpp/android_main.cpp
//
// The Android entry point for the Alpha Technical Demo. This is the ONE
// file in the whole engine written specifically for Android — everything
// it calls into (simengine_flight's ECS/physics systems, simengine_render's
// GLRenderer/aircraft_rig/mesh generators) is the exact same code already
// exercised by the desktop apps/renderer_demo, unchanged. Compare the two
// side by side: this file's job is only (a) EGL context/surface setup
// against the ANativeWindow NativeActivity hands us, (b) turning touch
// events into the same systems::InputSnapshot apps/renderer_demo builds
// from keyboard state, and (c) the fixed-timestep loop — the same
// Input -> FlightDynamics -> LandingGear -> Engine -> Animation -> Camera
// -> Render sequence.
//
// Touch mapping in this Alpha is intentionally minimal (left-half drag =
// virtual stick for pitch/roll, right-half vertical drag = throttle) —
// it is NOT the final touch UI. Wiring mobile_ui/index.html's actual
// button layout (flaps/gear/speedbrake/etc.) is the next step noted in
// docs/ROADMAP.md; this file's onInputEvent() is where that wiring lands,
// same InputSnapshot either way.

#include <android/log.h>
#include <android/native_window.h>
#include <android/asset_manager.h>
#include <android_native_app_glue.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <string>
#include <sys/stat.h>
#include <vector>

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

#define LOG_TAG "simengine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

using namespace simengine;

namespace {

// --- Everything the app needs, held in one place and reached via
// android_app::userData. Deliberately POD-ish/flat rather than a class
// hierarchy: NativeActivity's callback style (free functions receiving
// android_app*) reads more clearly against a flat struct than against
// member-function-pointer indirection. ---
struct EngineState {
    // EGL
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLSurface surface = EGL_NO_SURFACE;
    EGLContext context = EGL_NO_CONTEXT;
    EGLConfig config = nullptr;
    int width = 0, height = 0;
    bool hasSurface = false;

    // Engine
    render::GLRenderer renderer;
    bool rendererInitialized = false;
    render::GpuMesh aircraftGpu;
    render::GpuMesh groundGpu;
    core::World world;
    core::JobSystem jobs{4};
    core::Entity plane{};
    systems::InputSystem inputSystem;
    systems::FlightDynamicsSystem flightDynamics;
    systems::LandingGearSystem landingGear;
    systems::EngineSystem engineSystem;
    systems::AnimationSystem animationSystem;
    camera::ThirdPersonCamera chaseCam;

    // Touch state: one drag on each half of the screen. Pointer IDs
    // (not indices) are tracked so a second finger landing on the same
    // half mid-gesture doesn't hijack the active one.
    struct DragState {
        bool active = false;
        int pointerId = -1;
        float startX = 0, startY = 0, curX = 0, curY = 0;
    };
    DragState stickDrag;    // left half: roll/pitch virtual stick
    DragState throttleDrag; // right half: vertical throttle slider
    float throttleValue = 0.0f; // persists between drags (a slider, not a spring-back stick)

    // A320 variant picker (tap the top-left ~150x150px corner to cycle —
    // see onInputEvent; there's no text rendering yet so there's no
    // on-screen label, only a LOGI() of the new variant name — see
    // docs/ROADMAP.md "known rough edges" for the follow-up: a real
    // touch UI, per mobile_ui/index.html, would replace this).
    std::string extractedAssetRoot; // internalDataPath once assets are copied out
    std::size_t variantIndex = aircraft::kDefaultA320VariantIndex;
    bool assetsExtracted = false;

    bool running = false;
    bool loggedFirstFrame = false;
    double accumulator = 0.0;
    struct timespec lastTime {};
};

constexpr double kFixedDt = 1.0 / 60.0;

double nowSeconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Creates the EGLDisplay/EGLConfig/EGLContext exactly once and leaves
// them alive for the lifetime of the process. Splitting this out from
// surface creation (below) is the fix for a real bug seen on-device: a
// window can be destroyed and recreated (screen rotation, a system
// overlay changing insets, the app being backgrounded/foregrounded)
// without the process dying, and NativeActivity delivers that as
// APP_CMD_TERM_WINDOW followed by APP_CMD_INIT_WINDOW. If a *new* EGL
// context were created on every INIT_WINDOW (as an earlier version of
// this file did), every GPU resource tied to the old context — the
// compiled shader program and every uploaded mesh's VAO/VBO/EBO, all
// created once behind `rendererInitialized` — would silently become
// invalid, and every subsequent draw call would be a no-op against a
// blank framebuffer. Keeping one context alive for the process and only
// ever recreating the *surface* against it avoids that entirely.
bool ensureContext(EngineState& s) {
    if (s.context != EGL_NO_CONTEXT) return true; // already have one; nothing to do

    s.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (s.display == EGL_NO_DISPLAY) { LOGE("eglGetDisplay failed"); return false; }
    if (!eglInitialize(s.display, nullptr, nullptr)) { LOGE("eglInitialize failed"); return false; }
    eglBindAPI(EGL_OPENGL_ES_API);

    const EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_DEPTH_SIZE, 24,
        EGL_NONE
    };
    EGLint numConfigs = 0;
    if (!eglChooseConfig(s.display, configAttribs, &s.config, 1, &numConfigs) || numConfigs < 1) {
        LOGE("eglChooseConfig failed");
        return false;
    }

    const EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    s.context = eglCreateContext(s.display, s.config, EGL_NO_CONTEXT, contextAttribs);
    if (s.context == EGL_NO_CONTEXT) { LOGE("eglCreateContext failed"); return false; }

    LOGI("EGL context created (once for process lifetime)");
    return true;
}

// Creates (or recreates) just the EGLSurface against the current window
// and the process-lifetime context from ensureContext(). Safe to call
// every time APP_CMD_INIT_WINDOW fires.
bool createSurface(EngineState& s, ANativeWindow* window) {
    if (!ensureContext(s)) return false;

    EGLint format = 0;
    eglGetConfigAttrib(s.display, s.config, EGL_NATIVE_VISUAL_ID, &format);
    ANativeWindow_setBuffersGeometry(window, 0, 0, format);

    s.surface = eglCreateWindowSurface(s.display, s.config, window, nullptr);
    if (s.surface == EGL_NO_SURFACE) { LOGE("eglCreateWindowSurface failed"); return false; }

    if (!eglMakeCurrent(s.display, s.surface, s.surface, s.context)) {
        LOGE("eglMakeCurrent failed (0x%x)", eglGetError());
        return false;
    }

    eglQuerySurface(s.display, s.surface, EGL_WIDTH, &s.width);
    eglQuerySurface(s.display, s.surface, EGL_HEIGHT, &s.height);
    s.hasSurface = true;
    LOGI("EGL surface (re)created: %dx%d, GL_RENDERER=%s", s.width, s.height,
         reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
    return true;
}

// Tears down only the EGLSurface (window gone), NOT the context/display —
// those persist so createSurface() can rebind to a new window later
// without losing any GPU-resident state (shader program, mesh buffers).
void destroySurfaceEGL(EngineState& s) {
    if (s.display != EGL_NO_DISPLAY) {
        eglMakeCurrent(s.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (s.surface != EGL_NO_SURFACE) eglDestroySurface(s.display, s.surface);
    }
    s.surface = EGL_NO_SURFACE;
    s.hasSurface = false;
}

void shutdownEGL(EngineState& s) {
    if (s.display != EGL_NO_DISPLAY) {
        eglMakeCurrent(s.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (s.context != EGL_NO_CONTEXT) eglDestroyContext(s.display, s.context);
        if (s.surface != EGL_NO_SURFACE) eglDestroySurface(s.display, s.surface);
        eglTerminate(s.display);
    }
    s.display = EGL_NO_DISPLAY;
    s.context = EGL_NO_CONTEXT;
    s.surface = EGL_NO_SURFACE;
    s.hasSurface = false;
}

// --- Asset extraction --------------------------------------------------
// APK-bundled assets (see android/app/build.gradle.kts's
// sourceSets.main.assets.srcDirs pointing at the repo's assets/ folder)
// aren't directly openable via std::ifstream — io/jsbsim_import.cpp,
// io/xml.cpp, and render/obj_loader.cpp all take real filesystem paths,
// unchanged from the desktop build (see mesh_a320.hpp's header comment
// on why: keeping those files identical to the already-verified desktop
// code was judged more valuable than adding an AAssetManager-backed
// read path to each of them). So on first launch this copies the exact
// files needed out of the APK into the app's private internal storage
// (ANativeActivity::internalDataPath) via AAssetManager, then every
// loader downstream just gets handed an ordinary path under there,
// identical in shape to what apps/alpha_demo and apps/renderer_demo
// already pass on desktop.
//
// The file list is hardcoded rather than using AAssetManager_openDir
// recursively — that API's handling of nested asset subdirectories is
// inconsistent across AAPT2/NDK versions, and this project's asset set
// is small and fixed, so an explicit list is more robust than getting
// directory recursion right blind (this couldn't be tested on-device
// in the environment this was written in — see docs/ROADMAP.md).
const char* const kAssetFilesToExtract[] = {
    "fgfs_source/A320-211-set.xml",
    "fgfs_source/A320-211.xml",
    "fgfs_source/A320-main.xml",
    "fgfs_source/COPYING",
    "fgfs_source/cfm56_5a1.xml",
    "fgfs_source/variants/A318-111.xml",
    "fgfs_source/variants/A319-111.xml",
    "fgfs_source/variants/A319-131.xml",
    "fgfs_source/variants/A320-111.xml",
    "fgfs_source/variants/A320-211.xml",
    "fgfs_source/variants/A320-231.xml",
    "fgfs_source/variants/A321-211.xml",
    "fgfs_source/variants/A321-231.xml",
    "fgfs_source/variants/cfm56_5a1.xml",
    "fgfs_source/variants/cfm56_5b3.xml",
    "fgfs_source/variants/cfm56_5b5.xml",
    "fgfs_source/variants/cfm56_5b8.xml",
    "fgfs_source/variants/v2522_a5.xml",
    "fgfs_source/variants/v2533_a5.xml",
    "converted_models/cfm56.obj",
    "converted_models/fuselage.obj",
    "converted_models/hstab.obj",
    "converted_models/mlg_left.obj",
    "converted_models/mlg_right.obj",
    "converted_models/mlg_tires.obj",
    "converted_models/nacelle_cfm.obj",
    "converted_models/nlg.obj",
    "converted_models/nlg_tires.obj",
    "converted_models/pylon_cfm_left.obj",
    "converted_models/pylon_cfm_right.obj",
    "converted_models/vstab.obj",
    "converted_models/winglets.obj",
    "converted_models/wings.obj",
};

// mkdir -p for a single relative path's parent directories, under base.
void mkdirsFor(const std::string& base, const std::string& relPath) {
    std::string cur = base;
    mkdir(cur.c_str(), 0755);
    size_t pos = 0;
    while (true) {
        size_t slash = relPath.find('/', pos);
        if (slash == std::string::npos) break;
        cur += "/" + relPath.substr(pos, slash - pos);
        mkdir(cur.c_str(), 0755);
        pos = slash + 1;
    }
}

bool extractOneAsset(AAssetManager* mgr, const std::string& base, const char* relPath) {
    AAsset* asset = AAssetManager_open(mgr, relPath, AASSET_MODE_BUFFER);
    if (!asset) {
        LOGE("extractOneAsset: could not open bundled asset '%s'", relPath);
        return false;
    }
    const off_t size = AAsset_getLength(asset);
    const void* data = AAsset_getBuffer(asset);
    mkdirsFor(base, relPath);
    const std::string outPath = base + "/" + relPath;
    FILE* out = std::fopen(outPath.c_str(), "wb");
    bool ok = false;
    if (out) {
        ok = (size == 0) || (std::fwrite(data, 1, static_cast<size_t>(size), out) == static_cast<size_t>(size));
        std::fclose(out);
    } else {
        LOGE("extractOneAsset: could not open output file '%s'", outPath.c_str());
    }
    AAsset_close(asset);
    return ok;
}

// Extracts every file in kAssetFilesToExtract, once. Returns the base
// directory they were extracted under (== app->activity->internalDataPath),
// or an empty string if extraction failed outright (caller falls back to
// the generic placeholder aircraft rather than crashing).
std::string extractAssetsIfNeeded(android_app* app, bool& alreadyDone) {
    const std::string base = app->activity->internalDataPath;
    if (alreadyDone) return base;
    AAssetManager* mgr = app->activity->assetManager;
    int failures = 0;
    for (const char* rel : kAssetFilesToExtract) {
        if (!extractOneAsset(mgr, base, rel)) failures++;
    }
    if (failures > 0) {
        LOGE("extractAssetsIfNeeded: %d of %zu files failed to extract", failures,
             sizeof(kAssetFilesToExtract) / sizeof(kAssetFilesToExtract[0]));
    } else {
        LOGI("extractAssetsIfNeeded: all %zu files extracted to %s",
             sizeof(kAssetFilesToExtract) / sizeof(kAssetFilesToExtract[0]), base.c_str());
    }
    alreadyDone = (failures == 0);
    return base;
}

// Loads the given variant's real JSBSim FDM data and spawns it,
// replacing whatever aircraft entity currently exists in s.world. Falls
// back to leaving the current aircraft in place (just logs an error) if
// the import throws — e.g. asset extraction failed above.
void loadVariant(EngineState& s, std::size_t variantIndex) {
    const auto& v = aircraft::kA320Variants[variantIndex];
    const std::string dir = s.extractedAssetRoot + "/fgfs_source/variants";
    try {
        io::JSBSimAircraftData data = io::importJSBSimAircraft(
            dir + "/" + std::string(v.fdmFile), dir + "/" + std::string(v.engineFile));
        aircraft::SpawnParams spawn;
        spawn.positionNED = {0.0, 0.0, -2.0};
        // NOTE: the previous s.plane entity (if any) is intentionally
        // left in s.world rather than destroyed — core::World has no
        // entity-removal API yet (see docs/ROADMAP.md). Harmless for an
        // Alpha (nothing iterates "all entities", only the specific
        // s.plane handle the systems below are given each tick), but a
        // real picker UI that gets used heavily would want that added.
        s.plane = aircraft::spawnFromJSBSim(s.world, data, spawn);
        s.variantIndex = variantIndex;
        LOGI("loadVariant: switched to %s (%s), %zu gear legs, %zu import warnings",
             std::string(v.displayName).c_str(), data.name.c_str(), data.gearLegs.size(), data.warnings.size());
    } catch (const std::exception& e) {
        LOGE("loadVariant: failed to load %s: %s", std::string(v.displayName).c_str(), e.what());
    }
}

void ensureEngineInitialized(EngineState& s, android_app* app) {
    if (s.rendererInitialized) return;
    if (!s.renderer.init()) { LOGE("GLRenderer::init failed"); return; }

    s.extractedAssetRoot = extractAssetsIfNeeded(app, s.assetsExtracted);

    render::StaticMesh groundMesh = render::meshgen::buildBaseplate();
    s.groundGpu = s.renderer.upload(groundMesh);

    if (s.assetsExtracted) {
        try {
            render::StaticMesh aircraftMesh =
                render::meshgen::buildA320Aircraft(s.extractedAssetRoot + "/converted_models");
            s.aircraftGpu = s.renderer.upload(aircraftMesh);
            loadVariant(s, aircraft::kDefaultA320VariantIndex);
            LOGI("engine initialized (real A320 mesh + FDM loaded)");
        } catch (const std::exception& e) {
            LOGE("buildA320Aircraft/loadVariant failed (%s) — falling back to placeholder aircraft", e.what());
            s.assetsExtracted = false;
        }
    }
    if (!s.assetsExtracted) {
        // Extraction or mesh/FDM loading failed — fall back to the
        // original placeholder aircraft rather than leaving the app in
        // a half-initialized state.
        render::StaticMesh aircraftMesh = render::meshgen::buildPlaceholderAircraft(38.0f);
        s.aircraftGpu = s.renderer.upload(aircraftMesh);
        aircraft::SpawnParams spawn;
        spawn.positionNED = {0.0, 0.0, -2.0};
        s.plane = aircraft::spawnGenericNarrowbody(s.world, spawn);
        LOGI("engine initialized (placeholder aircraft — asset extraction or A320 load failed)");
    }

    s.rendererInitialized = true;
}

// Builds one frame's InputSnapshot from the current drag states. Called
// every fixed tick (not every input event) so held drags keep producing
// continuous control input between touch events, same as a real stick.
systems::InputSnapshot buildInputSnapshot(EngineState& s) {
    systems::InputSnapshot in;
    if (s.stickDrag.active) {
        const float dx = (s.stickDrag.curX - s.stickDrag.startX) / 300.0f; // px -> normalized, generic touch-stick radius
        const float dy = (s.stickDrag.curY - s.stickDrag.startY) / 300.0f;
        in.stickRoll = std::clamp(static_cast<double>(dx), -1.0, 1.0);
        in.stickPitch = std::clamp(static_cast<double>(dy), -1.0, 1.0); // drag down = nose up, matches a real yoke/stick
    }
    if (s.throttleDrag.active) {
        // Vertical position within the touch's screen half maps directly
        // to throttle (top of screen = full throttle), updated live while
        // held — a slider, not a delta-drag, matching a physical throttle
        // lever's absolute-position feel.
        const float norm = 1.0f - std::clamp(s.throttleDrag.curY / static_cast<float>(s.height), 0.0f, 1.0f);
        s.throttleValue = norm;
    }
    in.throttle = s.throttleValue * 2.0 - 1.0; // InputSystem expects [-1,1] (see readKeyboardInput's convention)
    return in;
}

int32_t onInputEvent(android_app* app, AInputEvent* event) {
    auto* s = static_cast<EngineState*>(app->userData);
    if (AInputEvent_getType(event) != AINPUT_EVENT_TYPE_MOTION) return 0;

    const int32_t action = AMotionEvent_getAction(event);
    const int32_t actionMasked = action & AMOTION_EVENT_ACTION_MASK;
    const int32_t actionIndex = (action & AMOTION_EVENT_ACTION_POINTER_INDEX_MASK)
        >> AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;

    auto assignHalf = [&](int32_t index) {
        const float x = AMotionEvent_getX(event, index);
        const float y = AMotionEvent_getY(event, index);
        const int32_t id = AMotionEvent_getPointerId(event, index);
        EngineState::DragState& d = (x < s->width * 0.5f) ? s->stickDrag : s->throttleDrag;
        d.active = true; d.pointerId = id; d.startX = x; d.startY = y; d.curX = x; d.curY = y;
    };

    // Aircraft variant picker: tap the top-left 150x150px corner to
    // cycle through aircraft::kA320Variants (see loadVariant()). This is
    // a placeholder for a real touch UI — there's no on-screen label for
    // it yet (see the EngineState::variantIndex comment) — but it is a
    // genuinely working way to pick a different real A320-family
    // aircraft on-device, not just via `alpha_demo --variant` on
    // desktop.
    constexpr float kPickerZonePx = 150.0f;
    auto inPickerZone = [&](int32_t index) {
        const float x = AMotionEvent_getX(event, index);
        const float y = AMotionEvent_getY(event, index);
        return x < kPickerZonePx && y < kPickerZonePx;
    };

    if (actionMasked == AMOTION_EVENT_ACTION_DOWN && inPickerZone(0) && s->assetsExtracted) {
        const std::size_t next = (s->variantIndex + 1) % aircraft::kA320Variants.size();
        loadVariant(*s, next);
        return 1;
    }

    switch (actionMasked) {
        case AMOTION_EVENT_ACTION_DOWN:
            assignHalf(0);
            return 1;
        case AMOTION_EVENT_ACTION_POINTER_DOWN:
            assignHalf(actionIndex);
            return 1;
        case AMOTION_EVENT_ACTION_MOVE: {
            const size_t count = AMotionEvent_getPointerCount(event);
            for (size_t i = 0; i < count; ++i) {
                const int32_t id = AMotionEvent_getPointerId(event, i);
                const float x = AMotionEvent_getX(event, i);
                const float y = AMotionEvent_getY(event, i);
                if (s->stickDrag.active && s->stickDrag.pointerId == id) { s->stickDrag.curX = x; s->stickDrag.curY = y; }
                if (s->throttleDrag.active && s->throttleDrag.pointerId == id) { s->throttleDrag.curX = x; s->throttleDrag.curY = y; }
            }
            return 1;
        }
        case AMOTION_EVENT_ACTION_UP:
            s->stickDrag.active = false;
            s->throttleDrag.active = false;
            return 1;
        case AMOTION_EVENT_ACTION_POINTER_UP: {
            const int32_t id = AMotionEvent_getPointerId(event, actionIndex);
            if (s->stickDrag.pointerId == id) s->stickDrag.active = false;
            if (s->throttleDrag.pointerId == id) s->throttleDrag.active = false;
            return 1;
        }
        default:
            return 0;
    }
}

void onAppCmd(android_app* app, int32_t cmd) {
    auto* s = static_cast<EngineState*>(app->userData);
    LOGI("onAppCmd: %d (running=%d hasSurface=%d)", cmd, s->running, s->hasSurface);
    switch (cmd) {
        case APP_CMD_INIT_WINDOW:
            if (app->window != nullptr) {
                if (createSurface(*s, app->window)) {
                    s->renderer.resize(s->width, s->height);
                    ensureEngineInitialized(*s, app);
                    s->running = true;
                    clock_gettime(CLOCK_MONOTONIC, &s->lastTime);
                    LOGI("running = true (window ready)");
                }
            }
            break;
        case APP_CMD_TERM_WINDOW:
            // The only event that should actually stop rendering: the
            // window itself is gone (about to be destroyed), so there's
            // nothing left to draw into.
            s->running = false;
            destroySurfaceEGL(*s);
            LOGI("running = false (window destroyed)");
            break;
        case APP_CMD_WINDOW_RESIZED:
        case APP_CMD_CONTENT_RECT_CHANGED:
            if (s->hasSurface) {
                eglQuerySurface(s->display, s->surface, EGL_WIDTH, &s->width);
                eglQuerySurface(s->display, s->surface, EGL_HEIGHT, &s->height);
                s->renderer.resize(s->width, s->height);
            }
            break;
        // Deliberately NOT touching s->running here. A prior version of
        // this file paused rendering on APP_CMD_LOST_FOCUS / resumed on
        // APP_CMD_GAINED_FOCUS — but focus is not the same thing as
        // window validity: a transient system overlay (a permission
        // prompt, an accessibility bubble, a screen-recording/assistant
        // indicator, etc.) can take focus without the window ever being
        // torn down, and if that overlay stays around, GAINED_FOCUS
        // never fires again — the app is then stuck paused forever on
        // whatever the last (possibly never-yet-rendered) frame was.
        // This is believed to be the actual cause of an on-device blank/
        // fully-transparent-on-screenshot symptom seen during Alpha
        // testing. Rendering now only ever stops for a genuinely gone
        // window (APP_CMD_TERM_WINDOW above); focus changes are logged
        // here for visibility but don't gate the render loop.
        case APP_CMD_GAINED_FOCUS:
        case APP_CMD_LOST_FOCUS:
        case APP_CMD_RESUME:
        case APP_CMD_PAUSE:
            break;
        default:
            break;
    }
}

void renderFrame(EngineState& s) {
    if (!s.hasSurface || !s.rendererInitialized) return;

    auto* body = s.world.getComponent<aircraft::RigidBody6DOFComponent>(s.plane);
    auto* anim = s.world.getComponent<aircraft::AnimationComponent>(s.plane);
    if (!body || !anim) return;

    const math::Vector3<double> renderOrigin{0.0, 0.0, 0.0};
    const math::Matrix4f aircraftWorld = render::nedStateToRenderWorld(
        body->state.positionNED, body->state.attitude, renderOrigin);

    const math::Vector3<double> camRel = s.chaseCam.position() - renderOrigin;
    const math::Vector3<double> lookRel = s.chaseCam.lookAtTarget() - renderOrigin;
    const math::Matrix4f view = math::Matrix4f::lookAt(
        {static_cast<float>(camRel.x), static_cast<float>(camRel.y), static_cast<float>(-camRel.z)},
        {static_cast<float>(lookRel.x), static_cast<float>(lookRel.y), static_cast<float>(-lookRel.z)},
        {0, 0, 1});
    const float aspect = s.height > 0 ? static_cast<float>(s.width) / static_cast<float>(s.height) : 1.0f;
    const math::Matrix4f proj = render::perspectiveGL(
        static_cast<float>(s.chaseCam.fovDegrees() * M_PI / 180.0), aspect, 0.5f, 50000.0f);

    std::vector<render::DrawItem> items;
    render::appendAircraftDrawItems(items, s.aircraftGpu, *anim, aircraftWorld);
    if (auto it = s.groundGpu.parts.find("ground"); it != s.groundGpu.parts.end()) {
        render::DrawItem g; g.part = &it->second; g.model = math::Matrix4f::identity();
        g.r = 0.42f; g.g = 0.42f; g.b = 0.44f; items.push_back(g);
    }
    if (auto it = s.groundGpu.parts.find("grid"); it != s.groundGpu.parts.end()) {
        render::DrawItem g; g.part = &it->second; g.model = math::Matrix4f::identity();
        g.r = 0.30f; g.g = 0.30f; g.b = 0.32f; items.push_back(g);
    }

    s.renderer.beginFrame(0.55f, 0.68f, 0.80f);
    render::DirectionalLight light;
    s.renderer.draw(items, view, proj, light);
    s.renderer.endFrame();

    if (!eglSwapBuffers(s.display, s.surface)) {
        LOGE("eglSwapBuffers failed (0x%x)", eglGetError());
    } else if (!s.loggedFirstFrame) {
        s.loggedFirstFrame = true;
        LOGI("first frame presented");
    }
}

} // namespace

void android_main(android_app* app) {
    EngineState state;
    app->userData = &state;
    app->onAppCmd = onAppCmd;
    app->onInputEvent = onInputEvent;

    LOGI("simengine Alpha Technical Demo starting");

    while (true) {
        int events = 0;
        android_poll_source* source = nullptr;
        // IMPORTANT: state.running ? 0 : -1 must be re-evaluated on every
        // single call, inline, right here — NOT hoisted into a local
        // computed once per outer-loop iteration. A prior version of
        // this file did exactly that (`const int timeoutMs = ...;` above
        // this loop), and it's a real, confirmed-on-device bug: if
        // running flips from false to true partway through processing a
        // burst of already-queued startup commands (START, RESUME,
        // INIT_WINDOW, WINDOW_RESIZED, ... all land back-to-back), the
        // loop keeps blocking with the STALE -1 timeout captured before
        // running became true, and only wakes up again on the next
        // genuinely new event (which can be seconds later, e.g. a focus
        // change) — during which the render path below is never reached
        // even once. Inlining the ternary here means every iteration of
        // this inner loop re-reads state.running fresh, so as soon as
        // it's true, the very next poll call correctly stops blocking.
        while (ALooper_pollAll(state.running ? 0 : -1, nullptr, &events, reinterpret_cast<void**>(&source)) >= 0) {
            if (source != nullptr) source->process(app, source);
            if (app->destroyRequested != 0) {
                shutdownEGL(state);
                LOGI("simengine Alpha Technical Demo exiting");
                return;
            }
        }

        if (!state.running) continue;

        const double now = nowSeconds();
        const double lastTime = static_cast<double>(state.lastTime.tv_sec) + static_cast<double>(state.lastTime.tv_nsec) * 1e-9;
        double frameTime = now - lastTime;
        clock_gettime(CLOCK_MONOTONIC, &state.lastTime);
        if (frameTime > 0.25) frameTime = 0.25; // clamp a resume-from-background hitch
        state.accumulator += frameTime;

        systems::InputSnapshot input = buildInputSnapshot(state);
        while (state.accumulator >= kFixedDt) {
            state.inputSystem.apply(state.world, state.plane, input, kFixedDt);
            state.flightDynamics.update(state.world, state.jobs, kFixedDt);
            state.landingGear.update(state.world, state.jobs, kFixedDt);
            state.engineSystem.update(state.world, state.jobs, kFixedDt);
            state.animationSystem.update(state.world, state.jobs, kFixedDt);
            state.accumulator -= kFixedDt;
        }

        if (auto* body = state.world.getComponent<aircraft::RigidBody6DOFComponent>(state.plane)) {
            state.chaseCam.update(body->state.positionNED, body->state.attitude, frameTime);
        }

        renderFrame(state);
    }
}
