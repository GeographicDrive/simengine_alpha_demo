# Alpha Technical Demo — What's Here, What Isn't, What's Next

This doc is the honest scope statement for this build. Read it before
assuming any given subsystem is more finished than it is.

## What actually runs and is tested

- `simengine_flight` static library: 6-DOF RK4 rigid body (your original
  `rigid_body_6dof.{hpp,cpp}`, unmodified), ISA atmosphere, a full
  per-wheel landing-gear/suspension/tire model (`physics/landing_gear.*`),
  and five ECS systems (`FlightDynamicsSystem`, `LandingGearSystem`,
  `EngineSystem`, `AnimationSystem`, `InputSystem`) that glue them into
  `core::World`/`core::JobSystem` from your existing `GeoEngine` core.
- `apps/alpha_demo`: a **headless console** loop that spawns one
  generic-narrowbody aircraft on the baseplate, runs Input → FlightDynamics
  → LandingGear → Engine → Animation → Camera every fixed tick for 20
  simulated seconds, and prints telemetry. It proves the pipeline
  integrates and runs to completion without crashing or diverging.
- `tests/test_flight_dynamics.cpp`: sanity tests — a dropped aircraft
  settles onto its gear and stops falling, full throttle produces
  forward acceleration and N1 spool-up, and control input reaches the
  animation channels. All pass (`ctest` or run the binary directly).
  One real bug (`FlightControls::gearDown` defaulting to `false`, so a
  freshly spawned aircraft had no ground contact at all) was caught by
  writing this test and is fixed in `aircraft_factory.cpp`.
- `mobile_ui/index.html`: a standalone, fully interactive touch-control
  mockup (throttle, virtual stick, rudder slider, flaps/gear/speedbrake/
  spoilers/trim/brakes/parking brake/reverse/pause). Every widget writes
  into an `InputSnapshot`-shaped object with field names matching
  `systems::InputSnapshot` exactly.

## FlightGear/JSBSim data compatibility (added on request — read this before assuming "A320 support")

The uploaded `A320-family.zip` turned out to be usable after all, just
not the way the original prompt assumed: its `.xml` files are **JSBSim**
format (the flight-dynamics-model JSBSim project, which FlightGear's
"A320-family" add-on happens to use — distinct from FlightGear's own
YASim format), and its `.ac` files are **AC3D** meshes. Both have a
real, parseable structure, so a real compatibility layer was built
rather than just noting the mismatch:

- **`io/xml.hpp` + `io/xml.cpp`**: a small dependency-free DOM XML
  parser, written from scratch (no internet-fetchable XML library was
  vendored) — sufficient for JSBSim's well-formed files, not a general
  XML implementation.
- **`io/jsbsim_import.hpp` + `io/jsbsim_import.cpp`**: reads a JSBSim
  aircraft file (`assets/fgfs_source/A320-211.xml`, bundled in this zip
  along with its GPL `COPYING` notice) and produces
  `MassProperties`/`AeroCoefficients`/`PropulsionModel`/gear-leg data
  populated from **real numbers**: wing area/span/chord, actual
  inertia, actual per-leg gear spring/damper/friction/geometry for
  every `<contact type="BOGEY">`, and real CFM56-5A1 static thrust from
  the referenced engine file. Run it yourself:
  `build/fgfs_import assets/fgfs_source/A320-211.xml assets/fgfs_source/cfm56_5a1.xml`
  — it prints the extracted JSON and every warning for anything it
  couldn't find (currently: none, against these two files).
- **`aircraft::spawnFromJSBSim()`** builds a real ECS aircraft entity
  from that imported data. `alpha_demo --a320 <fdm.xml> <engine.xml>`
  runs the same physics/gear/animation pipeline against it instead of
  the generic aircraft. **`tests/test_jsbsim_import.cpp`** checks the
  imported numbers land in a plausible A320 range (wing area, span,
  mass, thrust) and that the imported aircraft actually settles onto
  the baseplate under this engine's gear physics — all passing.
- **`tools/ac3d_to_obj/`**: a real AC3D→OBJ mesh converter (hierarchy,
  transforms, materials-as-groups) so the package's `.ac` meshes are
  usable by any future renderer without a bespoke AC3D loader.
  `assets/converted_models/cfm56_engine.obj` is a converted sample
  (1008 vertices, 675 faces) proving it works end to end; the rest of
  the package's meshes convert the same way, one `ac3d_to_obj in.ac
  out.obj` call at a time.

**What this is NOT**, so it isn't mistaken for more than it is:

- **Not a faithful JSBSim aerodynamics port.** JSBSim's `<aerodynamics>`
  defines coefficients as arbitrary nonlinear tables over multiple
  variables (alpha AND flap position, for example). The importer
  extracts one linearized derivative per coefficient (a slope through
  the table near zero) to fit this engine's much simpler static-
  derivative model. Flying the imported A320 in `alpha_demo --a320`
  is noticeably less stable/settled than the hand-tuned generic
  aircraft (visible pitch/roll oscillation in the telemetry) — that's
  the linearization showing, not a bug to chase down further here.
  Treat the imported handling qualities as a starting point, not a
  validated A320 flight model.
- **Not a general JSBSim importer.** The coordinate-frame conversion
  (JSBSim's aft-positive/up-positive structural frame → this engine's
  forward-positive/down-positive body frame) was derived from and
  checked against this one file (A320-211.xml). It has not been run
  against a second aircraft, so don't assume it generalizes without
  checking.
- **Not a renderer, on its own.** `ac3d_to_obj`'s `.obj` output has no
  material/texture data written out (materials are parsed but only
  passed through as OBJ group names — see that tool's own header
  comment). It's an asset-format bridge, not a finished art pipeline —
  but see the next section: this repo's renderer now does load the
  converted meshes, untextured.

## The real airframe mesh (`render/mesh_a320.cpp`, added on request)

`aircraft_rig.cpp` originally only had `meshgen::buildPlaceholderAircraft()`
to draw — procedural boxes/wedges sized to A320-ish proportions, not the
real geometry. `render/mesh_a320.cpp` + `render/obj_loader.cpp` replace
that with the actual converted A320-family.zip airframe:

- All 13 relevant `.ac` files (fuselage, wings, hstab, vstab, winglets,
  both landing-gear legs + tires, CFM56 engine/nacelle/pylons) are
  converted via `tools/ac3d_to_obj` into `assets/converted_models/`.
- `obj_loader.cpp` parses those `.obj` files back into `MeshPart`s, one
  per named AC3D object (the source files already split out things
  like `AileronL`, `FlapR1`, `SlatL3` as separate objects — no manual
  re-segmentation needed for those).
- `mesh_a320.cpp` maps ~28 of those parts onto every part name
  `AircraftRig` looks for (fuselage, both wings + all their control
  surfaces, both tail surfaces + elevators/rudder, all three gear legs
  + wheels, both engines + reversers + fans), each positioned using
  offsets traced through the aircraft's own FlightGear model-
  composition XML and cross-checked against the JSBSim gear data.
  **Full derivation, including a real bug that was caught (wingspan
  coming out ~2.3x too small from a second, different local axis
  convention in the wing/tail files) and how it was found and fixed:
  `assets/A320_MESH_NOTES.md`.**
- `apps/renderer_demo` now takes the same `--a320 <fdm.xml> <engine.xml>
  <converted_models-dir>` pattern as `alpha_demo`, spawning the real
  JSBSim-derived aircraft and drawing it with this real mesh instead of
  the placeholder. With no arguments it still falls back to the
  generic aircraft + placeholder mesh, unchanged.

**What this is NOT:**

- **Not rendered and visually confirmed in this environment.** This
  sandbox has the GLES3/EGL *runtime* libraries but not their
  development headers, and no network path to install them, so
  `aircraft_rig.cpp`/`gl_renderer.cpp`/`renderer_demo` could not be
  recompiled here to actually put pixels on screen. What was verified
  instead: a standalone harness (`tests/test_a320_mesh.cpp`,
  `tests/test_a320_mesh_detail.cpp`) that runs `buildA320Aircraft()`
  directly — all 28 parts populate, zero NaN vertices, zero degenerate
  triangles, and the resulting bounding box gives a 37.57 m fuselage
  length and 34.04 m wingspan, both matching the real A320 closely.
  That's meaningful evidence the offset/axis math is right, but it
  is not the same as confirming the meshes look correct once actually
  drawn, lit, and animated through `AircraftRig`. **Next person to
  touch this should build with real GLES3/EGL dev headers and actually
  run `renderer_demo --a320 ...` before trusting this further.**
- **wing_R has less geometry than wing_L** and **no hinge-line data**
  was available for animated surfaces (pivot is approximated as each
  surface's own centroid) — both explained in the notes doc, not
  hidden gaps.
- **No textures/materials** — same limitation `ac3d_to_obj` already had;
  parts render as flat-shaded geometry with `AircraftRig`'s existing
  solid colors, not the real livery.

## What is explicitly NOT done in this Alpha (do not assume otherwise)

- **No engine↔UI bridge yet.** `mobile_ui/index.html` still drives its own
  mock numbers (see `mockState` in its script), not the C++ simulation.
  `apps/renderer_demo` (see below) proves the engine→render pipeline end
  to end using desktop keyboard input as a stand-in for touch; wiring the
  actual HTML/touch UI needs a transport (WebAssembly build of
  `simengine_flight`+`simengine_render`, or a native shell with a WebView
  + JS bridge, or dropping the HTML UI in favor of a native-drawn touch
  overlay) that doesn't exist yet. This is the next concrete step.
- **No Android packaging yet.** `simengine_render` and every ECS system
  are written against GLES3-only APIs specifically so they need no
  changes for Android; `apps/renderer_demo/main.cpp`'s GLFW window/input
  layer is the one file that's desktop-only and needs a
  NativeActivity/EGL replacement (see "Suggested next steps" below).
- **Camera is a from-scratch placeholder, not a port of GeoDrive's
  camera.** GeoDrive's source wasn't part of what was uploaded for this
  build (only `GeoEngine.zip`, `rigid_body_6dof.{hpp,cpp}`, and
  `A320-family.zip` were). `camera/third_person_camera.hpp` implements
  the same *class* of behavior (damped spring-arm chase cam) from
  scratch. Swap this file for GeoDrive's real camera when it's available.
- **Aircraft geometry is placeholder, not modeled art.** `render/mesh.cpp`
  procedurally builds a low-poly box/wedge airliner shape — enough to
  see every animated surface move correctly, not final visual quality.
  Swapping in real modeled meshes only touches `mesh.cpp`'s builders and
  `aircraft_rig.cpp`'s part-name table.
- **A320 data is now real but approximate, not a validated flight
  model.** See the "FlightGear/JSBSim data compatibility" section above
  — mass/inertia/geometry/gear/thrust are real extracted numbers;
  aerodynamic derivatives are linearized approximations of JSBSim's
  nonlinear tables. `spawnGenericNarrowbody()`'s hand-picked coefficients
  are still the more stable/flyable option for now.
- **No JSON/data-file loader.** `assets/aircraft/generic_narrowbody.json`
  documents the intended per-aircraft data shape; `aircraft_factory.cpp`
  currently hardcodes the same numbers directly rather than parsing it.
- **Physics simplifications made for this Alpha, documented in code:**
  - Ground contact is applied as a post-RK4 semi-implicit velocity
    correction, not folded into the RK4 stages (avoids the stiff-spring
    timestep problem; see `landing_gear_system.hpp`'s header comment).
  - The gear correction's angular response uses diagonal inertia only
    (ignores the `Ixz` product-of-inertia term the in-flight RK4 model
    does account for).
  - Gear struts are treated as purely vertical (body -Z); canted strut
    geometry isn't modeled.
  - Slats are auto-scheduled from the flap handle (matches how most real
    transport aircraft actually work — slats aren't pilot-commanded
    directly) rather than having their own cockpit control; they are,
    however, now an independent aerodynamic/animation channel from flap,
    unlike the previous ganged approximation.
- **No cockpit, instruments, avionics, autopilot, weather, terrain, or
  multiple simultaneous aircraft demo** — the ECS/JobSystem plumbing
  (`parallelFor` over entity storages) supports N aircraft in parallel
  today; the demo just only spawns one.

## The renderer (`simengine_render` + `apps/renderer_demo`)

Added after the console-only Alpha above. `apps/renderer_demo/main.cpp`
opens a real window (GLFW, requesting a GLES 3.0 context specifically —
not desktop GL) and runs Input → FlightDynamics → LandingGear → Engine →
Animation → Camera → Render every frame, drawing a procedurally-built
placeholder aircraft (`render/mesh.cpp`) whose every animated part
(control surfaces, gear, wheels, engine fans, reversers — see
`render/aircraft_rig.cpp`) is driven directly by that frame's
`AnimationComponent` channels, over the grey baseplate, through a
third-person chase camera.

**Verified, not just written:** this environment has no display or GPU,
so verification here used a raw EGL surfaceless pbuffer context against
Mesa's llvmpipe software rasterizer (bypassing windowing entirely) to
run the full pipeline for 300 physics ticks with deflected controls,
then read back and inspect the framebuffer — confirmed the sky-clear
color, 22+ distinct shaded colors from the lit geometry (i.e. real
triangles at real depths, not a blank screen), and a CPU-side
projection check placing the aircraft's fuselage origin at screen-center
as expected for the chase camera's target. This is real evidence the
GL/shader/matrix/rig pipeline is correct, on real (if simulated) GPU
hardware — it is not a substitute for testing on an actual phone GPU,
which behaves differently in real ways (tiled rendering, driver quirks,
thermal throttling) that only on-device testing will surface.

`SIMENGINE_BUILD_RENDERER` in `CMakeLists.txt` auto-detects GLES3/EGL
dev headers and gracefully skips the renderer targets (falling back to
the physics/ECS-only build) if they're missing, so this doesn't break
building on a machine without them.

## Known rough edges to expect if you fly `alpha_demo`'s scripted scenario

The scripted takeoff roll in `alpha_demo/main.cpp` accelerates and rolls
correctly (weight-on-wheels, N1 spool, forward speed all behave), but
pitch response to the scripted elevator input is weak at the rotation
speeds reached in 20 simulated seconds — plausible given `Cmde=-1.3` and
untuned nose-gear unloading behavior, but not validated against any real
aircraft's rotation characteristics. Treat the aero coefficients in
`aircraft_factory.cpp` as a starting point for tuning, not ground truth.

## Suggested next steps, in the order they unblock the most

1. **Wire the touch UI** (`mobile_ui/index.html` or a native-drawn
   overlay) to a real `InputSnapshot` — `android/app/src/main/cpp/android_main.cpp`'s
   `onInputEvent()`/`buildInputSnapshot()` currently only implement a
   minimal drag-stick + throttle-slider mapping as a placeholder; that's
   the landing spot for real button-based controls (flaps/gear/speedbrake/
   spoiler/reverse/trim/parking brake).
2. **Build and run the Android project for real.** See `android/README.md`
   — the layer is written and believed correct against the real
   NDK/EGL/NativeActivity APIs (it mirrors the desktop `renderer_demo`'s
   already-verified engine calls closely), but this sandbox has no
   Android SDK/NDK/device, so it has NOT been compiled or run yet. This
   is the actual next milestone toward "on your phone."
3. Bring in GeoDrive's actual camera source and replace
   `third_person_camera.hpp`.
4. Add a real data-file loader (nlohmann/json is already MIT-licensed and
   trivial to vendor) and point `aircraft_factory.cpp` at
   `assets/aircraft/*.json` instead of hardcoding.
5. Replace the placeholder box-built aircraft mesh with real modeled
   geometry (glTF import is the natural target format for both desktop
   and Android).
6. Tune the aero coefficients against a real reference (a public
   textbook dataset, or a from-scratch derivation for a specific
   aircraft).
7. Ground-effect, icing, and structural-flex hooks are already called
   out as future extension points in `rigid_body_6dof.hpp`'s original
   header comment — still open.

## The Android layer (`android/`)

A NativeActivity APK project: EGL context creation, GLES 3.0 rendering
(the exact same `simengine_render` code the desktop renderer uses,
unmodified), and touch input, wired through
`android/app/src/main/cpp/android_main.cpp`. CMake
(`android/app/src/main/cpp/CMakeLists.txt`) builds it by
`add_subdirectory()`-ing this project's own root `CMakeLists.txt` rather
than duplicating source lists, and Gradle (`android/app/build.gradle.kts`)
drives that CMake build via `externalNativeBuild` to produce the APK.

**Bug found from an on-device screenshot, now fixed:** the mesh-
integration work above (`render/mesh_a320.cpp`) only touched
`apps/renderer_demo`'s desktop entry point. `android_main.cpp` is a
*separate* entry point (NativeActivity has its own `main`-equivalent)
that nobody had gone back and updated — it was still calling
`buildPlaceholderAircraft()` + `spawnGenericNarrowbody()` directly, so
the on-device build kept showing the old procedural placeholder (boxy
fuselage, red engine blocks) no matter how good the desktop mesh work
got. Fixed now:

- `android_main.cpp` extracts the real converted mesh + JSBSim FDM files
  out of the APK on first launch. Bundled Android assets aren't directly
  readable via `std::ifstream` the way desktop files are, so
  `extractAssetsIfNeeded()` copies a hardcoded list of the exact files
  needed (via `AAssetManager`) into the app's private internal storage,
  then hands `buildA320Aircraft()`/`importJSBSimAircraft()` ordinary
  filesystem paths under there — identical in shape to what
  `apps/alpha_demo`/`apps/renderer_demo` already pass on desktop, so
  none of that already-verified code needed to change. If extraction or
  loading fails for any reason, it falls back to the old placeholder
  aircraft rather than leaving the app half-initialized.
- `android/app/build.gradle.kts` now points `assets.srcDirs` at the
  repo's own `assets/` folder so those files actually end up in the APK
  (previously there was no `assets.srcDirs` at all — the files existed
  in the repo but Gradle was never told to bundle them).
- The file list in `extractAssetsIfNeeded()` is hardcoded rather than
  walking `AAssetManager_openDir()` recursively — that API's handling of
  nested asset subdirectories is inconsistent across AAPT2/NDK versions,
  and this project's asset set is small and fixed, so an explicit list
  was judged more robust than getting directory recursion right blind.

**Aircraft variant picker (added on request):** the A320-family.zip
package turns out to contain eight distinct real aircraft, not just the
A320-211 this project started with — A318-111, A319-111, A319-131,
A320-111/211/231, A321-211/231, each with its own real JSBSim mass/CG
and engine (CFM56-5A1/5B3/5B5/5B8 or IAE V2500-A5, depending on variant).
All eight were pulled into `assets/fgfs_source/variants/` and verified
end-to-end with `tools/fgfs_import` — zero import warnings for any of
them. `include/simengine/aircraft/a320_variants.hpp` is the shared
registry; `apps/alpha_demo --variant <name>` and
`apps/renderer_demo --variant <name>` both use it (see their `--help`-
equivalent usage text), and **both were actually run against several
variants in this environment** (A318-111, A320-211, A321-231 — all
loaded cleanly and ran the full scripted scenario). On Android, tapping
the screen's top-left ~150x150px corner cycles through the registry via
the same `loadVariant()` helper `ensureEngineInitialized()` uses for the
default — there's no on-screen label for it yet (no text-rendering
system exists in this Alpha), only a `LOGI()` of the new variant name,
so it's really a developer-facing picker today, not a finished touch UI
control. A real one belongs in the `mobile_ui/index.html` bridge work
already tracked below.

**Known simplification, stated plainly:** all eight variants currently
render with the *same* converted airframe mesh (the A320-211 shape).
That's not an oversight so much as a deliberate scope cut backed by real
data — the JSBSim FDM files for all eight report identical wing
reference geometry (S/b/c), so this package's own aero model doesn't
distinguish fuselage length either. But the real aircraft genuinely do
have different fuselage lengths (A318 shortest, A321 longest), and the
package has separate model-composition XML per variant (see
`assets/A320_MESH_NOTES.md`) that would need its own offset derivation,
same as was done for A320-211, to convert and place a per-variant
fuselage correctly. Picking a different variant right now changes the
real mass/CG/thrust the aircraft flies with; it does not change what
you see.

**Honest status on the Android build itself: written, not yet built or
run on a device.** This sandbox has no Android SDK, NDK, or device — see
`android/README.md`'s "What's verified, and what isn't" section for the
specifics and the most likely first-build friction points. That was
already true before this round of changes and remains true after it;
what's different this time is that the changes were driven by an actual
on-device screenshot showing the bug, and the non-Android-specific parts
of the fix (the variant registry, the JSBSim import per variant, the
mesh loading itself) were verified for real on desktop rather than only
reasoned about. The genuinely untested-in-this-environment pieces are
narrower than last round: `AAssetManager`/`extractAssetsIfNeeded()`'s
file I/O, and the touch-zone tap detection — both new, both native-only,
neither exercisable without a device or NDK.
