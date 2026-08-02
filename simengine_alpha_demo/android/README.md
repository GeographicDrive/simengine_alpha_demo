# android/ — Building the Alpha Technical Demo as an APK

## What this is

A plain **NativeActivity** APK: there is no Java/Kotlin application code
(`hasCode="false"` in the manifest). `android.app.NativeActivity` loads
`libsimengine_android.so` directly and calls into
`src/main/cpp/android_main.cpp`'s `android_main()`, which:

1. Creates an EGL context/surface against the `ANativeWindow` NativeActivity
   hands it (GLES 3.0, matching every other part of the render stack).
2. Runs the exact same ECS pipeline as the desktop
   `apps/renderer_demo` — Input → FlightDynamics → LandingGear → Engine →
   Animation → Camera → Render — using `simengine_flight` and
   `simengine_render`, unmodified, from the project root. On first
   launch it extracts the real converted A320 mesh + JSBSim FDM data out
   of the APK's bundled assets (see `docs/ROADMAP.md`'s Android section
   for why that extraction step exists) and loads the real A320-211
   aircraft, falling back to the procedural placeholder if that fails.
3. Reads touch input (left half of the screen = pitch/roll drag stick,
   right half = a vertical throttle slider, top-left 150x150px corner =
   cycle through the eight real A320-family variants — see
   `include/simengine/aircraft/a320_variants.hpp`) into the same
   `systems::InputSnapshot` the desktop keyboard path builds.

`src/main/cpp/CMakeLists.txt` does not duplicate the engine's source
list — it `add_subdirectory()`s the project's own root `CMakeLists.txt`
with `SIMENGINE_BUILD_RENDERER=ON`, `SIMENGINE_BUILD_APPS=OFF`,
`SIMENGINE_BUILD_TESTS=OFF`. Any change you make to the physics,
animation, or rendering code at the project root is picked up here
automatically — this Android project is not a fork or a copy.

## How to build it

You need [Android Studio](https://developer.android.com/studio) (which
bundles a compatible JDK) or a standalone Android SDK + NDK install. This
project targets **NDK 26.3.11579264** and **CMake 3.22.1** (see
`app/build.gradle.kts`); Android Studio's SDK Manager can install both.

**Easiest path:** open the `android/` folder directly in Android Studio
("Open" → select this folder) and press Run.

**Command line**, once you have `ANDROID_HOME`/`ANDROID_SDK_ROOT` set and
the NDK installed:

```bash
cd android
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

The Gradle Wrapper (`gradlew`, `gradlew.bat`, `gradle/wrapper/gradle-wrapper.jar`)
is included — no separate `gradle wrapper` bootstrap step needed. It's
pinned to Gradle 8.7 (see `gradle/wrapper/gradle-wrapper.properties`),
fetched from the same official `gradle/gradle` v8.7.0 tag as the jar/
scripts, so all three are a matched, authentic set. `./gradlew --version`
was run in the sandbox this project was built in to confirm the script
and jar correctly parse and attempt the distribution download — it fails
there only because that sandbox's network egress allowlist doesn't
include `services.gradle.org`, which is not a concern on your machine or
on GitHub Actions (see below).

## Continuous integration

`.github/workflows/android.yml` builds this project on every push/PR
that touches `android/`, `include/`, `src/`, or the root `CMakeLists.txt`,
on a stock `ubuntu-latest` GitHub-hosted runner — no Android Studio, no
self-hosted runner, no pre-baked image. It:

1. Installs JDK 17 (`actions/setup-java`).
2. Installs the exact SDK platform/build-tools/NDK/CMake versions this
   project pins (via `sdkmanager`), rather than relying on whatever the
   runner image happens to have preinstalled.
3. Runs `./gradlew assembleDebug --no-daemon --stacktrace`.
4. Verifies the resulting APK exists and is a structurally valid archive
   (`unzip -t`), then uploads it as a workflow artifact
   (`simengine-alpha-demo-debug-apk`) you can download from the Actions
   run summary — no device needed to get a real, installable APK out of
   this.

## What's verified, and what isn't (read this before assuming it just works)

The sandbox this project was built in has **no Android SDK, no NDK, and
no Android device/emulator** (it does have general internet access,
which is how `gradlew`/`gradlew.bat`/`gradle-wrapper.jar` were fetched
as authentic files from the official `gradle/gradle` v8.7.0 tag and then
test-run — see above) — so unlike the desktop renderer (which was
actually compiled and run against a software GL implementation with a
screenshot proving real triangles on screen, see `docs/ROADMAP.md`),
**the NDK/CMake/clang compile step itself has not been run here.** The
C++ (`android_main.cpp`) was written carefully against the real NDK/EGL/
NativeActivity APIs and mirrors the exact structure of the desktop
`apps/renderer_demo` (which *is* verified) as closely as possible, but
the honest status is: **the included GitHub Actions workflow's first run
is the first real compile of this.**

The likely first-build friction points, roughly in order of probability:
- A CMake/Gradle argument mismatch (wrong variable name, ABI flag) —
  usually a one-line fix, the error message will point at it directly.
- `android_native_app_glue.c`'s path assumes a standard NDK layout
  (`$ANDROID_NDK/sources/android/native_app_glue/`) — correct for every
  NDK version in recent memory, but worth checking first if that target
  fails to configure.
- Touch coordinate mapping (`onInputEvent` in `android_main.cpp`) is
  untested against a real touchscreen's coordinate/pointer-ID behavior;
  the logic is straightforward but device-specific quirks sometimes
  surface here.

Please push this to a GitHub repo and check the Actions tab (or run
`workflow_dispatch` manually) — the workflow log's first failure, if
any, is the fastest way to close the gap between "carefully written"
and "actually compiles." Touch behavior on a real screen still needs an
actual device/APK install to confirm, separately from the build itself.
