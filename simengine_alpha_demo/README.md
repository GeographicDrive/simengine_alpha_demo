> **Alpha Technical Demo addendum:** this checkout now includes a
> `simengine_flight` library (6-DOF physics + landing gear + ECS systems
> for flight dynamics/gear/engine/animation/input), a headless
> `alpha_demo` app, passing sanity tests, and a `mobile_ui/` touch-control
> mockup. **Read `docs/ROADMAP.md` first** — it states plainly what's
> real (compiles, runs, tested) versus not yet built (no renderer, no
> engine↔UI bridge, camera is a placeholder not a GeoDrive port, no real
> A320 data). Don't assume more is finished than that doc says.

# Simulation Engine

Next-generation simulation engine, built incrementally, one production-ready subsystem at a time.

See `docs/ARCHITECTURE.md` for the full module map, build order, and cross-cutting design invariants.

## This delivery: Mathematics module

Header-only C++20 library:

- `include/simengine/math/vector3.hpp`
- `include/simengine/math/quaternion.hpp`
- `include/simengine/math/matrix4.hpp`
- `include/simengine/math/geodetic.hpp` (WGS84)

## Build & test

With CMake:

```
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build
ctest --test-dir build --output-on-failure
```

Without CMake (direct compile, no external dependencies):

```
g++ -std=c++20 -O2 -Wall -Wextra -Iinclude tests/test_math.cpp -o test_math
./test_math
```

Expected output: `390/390 checks passed`, `RESULT: PASS`.

## Using the module in later subsystems

```cpp
#include "simengine/math/vector3.hpp"
#include "simengine/math/quaternion.hpp"
#include "simengine/math/matrix4.hpp"
#include "simengine/math/geodetic.hpp"

using namespace simengine::math;

Vector3d worldPos(...);
Quaterniond orientation = Quaterniond::fromEulerZYX(yaw, pitch, roll);
GeodeticCoord aircraftPos{latRad, lonRad, altMeters};
Vector3d ecef = geodeticToEcef(aircraftPos);
```

Link against the `simengine_math` CMake INTERFACE target, or just add `include/` to your include path.
