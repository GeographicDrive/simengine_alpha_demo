# Simulation Engine — Architecture

## Status

| Module | Status |
|---|---|
| **Mathematics** | ✅ Implemented |
| **Core Engine** | ✅ Implemented (this delivery) |
| HTML/CSS/JS Runtime | Not started |
| Physics | Not started |
| Flight Dynamics | Not started |
| Atmosphere | Not started |
| Weather | Not started |
| Terrain | Not started |
| Collision | Not started |
| Aircraft Systems | Not started |
| Engine Simulation | Not started |
| Landing Gear | Not started |
| Aerodynamics | Not started |
| Navigation | Not started |
| Avionics | Not started |
| Rendering | Not started |
| Audio | Not started |
| Asset Streaming | Not started |
| Networking | Not started |
| Plugin System | Not started |
| Replay System | Not started |
| UI | Not started |

Everything below "Mathematics" depends on it. Build order follows the dependency graph, not the order modules were listed in the brief.

## Honest scope note

A custom HTML/CSS/JS runtime that outperforms Blink while staying spec-compatible is, by itself, comparable in scope to an existing multi-year browser-engine project (V8/JavaScriptCore-class JS engine + a CSS layout/paint pipeline). That module will be designed with a real architecture and built incrementally like every other subsystem — but no single delivery will produce a "finished browser engine." Anything claiming otherwise would be exactly the kind of shortcut this project's own rules forbid. The plan below sequences it realistically: a minimal, sandboxed, spec-subset runtime purpose-built for simulation UI (HUDs, MFDs, instrument panels), not a general-purpose web browser replacement.

## Dependency-ordered build sequence

```
1. Mathematics        (done)
2. Core Engine         — ECS/data model, fixed-timestep scheduler, memory pools, job system
3. Physics             — rigid body dynamics, integrators, constraint solver (generic, non-flight-specific)
4. Atmosphere           — depends on Mathematics only; feeds Flight Dynamics & Rendering
5. Aerodynamics         — depends on Physics + Atmosphere
6. Flight Dynamics      — depends on Physics + Aerodynamics + Atmosphere (6-DOF integration)
7. Engine Simulation    — depends on Atmosphere (air density/temp), Flight Dynamics (thrust feedback)
8. Aircraft Systems     — depends on Core Engine (state model) + Engine Simulation (electrical/fuel draw)
9. Landing Gear         — depends on Physics (contact dynamics) + Flight Dynamics
10. Collision           — depends on Physics; generalizes gear contact solving to terrain/world
11. Terrain             — depends on Mathematics (geodetic) + Asset Streaming
12. Weather             — depends on Atmosphere; adds precipitation/icing state
13. World/Navigation    — depends on Mathematics (geodetic/ENU) + Terrain
14. Avionics            — depends on Navigation + Flight Dynamics + Aircraft Systems
15. Asset Streaming     — depends on Core Engine job system
16. Rendering            — depends on Core Engine + Asset Streaming + Terrain
17. HTML/CSS/JS Runtime — depends on Core Engine + Rendering (for UI compositing)
18. UI                  — built on the runtime above
19. Audio               — depends on Core Engine + World (spatialization)
20. Networking          — depends on Core Engine state model (replication)
21. Replay System        — depends on Core Engine (deterministic state snapshotting)
22. Plugin System        — cross-cutting; formalized once module boundaries above are stable
```

Rationale for reordering vs. the brief's listing: several named modules (Avionics, UI, Networking) structurally *consume* other modules' finished interfaces, so building them earlier would mean building against APIs that don't exist yet — a placeholder, which is explicitly disallowed.

## Cross-cutting invariants (apply to every subsystem)

1. **Simulation/render decoupling.** All physical/state modules (Physics, Flight Dynamics, Atmosphere, Aircraft Systems, Engine Simulation) run on a fixed timestep, independent of frame rate, using a standard accumulator pattern in the Core Engine scheduler. Rendering interpolates between the last two fixed states.
2. **No allocation in the hot loop.** Per-tick state lives in pre-sized memory pools (Core Engine module). Anything that must grow (e.g. streamed terrain tiles) grows on a background thread, never on the sim thread.
3. **Double precision at world scale, single precision locally.** Established in the Mathematics module (`Vector3d`/`Vector3f`, ECEF/ENU rebasing). Every downstream module must rebase to a local ENU or object-local frame before doing single-precision physics/rendering math — never do arithmetic directly in raw ECEF meters at float32.
4. **Determinism.** Fixed timestep + fixed-order system execution + no reliance on wall-clock or thread-scheduling-dependent floating point reduction order in any physics accumulation. This is required for both the Replay System and Networking (deterministic lockstep is one viable netcode strategy, TBD when that module is designed).
5. **No silent NaN/Inf propagation.** Degenerate math (zero-length normalize, singular matrix inverse) returns a defined fallback or an explicit failure flag, never a silently propagating NaN — enforced starting in Mathematics (see `Vector3::normalized`, `Matrix4::inverse`) and required of every subsystem built on top.
6. **Independent, pluggable modules.** Each module exposes a C++ interface (abstract base or C-ABI boundary, decided per-module) that the Plugin System can satisfy with a third-party implementation — e.g. a third-party Aerodynamics model replacing the built-in one without touching Flight Dynamics.

## What "Mathematics" delivers in this drop

- `Vector3<T>` — templated 3D vector, exact algebra, NaN-safe normalize.
- `Quaternion<T>` — orientation representation (no Euler-angle state storage anywhere else in the engine), axis-angle and ZYX-Euler construction/extraction, slerp.
- `Matrix4<T>` — column-major (Vulkan-native) transform matrix, TRS composition, Vulkan-convention perspective projection (depth [0,1], Y-flip), Gauss-Jordan inverse with singularity detection.
- `geodetic.hpp` — WGS84 ellipsoid geodetic↔ECEF conversion (Bowring's method), local ENU basis construction and conversion, forming the precision foundation for the future World/Navigation/Terrain modules.

All of it is header-only, allocation-free, and covered by round-trip/invariant unit tests (`tests/test_math.cpp`, 390 assertions, all passing on this build).

## What "Core Engine" delivers in this drop

- `memory_pool.hpp` — fixed-block-size pool allocator (`MemoryPool`) and a typed placement-new/delete wrapper (`TypedPool<T>`). O(1) alloc/free via freelist, single upfront allocation, returns `nullptr` on exhaustion rather than growing silently or invoking UB.
- `ecs.hpp` — sparse-set Entity Component System. Generational entity handles (`Entity{index, generation}`) detect stale references instead of aliasing recycled slots; `ComponentStorage<T>` keeps component data in a dense, contiguous, cache-friendly array with O(1) add/remove via swap-and-pop; `World` ties entity lifetime to automatic cleanup of every component type on destroy.
- `scheduler.hpp` — `FixedTimestepScheduler`, the accumulator that decouples simulation rate from render rate (required by the brief for Flight Dynamics). Deterministic step ordering, spiral-of-death protection via a max-steps-per-frame cap, and an interpolation alpha for the renderer.
- `job_system.hpp` — persistent thread-pool `JobSystem` with `parallelFor` for batch operations over ECS component arrays, and raw `submit`/`waitAll` for background work (streaming, terrain generation).

Integration-tested together in `tests/test_core.cpp`: 500 ECS entities updated in parallel across a thread pool, driven by the fixed-timestep scheduler, verified numerically correct to 1e-9 after 60 simulated steps. 38/38 checks passing.

## Next module: Physics

Per the dependency-ordered sequence above, **Physics** (generic rigid body dynamics, integrators, constraint solver — not flight-specific yet) is next. It sits directly on Core Engine (ECS components for rigid body state, scheduler for the fixed-step integration loop, job system for parallel constraint solving) and is itself a dependency of Flight Dynamics, Landing Gear, and Collision.
