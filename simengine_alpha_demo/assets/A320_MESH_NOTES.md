# A320 mesh integration notes

How `render/mesh_a320.cpp` places the real A320-family.zip airframe
meshes into this engine's body frame (X forward, Y right, Z down,
origin at the aircraft CG — the same frame `physics::RigidBody6DOF`
and `io::jsbsim_import.cpp` already use).

## Two different local axis conventions in the source files

`tools/ac3d_to_obj` converts each AC3D file's geometry "as authored" —
it applies that file's own internal OBJECT hierarchy transforms, but
each file's *root* coordinate system is whatever the modeler used, and
it turns out the A320-family.zip package doesn't use one convention
uniformly:

- `a320.fuselage.ac`, the landing gear `.ac` files, and the engine/
  nacelle/pylon `.ac` files use the same convention `io/jsbsim_import.cpp`
  already assumes for the JSBSim FDM data: X aft-positive, Y right-
  positive, Z up-positive.
- `a320.wings.ac`, `a320.hstab.ac`, `a320.vstab.ac`, and
  `a320.winglets.ac` use a different local layout: X is chordwise
  (small values, aft-positive), Y is up (small values), and Z is
  spanwise, with **positive Z on the left side**.

This wasn't assumed — it was checked directly by looking at the named
AC3D objects a real modeler would have to keep symmetric, e.g.
`AileronL` vs `AileronR` in `a320.wings.ac`: they mirror across local
**Z**, not local Y (`AileronL` Z range +13.4..+16.3, `AileronR` Z range
-16.3..-13.4; both have near-identical, small X and Y ranges — the
chord and thickness directions). The same check on `ElevatorL`/
`ElevatorR` and `WingletL`/`WingletR` confirmed the same layout, and
`a320.vstab.ac`'s single (non-mirrored) fin object showed local Y as
its ~6m-tall vertical extent, consistent with "Y = up" holding across
the whole wing/tail family.

`obj_loader.cpp` has two axis-conversion functions accordingly:
- `convertAxes`: `(x, y, z) -> (-x, y, -z)` — fuselage/gear/engine files.
- `convertWingAxes`: `(x, y, z) -> (-x, -z, -y)` — wing/tail-surface files.

`loadObjGroups(path, wingSurfaceAxes)` picks between them; `mesh_a320.cpp`
passes `true` only for `wings.obj`, `winglets.obj`, `hstab.obj`, `vstab.obj`.

## Part placement offsets

Every part's position in the aircraft is baked directly into its
vertices (not left as a separate `attachBody` — see the comment at the
top of `mesh_a320.hpp` for why). The offsets come from
`Aircraft/A320-family/XMLs/A320.xml` (the real FlightGear model-
composition file, which chains a `<model>` block with an `<offsets>`
per submodel: Fuselage, Wings, NoseGear, MLG left/right, HStab, VStab,
and — nested one level deeper, inside `XMLs/Wings/a320.wings.xml` —
the two `Engine.CFM` submodels relative to the wings' own origin).

Converting those into this engine's CG-centered frame needs one more
number: where the CG sits along the fuselage in that same model-space
X. That wasn't given directly, but it's derivable by combining two
independently-authored sources that should describe the same physical
aircraft:

- The model XML's nose-gear offset: `x = 7.83 m` (model-space, aft-positive).
- The JSBSim FDM's (`A320-211.xml`) nose-gear contact `<location>`
  (`x = 197.6772 in = 5.023 m`, JSBSim structural-frame) and CG
  `<location>` (`x = 642.122 in = 16.310 m`), giving a nose-gear-to-CG
  distance of `16.310 - 5.023 = 11.289 m`.

Adding those: `CG_model_x = 7.83 + 11.289 = 19.119 m`.

This number is also its own consistency check: running it back through
`engineFrame()` for the nose gear reproduces JSBSim's own nose-gear
body-frame X (11.289 m) exactly, and the main-gear Y offsets from the
model XML (±3.795 m) match JSBSim's main-gear Y exactly too — meaning
the model-XML geometry and the JSBSim FDM data, despite being authored
completely separately, agree on where the gear actually is. That
agreement is the main reason to trust the rest of the derived offsets
(wing root, tail surfaces, engines) even though there's no independent
FDM number to check those particular ones against.

## Known gaps / simplifications

- **wing_R renders less structure than wing_L.** The AC3D file's
  `Wingbox`/`Wings`/`FairingPylons`/`Flaps1` objects are single meshes
  that already span both sides (unlike `AileronL`/`AileronR` etc.,
  which are genuinely separate per-side objects), so they're only
  added to `wing_L` to avoid drawing the same geometry twice — `wing_R`
  currently only gets its own side-specific fairings and winglet. This
  is a visual gap, not a physics one (the mesh isn't used for
  aerodynamics).
- **No hinge-line data was extracted for animated surfaces** (ailerons,
  elevators, rudder, flaps, slats, spoilers). `mergeGroups(...,
  setPivotToCentroid=true)` uses each surface's own vertex centroid as
  an approximate rotation pivot. Visually reasonable for these thin
  chordwise surfaces; not aerodynamically exact.
- **No speedbrake-specific surface exists** in the source model (the
  real A320 uses its spoilers for both roles), so `AircraftRig`'s
  `speedbrake` part is simply absent — it's already written to skip
  missing parts silently, so this doesn't crash anything, but it means
  speedbrake deployment currently has no visible geometry change beyond
  whatever the spoiler parts already show.
- **`mlg_tires.obj` is reused unmodified for both the left and right
  main gear** — it's the only tire mesh in the package for that gear
  type, so both sides get the same local shape, mirrored purely by the
  attach offset's sign. The nose gear has its own dedicated
  `nlg_tires.obj`.
- **The nacelle/pylon/core files (`nacelle_cfm.obj`, `pylon_cfm_*.obj`,
  `cfm56.obj`) are shared between the two engines** — only the pylon
  has genuinely distinct left/right geometry in the source package
  (`a320.pylon.cfm.left.ac` / `a320.pylon.cfm.right.ac`); the nacelle
  and engine-core meshes are reused for both sides, mirrored by attach
  offset sign, same as the main gear tires above.
- **Not independently verified against a real GPU render pass in this
  environment** — this sandbox has the GLES3/EGL runtime libraries but
  not their development headers (and no network access to install
  them), so `renderer_demo`/`aircraft_rig.cpp`/`gl_renderer.cpp`
  couldn't be recompiled here. What *was* verified: `buildA320Aircraft()`
  runs standalone and produces all 28 expected parts, zero NaN
  vertices, zero degenerate triangles, a 37.57 m fuselage length and a
  34.04 m wingspan — both matching the real A320's dimensions almost
  exactly, which is a meaningful check on the offset/axis math above,
  but it is not the same as confirming the parts look right when
  actually drawn and lit through `AircraftRig`.
