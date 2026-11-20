# FRC Prototype Simulator: plan

Written 2026-11-20 for the model that builds it. The owner reads it too, so it says what is decided,
why, and what is still theirs to answer. Keep it current as work lands: mark milestones done here,
the way `rebuilt-sim-native/PLAN.md` does.

## In short

1. **A design tool, not a game.** You bring a robot (as rough blocks, or as CAD), mark what moves and
   what spins, and the app tells you whether it picks the game piece up, how fast it cycles, what it
   draws from the battery, and whether it tips. The 2026 simulators answered "how does this finished
   robot play". This one answers "should we build this".
2. **One robot description, two levels of physics.** *Concept* is for the first week: blocks, ideal
   pickup zones, real motors and battery. *Full* is for design review: the CAD's own surfaces,
   rollers and game pieces that squash, friction that slips. Each mechanism switches level on its
   own, so a robot is upgraded one mechanism at a time and nothing is rebuilt.
3. **MuJoCo runs the rigid bodies. The squashy contact is ours.** MuJoCo gives joints that do not
   stretch and geared motors that are stable. The roller-to-piece contact is an elastic foundation
   (a bed of springs) written here, with stiffness in real units taken from press tests. Both were
   tried before this plan was written (see "What was measured").
4. **The game is data.** 2025 REEFSCAPE is the first game pack and the test bed. A game pack is made
   in the same editor as a robot, so the 2027 field can be brought in during kickoff weekend.
5. **Kickoff is early January 2027.** The order of work puts what kickoff needs first: the bench,
   Concept robots, CAD rigging, and making a game pack. AI matches and running real robot code come
   after.

## What it is for

| | Concept | Full |
|---|---|---|
| When | Kickoff week: which archetype | Before cutting metal: does this design work |
| Robot is | Blocks from a palette, or CAD with nothing marked up | CAD with links, joints, rollers and materials marked |
| Game piece handling | Pickup zones and holders: in the zone with the intake running, it is yours after a set time | Contact: squash, friction, slip, jams |
| Joints, motors, battery | Simulated | Simulated |
| Drivetrain | Traction-limited swerve on a flat floor | The same, or wheels on real contact |
| Answers | Cycle time, reach, tipping, power budget, how a match might go | Does it intake from this angle, how much squeeze, what current, where it jams |
| Speed | Six robots far faster than real time | One robot in real time at 1 kHz or more |

A third place, the **Bench**, holds one mechanism on a stand with a piece feeder. It is where an
intake is tuned, and where sweeps run (squeeze against roller speed against current limit).

Not in scope: structural stress, part failure, wiring, a campaign mode, photoreal venues. The 2026
game's AI, neural driver and bit-exact port rules do not come along.

## What exists already (surveyed 2026-11-20)

`rebuilt-sim-2026` (browser, JS) and `rebuilt-sim-native` (Rust, Bevy 0.20.1, Rapier 0.36.1) are the
owner's 2026 simulators. What matters for this project:

- **The mechanism core is good and sphere-only.** `crates/sim/src/mech/mechanism.rs` (1,361 lines) is
  a soft-constraint Gauss-Seidel solve of FUEL against wheels and triangle surfaces, with a measured
  foam curve (AndyMark's test report: 19.8 lb/in, stiffening exponentially), tread stiffness in
  series, omni and mecanum friction, and the reaction on the chassis. Every contact routine is
  "centre distance minus R" with one global radius, mass and scalar inertia. A FUEL has no
  orientation. A pipe cannot be put through it.
- **`mech/drives.rs` (287 lines) is reusable as it stands.** Motors from datasheet numbers with
  stator and supply limits (the supply limit solved as a quadratic), shafts with stops and bearing
  friction, trains with coupled motors, racks, cams, a velocity loop, all integrated implicitly in
  the back-EMF.
- **Every robot is hand-written.** 200 to 370 lines of geometry and 270 to 460 lines of control per
  robot, plus a per-robot Python script that cuts the ball path out of the CAD from typed roller
  lists and seed points. There is no data-driven robot. This project exists to remove that.
- **The CAD tools are the right start.** `tools/step2glb.py` reads STEP through OpenCascade and keeps
  the assembly tree, names, colors and true normals. `tools/step-axes.py` finds every cylindrical
  face, merges coaxial ones per part, and writes axes, radii, volume and centre of mass.
  `tools/extract-parts.mjs` carries a recipe format (include by name, pivot, drop hardware) that is
  most of a markup schema.
- **Battery only on the drivetrain.** Mechanisms run at a fixed 12 V. Drive current sags a 12.6 V,
  0.02 ohm battery; mechanism current is not added. No breakers or brownout.
- **The swerve model is worth keeping for Concept.** `drivetrain.rs` solves the four wheel contacts
  against the chassis mass each step (motor limits, friction circle, battery sag) and applies one
  force and torque. It is engine-independent.
- **Arms are not dynamic.** A pivot is a shaft with a typed load torque. A raised arm does not move
  the robot's centre of mass, and tipping is damped so robots never fall over. For a design tool
  both have to be real.
- **The game is everywhere.** FUEL, HUB and TRENCH are globals in a dozen files. There is no line
  between game and engine.
- **No link to robot code** in either repo.
- **The app shell works**: Velopack installer and self-update from GitHub Releases, settings in app
  data, gamepad input, camera rig, CAD-look materials and edge bevels. Its UI is hand-built Bevy UI
  (menus and a HUD), with no property editors, trees or plots. The sim runs on the main schedule,
  not its own thread.

## What was measured (2026-11-20, 4-core Linux box)

Scripts are in `spikes/`. They are throwaway Python against the MuJoCo C engine; the numbers are
what the plan's decisions rest on.

**1. A compliant roller pulling a CORAL in against the carpet** (`spikes/roller_intake.py`). MuJoCo
holds the pipe and its floor contact. The roller contact is a bed of springs sliced along the
roller, with stick-slip friction per slice. The roller is a shaft on a Kraken X60 at 3:1 with a
stator limit.

| Case | Result |
|---|---|
| Grippy roller (friction 1.0), 1 to 30 mm squeeze, 40 A | In every time, 0.26 to 0.34 s. Peak current 8 A at 1 mm, at the 40 A limit from 10 mm. Squeeze force 25 to 945 N. |
| Slick roller (friction 0.25), same sweep | In up to 15 mm, slower. At 22 mm the pipe is pushed away and never enters. |
| 15 mm squeeze, stator limit 10 / 20 / 40 / 80 A | In each time. The roller sags to 911 rpm at 10 A against 1,715 rpm at 40 A. |
| Same run at 500 / 1,000 / 2,000 / 4,000 Hz | Entry time 0.28 to 0.29 s, peak force 347 to 350 N, slowest roller 1,725 to 1,764 rpm. |

So the model gives the answers a prototype gives (more squeeze costs current, a slick roller with
too much squeeze rejects the piece, a low limit bogs the roller), and it has converged by 1 kHz.
What it does not show: a pipe arriving turned 15 to 45 degrees left spun 30 to 90 degrees, which is
probably the script's crude geometry (no end faces, one roller, no side guides). Treat yaw
behaviour as unproven until M1 checks it against a real intake.

**2. A whole field** (`spikes/field_timing.py`): six robots, each with swerve on real wheel contacts,
a two-stage elevator, an arm and a wrist (11 geared actuators each), 60 CORAL and 18 ALGAE, everything
moving. Milliseconds of one core per simulated second:

| | 500 Hz | 1,000 Hz | 1,000 Hz, sleeping on |
|---|---:|---:|---:|
| CORAL as one cylinder | 290 to 330 | 550 to 590 | 434 |
| CORAL as a hollow ring of 12 boxes | 560 to 620 | 1,100 to 1,300 | 629 |

(The sleeping column was run on MuJoCo 4.0.1 only.) So a full field at 1 kHz fits in real time on one core of a slow machine only with MuJoCo's
sleeping on and the hollow shape used sparingly (D5). One robot on the Bench is hundreds of times
cheaper.

**3. A geared lift**: an 8 kg carriage on two Kraken X60 at 10:1 on a 1.75 in. spool, which is 28 kg
of reflected rotor. With the rotor as joint armature and the motor as an affine actuator it reaches
1 m in 1.09 s with 1.5 mm overshoot, and gives the same answer at 200, 500 and 1,000 Hz.

**4. Rust.** `mujoco-rs 7.1.0` (binds MuJoCo 3.17.0) builds here in 6 s and steps a CORAL for 100
simulated seconds at 1 kHz in 0.33 s. Its build script could not download MuJoCo through this
network; pointing `MUJOCO_DYNAMIC_LINK_DIR` at the release archive worked. The spikes give the same
results on MuJoCo 3.17.0 and 4.0.1.

**5. Onshape, on the test robot** (the owner's keys, an EDU Educator account, 2910's 2025 document):

| Call | Result |
|---|---|
| Document and element list | Served. The document grants `EXPORT` and anonymous access only. The workspace in the public link was stale; the document's `defaultWorkspace` is the one to use. |
| Assembly definition (the mates) | Refused, 403. The 2026 notes record the same for 7769's export-only document. |
| Bill of materials | Served, 5.3 MB in 8 s: 655 rows, 1,919 parts. 585 rows carry a mass, 557 a material. |
| STEP translation | STEP_RESULT |

The BOM names what the app needs to match: 16 Kraken X60 and 4 Kraken X44, SDS MK4i modules with
4 in. nitrile treads, two "4 in. OD x 1 in. Wide Straight Flex Wheel (... 45A) (WCP-1362)" and four
"AM-3562 3/8 in. Hex Bore 4 in. Compliant Wheel", with materials such as 6061, 7075, polycarbonate,
Delrin and printed Onyx. Hardware is 209 rows and more than half the part count. The whole exchange
cost about 10 API calls.

## Decisions

**D1. A new Rust workspace. Code is copied in, not depended on.** Bevy 0.20 for the app, plain Rust
with no ECS for the simulation crates (as before). Files taken from the 2026 repos are copied with a
comment naming the source file and commit. The bit-parity rules do not apply here: no `jsmath`, no
f32 rounding of impulses, no statement-order constraints, no pinned Rapier. Use `f64` and `std`
math.

**D2. MuJoCo is the rigid-body engine**, pinned to the version `mujoco-rs` binds (3.17.0 today),
shipped as its shared library beside the exe. Why, against staying on Rapier:

- Joints are generalized coordinates, so an elevator cannot stretch or sag at its joints under load.
- A geared motor's rotor is joint *armature*. Reflected rotor inertia is often larger than the
  mechanism (28 kg against 8 kg in the lift above). A maximal-coordinate engine has no way to state
  it for a sliding joint.
- A DC motor is an affine actuator (`force = gain * volts + bias * velocity`), so back-EMF is
  integrated implicitly and high ratios stay stable.
- Equality constraints and tendons cover continuous and cascade elevator rigging, four-bars, winches,
  ropes and constant-force springs.
- A model is built in code (`mjSpec`), and recompiling a robot takes milliseconds, which is what
  lets the editor apply a change while the sim runs.
- Sleeping islands (since 3.4) keep a field of resting game pieces cheap.

What it costs: a C library to ship and keep in step with the bindings; mesh collision is convex
only, so CAD needs decomposition (D7); its soft contact has no stiffness in N/m (D4); the existing
ball solver cannot be dropped in. `parry3d` is still used, for geometry queries and convex
decomposition, not for stepping.

All engine calls live in one module (`sim::engine`). Nothing else names MuJoCo types.

**D3. One description, two fidelities, chosen per mechanism.** A project stores what the robot *is*:
links, joints, transmissions, motors, rollers, surfaces, holders, sensors, controllers. Each
mechanism carries `fidelity: concept | full`, and a project-wide switch can force either. The
compiler (`sim::compile`) turns the description into an engine model:

| | Concept | Full |
|---|---|---|
| Link collision | Its blocks, or a box fitted to its CAD | Primitives and convex pieces from the CAD (D7) |
| Rollers | Drawn, spun, drawing current against a typed load. Not in contact | Foundation contact (D4) |
| Piece pickup | Zone and holder rules (below) | Contact only |
| Joints, motors, battery, sensors, controllers | The same code in both | |
| Step rate | 500 Hz | 1 kHz on the field, 2 kHz on the Bench, raised automatically when a contact needs it |

*Concept pickup.* A mechanism has a pickup zone (a box on a link) and one or more holders (a pose on
a link). A piece whose centre is in the zone while the intake runs is acquired after
`acquire_time`, then welded to the holder (a MuJoCo weld constraint switched on at run time, so the
piece's mass still loads the arm and moves the centre of mass). Handing off between holders takes a
set time. Release is place (weld off, piece keeps the holder's velocity plus an eject speed) or
launch (speed from the flywheel's surface speed times an efficiency). An optional "scoring assist"
seats a piece released within a set tolerance of a scoring spot, the tolerance standing in for
mechanism quality. `tools/nn/gpusim.py` in the 2026 repo is the precedent for this level.

**D4. Compliant contact is an elastic foundation, written here, in real units.**
MuJoCo's own soft contact is a spring in acceleration space: how far a piece sinks is independent
of its mass, and there is no N/m to set. Squeeze force decides traction and motor current, so it
has to be physical.

- *Who carries the springs.* A roller or wheel tread, a pad (foam or grip tape on a link), a belt (a
  pad whose surface moves along a path), or a soft piece (ALGAE against anything rigid).
- *Sampling.* A roller is sliced along its axis, and each slice samples points round the arc facing
  the piece. A pad samples a grid. A soft piece samples its own surface. Each sample asks the other
  body's signed distance for depth and normal.
- *Signed distance.* Pieces are analytic where they can be: sphere, cylinder, tube (hollow), box,
  capsule, cone, torus. Odd pieces and CAD surfaces use the triangle BVH and closest-point code from
  `mechanism.rs` (`build_tree`, `closest_on_tri`).
- *Normal force.* Pressure from depth by the material's law (linear, power, exponential-stiffening
  like the FUEL foam, or a table), times the sample's area. Where both sides are soft the depth is
  split so the pressures match (`squeeze()` in `mechanism.rs` is this for one contact).
- *Loss.* Each material has a loss factor: pressure is higher going in than coming out. Rolling
  resistance and the current to push a piece through a nip come out of that.
- *Friction.* A bristle per sample: it stretches with slip and lets go at `mu * pressure * area`.
  This holds a gripped piece still, which a smoothed Coulomb law does not. Separate static and
  sliding coefficients per material pair. Omni and mecanum treads grip along one direction only
  (the logic is in `mechanism.rs`).
- *Applying it.* Forces go on the piece and on the link that carries the roller (minus the torque
  about the roller's own axis, which goes to the shaft). Rollers are not engine bodies: their shafts
  and trains are `drives.rs`, integrated implicitly in the back-EMF.
- *Stability.* Forces are explicit, so the step must resolve the stiffest contact. The compiler
  works out each compliant pair's natural frequency from stiffness and the lighter mass, picks the
  step rate (floor 1 kHz), and warns when a pairing would need more than 4 kHz. If that proves too
  tight, the fallback is the implicit per-contact form already in `mechanism.rs` (`prepare` and
  `solve`: `hk = h(kh + c)`), run over the compliant contacts before the engine step.
- *Where stiffness comes from.* Published durometer first: the owner cannot press-test until the
  season, and most compliant wheels state a Shore A. The library turns durometer into a modulus
  (Gent's relation) and then into a foundation stiffness from the wheel's build (solid tread
  thickness, or spoke geometry for flex and compliant wheels). Every such value is marked
  *estimated from durometer*. Two things follow. Comparisons between wheels (35A against 60A on
  the same intake) are sound before absolute forces are, and the interface says so on every result
  that rests on an estimate. And a later press test replaces the estimate without touching the
  design: push the wheel `d` into a flat plate, read force `F`, and for radius `R` and width `w`,
  `F = k_A * w * (4/3) * sqrt(2R) * d^1.5` gives `k_A`.
- *Inflated pieces.* A ball held up by air pressure (ALGAE) has its own law: pressure on a sample in
  contact is the gauge pressure plus a small skin term, not a function of depth. The pack states
  the pressure as a range, and a sweep can run across it.

**D5. Everything else is rigid contact in MuJoCo.** Frame against field, robot against robot, a
piece on the carpet or on a polycarbonate plate. Elliptic friction cones, `impratio` 10, sleeping
on. A hollow piece (CORAL) is one smooth cylinder for ordinary contact plus a ring of boxes that
collides only with geometry flagged *enters a bore* (a REEF branch, a spear end effector). The ring
about doubles the cost of a step (measured), so it is built only for pieces within reach of a
flagged shape.

**D6. One electrical model for the whole robot.** A battery (open-circuit voltage from state of
charge, internal resistance, wiring resistance per branch), every motor's supply current, fixed
loads (controller, radio, compressor). Bus voltage feeds back to every motor each step. Motor
controllers apply stator and supply limits (`Motor::torque` in `drives.rs`). The main breaker and
branch breakers are thermal models that warn, and trip if the project asks. Brownout thresholds are
data, since the 2027 controller's differ from the roboRIO's. Reported per mechanism and in total:
current, peak, energy over a cycle and over a match, and an I²R heating estimate.

**D7. Collision shapes come from the CAD, with the user choosing how carefully.** Per link: *Box*
(one fitted box), *Simple* (a few fitted boxes and cylinders; tubing and plate are detected by their
proportions), *Detailed* (convex decomposition of the parts flagged as touching game pieces).
Fasteners and electronics are dropped by size and name before anything else. Surfaces used by the
foundation contact keep their triangles; they do not need to be convex.

**D8. CAD comes in as STEP, read by a sidecar built on OpenCascade. The front door is an Onshape
link.** The owner's CAD is in Onshape, and STEP carries the exact surfaces (true cylinders, so axes
and radii are read, not fitted), which Onshape's glTF export does not.

- *Paste a link.* The app asks Onshape to translate the assembly to STEP, downloads it, and fetches
  the bill of materials: each part's material, mass and vendor part number. The BOM fills materials
  and masses, and matches wheels and motors to the library by name ("4 in. Compliant Wheel",
  "WCP-1362 ... 45A", "Kraken X60"). Where the API serves the assembly definition (the owner's own
  documents), mates become proposed joints with their limits. On other teams' export-only
  documents it is refused (measured below), and joints come from the geometry.
- *The sidecar.* `tools/cadread` is a small C++ program, a port of `step2glb.py` and `step-axes.py`,
  whose calls map one to one onto the C++ API. It reads STEP and writes a *cadpack*: a GLB with the
  assembly tree, names and colors, plus a JSON of per-part facts (volume, centre of mass, bounding
  box, cylinder axes with radii and areas, planar faces). The app runs it as a child process with
  progress and a timeout, so a bad file cannot take the app down, and the OpenCascade build stays
  out of the Rust build. Until it exists (M3), the two Python scripts produce the same files.
- *Also.* A STEP file from disk, for CAD from anywhere else. Plain meshes (GLB, OBJ, STL) import
  without the per-part facts.
- *API budget.* The owner's plan is EDU Educator, capped at about 2,500 calls a year. An import
  costs about 10 (measured), re-import of an unchanged version costs one, and everything fetched is
  cached in the project. The app shows the calls used this year.
- *Keys.* Read from `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` or the system keychain. Never
  written to a project file or a log.

**D9. Markup survives a re-import.** Every part gets a stable id from its name path and instance
index, with a fingerprint (volume, centre of mass, bounding box) as the fallback when names change.
Links, joints, rollers and materials refer to those ids. Re-importing a changed CAD re-attaches
what matches, lists what did not, and never silently drops markup. Joint axes and roller geometry
picked from a face are stored as "this face of this part", so they move with the CAD.

**D10. A project is a folder of readable files.** `robot.json` (the description, schema-versioned,
with migrations from the first release), `cad/` (the cadpack cache), `runs/` (recordings). JSON
through serde with `float_roundtrip`, diffable and safe in git. Variants of a design are files side
by side.

**D11. The game is a pack, made in the same editor.** `assets/games/<name>/`: the field as a cadpack
with its collision shapes, the game pieces (shape, mass, materials, press curve), where pieces
start, feeders (a human player station is a feeder with a rate and a pose), scoring zones (a volume
plus a condition such as "a CORAL at rest here" or "a CORAL seated on this branch"), a points table
by match phase, and match timing. A field is rigged like a robot with no motors, so the importer,
the collision tools and the zone tools are the ones already built. Rules too odd for the format go
in a Rust hook compiled with the pack's name; no scripting language.

**D12. Controls without code.** Each motor group has a controller block: voltage, velocity
(PID and feedforward), position (PID, feedforward, trapezoid profile), or current. Gains have a
"from the model" button that fills kS, kG, kV and kA from the mechanism's own numbers. Named
*presets* hold a target per mechanism (Stow, Intake, L2, L3, L4). *Actions* bind a gamepad button to
presets and steps with waits (a time, a mechanism at its target, a sensor). Concept mode adds an
"ideal" controller that follows the fastest profile the motor allows.

**D13. Every motor and sensor sits on a device bus.** Each has an id and exposes what a real device
does (applied volts, position, velocity, stator and supply current, limit switches). D12's
controllers are one driver of the bus. Robot code is another (M9): a small library added to the
robot project that, in WPILib's desktop simulation, reads each vendor device's sim state (Phoenix 6
`TalonFXSimState`, REVLib's sim classes) and trades volts for position, velocity and bus voltage
with the app, plus WPILib's HAL WebSocket extension for DIO, analog, PWM and joysticks. WPILib 2027
is in alpha and vendor simulation has gaps, so this waits until after kickoff; the bus goes in at
M2 so nothing has to be reshaped for it.

**D14. The interface is egui inside Bevy.** `bevy_egui 0.43` on Bevy 0.20, with `egui_dock 0.21` for
panels and `egui_plot 0.37` for charts (that set resolves to one egui, 0.36; the newest dock and
plot releases want egui 0.37 and do not). An editor needs trees, property grids, drag-to-edit numbers,
docking and plots; egui has them and Bevy UI does not. The owner asked for an easy, quick
interface, so there is a theme pass (the 2026 app's fonts and palette) and the rules under "The
interface" are requirements, not polish. The match HUD stays Bevy UI, copied from the 2026 app.

**D15. The simulation runs on its own thread** and hands the app a snapshot per frame (poses, shaft
angles, contact patches, electrical state). Editing sends commands the other way. Sweeps run many
simulations at once with `rayon`, one engine state per thread.

**D16. Everything is recorded.** Each run keeps its state at every step in memory (and on disk on
request), so any run can be scrubbed, replayed in slow motion, and plotted afterwards. Export is
CSV first, WPILOG later so AdvantageScope can open it.

**D17. Tipping is real.** No tip damping. The view shows the centre of mass, the support polygon,
and the margin as mechanisms move.

**D18. Windows now, Linux for tests, nothing that shuts macOS out.** Installer and self-update by
Velopack, as the 2026 app. To keep a Mac build a matter of adding a release job: no Windows-only
crates or API calls outside one `platform` module, paths through the `dirs` crate, no DLSS, the
sidecar built with CMake, and a macOS `cargo check` in CI that reports but does not block. MuJoCo,
OpenCascade, Bevy and Velopack all support macOS.
SI inside; the interface shows inches and pounds or millimetres and kilograms, and number fields
accept units and arithmetic (`1.75in`, `30/11`, `6000rpm`).

**D19. Wheels are swappable.** Comparing wheel compliance is a main reason this tool exists. A
roller's geometry (axis, position, width) belongs to the design; what is on it is a reference to a
library wheel. Swapping is one pick from a list on the roller, on a selection of rollers, or on a
whole shaft, while the simulation runs. A swap to a different diameter shows the change in squeeze
before it is accepted. A swapped wheel is drawn from the library (colored by durometer), not from
the CAD part it replaced. On the Bench, wheel choice is a sweep axis like any number, and up to
four choices run side by side in one view with the same piece thrown at each. The library starts
with the common families at their published durometers (AndyMark compliant, WCP flex, Stealth and
solid treads, silicone and urethane tube rollers, mecanum and omni intake wheels), each entry
editable and each new wheel a short form.

## The robot description

Shown as an outline. The real schema is `crates/model`.

```
Robot
  parts[]        from the cadpack: id, name path, fingerprint, material, mass override
  links[]        a rigid group of parts (or blocks): name, parts, collision level, mass, centre of mass
  joints[]       hinge | slide | fixed | ball, parent and child link, axis (typed, or "this face"),
                 range, friction, spring. Couplings: stage rigging, gear, four-bar
  drives[]       motor type and count, ratio or stages, efficiency, stator and supply limits,
                 what it turns: a joint, or a train of roller shafts with a ratio each
  rollers[]      on a link and a shaft: axis, radius, width (from the CAD), wheel product or
                 material, kind (plain, omni, mecanum, belt)
  surfaces[]     parts that touch game pieces: material, pad or rigid, "enters a bore"
  drivetrain     swerve | tank | mecanum: module product, positions, wheel, ratio, limits
  mechanisms[]   a named group of the above with fidelity, pickup zones and holders (Concept),
                 controller, presets
  sensors[]      beam break (a ray), limit switch, encoder, distance, IMU, camera
  pneumatics[]   cylinder (bore, stroke, pressure), solenoid, tanks, compressor
  electrical     battery, wiring, breakers, fixed loads, brownout thresholds
  controls       presets, actions, gamepad bindings
```

The library (`assets/library/*.json`) supplies what a team would otherwise type: motors (the four
in `drivetrain.rs` with their rotor inertias, plus NEO, NEO Vortex, NEO 550, Minion and the rest
from the vendors' published curves), gearboxes, swerve modules, wheels and rollers (radius, tread
thickness, durometer, press curve, friction), materials (density, friction pairs, restitution,
stiffness and loss), game pieces of past seasons. Every library value carries where it came from
and whether it is measured or estimated.

## The interface

Five workspaces across the top. The 3D view is always present and the simulation can run in any of
them.

| Workspace | What you do there |
|---|---|
| **Build** | Concept robots from a palette: a drivetrain, then elevator, arm, wrist, intake, shooter, claw, climber. Each is a block with handles and a few numbers. Driveable from the first block. |
| **Rig** | CAD in. Group parts into links, click geometry to make joints and rollers, paint materials. |
| **Bench** | One mechanism on a stand, a feeder throwing pieces at it, live plots, sweeps. |
| **Field** | The game: practice, timed cycles, matches. |
| **Analyze** | Recordings, plots, A against B between variants, power budget, reports. |

Rules the interface is held to:

1. **Click the thing.** Selecting geometry offers what it can become. A cylinder offers "roller"
   and "joint axis here"; a flat face offers "slide along this" and "pad". No dialog asks for
   coordinates the CAD already holds.
2. **Nothing required is blank.** Every field starts from the CAD, the library or a sensible
   default, and shows which. A typed value is marked as typed and can be reset.
3. **Guess, then show the guess.** On import the app proposes links from the top-level
   subassemblies, rollers from coaxial cylinder parts with wheel-like proportions, swerve modules
   from four matching corner assemblies, and materials from part names and colors. Proposals are
   drawn as ghosts to accept, fix or dismiss, singly or all at once.
4. **Templates carry the hard parts.** "Elevator" asks for stages, rigging, spool diameter and
   motor, and builds the joints and couplings. The same for swerve, arm, wrist, telescope, four-bar,
   roller intake, flywheel, claw, turret, winch. A template can be laid over CAD later: the block
   becomes the link, and the numbers stay.
5. **Edit while it runs.** A number changed in the inspector reaches the running simulation at
   once. A change to the robot's structure recompiles in place and carries the state across where
   it can.
6. **A list of what is missing.** One panel names everything that stops or weakens the simulation
   ("Arm has no motor", "Intake rollers have no material: using 60A rubber"), and each line jumps to
   the fix.
7. **Undo everything, lose nothing.** Every edit is a command on the model with undo and redo.
   Autosave. Duplicate a design as a variant in one step.
8. **Batch work.** Multi-select, edit together, copy and paste a mechanism, mirror across the robot.
9. **See the physics.** View modes: by link, by material, collision shapes, contact pressure on
   rollers and pieces, force arrows, centre of mass and support polygon, current per motor.
   Slow motion, single step, and scrub back through any run.
10. **A command palette** (Ctrl+K) reaches every action by name.

The M3 gate times these: a stranger to the app rigs a four-mechanism robot from a STEP file in
under 30 minutes with the written guide.

## The first game pack: REEFSCAPE (2025)

- Field from FIRST's published field CAD, through the importer, with collision set per element.
- CORAL: a tube, 4 in. Schedule 40 PVC (4.5 in. outside), 11 7/8 in. long, rigid; the compliance is
  in whatever grips it. Take masses and tolerances from the game manual, not from this file.
- ALGAE: a 16.25 in. (plus or minus 1/4 in.) rubber playground ball, modelled as inflated (D4) with
  its pressure as a range. The owner asked for the published PSI range. The 2025 manual sets
  inflation by a diameter gauge, and no PSI figure was found for it, so the pack ships a low,
  middle and high pressure marked *estimated* until the owner names the source.
- REEF branches are flagged *enters a bore*. A CORAL counts as scored when seated on a branch and at
  rest; the trough, PROCESSOR and NET are zones.
- CORAL stations are feeders. CAGES hang from a ball joint, for climbs later.
- A second pack (2026 REBUILT's FUEL, whose foam curve and field colliders already exist in
  `rebuilt-sim-native/assets`) is built at M6 to prove the format holds more than one game, and to
  check the foundation contact against the 2026 simulator's tuned results.

**Kickoff weekend procedure** (written up as a guide at M6): import the field STEP, set collision
levels, flag bores and scoring zones, enter pieces from the manual with estimated materials, run
the press test on a real piece when one arrives, enter the points table. Target: a driveable field
the same day, measured pieces within the week.

## The test robot: 2910's Spectre

Jack in the Bot's 2025 robot, the world champion, released as a public Onshape document with a tech
binder. 1323 has no public CAD that could be found. Spectre is a useful test because it is not an
elevator: a pivoting, telescoping arm with a wristed end effector that handles both CORAL and
ALGAE, on SDS MK4i swerve.

- Document `f33ab032b00dc4b711aa86a6`, element `415a66cb7efdf4aba6199086` ("2025 Top Level
  Assembly"); take the workspace from the document's `defaultWorkspace`. A second assembly,
  "Scoring Positions", holds the arm in its scoring poses and gives joint ranges to check against.
- Release thread: https://www.chiefdelphi.com/t/2910-cad-and-tech-binder-release-2025/500310 (the
  tech binder is linked there). The team says in it that the intake's belts were changed to chain
  and gears after skipping under load, and that the intake's cross rod went from a 1/4 in. bolt to
  a 1/2 in. steel rod after bending twice. Both are loads the simulator should show as high.
- Its CAD is fetched with the owner's keys by `tools/fetch-fixture` into a cache outside the
  repository. It is another team's work: never commit it, its meshes or its cadpack. CI runs on a
  small generated robot; the Spectre gates are run locally and their results written here.
- It has no mates through the API, so it exercises the harder path: every joint from geometry.

## Layout

```
crates/
  model/     the project schema, ids and fingerprints, undo commands, migrations, validation. No engine
  library/   loads assets/library; unit parsing
  cad/       cadpack reading, re-import matching, proposals (links, rollers, modules), primitive
             fitting, convex decomposition
  sim/       headless. engine (MuJoCo), compile, drives (from drives.rs), electrical, contact
             (foundation, bristles, signed distances), pieces, concept (zones and holders), sensors,
             controllers, drivetrain (from drivetrain.rs), game (packs, scoring), record
  bridge/    the device bus's outside drivers (M9)
  app/       Bevy and egui: workspaces, view, gizmos, plots, HUD, updater
  cli/       run a scenario, a sweep or a benchmark with no window
tools/cadread/     the OpenCascade sidecar (C++)
assets/library/    motors, wheels, materials, modules, pieces
assets/games/      reefscape-2025/, rebuilt-2026/
spikes/            the measurements above
```

## What to take from the 2026 repos

| From | Take | For |
|---|---|---|
| `native/crates/sim/src/mech/drives.rs` | Whole file, parity rules removed | `sim::drives` |
| `native/crates/sim/src/drivetrain.rs` | `motor_spec`, `flat_out`, `supply_current`, the swerve solve | Motor library, `sim::drivetrain` |
| `native/crates/sim/src/mech/mechanism.rs` | `foam`, `squeeze`, `curvature`; the BVH and `closest_on_tri`; omni and mecanum friction; `prepare`/`solve` as the implicit fallback | `sim::contact` |
| `native/crates/sim/src/localize.rs` | Odometry, gyro and camera models | Sensors, M7 on |
| `native/crates/sim/src/nav.rs`, `ai.rs` | A* and the strategy-machine shape, not the 2026 logic | M7 |
| `native/crates/app/src/update.rs`, `main.rs`, `.github/workflows/release.yml` | Velopack and the release job (drop the DLSS and asset-sync steps) | M0 |
| `native/crates/app/src/settings.rs`, `camera.rs`, `materials.rs`, `bevel.rs`, `ui/mod.rs`, `ui/hud.rs` | Settings, camera rig, CAD look, palette and fonts, HUD | `app` |
| `2026/tools/step2glb.py`, `step-axes.py` | Development importer, then the spec for `cadread` | M0 to M3 |
| `2026/tools/extract-parts.mjs`, `tools/onshape-export.sh`, `cad/README.md` | Hardware filters, name matching (`like`), the Onshape translation calls and notes | `cad`, M3 |
| `2026/js/constants.js`, `native/assets/world.json` | FUEL's measured foam numbers, the 2026 field colliders | M1 check, M6 pack |
| `2026/tools/mechlab.mjs` | The `foam` and `pinch` scenes as cross-checks | M1 gate |

Leave behind: `crates/jsmath`, the parity harness, every `r*.rs` and `phys*.rs`, `physical.rs`,
`fuel.rs`, the match and rules files, `campaign`, the venue and atmosphere code, the neural driver.

## Milestones

Each ends with its gate passing in CI and a release the owner can install. Do them in order. M0 to
M6 are what kickoff needs.

### M0: foundation
Workspace and crates. CI: tests on Linux, build and Velopack release on Windows with the MuJoCo
library beside the exe. Bevy and egui shell with the five workspaces empty, the sim thread and
snapshot, recording and scrubbing. `model` with schema v1, ids, undo. The library with motors,
wheels and materials. `cli run`.
**Gate:** both spikes rewritten as Rust tests land within 5 % of the Python numbers. The installer
runs on Windows and updates itself.

### M1: the Bench
`sim::drives`, `sim::electrical`, `sim::contact` for rollers, pads and belts against analytic
pieces. Pieces: CORAL, ALGAE, FUEL. Bench scenes set from the inspector: roller over floor, two
rollers, roller against a backing plate, belt, with a feeder (pose and speed, fixed or random).
Live plots of current, voltage, squeeze force, slip. Pressure drawn on the roller.
Wheel swapping and the side-by-side view (D19), with the first wheel library.
**Gate:** closed-form checks (motor curve, battery sag, a block on a slope slips at `atan(mu)`, the
press-test formula, energy in against energy out). FUEL pressed between plates follows AndyMark's
measured curve, the one measured compression curve to hand. The two-wheel `pinch` scene throws FUEL
within 10 % of the 2026 simulator's speed. Swapping 35A for 60A on the same intake moves squeeze
force and current the right way and by a plausible ratio. The yaw behaviour from spike 1 is checked
against slow-motion footage of a real CORAL intake (public match video will do), and the slice
geometry fixed if it is wrong.
**Release:** tune a roller intake, swap its wheels, and watch the difference.

### M2: Concept robots
Joints, couplings and templates (elevator, arm, wrist, telescope, turret, four-bar, intake,
flywheel, claw, winch). Controllers, presets, actions, gamepad. The device bus. Swerve from
`drivetrain.rs`; tank after. Zones and holders. Springs and pneumatic cylinders. The Build
workspace. Power panel. Centre of mass and tipping view.
**Gate:** elevator time to height and arm holding current match hand calculation within 3 %. A
robot built from the palette in under 10 minutes drives, picks up and places.
**Release:** build an archetype and drive it on a flat floor.

### M3: CAD
`cadread` and its CI build. Import from an Onshape link (STEP translation, BOM, and mates where the
API serves them) and from a STEP file, with progress and the API call count. Parts tree. Links,
joints and rollers by clicking geometry. Proposals, including wheels and motors matched to the
library from BOM names. Materials and mass from the BOM, else density times volume, with a per-link
override and "scale to the weighed total". Collision levels. Re-import with markup kept. Laying a
template over CAD.
**Gate:** Spectre (see "The test robot") imports from its link in under 5 minutes plus Onshape's own
translation time, and the view stays above 60 fps. The 30-minute rigging test passes on it.
Re-import after moving, renaming and replacing parts keeps every piece of markup whose part still
exists, and lists the rest. One of the owner's own documents imports with its mates proposed as
joints.
**Release:** your CAD, moving under its own motors.

### M4: REEFSCAPE
The game pack format, zones and scoring, feeders, match timing. The Field workspace: practice and
timed cycles, with a cycle log (time per cycle, energy per cycle, pieces scored). The field made
with the M3 tools.
**Gate:** a Concept robot runs a full teleop of station-to-L4 cycles; the score and cycle log match
a hand count.
**Release:** drive an archetype through a 2025 match.

### M5: Full physics on a robot
Foundation contact against CAD surfaces. Soft pieces against rigid shapes. The hollow piece and
seating on a branch. Beam breaks and the other sensors. Wheels on real contact as a drivetrain
option. Sleeping, and full contact only near working mechanisms.
**Gate:** Spectre, rigged, picks a CORAL and an ALGAE off the floor and scores them with no assist.
Six robots on the field hold real time at 1 kHz on the owner's machine. A comparison against public
footage of the real robot (time from first touch to held, what happens to a CORAL that arrives
turned) is written into this file, with what matched and what did not. Until the owner can
press-test, that footage is the only check on Full mode's absolute numbers.
**Release:** the design-review tool.

### M6: sweeps, a second game, the kickoff guide
Sweeps on the Bench (choose variables, wheel choice among them, and outcomes; run in parallel; heat
map and table). Variants
side by side in Analyze. CSV export. The REBUILT pack. The kickoff guide, tested by building the
REBUILT pack from it.
**Gate:** a 3 by 3 by 3 sweep of squeeze, speed and limit over 20 random approaches each finishes in
under 2 minutes. The REBUILT pack is built from the guide without touching code.
**Release:** ready for kickoff.

### After kickoff
- **M7: matches.** Scripted alliance partners and opponents in Concept (cycle to the nearest piece,
  score, defend), six-robot matches, the HUD.
- **M8: press tests.** A form that takes force at a known squeeze for a wheel or a piece, replaces
  the durometer estimate, and reruns the affected results. When the owner has 2027 pieces in hand.
- **M9: robot code.** The bridge library (Java and Phoenix 6 first, as the owner's robots use), the
  HAL WebSocket client, then lock-step timing.
- **M10: WPILOG export, cameras and AprilTags, deformed drawing of squashed pieces.**

## Risks

- **A design tool that is confidently wrong is worse than none.** Every material and wheel is marked
  measured or estimated, results say which they rest on, and M1 and M5 each end with a comparison
  against something real. There are no press tests until the season, so until then Full mode is
  good for ranking options (this wheel against that one, more squeeze against less) and only
  indicative for absolute current and force. The interface has to say that plainly, not in a
  footnote.
- **Durometer is not stiffness.** Two 45A wheels with different spokes squash very differently.
  The estimate from durometer needs each wheel family's build, and flex and compliant wheels will be
  the least certain entries in the library. They are also the ones the owner most wants to compare,
  so they are the first to press-test at M8.
- **Explicit contact forces and stiff, light pieces.** Covered by the automatic step rate and the
  implicit fallback in D4. Watch for it with any piece under about 100 g.
- **Grip creep.** A piece held by friction in a rigid MuJoCo contact can slide slowly. The bristle
  model covers compliant grips; for rigid ones raise `impratio` or turn on `noslip` for that pair,
  and test a held piece for 30 s at M5.
- **Real STEP files.** Hundreds of megabytes, thousands of fasteners, odd units, missing names.
  The sidecar, the hardware filter and instancing are for this. Test on Spectre's real export at
  M3, not on a clean sample.
- **Thin plates and decomposition.** Polycarbonate guides are where pieces ride and where convex
  decomposition does worst. Plates are detected and built as boxes; piece-touching surfaces use
  triangles through the foundation path.
- **Full field in real time.** Measured at 0.43 to 0.63 s per simulated second with sleeping, on a
  slow box, with simple robots. CAD collision will cost more. The fallback is 500 Hz on the field
  and full contact only for the robot being studied.
- **egui looks like a tool.** Budget the theme pass in M0 and hold to the interface rules.
- **Version drift.** Pin MuJoCo and `mujoco-rs` together and upgrade them together. WPILib 2027 and
  vendor simulation are still moving, which is why M9 is last.
- **Licences.** MuJoCo is Apache 2.0. OpenCascade is LGPL 2.1 with an exception; the sidecar is a
  separate program. Ship both notices. Do not bundle other teams' CAD.

## The owner's answers (2026-11-20)

1. **MuJoCo: go ahead** (D2).
2. **CAD is in Onshape, and STEP is preferred** for its exact surfaces. So the import is "paste an
   Onshape link", translated to STEP (D8), and it is part of M3, not a later extra. The owner's API
   keys are in the environment as `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
3. **The test robot is another team's**, since the owner's 2025 robot is very simple: 1323 or 2910,
   whichever has public CAD and a parts list. That is 2910 (see "The test robot").
4. **No press tests for now** (the 2025 pieces are gone); perhaps during the season. Most compliant
   wheels publish a durometer, so stiffness starts from that (D4), and the press-test form moves to
   M8.
5. **Wheels must be easy to swap.** Comparing compliance is a main reason for the tool (D19).
6. **ALGAE from its published PSI range.** None was found (see the REEFSCAPE section), so this is
   the one answer still open.
7. **Windows only for now, with macOS left open** (D18).

Not answered, so the defaults stand: no AI matches before kickoff (M7).

Still needed from the owner:

- The source for ALGAE's PSI range, or agreement to use an estimated range.
- One of your own Onshape documents (any robot or mechanism with mates) for M3's mates-to-joints
  check.

## For the model building this

- Read this file, then `spikes/`, then the files in "What to take" before writing anything.
- Clone the two 2026 repos beside this one to copy from. Record the commit in each copied file's
  header.
- Fetch MuJoCo's release archive for the pinned version and set `MUJOCO_DYNAMIC_LINK_DIR`; do not
  rely on the binding's own download.
- A milestone's gate is its tests. Write the gate's tests first where they are closed-form.
- Keep physics in SI and `f64`. Keep every engine call inside `sim::engine`.
- When a number is not known (a mass, a stiffness, a threshold), put it in the library marked
  *estimated* with its source. Do not bury it in code.
- When a gate cannot be met as written, say so in this file with the measurement, and ask the owner
  before loosening it.
- Update this file at the end of each milestone: what was done, what was measured, what changed.
