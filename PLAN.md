# FRC Prototype Simulator: plan

Written 2026-11-20 for the model that builds it; progress added 2026-11-21. The owner reads it too, so it says what is decided,
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
4. **CAD comes from an Onshape link, as STEP.** The app proposes links, rollers, wheels and motors
   from the geometry and the bill of materials; you accept or fix them. Markup survives a
   re-import. The test robot is 2910's 2025 Spectre.
5. **Wheels swap in one pick.** Stiffness starts from each wheel's published durometer, so wheels
   can be compared on the same intake before anything is press-tested.
6. **The game is data.** 2025 REEFSCAPE is the first game pack and the test bed. A game pack is made
   in the same editor as a robot, so the 2027 field can be brought in during kickoff weekend.
7. **Kickoff is early January 2027.** The order of work puts what kickoff needs first: the bench,
   Concept robots, CAD rigging, and making a game pack. AI matches and running real robot code come
   after.

## Progress (2026-11-21: M0 to M6)

Built in one pass through M0 to M4. Every gate that can run without a person runs in CI on Linux
and on Windows. What each milestone has, and what is left:

| | State | Left to do |
|---|---|---|
| M0 | Done | The owner installs a release and sees it update itself (needs a push to `main`) |
| M1 | Done, one Bench scene | Two-roller, backing-plate and belt scenes; two gate checks (below) |
| M2 | Done | Four-bar, winch, pneumatics, tank drive; the 10-minute palette test (needs a person) |
| M3 | Done | Laying a template over CAD; the 30-minute rigging test and 60 fps check (need a person and a GPU) |
| M4 | Done, simplified field | The field made with the M3 tools instead of a script; energy per cycle in the log |
| M5 | Done, gate met for CORAL | ALGAE on Spectre, wheels on real contact, six robots at once (with M7), the footage comparison (needs a person) |
| M6 | Done | Wheel stiffness from press tests (M8) will sharpen the sweeps' absolute numbers |

**Measured at the gates** (`cargo test -p sim --release --test gates -- --nocapture`):

- M0 lift (spike B): 1.084 s to 1 m, 1.5 mm over. The spike: 1.088 s, 1.5 mm.
- M1 intake: a light squeeze takes a CORAL in 0.49 s at 25 A mean; a firm one in 0.46 s at 31 A,
  squeeze 63 N against 158 N. Entry time and peak squeeze agree to 0.3 % across 1, 0.5 and
  0.25 ms steps. 60A wheels squeeze 1.9 times harder than 35A on the same intake (453 N, 241 N).
- M1 closed-form: Kraken constants, battery sag and breaker trip, the press-test formula on a flat
  box, and FUEL between plates against AndyMark's curve all pass. A block on a slope (mu 0.6, the
  world's solver settings) creeps 0.15 mm in a second 2 degrees below atan(mu) and slides 279 mm
  2 degrees above. Energy: for every motor in the library, battery power covers work plus copper
  heat at every speed and voltage. (It didn't at first: the Kraken X60's published stall torque and
  free speed give kt = 1.014/kv, so the motor made 1.4 % more work than it was paid for. The
  battery now pays at the bigger of the two.)
- M2: a Concept intake picks a CORAL up 0.48 s after starting; the swerve base reaches 4.57 m/s
  (free speed 4.6), sags the battery to 9.3 V at 175 A, and stops.
- M4: a scripted elevator robot runs four CORAL STATION to L4 cycles; the score (20) and the
  cycle log match the hand count. Cycles of 8 to 14 s under the scripted driver.
- M3 on real CAD: 2910's Spectre (1,957 parts) reads in under 3 minutes, and two poses give the
  pivot (68 degrees), both 0.505 m slides and the wrist. The owner's 2026 robot, from its mates:
  turret (0 to 360 degrees), four steering axes, intake pivot with its limits, the hood and the
  flywheels, 157 lb, and it drives in the simulator. 8 API calls for that import.

**M5, measured** (`crates/sim/tests/gates.rs`, and `crates/sim/tests/spectre.rs` run locally):

- CAD collision: every imported part is a convex hull; thin parts (polycarbonate, sheet metal,
  tube and channel walls) are flat slabs, and a slab with windows or lightening holes is cut until
  each piece is mostly material. The owner's robot: about 470 hulls; Spectre: about 650.
- CORAL is hollow to the field (a ring of walls), so a branch goes up inside it, and solid to the
  robot. The branches have their L4 stop and L2/L3 arms. A piece that comes to rest within 12 cm
  of a seat, still for 0.3 s, is scored where it lies. Test: CORAL dropped over an L4 top slides to
  the stop and scores; one on an L3 arm scores.
- Soft pieces meet robot shapes through their own law, in newtons: an ALGAE on a plate sinks
  0.75 mm, where its 1 psi over the flattened patch holds its 6.67 N exactly. It bounces like a
  playground ball. A claw with wheels 14 in. apart pulls an ALGAE in at 4 V and holds it on 1.5 V
  (2 mm in 1.5 s).
- Beam breaks and limit switches; a button's run can stop on one. Test: an intake stops itself
  0.48 s after starting, holding its CORAL.
- **The gate, CORAL:** on 2910's Spectre as the importer rigged it, a CORAL fed lengthwise into
  the end effector is pulled in by its compliant wheels until a beam break trips (0.2 s), held on
  a speed loop, carried to the REEF, raised, brought in, pushed out, and scored on L4 by physics
  alone, on both branches of the face. Spectre in Full on the field runs about 8 times faster
  than real time at 1 kHz on a 4-core cloud machine.
- Two things a real driver does that the script had to learn: raise before closing on the REEF
  (the end effector sweeps forward as it rises), and take the last half metre slowly (the CORAL is
  held by the wheels' edges in this design, about 40 N of squeeze).

**M6, measured:**

- Sweeps on the Bench: up to three of speed, roller volts, current limit, piece turn, roller
  height and wheel; every combination's throws in parallel; a table, a heat map of any outcome
  over the first two, and CSV. **Gate:** a 3 x 3 x 3 sweep of squeeze, speed and limit, 20 random
  approaches each (540 throws), takes 3.5 s on 4 threads (allowed: 2 minutes); lower rollers
  squeeze harder (206, 308, 398 N peak).
- Game packs from folders ("Game packs" beside the projects), each with its problems listed in
  plain words; a new pack from a working template; `mirror`, `grid`, a goal's `exit` and a
  feeder's `stock` so a game is written once by hand.
- **Gate:** `docs/KICKOFF.md`, then the REBUILT (2026) pack written by following it, data only,
  from rebuilt-sim-2026's field dimensions. It reads with nothing to fix; a FUEL dropped in a HUB
  scores and comes back out; 504 FUEL on the field run at 2.4 times real time.
- Runs kept and compared in Analyze: plots overlaid, numbers side by side.
- Not modelled in game packs yet: points that change during a match (REBUILT's HUB shifts),
  climbing, and BUMPs as real ramps (the drivetrain drives on flat carpet, so they're drawn and
  driven through).

**M5, not met as written:**

1. *Off the floor.* Spectre's end effector can't reach the floor (its wheels stay about 10 cm
   above a lying CORAL with the wrist fully turned, and the arm stops at horizontal), so its CORAL
   comes from a station. The test feeds it in, moving, as a chute would.
2. *ALGAE on Spectre.* Its scoring CAD has no ALGAE and its CORAL end effector can't take a 16 in.
   ball, so how it handles ALGAE isn't known from the CAD. ALGAE in Full is shown on a template
   claw instead.
3. *Six robots at 1 kHz in real time.* The world holds one robot; six come with M7 (matches).
4. *Comparison with footage, and wheels on real contact as a drivetrain option:* not done.

**Gates not met as written** (the owner's call whether to loosen them):

1. M0 asks the intake spike to land within 5 % too. The contact law changed after the spike: each
   slice now follows the foundation law with both surfaces' curvature (force grows as depth to
   the 1.5), where the spike used a straight spring per slice, so that a press test on a flat
   plate means the same stiffness on a curved piece (`a_wheel_on_a_flat_box_matches_the_press_formula`). So the intake is checked by behaviour (more squeeze, more force
   and current; harder wheels squeeze harder; step-size convergence) instead of against the spike.
2. M1's FUEL `pinch` against the 2026 simulator and CORAL yaw against match footage are not done
   yet (the first needs the two-wheel scene, the second a person with the footage).

**Changed from the plan:**

- The CAD reader is OpenCascade from Python (`cadquery-ocp-novtk`), packaged with PyInstaller into
  a folder beside the app (418 MB), not a C++ sidecar. The same output, much less code; nobody
  installs Python.
- Onshape keys are kept in the app's data folder (`onshape-keys.json`), not the system keychain.
  Each user enters their own; they never go into a project.
- Rust 1.97.1 (Bevy 0.20 needs it), MuJoCo 3.17.0 through `mujoco-rs` 7.1.0.
- The engine splits each step in two (`mj_step1`, then our forces, then `mj_step2`) because body
  velocities read after `mj_step` are a step old; that stale read capped the drivetrain at 3.25 m/s.
- The intake template is over the bumper, with a ramp, so a CORAL rides up rather than jamming.
- A running Concept pickup holds a piece that enters it while it's drawn in, so a CORAL dropped
  from a station doesn't fall through a claw.
- The Bench tests the robot's own mechanism against a stream of pieces (distance, speed, yaw and
  offset, each with a spread) rather than separate fixture scenes.
- The import also: treats four upright hinges low on the chassis as swerve steering, merges
  anything spinning close to its own axis (rollers, pulleys) into its carrier, fixes groups with no
  joint to what they touch, and weighs reference bodies (clearance boxes, a camera's field-of-view
  solid) as nothing.
- CI has a Windows job that does everything the release does except publish, including running
  the packaged CAD reader on a STEP and the packaged app's `--self-test`.
- Two-pose import: a joint on a moving parent (a wrist on a telescope) had its pivot placed by the
  parent's move twice. Fixed; pivots come from the exact formula.
- The importer sizes its guessed joint gearing from what the joint carries (mass, reach, a
  telescope's travel): Spectre's arm pivot went from 1 motor at 60:1, which couldn't lift it, to
  2 at 139:1.

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
| STEP translation | Served. Onshape took about 2 minutes to translate; the file is 176 MB and downloaded in 24 s. |

The BOM names what the app needs to match: 16 Kraken X60 and 4 Kraken X44, SDS MK4i modules with
4 in. nitrile treads, two "4 in. OD x 1 in. Wide Straight Flex Wheel (... 45A) (WCP-1362)" and four
"AM-3562 3/8 in. Hex Bore 4 in. Compliant Wheel", with materials such as 6061, 7075, polycarbonate,
Delrin and printed Onyx. Hardware is 209 rows and more than half the part count. A clean import is
8 calls: the document, the BOM, the assembly definition, the translation request, three polls and
the download. Poll with a growing delay, since every poll counts against the year.

`step-axes.py` over that STEP, unfiltered: 157 s and 1.5 GB of memory for 1,957 part instances (483
distinct parts; 1,092 instances are hardware by name, 865 are not). What it shows for the importer:

- The tree gives the links. Under the top level are five subassemblies: Drivebase (838 parts),
  Pivot (269), Arm (434, itself split into Stage 0, 1 and 2), Intake & Wrist (299), Climber (117).
- Wheels are found exactly. The four intake compliant wheels and the two climber flex wheels each
  come back with a 2.00 in. radius on a tilted axis (the arm is not square to the frame), and the
  swerve treads at 1.98 in.
- Geometry alone over-proposes. "A round part that fills its bounding box about its axis" matched
  83 parts, nearly all shafts, columns, plugs and spacers, and only among axis-aligned parts. So
  roller proposals rank by name and BOM first (wheel, roller, tread, a vendor wheel number) and use
  geometry to confirm and to measure.
- Names differ between the two sources. The BOM row is "AM-3562 3/8 in. Hex Bore 4 in. Compliant
  Wheel"; the STEP part is "4IN Compliant Wheel FINAL Overmold". Matching BOM rows to STEP parts
  goes through the BOM's occurrence paths, not through names.
- Dropping hardware before meshing and measuring matters: it is 56 % of the instances.

**6. Mates, where Onshape serves them** (`spikes/onshape_mates.py`). One call returns every mate at
every level of an assembly, each with the two parts it joins, its type, its axis and origin, and its
limits where set. Which documents serve it:

| Document | Assembly definition |
|---|---|
| The owner's own (8793's 2026 robot) | Served: 4.1 MB in 26 s |
| Another team's fully public document (1706's 2026 robot) | Served: 2.2 MB in 7 s |
| Another team's link-shared, export-only document (2910's Spectre) | Refused, as are its feature list, mate values and configurations |

Turning the owner's robot's mates into links and joints (join parts across Fastened mates and mate
groups, treat a sub-assembly Onshape marks rigid as one link, and keep every other mate between two
different links):

- 2,071 parts become one 1,286-part chassis, a 156-part turret, four 42-part swerve steering
  modules, the shooter's moving pieces, and small groups.
- 25 joints, among them the turret's hinge with its 0 to 360 degree limit, the four steering axes
  at (plus or minus 0.254, plus or minus 0.254) m pointing straight up, the shooter's hinges, the
  intake's, and four bumper plungers as slides.
- 81 parts end up alone: parts nobody mated to anything. The importer attaches each to the nearest
  link in its sub-assembly and says so.
- 1706's robot shows the other untidiness: wheels held on shafts by a slider and a planar mate
  together, which add up to "fixed" but are neither alone. Mates between the same two links have
  to be combined before they are read. So mates are strong evidence, shown as proposals like
  everything else, not imported as truth.

**7. Two poses, where it does not** (`spikes/step_tree.py`, `spikes/pose_diff.py`). Spectre's
document has a second assembly, "Scoring Positions": the same robot at its L2 scoring pose, beside a
REEF with a CORAL on it. Exported as STEP (3 more API calls, 179 MB) and compared with the stowed
export, part by part:

- Parts with names that occur once (486 of 1,957) are matched directly and grouped by the rigid
  move they made between the poses. The rest are repeats (the same screw eight times), which the
  two exports list in no dependable order; each is given to the link whose move lands it on one of
  its twins. All 1,468 were placed.
- Result: 1,953 of 1,957 parts fall into six links. The base (1,046 parts). Arm stage 0 (313), which
  turned 68.0 degrees about the robot's x axis through (0, 0.260, 0.324) m: the pivot, found
  exactly. Stage 1 (147) and stage 2 (40), each of which slid 19.87 in. along the arm against the
  stage before, which also shows the rigging is one-to-one. The intake and wrist (301), which
  turned. And 106 swerve module parts that shifted 0.25 in., a configuration difference to dismiss.
- What it cannot do alone: it picked the wrong parent for the wrist (the link it turned least
  against, the base, when it rides on stage 2), so parents come from which links touch at the
  axis. A joint that did not move between the poses is not seen. And the travel seen is a lower
  bound, not a limit.

Reading placements from a 176 MB STEP takes 105 s here; the comparison itself is instant.

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
  "WCP-1362 ... 45A", "Kraken X60"). It also asks for the assembly definition, which holds the
  mates (D20).
- *The sidecar.* `tools/cadread` is a small C++ program, a port of `step2glb.py` and `step-axes.py`,
  whose calls map one to one onto the C++ API. It reads STEP and writes a *cadpack*: a GLB with the
  assembly tree, names and colors, plus a JSON of per-part facts (volume, centre of mass, bounding
  box, cylinder axes with radii and areas, planar faces). The app runs it as a child process with
  progress and a timeout, so a bad file cannot take the app down, and the OpenCascade build stays
  out of the Rust build. Until it exists (M3), the two Python scripts produce the same files.
- *Also.* A STEP file from disk, for CAD from anywhere else. Plain meshes (GLB, OBJ, STL) import
  without the per-part facts.
- *API budget.* The owner's plan is EDU Educator, capped at about 2,500 calls a year. An import
  costs about 8 (measured), re-import of an unchanged version costs one, and everything fetched is
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

**D20. Links and joints are proposed from three sources, best first.** All three produce the same
thing, a proposal the user accepts, fixes or dismisses, so the rest of the app does not care which
ran. All three were tried on real robots (measurements 6 and 7).

1. *Mates.* Onshape's assembly definition lists every mate. It is served for the owner's own
   documents and for fully public ones, in one API call. Fastened mates and mate groups make the
   links; revolute, slider and cylindrical mates between links make hinges and slides, with the
   mate's own axis and limits. Mates between the same two links are combined first. Parts nobody
   mated join the nearest link in their sub-assembly, flagged. A joint with a few small parts on
   one side (a wheel on a shaft, a roller in its bearings) is proposed as a roller, not as a
   mechanism joint.
2. *Two poses.* For a document that refuses its mates but has the robot in more than one pose (a
   second assembly, or a configuration), export both and compare. Parts that made the same move are
   a link; one link's move against its neighbour is a hinge (axis, point, angle) or a slide
   (direction, travel). More poses find more joints and widen the travel seen. The app offers this
   whenever a document has another assembly containing the same top-level sub-assembly.
3. *Geometry and a click.* Always available. Links from the sub-assembly tree; an axis from a
   picked cylinder face shared by two links, or a direction from a picked flat or edge; limits by
   dragging the joint to its stops.

For the owner's own CAD, the first is the normal path and costs nothing extra, so mating the real
freedoms in Onshape (a revolute on the arm, a slider on each stage, limits set) is the fastest way
to a rigged robot. The import guide says so.

**D21. The app updates itself, as the 2026 native simulator does.** The same mechanism, copied from
`rebuilt-sim-native` (`crates/app/src/update.rs`, `main.rs`, `.github/workflows/release.yml`):

- Every push to `main` builds the Windows app, packs it with Velopack's `vpk` and publishes a
  GitHub Release (version `0.1.<run number>`), with a delta from the last one. The first install is
  one `Setup.exe` from the latest release.
- The installed app checks at start and every minute, in a background thread, reading the latest
  release's files directly (not the GitHub API, so the check never hits a rate limit). It downloads
  a new version in the background and shows the state in the corner ("v0.1.212, up to date").
- Once a version is downloaded, the app restarts into it by itself after a 5 s notice when nothing
  would be lost: on the Home screen, or with a project open and saved and no run or sweep going. A
  click skips the wait. Otherwise a "Restart to update" button waits. Closing the app installs it
  too. This is the 2026 rule (wait for the match to end) restated for an editor.
- It autosaves before restarting and reopens the same project and workspace after.
- The `velopack` crate and the `vpk` tool are pinned to the same version. The repository must stay
  public, or the download needs a token.
- It is in M0, and M0's gate includes the owner watching one installed copy update itself.

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
| **Rig** | CAD in from an Onshape link or a STEP file. Group parts into links, click geometry to make joints and rollers, paint materials. |
| **Bench** | One mechanism on a stand, a feeder throwing pieces at it, wheels swapped and compared side by side, live plots, sweeps. |
| **Field** | The game: practice, timed cycles, matches. |
| **Analyze** | Recordings, plots, A against B between variants, power budget, reports. |

Rules the interface is held to:

1. **Click the thing.** Selecting geometry offers what it can become. A cylinder offers "roller"
   and "joint axis here"; a flat face offers "slide along this" and "pad". No dialog asks for
   coordinates the CAD already holds.
2. **Nothing required is blank.** Every field starts from the CAD, the library or a sensible
   default, and shows which. A typed value is marked as typed and can be reset.
3. **Guess, then show the guess.** On import the app proposes links from the subassembly tree,
   wheels and rollers from BOM and part names confirmed by their cylinder geometry, motors from
   the BOM, swerve modules from four matching corner assemblies, and materials from the BOM, then
   from part names and colors. Proposals are
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
  its pressure as a range. The 2025 manual sets inflation by a diameter gauge and gives no PSI,
  so the pack ships a low, middle and high pressure marked *estimated* (the owner agreed), chosen
  so the ball's squash under its own weight and in a hand press looks right, and open to edit.
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
  Assembly"); take the workspace from the document's `defaultWorkspace`.
- Release thread: https://www.chiefdelphi.com/t/2910-cad-and-tech-binder-release-2025/500310 (the
  tech binder is linked there). The team says in it that the intake's belts were changed to chain
  and gears after skipping under load, and that the intake's cross rod went from a 1/4 in. bolt to
  a 1/2 in. steel rod after bending twice. Both are loads the simulator should show as high.
- Its CAD is fetched with the owner's keys by `tools/fetch-fixture` into a cache outside the
  repository. It is another team's work: never commit it, its meshes or its cadpack. CI runs on a
  small generated robot; the Spectre gates are run locally and their results written here.
- It has no mates through the API, so it exercises the harder paths: joints from two poses (the
  "Scoring Positions" assembly, element `54598a0ed9dd23f1933582ac`, is the robot at L2 beside a
  REEF with a CORAL on it) and from geometry. That assembly also shows where the real robot holds a
  CORAL when it scores, which is the pose M5's gate has to reach.

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
**Gate:** both physics spikes rewritten as Rust tests land within 5 % of the Python numbers. The
installer runs on Windows, and the owner sees an installed copy find, download and restart into the
next release by itself (D21).

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
`cadread` and its CI build. Import from an Onshape link (STEP translation, BOM, mates) and from a
STEP file, with progress and the API call count. Links and joints proposed from mates, from two
poses, and from geometry (D20). Parts tree. Links,
joints and rollers by clicking geometry. Proposals, including wheels and motors matched to the
library from BOM names. Materials and mass from the BOM, else density times volume, with a per-link
override and "scale to the weighed total". Collision levels. Re-import with markup kept. Laying a
template over CAD.
**Gate:** Spectre (see "The test robot") imports from its link in under 5 minutes plus Onshape's own
translation time, and the view stays above 60 fps. The 30-minute rigging test passes on it.
Re-import after moving, renaming and replacing parts keeps every piece of markup whose part still
exists, and lists the rest. From mates: the owner's 2026 robot (document `e34c8122db6474f80cc7e909`)
imports with its turret, four steering axes and intake pivot proposed correctly, limits included.
From two poses: Spectre's pivot and both arm slides are proposed from its stowed and scoring
exports, and the wrist once parents are taken from contact.
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
6. **ALGAE at an estimated PSI.** No published figure was found, and the owner agreed to an
   estimate (see the REEFSCAPE section).
7. **Windows only for now, with macOS left open** (D18).
8. **Auto-update is required**, as in the Rust version of the 2026 simulator (D21).
9. **Mates: work out a solution before building.** Done (D20, measurements 6 and 7): mates from the
   API for the owner's own and public documents, two-pose comparison for locked ones, and clicking
   geometry always.

Not answered, so the default stands: no AI matches before kickoff (M7).

Nothing is waiting on the owner. Two things would help when there is time: mating the real freedoms
(with limits) in your own Onshape assemblies, and a press test of the wheels you most want to
compare.

## For the model building this

- Read this file, then `spikes/`, then the files in "What to take" before writing anything.
- Clone the two 2026 repos beside this one to copy from. Record the commit in each copied file's
  header.
- Fetch MuJoCo's release archive for the pinned version and set `MUJOCO_DYNAMIC_LINK_DIR`; do not
  rely on the binding's own download.
- Onshape: the owner's keys are in `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`. Never print,
  log or commit them. The account is capped at about 2,500 calls a year, so cache every response,
  never call the API from a test or from CI, and never poll faster than every 30 s.
- Spectre is another team's CAD. Keep it and everything derived from it out of the repository.
- A milestone's gate is its tests. Write the gate's tests first where they are closed-form.
- Keep physics in SI and `f64`. Keep every engine call inside `sim::engine`.
- When a number is not known (a mass, a stiffness, a threshold), put it in the library marked
  *estimated* with its source. Do not bury it in code.
- When a gate cannot be met as written, say so in this file with the measurement, and ask the owner
  before loosening it.
- Update this file at the end of each milestone: what was done, what was measured, what changed.
