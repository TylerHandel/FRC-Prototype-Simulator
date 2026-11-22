# Handoff: FRC Prototype Simulator

For the next person or AI picking this up. Read this first, then `PLAN.md` (the design, every
decision, and a "Progress" section with what was measured), then the docs it points to.

Last updated 2026-11-22. State: milestones **M0 to M6 are done** and released as **v0.1.3**.
Next up is **M7 (matches)**. Kickoff is early January 2027.

## What it is

A design tool for FRC robots, for team 8793. Build a robot from blocks or import it from Onshape
CAD, then see whether it picks up the game piece, how fast it cycles, what it draws from the
battery, and whether it tips. The owner's main interest is comparing compliant wheels on intakes.

- **Two levels of physics.** *Concept*: pickup zones grab pieces, with real motors and battery.
  *Full*: rollers really squeeze, grip and slip, pieces bump into the robot's real CAD shapes,
  ALGAE squashes like a ball. Each mechanism picks its own level.
- **Five workspaces:** Build, Rig CAD, Bench, Field, Analyze (plus Home).
- **Games are data** (game packs). REEFSCAPE 2025 and REBUILT 2026 are built in; teams add the
  2027 game at kickoff by following `docs/KICKOFF.md`, with no code.

## Where things are

- Repository: https://github.com/TylerHandel/FRC-Prototype-Simulator (public).
- Branches: work happens on `claude/compassionate-fermi-jjp9f9` (also the default branch).
  **`main` is the release branch**: every push to `main` builds and publishes a Windows release
  that installed copies update to by themselves. Only move `main` forward (fast-forward) to a
  commit whose CI has passed on Linux, macOS and Windows.
- Releases: https://github.com/TylerHandel/FRC-Prototype-Simulator/releases (v0.1.1, 0.1.2, 0.1.3).

| Folder | What's in it |
|---|---|
| `crates/model` | The robot description (links, joints, drives, rollers, mechanisms, sensors, controls), templates, undo, checks |
| `crates/library` | Motors, wheels, materials, pieces, swerve modules; durometer to stiffness; units |
| `crates/sim` | MuJoCo wrapper (`engine.rs`, the only file that names MuJoCo), the world, roller contact, motors, battery, swerve, the Bench, game packs |
| `crates/cad` | The cadpack format, the Onshape client, proposals from mates or two poses, collision hulls, re-import |
| `crates/app` | Bevy + egui app; `ui/` has one file per workspace; `shot.rs` is the screenshot harness |
| `crates/cli` | `fps`: check, bench, sweep, import, propose (offline), wheels |
| `assets/library`, `assets/games` | The data files (`tools/make-reefscape.py` writes the REEFSCAPE pack) |
| `tools/cadread` | The STEP reader (Python and OpenCascade, packaged with PyInstaller for release) |
| `docs/` | `KICKOFF.md` (game packs), `IMPORTING.md` (CAD), `BENCH.md`, `DEVELOPING.md` |

## Build, test, run

```bash
tools/fetch-mujoco.sh
```

```bash
cargo test --workspace --exclude app --release
```

```bash
cargo run -p app --release
```

The toolchain is pinned to Rust 1.97.1 (Bevy 0.20 needs it); MuJoCo 3.17.0 via `mujoco-rs` 7.1.0.
The milestone gates are tests in `crates/sim/tests/gates.rs`; run them with `-- --nocapture` to see
their numbers. Screenshots without a screen: `FPS_SHOT=... xvfb-run ...` (see `DEVELOPING.md` and
the env hooks at the top of `crates/app/src/shot.rs`). CI (`.github/workflows/ci.yml`) runs tests
on Linux, a macOS check, and a Windows job that does everything the release does except publish.

## Rules you must keep

1. **Onshape API keys** are in the environment as `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`.
   Never print, log or commit them. Each user enters their own; the app keeps them in the user's
   app data folder, never in a project. (Checked: no commit and no file of release v0.1.3 holds them.)
2. **Onshape API budget:** an education account gets about 2,500 calls a year. Never call the API
   from tests or CI; never poll faster than every 30 s. The app counts calls; about 40 used so far.
3. **Another team's CAD** (2910's Spectre) and anything taken from it must never be committed.
   `crates/sim/tests/spectre.rs` reads everything from a project named by `FPS_SPECTRE` and skips
   otherwise.
4. Commits end with the session's attribution lines; no model names in commits or code.
5. Don't open a pull request unless the owner asks.
6. When a gate can't be met as written, say so in `PLAN.md` with the measurement, and ask the
   owner before loosening it.

## What's done (details and numbers in PLAN.md "Progress")

- **M0:** workspace, MuJoCo engine, Velopack auto-update, Windows release pipeline.
- **M1:** the Bench. Foundation contact for rollers in real units; wheels swapped and compared
  side by side; closed-form checks (motor curve, battery, press formula, slope friction, energy).
- **M2:** Concept robots from templates, controllers, swerve, presets and buttons, pickup zones.
- **M3:** CAD import from an Onshape link (STEP + BOM + mates, or two poses); proposals for links,
  joints, rollers, wheels, motors, masses; click-to-mark; re-import keeps markup.
- **M4:** REEFSCAPE field, matches, scoring, cycle log; scripted station-to-L4 cycle test.
- **M5:** collision from CAD (convex hulls; thin parts as slabs, windows kept open), hollow CORAL
  that scores only by really landing, soft pieces in real units, beam breaks with
  run-until-sensor buttons. Gate met for CORAL on 2910's Spectre (local test).
- **M6:** sweeps over up to three things with heat map and CSV; game packs from folders with
  plain-word problem lists; the kickoff guide; REBUILT built from it; runs compared in Analyze.

## What's not done, and known gaps

- **M5 leftovers:** Spectre can't pick from the floor (its design), ALGAE on Spectre (mechanism not
  in its CAD), six robots at once (needs M7), comparison with match footage (needs a person),
  wheels on real contact as a drivetrain option.
- **M1 leftovers:** two-roller, backing-plate and belt Bench setups; FUEL throw compared with the
  2026 simulator; CORAL yaw against footage.
- **M2 leftovers:** four-bar, winch, pneumatics, tank drive.
- **M3 leftovers:** laying a template over CAD; the 30-minute rigging test and 60 fps check need a
  person and a GPU.
- **Game packs can't express:** points that change during a match (REBUILT HUB shifts), climbing
  points, ramps (BUMPs are drawn and driven through: the drivetrain model assumes flat carpet).
- **Owner asked, not yet built:** the import screen should say where the Onshape keys came from
  (environment variables or the saved file) and offer to forget them.
- The importer's motor and gear guesses are labelled "guessed: check it"; joint gearing is sized
  from the load, roller drives are not.

## Next: M7 and after (from PLAN.md)

- **M7, matches:** scripted alliance partners and opponents in Concept (cycle to the nearest
  piece, score, defend), six robots on the field, a HUD. The world holds one robot today
  (`World` in `crates/sim/src/world.rs`); this is the main change. The scripted drivers in
  `gates.rs` (`Driver`, `go`, `around`) are a starting point for the AI.
- **M8, press tests:** a form that replaces a wheel's durometer estimate with a measured
  force-at-squeeze, and reruns what depends on it.
- **M9, robot code:** the bridge library (Java, Phoenix 6), the HAL WebSocket client, lock-step timing.
- **M10:** WPILOG export, cameras and AprilTags, squashed pieces drawn deformed.

## Things that aren't in the repository

These lived only in the previous session's scratch space and won't exist in a new session:

- 2910's Spectre cadpack, BOM and scoring-pose export. Re-fetch only if needed, with the owner's
  keys (a few API calls), never commit them, and run the Spectre test from a scratch folder:
  `fps propose spectre.cadpack --bom bom.json --second scoring.cadpack --forward -Y --save <dir>`.
- The owner's 2026 robot import (8 API calls). It was used to tune the importer: turret, four
  swerve steering axes, intake pivot, hood and flywheels all proposed correctly; about 470 hulls; 157 lb.
- The owner's 2026 simulators are public: `TylerHandel/rebuilt-sim-native` (Rust) and
  `TylerHandel/rebuilt-sim-2026` (JS; `js/constants.js` has the REBUILT field from FIRST's CAD).

## Lessons that cost time

- MuJoCo body velocities read after `mj_step` are a step old: the engine splits each step
  (`begin_step`, our forces, `step`). Reading stale velocities capped the drivetrain at 3.25 m/s.
- Hollow pieces: the CORAL is round to the robot and a ring of walls only to the field (collision
  bits in `crates/sim/src/piece.rs`); a 16-sided tube caught on intake ramps.
- Concept pickups hold a piece that enters a running zone, or a falling piece passes straight through.
- Scripted drivers must raise before closing on the REEF and creep the last half metre; a held
  piece is only held by friction.
- Published motor constants aren't consistent (Kraken X60: kt is 1.4 % over 1/kv); the battery
  pays for work at the larger of the two.
