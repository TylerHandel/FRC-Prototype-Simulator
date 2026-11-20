# Developing

## Build and run

```bash
tools/fetch-mujoco.sh
```

```bash
cargo run -p app --release
```

On Windows, fetch MuJoCo with `powershell -File tools\fetch-mujoco.ps1` instead (it also puts
`mujoco.dll` next to the builds). The toolchain is pinned in `rust-toolchain.toml` (Bevy 0.20 needs
Rust 1.97.1). Linux needs `libudev-dev` and `libxkbcommon-dev` for the app.

## Test

```bash
cargo test --workspace --exclude app --release
```

The milestone gates are `crates/sim/tests/gates.rs` (the geared lift against spike B, an intake
taking a CORAL, step-size convergence, concept pickup, the drivetrain and battery, the field) and
`crates/cad/tests/import.rs` (the importer on a small generated robot).

## The command line

```bash
cargo run -p cli --release -- bench "path/to/project" am-compliant-4in-35a am-compliant-4in-60a
```

`fps check`, `fps bench`, `fps sweep`, `fps import` and `fps wheels`: see `crates/cli/src/main.rs`.

## The CAD reader

`tools/cadread/cadread.py` reads STEP with OpenCascade (`pip install cadquery-ocp`). The app runs it
as a separate program: in a release, the PyInstaller build in `cadread/` next to the exe; developing,
`python3 tools/cadread/cadread.py` (set `FPS_CADREAD_PYTHON` to a Python that has `cadquery-ocp`).
`tools/cadread/make_test_step.py` writes the small test robot the importer's tests use.

Never commit another team's CAD or anything made from it.

## Onshape

Your keys in `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY`. Education accounts get about 2,500 API
calls a year: never call the API from a test or CI, and never poll faster than every 30 s.

## Releasing

Every push to `main` runs `.github/workflows/release.yml`: tests, the Windows build, the packaged
CAD reader, and a Velopack release (`0.1.<run number>`). Installed copies find it within a minute.
The `velopack` crate and the `vpk` tool are pinned to the same version (1.2.161); change both together.

## Screenshots without a screen

```bash
FPS_SHOT=shot.png FPS_SHOT_WS=bench xvfb-run -s "-screen 0 1600x950x24" cargo run -p app --release
```
