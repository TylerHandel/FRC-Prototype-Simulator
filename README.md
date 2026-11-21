# FRC Prototype Simulator

A design tool for FRC robots. Build a robot from blocks or bring it in from CAD, then see whether
it picks the game piece up, how fast it cycles, what it draws from the battery, and whether it tips.
Compare wheels on an intake before you cut anything.

## Install

1. Go to the [latest release](https://github.com/TylerHandel/FRC-Prototype-Simulator/releases/latest).
2. Download `FrcPrototypeSim-win-Setup.exe` and run it.
3. That's all. The app keeps itself up to date: when a new version is out it downloads it in the
   background and restarts into it when your work is saved (or when you close it).

Windows only for now.

## Your first ten minutes

1. **Home**: pick a starting point. "Elevator, claw and ground intake" is a REEFSCAPE archetype
   ready to drive.
2. **Build**: drive it with W A S D and the arrow keys (or a gamepad). Click a mechanism in the
   list or in the 3D view to change its numbers on the right: the robot changes while it runs.
   Click one of its motors to try it by hand: hold **forward** or **back**, or slide **Go to**.
3. **Bench**: the intake on a stand with CORAL driven onto it. Pick wheels to compare under
   "Compare wheels" and press **Run the comparison**: each wheel's current, squeeze and speed,
   side by side.
4. **Field**: press **Start a match**. Drive to a CORAL STATION to get a CORAL, take it to the
   REEF, let go near a branch to score. The cycle log shows your cycle times.
5. **Analyze**: plots of the run, the power budget, and a CSV export.

Every number field takes units and sums: `1.75in`, `30mm`, `2ft + 3in`, `30/11`.
Ctrl+Z undoes anything. Ctrl+K finds any action by name. Projects save themselves.

## Bring in your CAD

Rig CAD, then paste your Onshape assembly's link. Full directions, including getting your API
keys: [docs/IMPORTING.md](docs/IMPORTING.md).

## Two levels of physics

- **Concept**: for the first week. A mechanism picks a piece up when the piece is in its pickup
  zone with its rollers running. Joints, motors and the battery are fully simulated.
- **Full**: for design review. Rollers really squeeze, grip and slip on the piece; the squeeze
  sets the current. Each mechanism has its own setting.

Wheel stiffness comes from each wheel's published durometer until you measure it, so trust the
ranking between wheels more than the exact amps and newtons. [docs/BENCH.md](docs/BENCH.md) says
more.

## For developers

[docs/DEVELOPING.md](docs/DEVELOPING.md): building, testing, releasing. [PLAN.md](PLAN.md): the
design, what was measured, and what's left.
