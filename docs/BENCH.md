# The Bench: tuning an intake

The Bench holds the robot still and sends game pieces at it as if it drove onto them, at the speed
you set. It is where you choose wheels, squeeze and current limits.

## Comparing wheels

1. Open your project and go to **Bench**. The 3D view throws a piece at the intake every couple of
   seconds; green lines are rollers gripping, orange is slipping.
2. Under **Compare wheels**, add up to four wheels.
3. Press **Run the comparison**. Each wheel runs the same throw, at the same time, on every roller.
4. The right panel shows for each: did the piece come in and how fast, the rollers' current, the
   squeeze force, how far the rollers slowed, how much of the time they slipped. The plots below show
   them over time.

## Changing the squeeze

The squeeze is how much smaller the gap is than the piece. For the template intake it's set by
**Roller height** (Build, select the intake); the inspector shows the squeeze it gives on each game
piece as you change it.

More squeeze grips harder but costs current, slows the rollers, and pushes the piece into the ramp
harder (more friction to overcome). The Bench shows where that stops being worth it.

## Sweeps

Pick a number (driving speed, roller volts, stator limit, how the piece is turned, roller height),
type the values to try, and how many throws each. Each throw takes a random turn and offset within the
"give or take" you set. The table shows how many came in at each value.

## How much to trust the numbers

- **Wheel stiffness** comes from the wheel's published durometer (Shore A) and how it's built (solid,
  spoked, flex). Two 45A wheels with different spokes squash differently, so the estimates for
  compliant and flex wheels are the least certain numbers in the library. They're marked "estimated"
  everywhere. Comparisons between wheels are sounder than absolute amps and newtons.
- **A press test replaces an estimate.** Push a wheel a known distance into a flat plate on a
  bathroom scale and note the force. Add it to the wheel (a press-test form is planned; until then
  the wheel's `k_area` in your library file): `k_area = F / (w * 4/3 * sqrt(2R) * d^1.5)`.
- **ALGAE's pressure** isn't published: the 2025 manual sizes it with a gauge. The library has a low,
  middle and high estimate.
- **CORAL** is rigid PVC: all of the squeeze is in the wheels.
