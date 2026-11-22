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

On **Home**, open **Elevator and claw example** under **Archetype builder**, or start with a **Blank swerve chassis**.

1. In **Layout**, click **Elevator** to add one. Drag **Roller claw** onto the blue **Elevator carriage** marker. A ghost shows where the claw will land. Dropping in open space places it on the chassis. Drag an existing part to change its position. Under **Placement**, enter a precise position or adjust **Rotation from mount**. For example, set a claw’s yaw to 90° to face the robot’s left side on a level, forward-facing mount. Parts on a tilted carriage follow its angle. Use **Side** for height and pitch and **Top** for left/right placement and yaw.
2. In **Controls**, choose **Elevator mid** or **+ New position**. Drag the blue carriage handle to the scoring height, or enter the height beside the slider. To run a roller with that position, select the roller under **Mechanism actions** and choose the same gamepad button, marked **share with**. Pressing it selects the position and runs the roller. Releasing it leaves the position selected and returns the rollers to their idle setting. Under **Roller hold**, choose **Forward** or **Reverse** and set the current per motor. A setting of **0 A** turns holding off. The button's keyboard shortcut does the same. You can also share just the keyboard shortcut while keeping separate gamepad buttons.
3. Open **Test drive**. Claws and intakes use roller contact. Choose the piece and its starting orientation, then **Place in front of robot** or **Feed into claw**. Feed presents a loose piece to the rollers at their current height. Hold the claw’s forward control while feeding. Its gap, depth and mounting angle determine whether the piece stays in.

For vertical CORAL, select the claw in **Layout** and click **Set up for upright CORAL**. This rolls the claw 90°, sets a 3.75 in depth and a 4.25 in roller gap, and opens the side plates so a branch can pass through the pipe. Choose **Upright** in Test drive. A seated CORAL counts after it rests on the reef. Placing the next piece clears the branch for another attempt.

The **Upright CORAL scorer** on Home has a complete L4 setup with an 8 A forward roller hold. In Test drive, hold the intake and click **Feed into claw**, then release the intake. Select **Clear L4**, hold a bumper to align, and select **Seat L4**. Once the elevator lowers the pipe over the branch, release the bumper, reverse the rollers, and back away.

On the REEFSCAPE field, hold **LB** or **RB** (keyboard **[** or **]**) for the screen-left or screen-right branch on the closest face. Release to drive manually. In **Controls**, choose each saved position and enter its **Bumper clearance**. The most recently activated position sets the gap, including while you hold alignment. Before selecting a position the default is 5 cm. The operator list shows the effective gamepad buttons. An existing bumper-bound mechanism moves to a spare pad button for reef driving and keeps its keyboard shortcut.

In **Layout**, select **Swerve chassis** to edit its mass in lb or kg. The total below includes the mechanisms. In REEFSCAPE, **Reef outline** shows the scoring face and L1–L4 in **Side**, or the hexagonal footprint and twelve branches in **Top**. Set **Bumper gap** to the distance where your driver expects to stop. The same outline is available in **Controls** while teaching a scoring position.

To check a design for a season, choose its game next to **Test drive**. In **Layout**, open **Starting limits** above the drawing to see the measurements and dashed limits. Switch to **Deployed preview** to check operating reach. In **Controls**, select a saved position and check **Use as starting position** to use that pose in Layout and each time Test drive starts or resets. Other saved positions preview only the joints they command, using the starting pose for the rest.

**Test drive** opens on the selected season’s field with one practice piece. The camera follows the robot’s position and keeps the view facing downfield from the blue driver station. W or left stick forward drives up the screen, even while the robot turns. Choose **Show whole field** for the same orientation from above or **Open carpet** to practice without obstacles. Both built-in fields use FIRST’s competition STEP assemblies for the visual model and simpler surfaces for contact. The REBUILT BUMPs have driveable slopes. [Official field assets](https://www.firstinspires.org/resources/library/frc/archived-games)

### CAD and simulation

1. **Home**: open **Simulation starters** under **CAD and simulation**. "Elevator, claw and ground intake" is a REEFSCAPE robot ready to drive.
2. **Build**: drive it with W A S D and the arrow keys (or a gamepad). Click a mechanism in the
   list or in the 3D view to change its numbers on the right: the robot changes while it runs.
   Click one of its motors to try it by hand: hold **forward** or **back**, or slide **Go to**.
3. **Bench**: the intake on a stand with CORAL driven onto it. Pick wheels to compare under
   "Compare wheels" and press **Run the comparison**: each wheel's current, squeeze and speed,
   side by side.
4. **Field**: press **Start a match**. Drive to a CORAL STATION to get a CORAL, take it to the
   REEF, let go near a branch to score. The cycle log shows your cycle times.
5. **Analyze**: plots of the run, the power budget, and a CSV export.

The **Field** has REEFSCAPE (2025) and REBUILT (2026). At kickoff, add the new game yourself, with
no programming: [docs/KICKOFF.md](docs/KICKOFF.md) takes you through it step by step.

Every number field takes units and sums: `1.75in`, `30mm`, `2ft + 3in`, `30/11`.
Ctrl+Z undoes anything. Ctrl+K finds any action by name. Projects save themselves.

## Bring in your CAD

Rig CAD, then paste your Onshape assembly's link. Full directions, including getting your API
keys: [docs/IMPORTING.md](docs/IMPORTING.md).

## Two levels of physics

- **Concept**: for the first week. A mechanism picks a piece up when the piece is in its pickup
  zone with its rollers running. Joints, motors and the battery are fully simulated.
- **Full**: for design review. Rollers really squeeze, grip and slip on the piece; the squeeze
  sets the current. Pieces bump into the robot's real parts from the CAD, ALGAE squashes like a
  ball, and a CORAL only scores if it really lands on a branch. Each mechanism has its own
  setting.

Add a **beam break** to any link (click it, then "Add a beam break"), and set a button to run a
motor "until" it trips, the way an intake stops when it has a piece.

In **Layout**, select an intake, roller claw, or shooter and open **Wheel profile and grip**. Click the side profile to cut a full-width V notch, then set its depth, width, angle, and repeat count. The cuts apply to the selected mechanism's wheels. The **Piece fit** readout checks whether a sideways-fed CORAL can seat behind the claw's roller axles.

The 2-, 3-, and 4-inch AndyMark compliant wheels and the 3- and 5-inch AndyMark Compliant Stars use center sections sampled every two degrees from the manufacturer's 1/2-inch hex STEP files. A plane-stress elastic solve on those sections estimates directional stiffness with the shaft constrained. Wheel hardness scales the estimated tread modulus. The realtime solver uses that stiffness in its roller contact model. Through-width extrusion, large pocket collapse, the stiffness of edited cuts, and dry tread friction are approximations. **Use a measured press force** calibrates one wheel against a flat plate at the indicated compression. **Tune CORAL friction** accepts static and sliding coefficients for the selected mechanism. [AndyMark wheel CAD](https://andymark.com/products/compliant-wheels), [AndyMark star CAD](https://andymark.com/products/compliant-stars).

## For developers

[docs/DEVELOPING.md](docs/DEVELOPING.md): building, testing, releasing. [PLAN.md](PLAN.md): the
design, what was measured, and what's left.
