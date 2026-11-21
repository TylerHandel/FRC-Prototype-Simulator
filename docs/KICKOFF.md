# Kickoff: putting a new game in the simulator

On kickoff weekend you can have the new field, its game pieces and its scoring in the simulator
in an afternoon, with no programming. A game is a **game pack**: a folder holding a text file,
`game.json`, that lists the field's shapes, where pieces start, where human players hand pieces
in, and where pieces score. If the game has a new piece, a second file, `pieces.json`, describes it.

You need: the game manual, the field drawings (FIRST posts them at kickoff), WPILib's AprilTag
field layout for the year (it gives exact places for most field elements), and a text editor.
Any editor works; one that checks JSON (VS Code, Notepad++) helps.

## 1. Start a pack

1. Open the simulator, go to **Field**, and press **New game pack**. It makes a folder called
   `new-game` in your game packs folder, with a small working example in it, and opens the folder.
2. Rename the folder (for example `my-game-2027`). Open `game.json`. Change `"id"` to the same
   name and `"name"` to the game's name.
3. In the simulator, press **Reload** and pick your game in the **Game** list. You'll see the
   example field. Every time you save `game.json`, press **Reload** to see your changes.

If something in the file is wrong, the Field panel says what and where ("game.json line 40,
column 7: ..." or "zone 'Goal': every size must be more than 0"). Fix it, save, Reload.

## 2. Where things are

Everything is in **metres** and **degrees**. To turn inches into metres, multiply by 0.0254.

- The middle of the field is `0, 0`.
- **x** runs the length of the field: the blue alliance wall is at minus half the length, the red
  wall at plus half.
- **y** runs across: plus is to the left of a blue driver looking out at the field.
- **z** is up from the carpet.

WPILib's field coordinates (and its AprilTag layout) start at the blue wall's right-hand corner.
To turn one of those into ours: **x = WPILib x - length / 2**, **y = WPILib y - width / 2**,
z the same.

A position is `[x, y, z]`. A shape's turn is `"euler_deg": [0, 0, 30]`: the last number turns it
about the vertical, which is the only one most things need. (The first two tilt it about x, then
y, before that.)

Most games are the same for both alliances, turned half round the field's middle. Write the blue
half and add `"mirror": true` to each thing: the red copy is made for you, with "Blue" in its name
changed to "Red". (`"mirror_color"` gives the copy its own colour.)

## 3. The field

```json
"field": { "length": 16.541, "width": 8.069, "carpet_friction": 1.0 },
```

Length and width are the carpet's, from the field drawings. Leave the friction at 1.0 unless the
manual says the carpet changed.

## 4. Field elements: `"statics"`

Each element is a box or a cylinder, placed by its middle:

```json
{ "name": "Blue HUB", "shape": { "box": { "size": [1.194, 1.194, 1.829] } },
  "pos": [-3.641, 0, 0.914], "color": "#3b6fc4", "mirror": true, "mirror_color": "#c94f4f" }
```

- `"box"`: `"size"` is its full length, width and height (x, y, z) before turning.
- `"cylinder"`: `"radius"` and `"length"`; it stands upright unless you tilt it with `euler_deg`.
- A ramp: a thin box tilted with `"euler_deg": [0, 15, 0]`.
- `"ghost": true` draws it without it being solid (tape lines, zone markings).
- `"bore": true` on a post means a hollow game piece slides down over it.

Keep it simple. Robots and pieces only meet the outside of these shapes, so a few boxes in the
right places beat a detailed shape in the wrong one. Walls go just outside the carpet. A goal that
pieces fall into is an open box: four thin walls and a floor, not one solid block.

## 5. New game pieces: `pieces.json`

Skip this if the game reuses a piece the simulator knows (CORAL, ALGAE, FUEL). Otherwise, put a
`pieces.json` beside `game.json`:

```json
{ "pieces": [
  { "id": "widget-2027", "name": "WIDGET", "game": "2027",
    "shape": { "sphere": { "d_in": 7.0 } },
    "mass_kg": 0.30, "surface": "foam", "law": "rigid",
    "color": "#ff8800", "source": "2027 manual section 5.3", "measured": false }
] }
```

- **shape**, in inches: `{ "sphere": { "d_in": ... } }`, `{ "cylinder": { "d_in": ..., "length_in": ... } }`,
  `{ "tube": { "outer_d_in": ..., "inner_d_in": ..., "length_in": ... } }` (hollow) or
  `{ "box": { "x_in": ..., "y_in": ..., "z_in": ... } }`.
- **mass_kg**: weigh one, or take it from the manual.
- **surface**: what it's made of, for grip: `"pvc"`, `"foam"`, `"playground-rubber"`, or anything
  else for a middling grip.
- **law**, how it squashes:
  - `"rigid"`: hard (plastic pipe, wood);
  - `{ "inflated": { "psi": [0.5, 1.0, 1.5], "choice": 1, "skin_n_per_m": 400 } }`: a ball
    (the pressure in use is `psi[choice]`);
  - `{ "foam": { "k0_lb_per_in": 20, "xc_in": 1.0 } }`: foam (from a press test: lb per inch when
    first squeezed, and the squeeze in inches over which it gets about 2.7 times firmer).
- **source** says where the numbers came from; **measured** is `true` once you've tested a real one.

## 6. Pieces on the field at the start: `"pieces"`

```json
{ "piece": "fuel-2026", "pos": [-7.4, 1.5, 0.08], "grid": [4, 6], "spacing": [0.16, 0.16], "mirror": true }
```

`pos` is the first piece's place (z is about the piece's radius, so it sits on the carpet).
`"grid": [4, 6]` makes a block, 4 along x and 6 along y, `spacing` metres apart. Leave out `grid`
for a single piece. A tube lies on its side unless you add `"upright": true`; `"yaw_deg"` turns it.

## 7. Human players: `"feeders"`

A feeder hands a piece to a robot that comes close:

```json
{ "name": "Blue OUTPOST", "piece": "fuel-2026", "alliance": "blue",
  "pos": [-8.1, -3.4, 0.9], "dir": [1, 0, -0.2], "speed": 2.0,
  "interval": 1.0, "reach": 1.5, "stock": 24, "mirror": true }
```

`pos` is where the piece appears; `dir` the way it's sent and `speed` how fast (m/s); `interval`
the fewest seconds between pieces; `reach` how close (m) a robot has to be; `stock` how many it
holds. Pieces that leave the field go back to the feeders.

## 8. Scoring: `"zones"`

A zone is a box. A piece of the right kind in it scores:

```json
{ "name": "Blue HUB", "alliance": "blue", "piece": "fuel-2026",
  "center": [-3.641, 0, 1.7], "size": [1.0, 1.0, 0.4],
  "auto_points": 1, "teleop_points": 1, "capacity": 1000,
  "exit": { "pos": [-3.0, 0, 0.8], "dir": [1, 0, 0], "speed": 1.5 }, "mirror": true }
```

- `capacity`: how many it can hold and count (a branch holds 1; a goal pieces fall through, lots).
- `"exit"`: for a goal that gives pieces back (a HUB, a processor): each piece is counted, then
  sent out from `pos` along `dir` at `speed`. Without `exit`, a scored piece stays where it
  scored.
- `"seat"`: for a place a piece is put (a branch, a peg). In Concept, letting go of a piece within
  `tolerance` metres of `pos` puts it there, turned by `euler_deg`, and scores it (the scoring
  assist):

  ```json
  "seat": { "pos": [-3.55, 0.16, 1.83], "euler_deg": [0, 0, 0], "tolerance": 0.2 }
  ```

  In Full, nothing is assisted: a piece that comes to rest within 12 cm of a seat scores.

Points that change during a match (bonuses, a goal that's only sometimes active) and points for
climbing aren't modelled yet: give each zone its usual points.

## 9. Timing and the start

```json
"timing": { "auto": 20, "teleop": 140 },
"start": { "pos": [-4.2, 0], "yaw_deg": 0 }
```

`timing` is seconds of autonomous and of teleop. `start` is where your robot starts (x, y) and
which way it faces (0 is toward the red wall).

## 10. Check it

1. Press **Reload**. Fix anything the Field panel lists.
2. Drive round the field. Does the robot fit where the manual says it should? Do pieces sit on
   the carpet?
3. Score a piece in each zone and watch the score and the cycle log.
4. Put a Concept robot on it and time a few cycles.

When it works, zip the folder to share it. Anyone with the simulator puts it in their game packs
folder and presses Reload.

The REBUILT (2026) pack that comes with the simulator was made by following this guide; its
`game.json` (in `assets/games/rebuilt-2026/` in the source) is a full example.
