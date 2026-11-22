"""Writes assets/games/reefscape-2025/game.json: a simplified REEFSCAPE field.

Dimensions from the 2025 game manual where known (field 690.876 x 317 in., REEF centres 176.746 in.
from each ALLIANCE WALL on the field's centre line, branch heights L2 31.875 in., L3 47.625 in.,
L4 72 in., the trough's top 18 in.). The REEF is drawn as a hexagonal base with twelve branch posts;
the CORAL STATIONS, PROCESSOR and BARGE are blocks in roughly the right places. It is good for
archetype cycles. The separate field.glb supplies the official field geometry for rendering.

    python3 tools/make-reefscape.py
"""
import json, math, os

IN = 0.0254
L, W = 690.876 * IN, 317.0 * IN
HX = L / 2
REEF_X = HX - 176.746 * IN
APO = 32.75 * IN          # REEF base, centre to a flat face
BASE_H = 18 * IN          # the trough's top
POST_R = 0.835 * IN       # 1.66 in. pipe
CORAL_HALF = 11.875 * IN / 2
CORAL_IN_R = 4.026 * IN / 2
LEVELS = [("L2", 31.875 * IN, 35.0, 4, 3), ("L3", 47.625 * IN, 35.0, 6, 4), ("L4", 72.0 * IN, 90.0, 7, 5)]

statics, zones, pieces, feeders, reef_faces = [], [], [], [], []

def box(name, pos, size, yaw=0.0, color="#7d838a", **kw):
    statics.append(dict(name=name, shape={"box": {"size": [round(s, 4) for s in size]}}, pos=[round(p, 4) for p in pos], euler_deg=[0, 0, round(yaw, 3)], color=color, **kw))

def cyl(name, pos, r, length, euler=(0, 0, 0), color="#9a6bd0", **kw):
    statics.append(dict(name=name, shape={"cylinder": {"radius": round(r, 4), "length": round(length, 4)}}, pos=[round(p, 4) for p in pos], euler_deg=[round(e, 3) for e in euler], color=color, **kw))

def tilt_euler(nx, ny, up_deg):
    """euler angles (x, y, z, deg) turning a piece's z axis to point out along (nx, ny), up_deg above level"""
    yaw = math.degrees(math.atan2(ny, nx))
    pitch = 90.0 - up_deg     # about y: z toward +x by (90 - up)
    return [0.0, round(pitch, 3), round(yaw, 3)]

# the carpet's edge: walls
wall_h = 0.5
box("North wall", [0, W / 2 + 0.05, wall_h / 2], [L, 0.1, wall_h], color="#4a4f55")
box("South wall", [0, -W / 2 - 0.05, wall_h / 2], [L, 0.1, wall_h], color="#4a4f55")
box("Blue alliance wall", [-HX - 0.05, 0, 1.0], [0.1, W, 2.0], color="#2b5fad")
box("Red alliance wall", [HX + 0.05, 0, 1.0], [0.1, W, 2.0], color="#b8312f")

for side, sx, alliance, col in ((-1, -REEF_X, "blue", "#3b6fc4"), (1, REEF_X, "red", "#c94f4f")):
    A = alliance.capitalize()
    cyl(f"{A} REEF base", [sx, 0, 0.195], APO * 0.98, 0.39, color="#6e6e78")
    cyl(f"{A} REEF column", [sx, 0, 0.95], 0.12, 1.9, color="#4f4f58")
    for k in range(6):
        a = math.radians(60 * k)
        nx, ny = math.cos(a), math.sin(a)
        tx, ty = -ny, nx
        face = [sx + nx * APO, ny * APO]
        reef_faces.append(dict(center=[round(v, 5) for v in face], yaw_deg=60*k, branch_offset=6.47*IN))
        face_width = 2 * APO * math.tan(math.radians(30))
        box(f"{A} REEF face {k + 1}", [face[0] - nx * 0.0065, face[1] - ny * 0.0065, BASE_H / 2], [0.013, face_width, BASE_H], yaw=60 * k, color=col)
        trough_name = f"{A} trough {k + 1}"
        box(trough_name, [face[0]-nx*0.132, face[1]-ny*0.132, 0.436], [math.hypot(0.234, 0.066), face_width-0.08, 0.006], color=col)
        statics[-1]["euler_deg"] = [0, round(math.degrees(math.atan2(0.066,0.234)),3), 60*k]
        # the trough (L1) on top of the base by this face
        zones.append(dict(name=f"{A} L1 face {k + 1}", alliance=alliance, piece="coral-2025",
                          center=[round(face[0] - nx * 0.12, 4), round(face[1] - ny * 0.12, 4), round(BASE_H + 0.08, 4)],
                          size=[0.3, 0.9, 0.25], euler_deg=[0, 0, 60 * k], support=trough_name, auto_points=3, teleop_points=2, capacity=4))
        for b, off in (("left", 6.47 * IN), ("right", -6.47 * IN)):
            # Centre lines of the pipes in the same CAD section used by the drawing below.
            def point(inset, z):
                return [face[0] - nx*inset + tx*off, face[1] - ny*inset + ty*off, z]

            def pipe(label, start, end):
                a, bpos = point(*start), point(*end)
                along = start[0] - end[0]
                elevation = math.degrees(math.atan2(end[1] - start[1], along))
                cyl(f"{A} branch {k + 1}{b[0]} {label}", [(x+y)/2 for x,y in zip(a,bpos)],
                    POST_R, math.dist(a,bpos), euler=tilt_euler(nx, ny, elevation), bore=True)

            trunk = [(0.305, 0.019), (0.305, 1.368), (0.296, 1.410), (0.272, 1.446),
                     (0.099, 1.569), (0.070, 1.601), (0.055, 1.632), (0.051, 1.654), (0.051, 1.826)]
            for i, (start, end) in enumerate(zip(trunk, trunk[1:])):
                pipe(f"trunk {i+1}", start, end)
            for lv, _, up, pa, pt in LEVELS:
                if lv == "L4":
                    tip = point(0.051, 1.826)
                    seat = point(0.051, 1.734)
                else:
                    dz = 0.403 if lv == "L3" else 0
                    pipe(lv, (0.305, 0.616 + dz), (0.055, 0.791 + dz))
                    tip = point(0.055, 0.791 + dz)
                    seat = point(0.163, 0.715 + dz)
                e = tilt_euler(nx, ny, up)
                zones.append(dict(name=f"{A} {lv} {k + 1}{b[0]}", alliance=alliance, piece="coral-2025",
                                  center=[round(s, 4) for s in seat], size=[0.35, 0.35, 0.35], auto_points=pa, teleop_points=pt, capacity=1,
                                  seat=dict(pos=[round(s, 4) for s in seat], euler_deg=e, tolerance=0.2,
                                            peg_tip=[round(s, 4) for s in tip], peg_radius=round(POST_R, 5))))
    # CORAL STATIONS in the corners of the alliance's own wall, feeding toward the field (AprilTags
    # 12 and 13 for blue, 1 and 2 for red)
    wx = side * HX
    for yy in (1, -1):
        p = [wx - side * 0.55, yy * (W / 2 - 0.55), 1.0]
        box(f"{A} CORAL STATION {'north' if yy > 0 else 'south'}", [wx - side * 0.35, yy * (W / 2 - 0.35), 0.6], [0.25, 1.9, 1.2], yaw=yy * side * 54, color="#8d8d93")
        d = [-side * 0.6, -yy * 0.8, 0.0]
        n = math.hypot(d[0], d[1])
        feeders.append(dict(name=f"{A} CORAL STATION {'north' if yy > 0 else 'south'}", piece="coral-2025", alliance=alliance,
                            pos=[round(p[0] - side * 0.35, 4), round(p[1] - yy * 0.45, 4), 0.9], dir=[round(d[0] / n, 4), round(d[1] / n, 4), -0.3],
                            speed=1.2, interval=1.5, reach=1.3))
    # PROCESSOR in the side wall between the alliance's REEF and the centre, scored with ALGAE (blue's
    # in the south wall at AprilTag 16, red's in the north wall at 3)
    px = sx - side * 1.5
    box(f"{A} PROCESSOR", [px, side * (W / 2 + 0.2), 0.3], [0.8, 0.4, 0.6], color="#5d6168")
    zones.append(dict(name=f"{A} PROCESSOR", alliance=alliance, piece="algae-2025", center=[round(px, 4), round(side * (W / 2 - 0.2), 4), 0.35],
                      size=[0.8, 0.5, 0.6], auto_points=6, teleop_points=6, capacity=99))
    # staged pieces on their marks 48 in. from the alliance wall: CORAL upright with ALGAE on top
    for yy in (-1.83, 0.0, 1.83):
        pieces.append(dict(piece="coral-2025", pos=[round(side * (HX - 48 * IN), 4), yy, 0.151], upright=True))
        pieces.append(dict(piece="algae-2025", pos=[round(side * (HX - 48 * IN), 4), yy, 0.302 + 0.207]))
    # NET: ALGAE thrown over the BARGE's top on this alliance's side
    zones.append(dict(name=f"{A} NET", alliance=alliance, piece="algae-2025", center=[round(side * 0.25, 4), round(side * 1.8, 4), 2.3],
                      size=[0.5, 3.6, 0.6], auto_points=4, teleop_points=4, capacity=99))

# the BARGE on the centre line: posts and a beam (the CAGES hang from it)
for yy in (-3.6, 0.0, 3.6):
    box("BARGE post", [0, yy, 1.05], [0.1, 0.1, 2.1], color="#4a4f55")
box("BARGE beam", [0, 0, 2.15], [0.15, 7.3, 0.1], color="#4a4f55")
box("Centre line", [0, 0, 0.001], [0.05, W, 0.002], color="#ffffff", ghost=True)

# Section through a scoring branch in FIRST's FE-2025 STEP. Coordinates below are
# millimetres from the outside of panel GE-25302 and the carpet. The trunk and
# branch outlines follow GE-25301-02 and -01, simplified to about 2 mm.
def metres(points):
    return [[x / 1000, y / 1000] for x, y in points]

trunk = [(284, 19), (284, 1374), (277, 1401), (257, 1428), (80, 1555),
         (60, 1573), (43, 1600), (33, 1627), (30, 1654), (30, 1826),
         (72, 1826), (72, 1654), (77, 1627), (86, 1609), (114, 1582),
         (269, 1473), (290, 1455), (311, 1428), (324, 1392), (326, 1365), (326, 19)]
branch = [(284, 605), (43, 774), (67, 808), (284, 656)]
side = dict(contours=[metres(path) for path in (
    [(0, 3), (0, 457), (13, 457), (13, 3)],
    [(17, 403), (17, 409), (246, 471), (248, 469), (248, 518), (357, 518),
     (357, 511), (254, 511), (254, 464), (247, 464)],
    trunk, branch, [(x, y + 403) for x, y in branch])],
    labels=[dict(text=level, at=[0.43, z], tip=[x, z]) for level, x, z in
            [("L1", 0.013, 0.457), ("L2", 0.055, 0.791), ("L3", 0.055, 1.194), ("L4", 0.072, 1.826)]])

# Looking down on the same branch, the robot centreline lines up with one of the
# two pipes on the near face. The other five faces repeat the same branch pattern.
half_spacing = 6.47 * IN
def plan_point(radial, tangent, a):
    return [round(APO + radial * math.cos(a) - tangent * math.sin(a), 4),
            round(-half_spacing + radial * math.sin(a) + tangent * math.cos(a), 4)]

corner_radius = APO / math.cos(math.pi / 6)
top_contours = [[plan_point(corner_radius, 0, math.radians(30 + k * 60)) for k in range(6)]]
for k in range(6):
    for offset in [-half_spacing, half_spacing]:
        top_contours.append([plan_point(APO - inset, offset + width, math.radians(k * 60))
                             for inset, width in [(0.030, 0), (0.051, 0.021), (0.305, 0.021),
                                                  (0.326, 0), (0.305, -0.021), (0.051, -0.021)]])
top = dict(contours=top_contours, labels=[dict(text="REEF", at=[round(APO, 4), round(-half_spacing, 4)])])

game = dict(
    id="reefscape-2025", name="REEFSCAPE (2025)",
    about="Field model from FIRST’s official REEFSCAPE STEP assembly. Simplified collision surfaces and scoring zones support CORAL on branches and in the trough, and ALGAE in the PROCESSOR and NET.",
    robot_size=dict(perimeter=3.048, start_height=1.0668, start_extension=0.0, extension=0.4572,
                    source="https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm"),
    field=dict(length=round(L, 4), width=round(W, 4), carpet_friction=1.0),
    design_reference=dict(name="Reef outline", side=side, top=top),
    statics=statics, pieces=pieces, feeders=feeders, zones=zones, reef_faces=reef_faces,
    timing=dict(auto=15, teleop=135), start=dict(pos=[round(-HX + 7.2, 4), -1.5], yaw_deg=0),
    practice_start=dict(pos=[-7.0, round(half_spacing, 4)], yaw_deg=0))

out = os.path.join(os.path.dirname(__file__), "..", "assets", "games", "reefscape-2025", "game.json")
json.dump(game, open(out, "w"), indent=1)
print(f"{len(statics)} shapes, {len(zones)} zones, {len(pieces)} pieces, {len(feeders)} feeders -> {os.path.normpath(out)}")
