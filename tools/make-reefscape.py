"""Writes assets/games/reefscape-2025/game.json: a simplified REEFSCAPE field.

Dimensions from the 2025 game manual where known (field 690.876 x 317 in., REEF centres 176.746 in.
from each ALLIANCE WALL on the field's centre line, branch heights L2 31.875 in., L3 47.625 in.,
L4 72 in., the trough's top 18 in.). The REEF is drawn as a hexagonal base with twelve branch posts;
the CORAL STATIONS, PROCESSOR and BARGE are blocks in roughly the right places. It is good for
archetype cycles; import the official field CAD for exact geometry (PLAN.md, kickoff procedure).

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
LEVELS = [("L2", 31.875 * IN, 35.0, 4, 3), ("L3", 47.625 * IN, 35.0, 6, 4), ("L4", 72.0 * IN, 90.0, 7, 5)]

statics, zones, pieces, feeders = [], [], [], []

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
    cyl(f"{A} REEF base", [sx, 0, BASE_H / 2], APO * 0.98, BASE_H, color="#6e6e78")
    cyl(f"{A} REEF column", [sx, 0, 0.95], 0.12, 1.9, color="#4f4f58")
    for k in range(6):
        a = math.radians(60 * k)
        nx, ny = math.cos(a), math.sin(a)
        tx, ty = -ny, nx
        face = [sx + nx * APO, ny * APO]
        box(f"{A} REEF face {k + 1}", [face[0] - nx * 0.05, face[1] - ny * 0.05, BASE_H / 2], [0.1, 2 * APO * math.tan(math.radians(30)), BASE_H], yaw=60 * k, color=col)
        # the trough (L1) on top of the base by this face
        zones.append(dict(name=f"{A} L1 face {k + 1}", alliance=alliance, piece="coral-2025",
                          center=[round(face[0] - nx * 0.12, 4), round(face[1] - ny * 0.12, 4), round(BASE_H + 0.08, 4)],
                          size=[0.3, 0.9, 0.25], euler_deg=[0, 0, 60 * k], auto_points=3, teleop_points=2, capacity=4))
        for b, off in (("left", 6.47 * IN), ("right", -6.47 * IN)):
            px, py = sx + nx * (APO - 0.1) + tx * off, ny * (APO - 0.1) + ty * off
            cyl(f"{A} branch {k + 1}{b[0]}", [px, py, (BASE_H + 1.85) / 2], POST_R, 1.85 - BASE_H, color="#9a6bd0", bore=True)
            for lv, h, up, pa, pt in LEVELS:
                out = 0.0 if lv == "L4" else 0.11
                seat = [px + nx * out, py + ny * out, h + (0.0 if lv == "L4" else 0.05)]
                e = [0.0, 0.0, 0.0] if lv == "L4" else tilt_euler(nx, ny, up)
                zones.append(dict(name=f"{A} {lv} {k + 1}{b[0]}", alliance=alliance, piece="coral-2025",
                                  center=[round(s, 4) for s in seat], size=[0.35, 0.35, 0.35], auto_points=pa, teleop_points=pt, capacity=1,
                                  seat=dict(pos=[round(s, 4) for s in seat], euler_deg=e, tolerance=0.2)))
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

game = dict(
    id="reefscape-2025", name="REEFSCAPE (2025)",
    about="A simplified field: REEF, BARGE, CORAL STATIONS and PROCESSOR as blocks in roughly the right places. Scoring zones count CORAL on branches and in the trough, ALGAE in the PROCESSOR and NET. Import the official field CAD for exact geometry.",
    field=dict(length=round(L, 4), width=round(W, 4), carpet_friction=1.0),
    statics=statics, pieces=pieces, feeders=feeders, zones=zones,
    timing=dict(auto=15, teleop=135), start=dict(pos=[round(-HX + 7.2, 4), -1.5], yaw_deg=0))

out = os.path.join(os.path.dirname(__file__), "..", "assets", "games", "reefscape-2025", "game.json")
json.dump(game, open(out, "w"), indent=1)
print(f"{len(statics)} shapes, {len(zones)} zones, {len(pieces)} pieces, {len(feeders)} feeders -> {os.path.normpath(out)}")
