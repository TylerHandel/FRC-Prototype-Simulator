"""Spike 4: links and joints from two poses of the same robot, with no mates at all.

For CAD whose mates the API will not serve (another team's export-only document). Export the
assembly twice, in two poses (two configurations, or a "scoring positions" assembly beside the
stowed one), read each part's placement with `step_tree.py`, and compare:

  - parts that made the same rigid move between the poses are one link;
  - the move of one link relative to its neighbour is a rotation about a line (a hinge, with its
    axis, a point on it and the angle turned) or a straight shift (a slide, with its direction and
    travel).

    python spikes/pose_diff.py tree-a.json tree-b.json [name of the sub-assembly both share]
"""
import collections
import json
import math
import sys

import numpy as np

a_path, b_path = sys.argv[1:3]
anchor = sys.argv[3] if len(sys.argv) > 3 else None


def load(path):
    rows = json.load(open(path))
    out = collections.defaultdict(list)
    for r in rows:
        p = r["path"]
        if anchor:
            hits = [i for i, n in enumerate(p) if n.startswith(anchor)]
            if not hits:
                continue
            p = p[hits[0] + 1:]             # the names below the shared sub-assembly
        out[tuple(p) + (r["part"],)].append(np.array(r["m"]).reshape(4, 4))
    return out


A, B = load(a_path), load(b_path)
# Parts whose names occur once are matched for certain. Repeated parts (eight of the same screw in
# one sub-assembly) come out of the two exports in no dependable order, so they wait: once the
# links' moves are known, each is given to the link whose move lands it on one of its twins.
unique = [k for k in A if k in B and len(A[k]) == 1 and len(B[k]) == 1]
repeated = [k for k in A if k in B and (len(A[k]) > 1 or len(B[k]) > 1)]
print(f"parts: {sum(len(v) for v in A.values())} in the first pose, {sum(len(v) for v in B.values())} in the second; "
      f"{len(unique)} have names that occur once, {sum(len(A[k]) for k in repeated)} are repeats")

# each part's move from pose A to pose B, in the world: D = B * inverse(A)
moves = {k: B[k][0] @ np.linalg.inv(A[k][0]) for k in unique}
common = unique


def same(m, n, tol_r=2e-3, tol_t=3e-4):
    return np.abs(m[:3, :3] - n[:3, :3]).max() < tol_r and np.abs(m[:3, 3] - n[:3, 3]).max() < tol_t


# group parts by the move they made
groups = []          # [representative move, [keys]]
for k in common:
    for g in groups:
        if same(g[0], moves[k]):
            g[1].append(k)
            break
    else:
        groups.append([moves[k], [k]])
# the repeats: which link's move carries this instance onto a same-named instance in pose B?
placed = lost = 0
for k in repeated:
    targets = [m[:3, 3] for m in B[k]]
    for m in A[k]:
        for g in sorted(groups, key=lambda g: -len(g[1])):
            landed = (g[0] @ m)[:3, 3]
            if min(np.linalg.norm(landed - t) for t in targets) < 5e-4:
                g[1].append(k)
                placed += 1
                break
        else:
            lost += 1
print(f"repeats given to a link by where its move lands them: {placed}; not explained by any link: {lost}")
groups.sort(key=lambda g: -len(g[1]))
big = [g for g in groups if len(g[1]) >= 5]
print(f"distinct moves: {len(groups)}; {len(big)} of them shared by 5 parts or more, covering "
      f"{sum(len(g[1]) for g in big)} parts; {sum(1 for g in groups if len(g[1]) == 1)} parts moved alone")


def where(keys):
    c = collections.Counter(" / ".join(k[0:2])[:60] for k in keys)
    return "; ".join(f"{n} in {w}" for w, n in c.most_common(2))


def screw(rel):
    """a relative move as a turn about a line, or a straight shift"""
    r, t = rel[:3, :3], rel[:3, 3]
    ang = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if ang < 1e-3:
        d = float(np.linalg.norm(t))
        return ("fixed", None, None, 0.0) if d < 3e-4 else ("slide", t / d, None, d)
    w, v = np.linalg.eig(r)                      # the axis is the direction the rotation leaves alone
    axis = np.real(v[:, np.argmin(np.abs(w - 1))])
    axis /= np.linalg.norm(axis)
    if axis @ np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) < 0:
        axis = -axis
    along = float(t @ axis)
    # a point on the axis: solve (I - R) p = t - along * axis, in the plane across the axis
    p, *_ = np.linalg.lstsq(np.eye(3) - r, t - along * axis, rcond=None)
    p -= (p @ axis) * axis
    return ("hinge" if abs(along) < 1e-3 else "hinge+slide", axis, p, ang)


print("\nlinks (largest first), and how each moved against the one that did not move:")
base = big[0]
for g in big[:12]:
    rel = np.linalg.inv(base[0]) @ g[0]
    kind, axis, point, amount = screw(rel)
    if kind == "fixed":
        desc = "did not move (the base)"
    elif kind == "slide":
        desc = f"slid {amount / 0.0254:.2f} in. along {np.round(axis, 3)}"
    else:
        desc = f"turned {math.degrees(amount):.1f} deg about {np.round(axis, 3)} through {np.round(point, 3)} m"
        if kind == "hinge+slide":
            desc += " and shifted along it"
    print(f"  {len(g[1]):>4} parts  {desc}\n        {where(g[1])}")

# joints: each link against the link it rides on. The parent is the link whose own move, undone,
# leaves the simplest move: try every larger-or-earlier link and keep a pure hinge or a pure slide.
print("\njoints, each link against the link it rides on:")
for i, g in enumerate(big[:12]):
    if g is base:
        continue
    best = None
    for h in big[:12]:
        if h is g:
            continue
        kind, axis, point, amount = screw(np.linalg.inv(h[0]) @ g[0])
        if kind in ("hinge", "slide"):
            # the parent is the link against which this one turned least; between equals, moved least
            rel = np.linalg.inv(h[0]) @ g[0]
            size = (round(amount, 3) if kind == "hinge" else 0.0, float(np.linalg.norm(rel[:3, 3])))
            if best is None or size < best[0]:
                best = (size, h, kind, axis, point, amount)
    if best is None:
        print(f"  {len(g[1]):>4} parts: no single hinge or slide explains it against any other link")
        continue
    _, h, kind, axis, point, amount = best
    if kind == "slide":
        print(f"  {len(g[1]):>4} parts slide on the {len(h[1])}-part link: {amount / 0.0254:.2f} in. along {np.round(axis, 3)}"
              f"   [{where(g[1])}]")
    else:
        print(f"  {len(g[1]):>4} parts hinge on the {len(h[1])}-part link: {math.degrees(amount):.1f} deg about "
              f"{np.round(axis, 3)} through {np.round(point, 3)} m   [{where(g[1])}]")
