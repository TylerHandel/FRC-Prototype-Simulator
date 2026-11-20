"""Spike 3: links and joints from an Onshape assembly's mates.

Reads one assembly definition, as Onshape's API returns it from

    GET /api/v6/assemblies/d/{did}/w/{wid}/e/{eid}?includeMateFeatures=true&includeMateConnectors=true

and works out what PLAN.md D8 needs from it:

  - links: the sets of parts that cannot move against each other (joined by Fastened mates and
    mate groups, at every level of the assembly);
  - joints: every mate that leaves a freedom between two links, with its axis in the robot's frame;
  - which joints are mechanisms (an arm, a slide) and which are just something spinning on a shaft.

    python spikes/onshape_mates.py assembly.json

The JSON is the owner's or another team's CAD: keep it out of the repository.
"""
import collections
import json
import sys

import numpy as np

MOVING = {"REVOLUTE": "hinge", "SLIDER": "slide", "CYLINDRICAL": "hinge+slide", "PIN_SLOT": "hinge+slide",
          "PLANAR": "planar", "BALL": "ball"}


def load(path):
    d = json.load(open(path))
    root = d["rootAssembly"]
    subs = {(s["documentId"], s["elementId"], s.get("fullConfiguration", s.get("configuration", ""))): s
            for s in d.get("subAssemblies", [])}
    return d, root, subs


def key_of(inst):
    return (inst.get("documentId"), inst.get("elementId"), inst.get("fullConfiguration", inst.get("configuration", "")))


def main(path):
    d, root, subs = load(path)

    # every instance by id, from the root and from each sub-assembly
    inst = {i["id"]: i for i in root["instances"]}
    for s in subs.values():
        for i in s["instances"]:
            inst[i["id"]] = i

    occ = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in root["occurrences"]}
    parts = [p for p in occ if inst[p[-1]].get("type") == "Part"]
    part_set = set(parts)

    def leaves(prefix):
        """the part occurrences at or under an occurrence path"""
        n = len(prefix)
        return [p for p in parts if p[:n] == prefix]

    # union-find over part occurrences, plus one node per assembly for its own origin
    parent = {p: p for p in parts}

    def find(p):
        parent.setdefault(p, p)
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[a] = b

    # every assembly's features, once per place that assembly is used
    owners = [((), root)]
    for path in occ:
        i = inst[path[-1]]
        if i.get("type") == "Assembly" and key_of(i) in subs:
            owners.append((path, subs[key_of(i)]))

    def node(prefix, mated):
        """what one side of a mate holds on to: a part, or (with nothing named) the assembly's own origin"""
        if not mated:
            return ("origin",) + prefix
        path = prefix + tuple(mated)
        return path if path in part_set else None

    moving = []          # (kind, name, world point, world axis, side A parts, side B parts)
    counts = collections.Counter()
    unresolved = 0
    rigid = 0
    for prefix, asm in owners:
        if asm.get("isRigid") and prefix:
            # Onshape marks a sub-assembly with no freedom left in it: everything under it is one link
            group = leaves(prefix)
            for p in group[1:]:
                union(group[0], p)
            rigid += 1
            continue
        for f in asm.get("features", []):
            if f.get("suppressed"):
                continue
            fd = f.get("featureData", {})
            if f["featureType"] == "mateGroup":
                group = []
                for o in fd.get("occurrences", []):
                    group += leaves(prefix + tuple(o["occurrence"]))
                for p in group[1:]:
                    union(group[0], p)
                counts["mate group"] += 1
                continue
            if f["featureType"] != "mate":
                continue
            ents = fd.get("matedEntities", [])
            if len(ents) != 2:
                unresolved += 1
                continue
            sides = [node(prefix, e["matedOccurrence"]) for e in ents]
            if sides[0] is None or sides[1] is None:
                unresolved += 1          # a side that is not a part: a sketch, a deleted instance
                continue
            kind = fd.get("mateType")
            counts[kind] += 1
            if kind == "FASTENED":
                union(sides[0], sides[1])
            elif kind in MOVING:
                e = ents[0]
                path0 = prefix + tuple(e["matedOccurrence"])
                t = occ.get(path0, np.eye(4))
                cs = e["matedCS"]
                point = (t @ np.array(cs["origin"] + [1.0]))[:3]
                axis = t[:3, :3] @ np.array(cs["zAxis"])
                moving.append((kind, fd.get("name", ""), point, axis, sides[0], sides[1], fd.get("mateLimits") or {}))

    links = collections.defaultdict(list)
    for p in parts:
        links[find(p)].append(p)
    sizes = sorted((len(v) for v in links.values()), reverse=True)

    print(f"parts: {len(parts)}   assemblies used: {len(owners)}, of which Onshape marks {rigid} rigid")
    print("mates:", dict(counts), f"  (skipped, one-sided: {unresolved})")
    print(f"links (sets of parts that move together): {len(links)}")
    print(f"  the 12 largest hold {sizes[:12]} parts; {sum(1 for s in sizes if s == 1)} are a single part")

    def name(p):
        return "(origin)" if p[0] == "origin" else inst[p[-1]].get("name", "?")

    def where(p):
        p = p[1:] + ("",) if p[0] == "origin" else p
        return " / ".join(inst[i].get("name", "?") for i in p[:-1])[-60:]

    joints = collections.defaultdict(list)
    internal = 0
    for kind, mname, point, axis, a, b, limits in moving:
        la, lb = find(a), find(b)
        if la == lb:
            internal += 1               # both sides already rigid together: the mate is redundant
            continue
        joints[frozenset((la, lb))].append((kind, mname, point, axis, a, b, limits))
    print(f"joints between different links: {len(joints)}   (mates inside one link, ignored: {internal})")

    mech, spin = [], []
    for pair, ms in joints.items():
        la, lb = tuple(pair)
        small = min(len(links[la]), len(links[lb]))
        (mech if small >= 8 else spin).append((small, ms, la, lb))
    print(f"  mechanism joints (both sides 8 parts or more): {len(mech)}")
    for small, ms, la, lb in sorted(mech, key=lambda x: -x[0]):
        kind, mname, point, axis, a, b, limits = ms[0]
        print(f"    {MOVING[kind]:<11} '{mname[:28]}'  {len(links[la])} parts <-> {len(links[lb])} parts   "
              f"at {np.round(point, 3)} m along {np.round(axis, 2)}  limits {limits or 'none set'}   in {where(a)}")
    print(f"  small joints (a roller, wheel, plunger or latch on a link): {len(spin)}")
    for small, ms, la, lb in sorted(spin, key=lambda x: -x[0])[:14]:
        kind, mname, point, axis, a, b, limits = ms[0]
        little = la if len(links[la]) <= len(links[lb]) else lb
        print(f"    {MOVING[kind]:<11} {small:>2} parts: {', '.join(sorted({name(p) for p in links[little]}))[:70]}")


if __name__ == "__main__":
    main(sys.argv[1])
