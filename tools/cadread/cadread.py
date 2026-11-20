"""cadread: a STEP assembly to a cadpack (PLAN.md D8).

One pass over the file with OpenCascade, as rebuilt-sim-2026's tools/step2glb.py and
tools/step-axes.py did in two: the assembly tree with each part's name path, placement and colour;
each distinct part's facts (volume, centre of mass, bounding box, its cylindrical faces' axes and
radii); and each distinct part meshed once, with the surfaces' own normals so round parts look round.

    cadread <in.step> <out-dir> [--lin 0.0012] [--ang 0.45] [--hardware]

Writes <out-dir>/parts.json and <out-dir>/meshes.bin (little-endian: for each mesh, float32
positions xyz, float32 normals xyz, uint32 triangle indices, at the offsets parts.json gives).
Fasteners and other hardware (by name) are listed but not meshed or measured unless --hardware.
Prints "PROGRESS <0..1> <what>" lines so the app can show how far along it is.

Runs as a child process of the app (a bad file can't take the app down); packaged with
PyInstaller for people who don't have Python (tools/cadread/build.py).
"""
import argparse
import json
import math
import os
import re
import struct
import sys
import time

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_ColorRGBA
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence

def face(shape):
    """TopoDS.Face_s in older bindings, TopoDS.Face in newer"""
    f = getattr(TopoDS, "Face_s", None) or getattr(TopoDS, "Face")
    return f(shape)


HARDWARE = re.compile(
    r"screw|bolt|\bnut\b|nutstrip|washer|rivet|spacer|standoff|bearing|shcs|bhcs|fhcs|\bpin\b|retaining|collar|"
    r"zip ?tie|grommet|insert|rivnut|clevis|^9\d{4}a|shoulder|set ?screw|e-clip|snap ring|thread", re.I)


def progress(f, what):
    print(f"PROGRESS {f:.3f} {what}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--lin", type=float, default=0.0012, help="largest distance from the true surface, m")
    ap.add_argument("--ang", type=float, default=0.45, help="largest angle between facets, rad")
    ap.add_argument("--hardware", action="store_true", help="mesh and measure fasteners too")
    a = ap.parse_args()
    t0 = time.time()
    os.makedirs(a.out, exist_ok=True)

    progress(0.0, "reading the STEP file")
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    rd = STEPCAFControl_Reader()
    rd.SetColorMode(True)
    rd.SetNameMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
    if rd.ReadFile(a.src) != IFSelect_RetDone:
        sys.exit("could not read " + a.src)
    progress(0.15, "building the assembly")
    rd.Transfer(doc)
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(label):
        n = TDataStd_Name()
        return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""

    def color_of(label, shape):
        c = Quantity_ColorRGBA()
        for ct in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if XCAFDoc_ColorTool.GetColor_s(label, ct, c) or colors.GetColor(shape, ct, c):
                rgb = c.GetRGB()
                return "#%02x%02x%02x" % tuple(int(round(max(0, min(1, v)) * 255)) for v in (rgb.Red(), rgb.Green(), rgb.Blue()))
        return None

    def mat(loc):
        t = loc.Transformation()
        return [[t.Value(r + 1, c + 1) for c in range(4)] for r in range(3)] + [[0.0, 0.0, 0.0, 1.0]]

    def mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

    # ---- the tree: every placed part, its name path and placement ----
    placed = []           # (path, label key, label, world matrix, colour)
    seen_names = {}

    def walk(label, path, M, inherited):
        shape = shapes.GetShape_s(label)
        col = color_of(label, shape) or inherited
        if shapes.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            shapes.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                c = comps.Value(i)
                ref = TDF_Label()
                if not shapes.GetReferredShape_s(c, ref):
                    continue
                cname = name_of(c)
                if not cname or cname.startswith("=>"):
                    cname = name_of(ref)
                ccol = color_of(c, shapes.GetShape_s(c)) or col
                walk(ref, path + [cname], mul(M, mat(shapes.GetShape_s(c).Location())), ccol)
            return
        placed.append((path, label.Tag(), label, M, col))

    free = TDF_LabelSequence()
    shapes.GetFreeShapes(free)
    I = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    for i in range(1, free.Length() + 1):
        lab = free.Value(i)
        walk(lab, [name_of(lab)], I, None)

    # ---- each distinct part once: facts and a mesh ----
    distinct = {}
    for path, key, label, M, col in placed:
        distinct.setdefault(key, label)
    blob = bytearray()
    shape_rows = {}
    n = len(distinct)
    for k, (key, label) in enumerate(distinct.items()):
        if k % 20 == 0:
            progress(0.2 + 0.75 * k / max(n, 1), f"measuring and meshing parts ({k} of {n})")
        nm = name_of(label)
        hw = bool(HARDWARE.search(nm))
        row = {"name": nm, "hardware": hw, "volume": 0.0, "com": [0, 0, 0], "bbox": [[0, 0, 0], [0, 0, 0]], "axes": [], "mesh": None}
        shape_rows[key] = row
        if hw and not a.hardware:
            continue
        shape = shapes.GetShape_s(label)
        # facts
        bb = Bnd_Box()
        BRepBndLib.Add_s(shape, bb)
        if not bb.IsVoid():
            lo, hi = bb.CornerMin(), bb.CornerMax()
            row["bbox"] = [[lo.X(), lo.Y(), lo.Z()], [hi.X(), hi.Y(), hi.Z()]]
        gp = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, gp)
        row["volume"] = abs(gp.Mass())
        c = gp.CentreOfMass()
        row["com"] = [c.X(), c.Y(), c.Z()]
        groups = []
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        while ex.More():
            f = face(ex.Current())
            s = BRepAdaptor_Surface(f)
            if s.GetType() == GeomAbs_Cylinder:
                cyl = s.Cylinder()
                ax = cyl.Axis()
                d, p = ax.Direction(), ax.Location()
                dv = [d.X(), d.Y(), d.Z()]
                big = max(range(3), key=lambda i: abs(dv[i]))
                if dv[big] < 0:
                    dv = [-x for x in dv]
                pv = [p.X(), p.Y(), p.Z()]
                fp = GProp_GProps()
                BRepGProp.SurfaceProperties_s(f, fp)
                area = fp.Mass()
                for g in groups:
                    dot = sum(g["d"][i] * dv[i] for i in range(3))
                    diff = [pv[i] - g["p"][i] for i in range(3)]
                    cr = [diff[1] * dv[2] - diff[2] * dv[1], diff[2] * dv[0] - diff[0] * dv[2], diff[0] * dv[1] - diff[1] * dv[0]]
                    if abs(abs(dot) - 1) < 1.5e-4 and math.sqrt(sum(x * x for x in cr)) < 5e-4:
                        g["r"].add(round(cyl.Radius(), 5))
                        g["area"] += area
                        break
                else:
                    groups.append({"d": dv, "p": pv, "r": {round(cyl.Radius(), 5)}, "area": area})
            ex.Next()
        groups.sort(key=lambda g: -g["area"])
        row["axes"] = [{"d": g["d"], "p": g["p"], "r": sorted(g["r"]), "area": g["area"]} for g in groups[:6]]
        # mesh
        BRepMesh_IncrementalMesh(shape, a.lin, False, a.ang, True)
        pos, nrm, idx = [], [], []
        ex = TopExp_Explorer(shape, TopAbs_FACE)
        while ex.More():
            f = face(ex.Current())
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(f, loc)
            if tri is not None:
                if not tri.HasNormals():
                    BRepLib_ToolTriangulatedShape.ComputeNormals_s(f, tri)
                tr = loc.Transformation()
                rev = f.Orientation() == TopAbs_REVERSED
                base = len(pos) // 3
                for i in range(1, tri.NbNodes() + 1):
                    p = tri.Node(i).Transformed(tr)
                    pos += [p.X(), p.Y(), p.Z()]
                    nn = tri.Normal(i)
                    v = [nn.X(), nn.Y(), nn.Z()]
                    # the normal turns with the face's own placement
                    m = [[tr.Value(r + 1, cc + 1) for cc in range(3)] for r in range(3)]
                    v = [sum(m[r][cc] * v[cc] for cc in range(3)) for r in range(3)]
                    if rev:
                        v = [-x for x in v]
                    nrm += v
                for i in range(1, tri.NbTriangles() + 1):
                    t1, t2, t3 = tri.Triangle(i).Get()
                    if rev:
                        t2, t3 = t3, t2
                    idx += [base + t1 - 1, base + t2 - 1, base + t3 - 1]
            ex.Next()
        if idx:
            off = len(blob)
            blob += struct.pack(f"<{len(pos)}f", *pos)
            blob += struct.pack(f"<{len(nrm)}f", *nrm)
            blob += struct.pack(f"<{len(idx)}I", *idx)
            row["mesh"] = {"offset": off, "vertices": len(pos) // 3, "indices": len(idx)}

    # ---- parts, with ids that survive a re-import (D9): the name path, then the instance number ----
    keys = list(shape_rows.keys())
    index_of = {k: i for i, k in enumerate(keys)}
    parts = []
    count = {}
    for path, key, label, M, col in placed:
        base = "/".join(path)
        count[base] = count.get(base, 0) + 1
        parts.append({"id": f"{base}#{count[base]}", "name": path[-1] if path else "", "path": path,
                      "shape": index_of[key], "m": [x for r in M[:3] for x in r], "color": col})
    out = {"format": "cadpack 1", "source": os.path.basename(a.src), "units": "m", "parts": parts,
           "shapes": [shape_rows[k] for k in keys], "seconds": round(time.time() - t0, 1)}
    with open(os.path.join(a.out, "meshes.bin"), "wb") as fh:
        fh.write(blob)
    with open(os.path.join(a.out, "parts.json"), "w") as fh:
        json.dump(out, fh)
    progress(1.0, f"done: {len(parts)} parts, {sum(1 for r in shape_rows.values() if r['mesh'])} meshed, {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
