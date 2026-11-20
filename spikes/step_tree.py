"""Every part of a STEP assembly with where it sits: its names down the tree and its 4x4 placement.

No geometry is measured, so this is the quick half of `step-axes.py` (rebuilt-sim-2026/tools). It
feeds `pose_diff.py`. Needs OpenCascade's Python bindings:

    pip install cadquery-ocp numpy
    python spikes/step_tree.py robot.step tree.json
"""
import json
import sys

import numpy as np
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool

src, out = sys.argv[1:3]
doc = TDocStd_Document(TCollection_ExtendedString("step"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
Interface_Static.SetCVal_s("xstep.cascade.unit", "M")
if reader.ReadFile(src) != IFSelect_RetDone:
    sys.exit("could not read " + src)
reader.Transfer(doc)
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name_of(label):
    n = TDataStd_Name()
    return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""


def matrix(loc):
    t = loc.Transformation()
    m = np.eye(4)
    for r in range(3):
        for c in range(4):
            m[r, c] = t.Value(r + 1, c + 1)
    return m


rows = []


def walk(label, world, path):
    if not tool.IsAssembly_s(label):
        rows.append({"path": path, "part": name_of(label) or path[-1],
                     "m": [round(float(x), 9) for x in world.flatten()]})
        return
    comps = TDF_LabelSequence()
    tool.GetComponents_s(label, comps, False)
    for i in range(1, comps.Length() + 1):
        comp = comps.Value(i)
        ref = TDF_Label()
        if not tool.GetReferredShape_s(comp, ref):
            continue
        walk(ref, world @ matrix(tool.GetShape_s(comp).Location()), path + [name_of(comp) or name_of(ref)])


free = TDF_LabelSequence()
tool.GetFreeShapes(free)
for i in range(1, free.Length() + 1):
    walk(free.Value(i), np.eye(4), [name_of(free.Value(i))])
json.dump(rows, open(out, "w"))
print(len(rows), "parts")
