"""Writes a small STEP assembly for testing cadread: a base plate with two rollers (a 2 in. and a
4 in. 'Compliant Wheel') and an arm, each a named part in a named sub-assembly.

    python tools/cadread/make_test_step.py out.step
"""
import sys

from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool

IN = 0.0254 * 1000  # the STEP is written in mm

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name(label, s):
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(s))


def at(x, y, z):
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(x, y, z))
    return TopLoc_Location(t)


top = tool.NewShape()
name(top, "Test Robot")
base = tool.NewShape()
name(base, "10 - Drivebase")
plate = tool.AddShape(BRepPrimAPI_MakeBox(27 * IN, 27 * IN, 0.25 * IN).Shape(), False)
name(plate, "Belly Pan")
c = tool.AddComponent(base, plate, at(-13.5 * IN, -13.5 * IN, 2 * IN))
intake = tool.NewShape()
name(intake, "20 - Intake")
wheel4 = tool.AddShape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), 2 * IN, 1 * IN).Shape(), False)
name(wheel4, "4in Compliant Wheel 35A")
for k in range(3):
    tool.AddComponent(intake, wheel4, at(16 * IN, (-6 + 5 * k) * IN, 6 * IN))
arm = tool.NewShape()
name(arm, "30 - Arm")
tube = tool.AddShape(BRepPrimAPI_MakeBox(20 * IN, 1 * IN, 2 * IN).Shape(), False)
name(tube, "Arm Tube")
tool.AddComponent(arm, tube, at(0, -0.5 * IN, 20 * IN))
screw = tool.AddShape(BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 0.1 * IN, 0.5 * IN).Shape(), False)
name(screw, "Socket head cap screw #10-32 x 0.5")
tool.AddComponent(arm, screw, at(1 * IN, 0, 22 * IN))
for sub in (base, intake, arm):
    tool.AddComponent(top, sub, TopLoc_Location())
tool.UpdateAssemblies()
w = STEPCAFControl_Writer()
w.SetNameMode(True)
w.Transfer(doc)
assert w.Write(sys.argv[1]) == IFSelect_RetDone
print("wrote", sys.argv[1])
