"""Extract normalized mid-plane solid intervals from the AndyMark 1/2 hex STEP models.
Run with the project's OCP Python environment and the three downloaded STEPs in target.
"""
import json, math
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Pnt, gp_Dir, gp_Lin
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID

profiles = {}
for sku in ('am-3462','am-3945','am-3480','am-5740','am-5123'):
    reader = STEPControl_Reader()
    reader.ReadFile(str(Path('target') / (sku + '.step')))
    reader.TransferRoots()
    shape = reader.OneShape()
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box)
    lo, hi = box.CornerMin().Coord(), box.CornerMax().Coord()
    center = gp_Pnt(*[(a+b)/2 for a,b in zip(lo, hi)])
    radius = {"am-3462": 25.4, "am-3945": 38.1, "am-3480": 50.8, "am-5740": 38.1, "am-5123": 63.5}[sku]
    sol = TopExp_Explorer(shape, TopAbs_SOLID)
    probes = []
    while sol.More():
        probe = IntCurvesFace_ShapeIntersector()
        probe.Load(sol.Current(), 1e-7)
        probes.append(probe)
        sol.Next()
    profiles[sku] = []
    for i in range(180):
        angle = (i+0.5)*math.tau/180
        ray = gp_Lin(center, gp_Dir(math.cos(angle), math.sin(angle), 0))
        intervals = []
        for probe in probes:
            probe.Perform(ray, 0., radius * 1.02)
            hits = sorted(set(round(probe.WParameter(k)/radius,5) for k in range(1,probe.NbPnt()+1)))
            assert len(hits)%2 == 0, (sku,i,hits)
            intervals.extend(zip(hits[::2],hits[1::2]))
        merged = []
        for a,b in sorted(intervals):
            if merged and a <= merged[-1][1]+1e-4: merged[-1][1]=max(b,merged[-1][1])
            else: merged.append([a,b])
        assert merged, (sku,i)
        profiles[sku].append(merged)
    print(sku, profiles[sku][0], max(map(len,profiles[sku])))
Path('assets/library/wheel-sections.json').write_text(json.dumps(profiles,separators=(',',':'))+'\n')
