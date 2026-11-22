"""Solve unit-modulus, unit-width plane-stress wheel sections at a reference flat-plate indentation.

Reads extract-wheel-sections.py output. NumPy and SciPy are build-time tools only.
The sparse solve accounts for bending around holes. The result is a small-strain
estimate; large finger deflection, hysteresis and pocket self-contact need physical testing.
"""
import json
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu

assets = Path("assets/library")
results = {}
# Rigid core bounds from the individual manufacturer's STEP solids (fractions of OD radius).
cores = {"am-3462": 0., "am-3945": 10/38.1, "am-3480": 20.955/50.8, "am-5740": 10/38.1, "am-5123": 10/63.5}
for sku, bands in json.loads((assets/"wheel-sections.json").read_text()).items():
    sectors, nr = len(bands), 48
    angle = np.arange(sectors)*2*np.pi/sectors
    unit = np.stack([np.cos(angle), np.sin(angle)],axis=1)
    xy = (unit[:,None,:]*np.arange(nr+1)[None,:,None]/nr).reshape(-1,2)
    def node(i,j): return (i%sectors)*(nr+1)+j
    triangles, pins, outside = [], set(), []
    for i, spans in enumerate(bands):
        cells = [j for j in range(nr) if any(a <= (j+0.5)/nr <= b for a,b in spans)]
        for j in cells:
            a,b,c,d = node(i,j),node(i,j+1),node(i+1,j+1),node(i+1,j)
            triangles.extend(((a,b,c),(a,c,d)))
            if j == cells[0] or j/nr < cores[sku]:
                pins.update((a,d))
        outside.append(node(i,cells[-1]+1))
    tri = np.array(triangles)
    coords = xy[tri]
    b = np.roll(coords[:,:,1],-1,axis=1)-np.roll(coords[:,:,1],-2,axis=1)
    c = np.roll(coords[:,:,0],-2,axis=1)-np.roll(coords[:,:,0],-1,axis=1)
    area2 = np.sum(coords[:,:,0]*b,axis=1)
    B = np.zeros((len(tri),3,6))
    B[:,0,::2], B[:,1,1::2], B[:,2,::2], B[:,2,1::2] = b,c,c,b
    nu = 0.47
    D = np.array([[1,nu,0],[nu,1,0],[0,0,(1-nu)/2]])/(1-nu**2)
    stiffness = (B.transpose(0,2,1)@D@B)/(2*area2[:,None,None])
    dofs = np.stack((2*tri,2*tri+1),axis=-1).reshape(-1,6)
    ndof = 2*len(xy)
    K = coo_matrix((stiffness.ravel(),(np.repeat(dofs,6,axis=1).ravel(),np.tile(dofs,(1,6)).ravel())),shape=(ndof,ndof)).tocsc()
    free = np.setdiff1d(np.unique(dofs),[2*n+d for n in pins for d in (0,1)])
    solver = splu(K[free,:][:,free])
    outside = np.array(outside)
    # Flat platen with a pressure distribution proportional to initial overlap.
    # Work-conjugate mean displacement is f.T K^-1 f for a unit resultant.
    values = []
    for i in range(sectors):
        normal = unit[i]
        reach = xy[outside]@normal
        weights = np.maximum(reach - (reach.max()-0.1),0.)
        weights /= weights.sum()
        f = np.zeros((len(xy),2))
        f[outside] = weights[:,None]*normal
        load = f.ravel()[free]
        compliance = load @ solver.solve(load)
        # Dimensionless k_A R/E, equivalent to the full-wheel plate force at 10% R.
        values.append(round(0.1/(compliance*(4/3)*2**0.5*0.1**1.5),6))
    results[sku] = values
    print(sku, 'k_A R/E', min(values),sum(values)/len(values),max(values),flush=True)
(assets/"wheel-stiffness.json").write_text(json.dumps(results,separators=(',',':'))+'\n')
