//! A cadpack (PLAN.md D8): what `tools/cadread` writes from a STEP file. `parts.json` lists every
//! placed part (name path, placement, colour, which distinct shape it is); `meshes.bin` holds each
//! distinct shape's triangles once.
use model::math::*;
use model::{Pose, V3};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Axis {
    /// direction (unit) and a point, in the shape's own frame
    pub d: V3,
    pub p: V3,
    /// radii of the cylinders on this axis
    pub r: Vec<f64>,
    pub area: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeshRef { pub offset: usize, pub vertices: usize, pub indices: usize }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShapeFacts {
    pub name: String,
    pub hardware: bool,
    pub volume: f64,
    pub com: V3,
    pub bbox: [V3; 2],
    pub axes: Vec<Axis>,
    pub mesh: Option<MeshRef>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PartRow {
    pub id: String,
    pub name: String,
    pub path: Vec<String>,
    pub shape: usize,
    /// 3x4 row-major placement in the CAD frame
    pub m: Vec<f64>,
    pub color: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cadpack {
    pub format: String,
    pub source: String,
    pub parts: Vec<PartRow>,
    pub shapes: Vec<ShapeFacts>,
    #[serde(default)]
    pub seconds: f64,
    #[serde(skip)]
    pub blob: Vec<u8>,
}

/// Triangles of one shape, in its own frame.
#[derive(Debug, Clone, Default)]
pub struct Mesh { pub positions: Vec<[f32; 3]>, pub normals: Vec<[f32; 3]>, pub indices: Vec<u32> }

impl PartRow {
    pub fn pose(&self) -> Pose {
        let m = &self.m;
        Pose::from_rows(&[m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]])
    }
}

impl Cadpack {
    pub fn load(dir: &Path) -> anyhow::Result<Cadpack> {
        let mut c: Cadpack = serde_json::from_str(&std::fs::read_to_string(dir.join("parts.json"))?)?;
        c.blob = std::fs::read(dir.join("meshes.bin")).unwrap_or_default();
        Ok(c)
    }
    pub fn mesh(&self, shape: usize) -> Option<Mesh> {
        let r = self.shapes.get(shape)?.mesh.as_ref()?;
        let f = |o: usize| f32::from_le_bytes(self.blob[o..o + 4].try_into().unwrap());
        let n = r.vertices;
        let mut m = Mesh::default();
        let pos0 = r.offset;
        let nrm0 = pos0 + n * 12;
        let idx0 = nrm0 + n * 12;
        if idx0 + r.indices * 4 > self.blob.len() { return None; }
        for i in 0..n {
            m.positions.push([f(pos0 + i * 12), f(pos0 + i * 12 + 4), f(pos0 + i * 12 + 8)]);
            m.normals.push([f(nrm0 + i * 12), f(nrm0 + i * 12 + 4), f(nrm0 + i * 12 + 8)]);
        }
        for i in 0..r.indices { m.indices.push(u32::from_le_bytes(self.blob[idx0 + i * 4..idx0 + i * 4 + 4].try_into().unwrap())); }
        Some(m)
    }
    /// a part's facts in the CAD frame: (volume, centre of mass, world bounding box corners)
    pub fn facts(&self, p: &PartRow) -> (f64, V3, [V3; 2]) {
        let s = &self.shapes[p.shape];
        let pose = p.pose();
        let com = pose.apply(s.com);
        let (lo, hi) = (s.bbox[0], s.bbox[1]);
        let mut a = [f64::INFINITY; 3];
        let mut b = [f64::NEG_INFINITY; 3];
        for i in 0..8 {
            let c = [if i & 1 == 0 { lo[0] } else { hi[0] }, if i & 2 == 0 { lo[1] } else { hi[1] }, if i & 4 == 0 { lo[2] } else { hi[2] }];
            let w = pose.apply(c);
            for k in 0..3 { a[k] = a[k].min(w[k]); b[k] = b[k].max(w[k]); }
        }
        (s.volume, com, [a, b])
    }
    /// a part's cylinder axes in the CAD frame
    pub fn axes(&self, p: &PartRow) -> Vec<Axis> {
        let pose = p.pose();
        self.shapes[p.shape].axes.iter().map(|a| Axis { d: pose.rotate(a.d), p: pose.apply(a.p), r: a.r.clone(), area: a.area }).collect()
    }
    pub fn is_hardware(&self, p: &PartRow) -> bool { self.shapes[p.shape].hardware }
}

/// The CAD frame to the robot frame: which way is forward and up in the CAD, and where the floor and
/// centre are (the importer's two questions).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Dir { PX, NX, PY, NY, PZ, NZ }

impl Dir {
    pub const ALL: [Dir; 6] = [Dir::PX, Dir::NX, Dir::PY, Dir::NY, Dir::PZ, Dir::NZ];
    pub fn v(self) -> V3 { match self { Dir::PX => [1.0, 0.0, 0.0], Dir::NX => [-1.0, 0.0, 0.0], Dir::PY => [0.0, 1.0, 0.0], Dir::NY => [0.0, -1.0, 0.0], Dir::PZ => [0.0, 0.0, 1.0], Dir::NZ => [0.0, 0.0, -1.0] } }
    pub fn label(self) -> &'static str { match self { Dir::PX => "+X", Dir::NX => "-X", Dir::PY => "+Y", Dir::NY => "-Y", Dir::PZ => "+Z", Dir::NZ => "-Z" } }
}

/// The rotation taking CAD `forward` to +x and CAD `up` to +z, and a shift putting the lowest
/// non-hardware point on the floor under the parts' centre.
pub fn to_robot(c: &Cadpack, forward: Dir, up: Dir) -> Pose {
    let f = forward.v();
    let u = up.v();
    let l = cross(u, f);
    // rows: robot x, y, z in CAD coordinates
    let m = [f[0], f[1], f[2], l[0], l[1], l[2], u[0], u[1], u[2]];
    let rot = Pose::new([0.0; 3], mat_to_quat(m));
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &c.parts {
        if c.is_hardware(p) || c.shapes[p.shape].volume <= 0.0 { continue; }
        let (_, _, bb) = c.facts(p);
        for i in 0..8 {
            let q = rot.apply([bb[i & 1][0], bb[(i >> 1) & 1][1], bb[(i >> 2) & 1][2]]);
            for k in 0..3 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); }
        }
    }
    if !lo[0].is_finite() { return rot; }
    let shift = [-(lo[0] + hi[0]) / 2.0, -(lo[1] + hi[1]) / 2.0, -lo[2]];
    Pose::new(shift, rot.quat)
}

/// A good guess at which way is up: the axis the parts are least spread along is usually not it
/// (robots are about as tall as they are wide); the most common FRC habit is +Z up, Onshape's default.
pub fn guess_up(_c: &Cadpack) -> Dir { Dir::PZ }
