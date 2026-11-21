//! Collision shapes from CAD meshes (PLAN.md D7, M5). MuJoCo collides meshes by their convex hull,
//! so each part becomes one or more point sets whose hulls follow it:
//!
//! - a solid part (a hub, a gear, a block) is one hull;
//! - a thin part (polycarbonate, sheet metal, the walls of a tube or a C-channel) is split into
//!   flat slabs, one per face direction and depth, so a bent guide or a channel stays open instead
//!   of becoming a solid block.
//!
//! Each set is cut down to its support points along a fixed set of directions, which keeps the
//! robot file small and the hull faithful where it matters (its outline).
use crate::cadpack::Mesh;
use model::math::*;
use model::{Pose, V3};

/// thinner than this (twice the volume over the area) and a part is a sheet
const SHEET: f64 = 0.008;
/// faces within this angle are one slab direction
const SAME_DIR_DEG: f64 = 20.0;
/// a slab is at least this thick, so its hull isn't flat
const MIN_THICK: f64 = 0.001;
/// at most this many slabs from one part; more than that and it's one hull
const MAX_SLABS: usize = 16;

/// Directions to take support points along: the axes, the diagonals, and a spiral round the
/// sphere. 74 in all.
fn directions() -> &'static [V3] {
    static D: std::sync::OnceLock<Vec<V3>> = std::sync::OnceLock::new();
    D.get_or_init(|| {
        let mut d: Vec<V3> = vec![];
        for a in [-1.0, 0.0, 1.0] { for b in [-1.0, 0.0, 1.0] { for c in [-1.0, 0.0, 1.0] {
            if a != 0.0 || b != 0.0 || c != 0.0 { d.push(norm([a, b, c])); }
        } } }
        let n = 48;
        let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
        for i in 0..n {
            let z = 1.0 - 2.0 * (i as f64 + 0.5) / n as f64;
            let r = (1.0 - z * z).sqrt();
            let t = golden * i as f64;
            d.push([r * t.cos(), r * t.sin(), z]);
        }
        d
    })
}

/// The points of `pts` that stick out furthest along each direction (no repeats).
pub fn support(pts: &[V3]) -> Vec<V3> {
    let mut out: Vec<V3> = vec![];
    for d in directions() {
        let Some(p) = pts.iter().max_by(|a, b| dot(**a, *d).total_cmp(&dot(**b, *d))) else { break };
        if !out.iter().any(|q| len(sub(*q, *p)) < 1e-6) { out.push(*p); }
    }
    out
}

/// One part's collision point sets, in the frame `pose` takes the mesh to. `volume` is the
/// part's own (from the CAD), for telling sheets from solids.
pub fn part_hulls(mesh: &Mesh, pose: &Pose, volume: f64) -> Vec<Vec<V3>> {
    let v: Vec<V3> = mesh.positions.iter().map(|p| pose.apply([p[0] as f64, p[1] as f64, p[2] as f64])).collect();
    if v.len() < 4 { return vec![]; }
    let tris: Vec<[usize; 3]> = mesh.indices.chunks_exact(3).map(|t| [t[0] as usize, t[1] as usize, t[2] as usize]).filter(|t| t.iter().all(|i| *i < v.len())).collect();
    let mut area = 0.0;
    let faces: Vec<(V3, f64, V3)> = tris.iter().map(|t| {
        let c = cross(sub(v[t[1]], v[t[0]]), sub(v[t[2]], v[t[0]]));
        let a = 0.5 * len(c);
        area += a;
        let centre = scale(add(add(v[t[0]], v[t[1]]), v[t[2]]), 1.0 / 3.0);
        (if a > 0.0 { scale(c, 0.5 / a) } else { [0.0, 0.0, 1.0] }, a, centre)
    }).collect();
    let thick = if area > 0.0 && volume > 0.0 { 2.0 * volume / area } else { f64::INFINITY };
    let (lo, hi) = bounds(&v);
    let biggest = (0..3).map(|k| hi[k] - lo[k]).fold(0.0, f64::max);
    if thick >= SHEET || biggest < 8.0 * thick {
        return vec![thicken(support(&v), None)];
    }
    // a sheet: group the faces by direction (either way along it), then by depth along it
    let cos = SAME_DIR_DEG.to_radians().cos();
    let mut order: Vec<usize> = (0..faces.len()).collect();
    order.sort_by(|a, b| faces[*b].1.total_cmp(&faces[*a].1));
    let mut dirs: Vec<(V3, f64, Vec<usize>)> = vec![];
    for i in order {
        let (n, a, _) = faces[i];
        if a <= 0.0 { continue; }
        match dirs.iter_mut().find(|d| dot(d.0, n).abs() >= cos) {
            Some(d) => { d.1 += a; d.2.push(i); }
            None => dirs.push((n, a, vec![i])),
        }
    }
    // the rims of a sheet (its edges, the walls of its holes) are small: drop them
    let keep = 0.05 * area;
    let mut slabs: Vec<Vec<V3>> = vec![];
    let gap = (3.0 * thick).max(0.003);
    for (n, a, fs) in dirs.iter().filter(|d| d.1 >= keep) {
        let _ = a;
        let mut by_depth: Vec<(f64, usize)> = fs.iter().map(|&i| (dot(faces[i].2, *n), i)).collect();
        by_depth.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut group: Vec<usize> = vec![];
        let mut last = f64::NEG_INFINITY;
        // a group at one depth is kept only if it's a real face, not the rims of other faces
        // (a channel's flange edges would otherwise close the channel)
        let mut flush = |g: &mut Vec<usize>| {
            let ga: f64 = g.iter().map(|&i| faces[i].1).sum();
            if g.is_empty() || ga < keep { g.clear(); return; }
            let pts: Vec<V3> = g.iter().flat_map(|&i| tris[i].iter().map(|&k| v[k])).collect();
            slabs.push(thicken(support(&pts), Some(*n)));
            g.clear();
        };
        for (d, i) in by_depth {
            if d - last > gap { flush(&mut group); }
            group.push(i);
            last = d;
        }
        flush(&mut group);
    }
    if slabs.is_empty() || slabs.len() > MAX_SLABS { return vec![thicken(support(&v), None)]; }
    slabs
}

fn bounds(v: &[V3]) -> (V3, V3) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in v { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    (lo, hi)
}

/// A point set flat along `n` (or along its thinnest box side) gets a copy moved off that plane,
/// so its hull has some thickness.
fn thicken(mut pts: Vec<V3>, n: Option<V3>) -> Vec<V3> {
    if pts.is_empty() { return pts; }
    let n = n.unwrap_or_else(|| {
        let (lo, hi) = bounds(&pts);
        let k = (0..3).min_by(|a, b| (hi[*a] - lo[*a]).total_cmp(&(hi[*b] - lo[*b]))).unwrap();
        let mut e = [0.0; 3];
        e[k] = 1.0;
        e
    });
    let ds: Vec<f64> = pts.iter().map(|p| dot(*p, n)).collect();
    let span = ds.iter().cloned().fold(f64::NEG_INFINITY, f64::max) - ds.iter().cloned().fold(f64::INFINITY, f64::min);
    if span < MIN_THICK || pts.len() < 4 {
        let extra: Vec<V3> = pts.iter().map(|p| add(*p, scale(n, MIN_THICK - span.max(0.0)))).collect();
        pts.extend(extra);
    }
    pts
}

#[cfg(test)]
mod tests {
    use super::*;

    /// a box as a mesh (12 triangles), `lo` to `hi`
    fn boxed(lo: V3, hi: V3) -> (Vec<V3>, Vec<[usize; 3]>) {
        let v: Vec<V3> = (0..8).map(|i| [if i & 1 == 0 { lo[0] } else { hi[0] }, if i & 2 == 0 { lo[1] } else { hi[1] }, if i & 4 == 0 { lo[2] } else { hi[2] }]).collect();
        let q = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
        let t = q.iter().flat_map(|f| [[f[0], f[1], f[2]], [f[0], f[2], f[3]]]).collect();
        (v, t)
    }
    fn mesh(parts: &[(V3, V3)]) -> Mesh {
        let mut m = Mesh::default();
        for (lo, hi) in parts {
            let (v, t) = boxed(*lo, *hi);
            let o = m.positions.len() as u32;
            m.positions.extend(v.iter().map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]));
            m.indices.extend(t.iter().flat_map(|t| t.iter().map(|i| *i as u32 + o)));
        }
        m
    }

    #[test]
    fn a_block_is_one_hull() {
        let m = mesh(&[([0.0; 3], [0.1, 0.05, 0.05])]);
        let h = part_hulls(&m, &Pose::default(), 0.1 * 0.05 * 0.05);
        assert_eq!(h.len(), 1);
        assert_eq!(h[0].len(), 8, "a box's hull is its corners");
    }

    #[test]
    fn a_channel_stays_open() {
        // a C-channel 3 mm thick: a web and two flanges, as one part
        let t = 0.003;
        let parts = [([0.0, 0.0, 0.0], [0.5, t, 0.05]), ([0.0, 0.0, 0.0], [0.5, 0.04, t]), ([0.0, 0.0, 0.05 - t], [0.5, 0.04, 0.05])];
        let vol = 0.5 * t * 0.05 + 2.0 * 0.5 * 0.04 * t;
        let h = part_hulls(&mesh(&parts), &Pose::default(), vol);
        assert!(h.len() >= 3, "{} slabs", h.len());
        // nothing fills the channel's middle
        let mid = [0.25, 0.02, 0.025];
        for s in &h {
            let (lo, hi) = bounds(s);
            let inside = (0..3).all(|k| mid[k] > lo[k] + 1e-4 && mid[k] < hi[k] - 1e-4);
            assert!(!inside, "a slab fills the channel: {lo:?} {hi:?}");
        }
    }

    #[test]
    fn a_flat_sheet_keeps_some_thickness() {
        let m = mesh(&[([0.0; 3], [0.3, 0.2, 0.0005])]);
        let h = part_hulls(&m, &Pose::default(), 0.3 * 0.2 * 0.0005);
        assert_eq!(h.len(), 1);
        let (lo, hi) = bounds(&h[0]);
        assert!(hi[2] - lo[2] >= MIN_THICK - 1e-9);
    }
}
