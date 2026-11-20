//! The roller-to-piece contact (PLAN.md D4): an elastic foundation on the roller, in real units,
//! sliced along its width, with a stick-slip bristle per slice.
//!
//! Per slice, the rim point facing the piece is found from the piece's signed distance; the depth
//! there and the two surfaces' curvatures give the force by the foundation law
//! `f = k_A w (4/3) sqrt(2 R_eff) d^1.5`. A piece that gives too (an inflated ball, foam) takes its
//! share of the depth, so the two forces match. Friction is a bristle that stretches with slip and
//! lets go at `mu_static f`, sliding at `mu_sliding f` until the slip stops.
use crate::piece::{Law, PieceKind};
use model::math::*;
use model::{RollerKind, V3};
use std::collections::HashMap;

/// A roller's contact properties.
#[derive(Debug, Clone)]
pub struct RollerSurface {
    pub radius: f64,
    pub width: f64,
    /// foundation stiffness, Pa per m of squeeze
    pub k_area: f64,
    pub mu_static: f64,
    pub mu_sliding: f64,
    pub kind: RollerKind,
    /// the tread's hysteresis: pressure is higher going in than coming out by this share
    pub loss: f64,
    pub slices: usize,
}

impl RollerSurface {
    pub fn slices_for(width: f64) -> usize { ((width / 0.0125).ceil() as usize).clamp(1, 24) }
}

/// Where the roller is now.
#[derive(Debug, Clone, Copy)]
pub struct RollerState {
    pub center: V3,
    pub axis: V3,
    /// its spin about `axis`, rad/s
    pub spin: f64,
    /// the velocity of the link carrying it, at its centre, and the link's angular velocity
    pub lin: V3,
    pub ang: V3,
}

impl RollerState {
    /// the roller's surface velocity at a point
    pub fn surface_velocity(&self, p: V3) -> V3 {
        let r = sub(p, self.center);
        let link = add(self.lin, cross(self.ang, r));
        add(link, cross(scale(self.axis, self.spin), r))
    }
}

/// Where a piece is now.
#[derive(Debug, Clone, Copy)]
pub struct PieceState {
    pub pos: V3,
    /// row-major rotation, piece frame to world
    pub mat: [f64; 9],
    pub lin: V3,
    pub ang: V3,
}

impl PieceState {
    pub fn to_local(&self, p: V3) -> V3 {
        let d = sub(p, self.pos);
        let m = self.mat;
        [m[0] * d[0] + m[3] * d[1] + m[6] * d[2], m[1] * d[0] + m[4] * d[1] + m[7] * d[2], m[2] * d[0] + m[5] * d[1] + m[8] * d[2]]
    }
    pub fn dir_to_world(&self, v: V3) -> V3 {
        let m = self.mat;
        [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]]
    }
    pub fn dir_to_local(&self, v: V3) -> V3 {
        let m = self.mat;
        [m[0] * v[0] + m[3] * v[1] + m[6] * v[2], m[1] * v[0] + m[4] * v[1] + m[7] * v[2], m[2] * v[0] + m[5] * v[1] + m[8] * v[2]]
    }
    pub fn velocity_at(&self, p: V3) -> V3 { add(self.lin, cross(self.ang, sub(p, self.pos))) }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Bristle { pub s: V3, pub sliding: bool }

/// One slice's contact this step.
#[derive(Debug, Clone, Copy)]
pub struct SliceContact {
    pub point: V3,
    /// force on the piece, N (the roller gets the opposite)
    pub force: V3,
    /// unit, from the roller into the piece
    pub normal: V3,
    pub depth: f64,
    /// the wheel's own squeeze (the rest is the piece's)
    pub wheel_depth: f64,
    pub normal_force: f64,
    pub sliding: bool,
    /// normal stiffness, N/m (for the step-rate check)
    pub stiffness: f64,
}

/// The bristles of every roller slice touching every piece, kept between steps.
pub type Bristles = HashMap<(u32, u16, u32), Bristle>;

const MU_FREE: f64 = 0.04;

/// Contacts between one roller and one piece. `key` names the roller and piece for the bristles.
#[allow(clippy::too_many_arguments)]
pub fn roller_piece(
    surf: &RollerSurface, rs: &RollerState, kind: &PieceKind, ps: &PieceState, dt: f64,
    bristles: &mut Bristles, roller_key: u32, piece_key: u32, out: &mut Vec<SliceContact>,
) {
    // far apart: nothing, and forget any bristles
    let reach = surf.radius + kind.bound() + surf.width / 2.0;
    if len(sub(ps.pos, rs.center)) > reach {
        for s in 0..surf.slices { bristles.remove(&(roller_key, s as u16, piece_key)); }
        return;
    }
    let n = surf.slices;
    let w = surf.width / n as f64;
    for s in 0..n {
        let key = (roller_key, s as u16, piece_key);
        let t = (s as f64 + 0.5) / n as f64 - 0.5;
        let c = add(rs.center, scale(rs.axis, t * surf.width));
        // the direction from the slice's centre to the piece, across the roller's axis
        let cl = ps.to_local(c);
        let g = ps.dir_to_world(kind.grad_local(cl));
        let toward = neg(g);
        let across = sub(toward, scale(rs.axis, dot(toward, rs.axis)));
        if len(across) < 1e-6 { bristles.remove(&key); continue; }
        let nrm = norm(across);
        let rim = add(c, scale(nrm, surf.radius));
        let rim_l = ps.to_local(rim);
        let depth = -kind.sdf_local(rim_l);
        if depth <= 0.0 { bristles.remove(&key); continue; }
        // relative curvature in the roller's plane
        let tw = cross(rs.axis, nrm);
        let kp = kind.curvature_local(rim_l, ps.dir_to_local(tw));
        let r_eff = 1.0 / (1.0 / surf.radius + kp);
        let fw = |d: f64| surf.k_area * w * (4.0 / 3.0) * (2.0 * r_eff).sqrt() * d.max(0.0).powf(1.5);
        // the depth's split between the wheel and the piece
        let dw = match kind.law {
            Law::Rigid => depth,
            _ => {
                let (mut lo, mut hi) = (0.0, depth);
                for _ in 0..32 {
                    let m = 0.5 * (lo + hi);
                    if fw(m) > kind.force(depth - m, w, r_eff, surf.radius) { hi = m; } else { lo = m; }
                }
                0.5 * (lo + hi)
            }
        };
        let f_static = fw(dw);
        // where the two surfaces meet
        let point = sub(rim, scale(nrm, depth - dw));
        let vs = rs.surface_velocity(point);
        let vp = ps.velocity_at(point);
        let vrel = sub(vs, vp);
        let closing = dot(vrel, nrm);
        let fnorm = (f_static * (1.0 + surf.loss * (closing / 0.05).tanh())).max(0.0);
        // the wheel and the piece in series
        let kw = 1.5 * f_static / dw.max(1e-6);
        let kpiece = match kind.law { Law::Rigid => f64::INFINITY, _ => { let dp = (depth - dw).max(1e-6); (kind.force(dp * 1.01, w, r_eff, surf.radius) - kind.force(dp, w, r_eff, surf.radius)) / (0.01 * dp) } };
        let k = 1.0 / (1.0 / kw.max(1.0) + 1.0 / kpiece.max(1.0));
        // friction: the bristle stretches with slip
        let vt = sub(vrel, scale(nrm, dot(vrel, nrm)));
        let b = bristles.entry(key).or_default();
        b.s = add(b.s, scale(vt, dt));
        b.s = sub(b.s, scale(nrm, dot(b.s, nrm)));
        let kt = 0.8 * k;
        let mut ft = scale(b.s, kt);
        let (grip, free) = match surf.kind {
            RollerKind::Plain => (None, None),
            RollerKind::Omni => (Some(norm(tw)), Some(rs.axis)),
            RollerKind::Mecanum => {
                let gdir = norm(add(scale(norm(tw), std::f64::consts::FRAC_1_SQRT_2), scale(rs.axis, std::f64::consts::FRAC_1_SQRT_2)));
                (Some(gdir), Some(norm(cross(nrm, gdir))))
            }
        };
        let cap_s = surf.mu_static * fnorm;
        let cap_k = surf.mu_sliding * fnorm;
        if let (Some(gd), Some(fd)) = (grip, free) {
            // a wheel of rollers grips one way only
            let along = dot(ft, gd);
            let cross_ = dot(ft, fd);
            let cap = if b.sliding { cap_k } else { cap_s };
            let a2 = along.clamp(-cap, cap);
            let c2 = cross_.clamp(-MU_FREE * fnorm, MU_FREE * fnorm);
            b.sliding = along.abs() > cap;
            ft = add(scale(gd, a2), scale(fd, c2));
            b.s = scale(ft, 1.0 / kt.max(1e-9));
        } else {
            let mag = len(ft);
            let cap = if b.sliding { cap_k } else { cap_s };
            if mag > cap && mag > 0.0 {
                ft = scale(ft, cap_k / mag);
                b.s = scale(ft, 1.0 / kt.max(1e-9));
                b.sliding = true;
            } else if b.sliding && len(vt) < 0.01 {
                b.sliding = false;
            }
        }
        out.push(SliceContact { point, force: add(scale(nrm, fnorm), ft), normal: nrm, depth, wheel_depth: dw, normal_force: fnorm, sliding: b.sliding, stiffness: k });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::piece::PieceKind;

    fn wheel(id: &str, w: f64) -> RollerSurface {
        let lib = library::Library::builtin();
        let wh = lib.wheel(id).unwrap();
        RollerSurface { radius: wh.radius(), width: w, k_area: wh.k_area(), mu_static: 1.1, mu_sliding: 0.9, kind: RollerKind::Plain, loss: 0.0, slices: RollerSurface::slices_for(w) }
    }

    #[test]
    fn a_wheel_on_a_flat_box_matches_the_press_formula() {
        let s = wheel("am-compliant-4in-35a", 0.0254);
        let kind = PieceKind { id: "plate".into(), name: "plate".into(), shape: crate::piece::Shape::Box { half: [0.3, 0.3, 0.05] }, mass: 1.0, law: Law::Rigid, surface: "pvc".into(), color: String::new(), measured: true };
        let d = 0.006;
        // the wheel's axis along y, its bottom d into the plate's top at z = 0
        let rs = RollerState { center: [0.0, 0.0, s.radius - d], axis: [0.0, 1.0, 0.0], spin: 0.0, lin: [0.0; 3], ang: [0.0; 3] };
        let ps = PieceState { pos: [0.0, 0.0, -0.05], mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], lin: [0.0; 3], ang: [0.0; 3] };
        let mut out = vec![];
        roller_piece(&s, &rs, &kind, &ps, 1e-3, &mut Bristles::new(), 0, 0, &mut out);
        let f: f64 = -out.iter().map(|c| c.force[2]).sum::<f64>();
        let want = library::stiffness::plate_force(s.k_area, s.radius, s.width, d);
        assert!((f / want - 1.0).abs() < 1e-6, "{f} vs {want}");
    }

    #[test]
    fn a_soft_ball_takes_some_of_the_squeeze() {
        let lib = library::Library::builtin();
        let algae = PieceKind::from_library(lib.piece("algae-2025").unwrap());
        let s = wheel("am-compliant-4in-60a", 0.0254);
        let crate::piece::Shape::Sphere { r } = algae.shape else { panic!() };
        let rs = RollerState { center: [0.0, 0.0, r + s.radius - 0.01], axis: [0.0, 1.0, 0.0], spin: 0.0, lin: [0.0; 3], ang: [0.0; 3] };
        let ps = PieceState { pos: [0.0; 3], mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], lin: [0.0; 3], ang: [0.0; 3] };
        let mut out = vec![];
        roller_piece(&s, &rs, &algae, &ps, 1e-3, &mut Bristles::new(), 0, 0, &mut out);
        let c = out[0];
        assert!(c.wheel_depth > 0.0 && c.wheel_depth < c.depth, "{c:?}");
        assert!(c.force[2] < 0.0, "pushes the ball down");
    }
}
