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
    pub section: Option<std::sync::Arc<library::profile::Section>>,
    pub mean_support: f64,
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
    pub phase: f64,
    pub zero: V3,
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
        for i in 0..3 * surf.slices { bristles.remove(&(roller_key, i as u16, piece_key)); }
        return;
    }
    let n = surf.slices;
    let w = surf.width / n as f64;
    for s in 0..n {
        let key = (roller_key, (3 * s) as u16, piece_key);
        let clear = |bristles: &mut Bristles| { for i in 0..3 { bristles.remove(&(roller_key, key.1 + i, piece_key)); } };
        let t = (s as f64 + 0.5) / n as f64 - 0.5;
        let c = add(rs.center, scale(rs.axis, t * surf.width));
        // the direction from the slice's centre to the piece, across the roller's axis
        let cl = ps.to_local(c);
        let g = ps.dir_to_world(kind.grad_local(cl));
        let toward = neg(g);
        let across = sub(toward, scale(rs.axis, dot(toward, rs.axis)));
        if len(across) < 1e-6 { clear(bristles); continue; }
        let nrm = norm(across);
        let mut rim = add(c, scale(nrm, surf.radius));
        let mut local_k = surf.k_area;
        if let Some(section) = &surf.section {
            let zero = rs.zero;
            let view = dot(nrm, cross(rs.axis, zero)).atan2(dot(nrm, zero)) - rs.phase;
            let mut index = library::profile::Section::index(view);
            if section.irregular {
                // Look for the deepest actual lobe; a nearby tip can touch before the valley.
                let sideways = cross(rs.axis, nrm);
                let mut deepest = f64::NEG_INFINITY;
                let middle = index as isize;
                let sectors = library::profile::SECTORS as isize;
                for offset in -30..=30 {
                    let at = (middle + offset).rem_euclid(sectors) as usize;
                    let angle = std::f64::consts::TAU * (middle + offset) as f64 / sectors as f64
                        + std::f64::consts::PI / sectors as f64 - view.rem_euclid(std::f64::consts::TAU);
                    let radial = add(scale(nrm, angle.cos()), scale(sideways, angle.sin()));
                    let tip = add(c, scale(radial, surf.radius * section.edge(at)));
                    let depth = kind.wheel_penetration(cl, ps.to_local(tip));
                    if depth > deepest { deepest = depth; rim = tip; index = at; }
                }
            }
            local_k *= section.support[index] / surf.mean_support;
        }
        let rim_l = ps.to_local(rim);
        let depth = kind.wheel_penetration(cl, rim_l);
        if depth <= 0.0 { clear(bristles); continue; }
        // relative curvature in the roller's plane
        let tw = cross(rs.axis, nrm);
        let kp = kind.curvature_local(rim_l, ps.dir_to_local(tw));
        let r_eff = 1.0 / (1.0 / surf.radius + kp);
        let fw = |d: f64| local_k * w * (4.0 / 3.0) * (2.0 * r_eff).sqrt() * d.max(0.0).powf(1.5);
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
        let point = sub(rim, scale(nrm, dw));
        // the wheel and the piece in series
        let kw = 1.5 * f_static / dw.max(1e-6);
        let kpiece = match kind.law { Law::Rigid => f64::INFINITY, _ => { let dp = (depth - dw).max(1e-6); (kind.force(dp * 1.01, w, r_eff, surf.radius) - kind.force(dp, w, r_eff, surf.radius)) / (0.01 * dp) } };
        let k = 1.0 / (1.0 / kw.max(1.0) + 1.0 / kpiece.max(1.0));
        // Integrate the pressure footprint on a curved piece. Each station has its
        // own surface normal, moment arm and tangential spring as the tread wraps.
        let stations = wall_patch(kind, ps, point, nrm, tw, r_eff, dw);
        for (i, (point, normal, weight)) in stations.into_iter().enumerate() {
            let station = (roller_key, key.1 + i as u16, piece_key);
            if weight == 0.0 { bristles.remove(&station); continue; }
            let vrel = sub(rs.surface_velocity(point), ps.velocity_at(point));
            let closing = dot(vrel, normal);
            let fnorm = f_static * weight * (1.0 + surf.loss * (closing / 0.05).tanh());
            let stiffness = k * weight;
            let (ft, sliding) = tread_friction(surf, rs.axis, normal, vrel, dt, stiffness, fnorm, bristles.entry(station).or_default());
            out.push(SliceContact { point, force: add(scale(normal, fnorm), ft), normal, depth, wheel_depth: dw, normal_force: fnorm, sliding, stiffness });
        }
    }
}

/// Three pressure-weighted stations across the footprint. For the parabolic
/// foundation pressure, a pair at +/-sqrt(3/5) of the half-width with 1/6
/// of the load each reproduces the second moment of the pressure distribution.
fn wall_patch(kind: &PieceKind, ps: &PieceState, point: V3, normal: V3, tangent: V3, r_eff: f64, compression: f64) -> [(V3, V3, f64); 3] {
    let mut patch = [(point, normal, 1.), (point, normal, 0.), (point, normal, 0.)];
    let center = ps.to_local(point);
    let offset = (1.2 * r_eff * compression).sqrt();
    let local_tangent = ps.dir_to_local(tangent);
    let spherical = match kind.shape {
        crate::piece::Shape::Sphere { .. } => true,
        crate::piece::Shape::Tube { r, half, .. } | crate::piece::Shape::Cylinder { r, half } => {
            if (center[0].hypot(center[1]) - r).abs() > 1e-5
                || center[2].abs() + offset * local_tangent[2].abs() >= half { return patch; }
            false
        }
        crate::piece::Shape::Box { .. } => return patch,
    };
    // On soft balls the centre station already includes the ball's share of compression.
    let radius = if spherical { len(center) } else { center[0].hypot(center[1]) };
    patch[0].2 = 2. / 3.;
    for (i, sign) in [(1, -1.), (2, 1.)] {
        let q = add(center, scale(local_tangent, sign * offset));
        let radial = norm(if spherical { q } else { [q[0], q[1], 0.] });
        let mut surface = scale(radial, radius);
        if !spherical { surface[2] = q[2]; }
        patch[i] = (add(ps.pos, ps.dir_to_world(surface)), neg(ps.dir_to_world(radial)), 1. / 6.);
    }
    patch
}

#[allow(clippy::too_many_arguments)]
fn tread_friction(surf: &RollerSurface, axis: V3, nrm: V3, vrel: V3, dt: f64, k: f64, fnorm: f64, b: &mut Bristle) -> (V3, bool) {
    let tw = cross(axis, nrm);
    // friction: the bristle stretches with slip
    let vt = sub(vrel, scale(nrm, dot(vrel, nrm)));
    b.s = add(b.s, scale(vt, dt));
    b.s = sub(b.s, scale(nrm, dot(b.s, nrm)));
    let kt = 0.8 * k;
    let mut ft = scale(b.s, kt);
    let (grip, free) = match surf.kind {
        RollerKind::Plain => (None, None),
        RollerKind::Omni => (Some(norm(tw)), Some(norm(cross(nrm, tw)))),
        RollerKind::Mecanum => {
            let gdir = norm(add(scale(norm(tw), std::f64::consts::FRAC_1_SQRT_2), scale(norm(cross(nrm, tw)), std::f64::consts::FRAC_1_SQRT_2)));
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
        let slipping = along.abs() > cap;
        let a2 = if slipping { along.signum() * cap_k } else { along };
        let c2 = cross_.clamp(-MU_FREE * fnorm, MU_FREE * fnorm);
        b.sliding = slipping || (b.sliding && dot(vt, gd).abs() >= 0.01);
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
    (ft, b.sliding)
}

/// A soft piece against a rigid surface (a plate, a bar, a robot's hull) at one contact, in real
/// units (M5): the piece's own plate force at `depth`, higher going in than coming out by `LOSS`,
/// and a bristle for friction. `normal` points from the surface into the piece; `vrel` is the
/// surface's velocity less the piece's at the contact. Returns the force on the piece.
#[allow(clippy::too_many_arguments)]
pub fn soft_rigid(kind: &PieceKind, depth: f64, normal: V3, point: V3, vrel: V3, mu_static: f64, mu_sliding: f64, dt: f64, b: &mut Bristle) -> Option<SliceContact> {
    const LOSS: f64 = 0.15;
    let f0 = kind.plate_force(depth)?;
    if depth <= 0.0 || f0 <= 0.0 { return None; }
    let closing = dot(vrel, normal);
    let fnorm = (f0 * (1.0 + LOSS * (closing / 0.05).tanh())).max(0.0);
    let k = ((kind.plate_force(depth * 1.01)? - f0) / (0.01 * depth)).max(1.0);
    let vt = sub(vrel, scale(normal, closing));
    b.s = add(b.s, scale(vt, dt));
    b.s = sub(b.s, scale(normal, dot(b.s, normal)));
    let kt = 0.8 * k;
    let mut ft = scale(b.s, kt);
    let mag = len(ft);
    let cap = if b.sliding { mu_sliding * fnorm } else { mu_static * fnorm };
    if mag > cap && mag > 0.0 {
        ft = scale(ft, mu_sliding * fnorm / mag);
        b.s = scale(ft, 1.0 / kt);
        b.sliding = true;
    } else if b.sliding && len(vt) < 0.01 {
        b.sliding = false;
    }
    Some(SliceContact { point, force: add(scale(normal, fnorm), ft), normal, depth, wheel_depth: 0.0, normal_force: fnorm, sliding: b.sliding, stiffness: k })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::piece::PieceKind;

    fn wheel(id: &str, w: f64) -> RollerSurface {
        let lib = library::Library::builtin();
        let wh = lib.wheel(id).unwrap();
        RollerSurface { section: None, mean_support: 1., radius: wh.radius(), width: w, k_area: wh.k_area(), mu_static: 1.1, mu_sliding: 0.9, kind: RollerKind::Plain, loss: 0.0, slices: RollerSurface::slices_for(w) }
    }

    #[test]
    fn a_wheel_on_a_flat_box_matches_the_press_formula() {
        let s = wheel("am-compliant-4in-35a", 0.0254);
        let kind = PieceKind { id: "plate".into(), name: "plate".into(), shape: crate::piece::Shape::Box { half: [0.3, 0.3, 0.05] }, mass: 1.0, law: Law::Rigid, surface: "pvc".into(), color: String::new(), measured: true };
        let d = 0.006;
        // the wheel's axis along y, its bottom d into the plate's top at z = 0
        let rs = RollerState { zero: [1.,0.,0.], phase: 0., center: [0.0, 0.0, s.radius - d], axis: [0.0, 1.0, 0.0], spin: 0.0, lin: [0.0; 3], ang: [0.0; 3] };
        let ps = PieceState { pos: [0.0, 0.0, -0.05], mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], lin: [0.0; 3], ang: [0.0; 3] };
        let mut out = vec![];
        roller_piece(&s, &rs, &kind, &ps, 1e-3, &mut Bristles::new(), 0, 0, &mut out);
        let f: f64 = -out.iter().map(|c| c.force[2]).sum::<f64>();
        let want = library::stiffness::plate_force(s.k_area, s.radius, s.width, d);
        assert!((f / want - 1.0).abs() < 1e-6, "{f} vs {want}");
        assert!(out.iter().all(|c| c.point[2].abs() < 1e-9), "contact at the compressed tread");
        for kind_of_roller in [RollerKind::Plain, RollerKind::Omni] {
            let surface = RollerSurface { kind: kind_of_roller, ..s.clone() };
            let spinning = RollerState { spin: 300., ..rs };
            let mut bristles = Bristles::new();
            for _ in 0..3 {
                out.clear();
                roller_piece(&surface, &spinning, &kind, &ps, 1e-3, &mut bristles, 0, 0, &mut out);
                assert!(!out.is_empty() && out.iter().all(|c| c.sliding
                    && (c.force[0] / c.normal_force + s.mu_sliding).abs() < 1e-9));
            }
        }
    }

    #[test]
    fn plain_tread_resists_horizontal_coral_motion_along_and_across_the_axle() {
        let lib = library::Library::builtin();
        let coral = PieceKind::from_library(lib.piece("coral-2025").unwrap());
        let tread = lib.wheel("am-compliant-2in-35a").unwrap();
        let (mu_static, mu_sliding) = lib.friction(&tread.tread, &coral.surface);
        let mut surface = wheel(&tread.id, 0.012);
        surface.mu_static = mu_static;
        surface.mu_sliding = mu_sliding;
        let crate::piece::Shape::Tube { r, .. } = coral.shape else { panic!() };
        // CORAL on its side, squeezed from above. X runs along the roller axle.
        for motion in [[1.,0.,0.], [0.,1.,0.], [-0.6,0.8,0.]] {
            let piece = PieceState { pos: [0.;3], mat: [0.,0.,1., 0.,1.,0., -1.,0.,0.], lin: motion, ang: [0.;3] };
            let mut bristles = Bristles::new();
            let mut previous_force = 0.;
            let mut previous_width = 0.;
            for squeeze in [0.001, 0.006, -0.001] {
                let roller = RollerState { zero: [1.,0.,0.], phase: 0., center: [0.,0.,r + surface.radius - squeeze], axis: [1.,0.,0.],
                    spin: 0., lin: [0.;3], ang: [0.;3] };
                let mut contacts = Vec::new();
                for _ in 0..12 {
                    contacts.clear();
                    roller_piece(&surface, &roller, &coral, &piece, 0.001, &mut bristles, 0, 0, &mut contacts);
                }
                if squeeze < 0. {
                    assert!(contacts.is_empty() && bristles.is_empty());
                    continue;
                }
                assert_eq!(contacts.len(), 3 * surface.slices);
                for c in &contacts {
                    let shear = sub(c.force, scale(c.normal, c.normal_force));
                    assert!(c.sliding && dot(shear, motion) < 0.);
                    assert!((len(shear) / c.normal_force - mu_sliding).abs() < 1e-8);
                    assert!((coral.sdf_local(piece.to_local(c.point))).abs() < 1e-8);
                }
                let resistance: f64 = contacts.iter().map(|c| -dot(c.force, motion)).sum();
                let width = contacts.iter().map(|c| c.point[1].abs()).fold(0., f64::max);
                assert!(width > previous_width && resistance > previous_force);
                previous_width = width;
                previous_force = resistance;
            }
        }
    }

    #[test]
    fn rotating_a_star_alternates_between_finger_contact_and_clearance() {
        let lib = library::Library::builtin();
        let wheel_spec = lib.wheel("am-star-3in-35a").unwrap();
        let section = library::profile::Section::new(wheel_spec, &Default::default());
        let mut surface = wheel(&wheel_spec.id, wheel_spec.width());
        surface.mean_support = section.mean_support();
        surface.section = Some(std::sync::Arc::new(section));
        let coral = PieceKind::from_library(lib.piece("coral-2025").unwrap());
        let crate::piece::Shape::Tube { r, .. } = coral.shape else { panic!() };
        let piece = PieceState { pos: [0.;3], mat: [1.,0.,0.,0.,1.,0.,0.,0.,1.], lin:[0.;3], ang:[0.;3] };
        let mut had_contact = false;
        let mut had_clearance = false;
        for i in 0..36 {
            let rs = RollerState { zero:[1.,0.,0.], phase: i as f64*std::f64::consts::TAU/36.,
                center:[r+surface.radius*0.96,0.,0.],axis:[0.,0.,1.],spin:0.,lin:[0.;3],ang:[0.;3] };
            let mut contacts = Vec::new();
            roller_piece(&surface,&rs,&coral,&piece,0.001,&mut Bristles::new(),0,0,&mut contacts);
            had_contact |= !contacts.is_empty();
            had_clearance |= contacts.is_empty();
        }
        assert!(had_contact && had_clearance);
    }

    #[test]
    fn a_soft_ball_takes_some_of_the_squeeze() {
        let lib = library::Library::builtin();
        let algae = PieceKind::from_library(lib.piece("algae-2025").unwrap());
        let s = wheel("am-compliant-4in-60a", 0.0254);
        let crate::piece::Shape::Sphere { r } = algae.shape else { panic!() };
        let rs = RollerState { zero: [1.,0.,0.], phase: 0., center: [0.0, 0.0, r + s.radius - 0.01], axis: [0.0, 1.0, 0.0], spin: 0.0, lin: [0.0; 3], ang: [0.0; 3] };
        let ps = PieceState { pos: [0.0; 3], mat: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], lin: [0.0; 3], ang: [0.0; 3] };
        let mut out = vec![];
        roller_piece(&s, &rs, &algae, &ps, 1e-3, &mut Bristles::new(), 0, 0, &mut out);
        let c = out[0];
        assert!(c.wheel_depth > 0.0 && c.wheel_depth < c.depth, "{c:?}");
        assert!(c.force[2] < 0.0, "pushes the ball down");
    }
}
