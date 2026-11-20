//! Game pieces: their shapes as signed distances (PLAN.md D4: analytic where they can be), how they
//! give under a squeeze, and how MuJoCo sees them for ordinary rigid contact.
use library::{PieceLaw, PieceShape, IN, LBF, PSI};
use model::math::*;
use model::V3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    Sphere { r: f64 },
    /// along local z
    Tube { r: f64, r_in: f64, half: f64 },
    Cylinder { r: f64, half: f64 },
    Box { half: V3 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Law {
    Rigid,
    /// gauge pressure (Pa) and a skin stiffness (N/m per m of width)
    Inflated { pressure: f64, skin: f64 },
    /// plate force k0*xc*(e^(s/xc)-1), s the total squeeze
    Foam { k0: f64, xc: f64 },
}

#[derive(Debug, Clone)]
pub struct PieceKind {
    pub id: String,
    pub name: String,
    pub shape: Shape,
    pub mass: f64,
    pub law: Law,
    /// friction surface id, for the library's tread-surface pairs
    pub surface: String,
    pub color: String,
    pub measured: bool,
}

impl PieceKind {
    pub fn from_library(p: &library::Piece) -> Self {
        let shape = match p.shape {
            PieceShape::Sphere { d_in } => Shape::Sphere { r: d_in * IN / 2.0 },
            PieceShape::Tube { outer_d_in, inner_d_in, length_in } => Shape::Tube { r: outer_d_in * IN / 2.0, r_in: inner_d_in * IN / 2.0, half: length_in * IN / 2.0 },
            PieceShape::Cylinder { d_in, length_in } => Shape::Cylinder { r: d_in * IN / 2.0, half: length_in * IN / 2.0 },
            PieceShape::Box { x_in, y_in, z_in } => Shape::Box { half: [x_in * IN / 2.0, y_in * IN / 2.0, z_in * IN / 2.0] },
        };
        let law = match &p.law {
            PieceLaw::Rigid => Law::Rigid,
            PieceLaw::Inflated { psi, choice, skin_n_per_m } => Law::Inflated { pressure: psi.get(*choice).or(psi.first()).copied().unwrap_or(1.0) * PSI, skin: *skin_n_per_m },
            PieceLaw::Foam { k0_lb_per_in, xc_in } => Law::Foam { k0: k0_lb_per_in * LBF / IN, xc: xc_in * IN },
        };
        PieceKind { id: p.id.clone(), name: p.name.clone(), shape, mass: p.mass_kg, law, surface: p.surface.clone(), color: p.color.clone(), measured: p.measured }
    }

    /// the radius of a sphere that holds it (for quick distance checks)
    pub fn bound(&self) -> f64 {
        match self.shape {
            Shape::Sphere { r } => r,
            Shape::Tube { r, half, .. } | Shape::Cylinder { r, half } => (r * r + half * half).sqrt(),
            Shape::Box { half } => len(half),
        }
    }

    /// How high its centre sits when lying on the floor, and the turn that lays it down.
    pub fn resting(&self) -> (f64, Quat) {
        match self.shape {
            Shape::Sphere { r } => (r, QI),
            // lying on its side, its axis along y
            Shape::Tube { r, .. } | Shape::Cylinder { r, .. } => (r, axis_angle([1.0, 0.0, 0.0], std::f64::consts::FRAC_PI_2)),
            Shape::Box { half } => (half[2], QI),
        }
    }

    /// Signed distance from a point in the piece's own frame to its outer surface (negative inside).
    pub fn sdf_local(&self, p: V3) -> f64 {
        match self.shape {
            Shape::Sphere { r } => len(p) - r,
            Shape::Tube { r, half, .. } | Shape::Cylinder { r, half } => {
                let dr = (p[0] * p[0] + p[1] * p[1]).sqrt() - r;
                let dz = p[2].abs() - half;
                let out = (dr.max(0.0).powi(2) + dz.max(0.0).powi(2)).sqrt();
                out + dr.max(dz).min(0.0)
            }
            Shape::Box { half } => {
                let q = [p[0].abs() - half[0], p[1].abs() - half[1], p[2].abs() - half[2]];
                len([q[0].max(0.0), q[1].max(0.0), q[2].max(0.0)]) + q[0].max(q[1]).max(q[2]).min(0.0)
            }
        }
    }

    /// The outward direction of the surface nearest a point (own frame).
    pub fn grad_local(&self, p: V3) -> V3 {
        match self.shape {
            Shape::Sphere { .. } => norm(p),
            _ => {
                let h = 1e-5;
                let g = [
                    self.sdf_local([p[0] + h, p[1], p[2]]) - self.sdf_local([p[0] - h, p[1], p[2]]),
                    self.sdf_local([p[0], p[1] + h, p[2]]) - self.sdf_local([p[0], p[1] - h, p[2]]),
                    self.sdf_local([p[0], p[1], p[2] + h]) - self.sdf_local([p[0], p[1], p[2] - h]),
                ];
                norm(g)
            }
        }
    }

    /// The surface's curvature (1/m) at a point near it (own frame), along a direction `t` that
    /// lies in the surface (own frame).
    pub fn curvature_local(&self, p: V3, t: V3) -> f64 {
        match self.shape {
            Shape::Sphere { r } => 1.0 / r,
            Shape::Tube { r, half, .. } | Shape::Cylinder { r, half } => {
                // on the round side: 1/r across the axis, 0 along it; the ends are flat
                let side = (p[0] * p[0] + p[1] * p[1]).sqrt() - r;
                let end = p[2].abs() - half;
                if end > side { 0.0 } else { (1.0 - t[2] * t[2]).max(0.0) / r }
            }
            Shape::Box { .. } => 0.0,
        }
    }

    /// Force a slice of a hard surface (width `w`, relative radius of curvature `r_eff`) takes to
    /// push `d` into the piece, and the piece's own share of the stiffness. Rigid pieces give
    /// nothing (infinite).
    pub fn force(&self, d: f64, w: f64, r_eff: f64, r_roller: f64) -> f64 {
        if d <= 0.0 { return 0.0; }
        match self.law {
            Law::Rigid => f64::INFINITY,
            Law::Inflated { pressure, skin } => {
                // the flattened patch is about 2*sqrt(2 r d) long round the ball, `w` wide
                pressure * w * 2.0 * (2.0 * r_eff * d).sqrt() + skin * w * d
            }
            Law::Foam { k0, xc } => {
                // a plate pressing d into the ball squeezes it as two plates would by 2d; a slice
                // covers its share of the patch, and a curved roller presses less than a plate
                // (rebuilt-sim-native mechanism.rs `foam` and `curvature`)
                let s = 2.0 * d;
                let plate = k0 * xc * ((s / xc).min(6.0).exp() - 1.0);
                let r = match self.shape { Shape::Sphere { r } => r, _ => r_eff };
                let patch = 2.0 * (2.0 * r * d).sqrt();
                let share = (w / patch.max(1e-6)).min(1.0);
                let curv = if r_roller.is_finite() { (r_roller / (r + r_roller)).sqrt() } else { 1.0 };
                plate * share * curv
            }
        }
    }

    /// The MJCF for a free piece named `name` at a pose, colliding as `contype`/`conaffinity`.
    pub fn mjcf(&self, name: &str, pos: V3, quat: Quat, friction: f64) -> String {
        let fr = format!("friction=\"{friction} 0.005 0.0001\"");
        let col = "contype=\"4\" conaffinity=\"7\"";
        let soft = match self.law { Law::Rigid => "", _ => " solref=\"0.02 0.5\"" };
        let geoms = match self.shape {
            Shape::Sphere { r } => format!("<geom type=\"sphere\" size=\"{r}\" mass=\"{}\" {fr} {col}{soft}/>", self.mass),
            Shape::Tube { r, half, .. } | Shape::Cylinder { r, half } => format!("<geom type=\"cylinder\" size=\"{r} {half}\" mass=\"{}\" {fr} {col}{soft}/>", self.mass),
            Shape::Box { half } => format!("<geom type=\"box\" size=\"{} {} {}\" mass=\"{}\" {fr} {col}{soft}/>", half[0], half[1], half[2], self.mass),
        };
        // a hollow tube's mass sits in its wall: its inertia is a thin-walled tube's, not a rod's
        let inertial = match self.shape {
            Shape::Tube { r, r_in, half } => {
                let m = self.mass;
                let ixx = m / 12.0 * (3.0 * (r * r + r_in * r_in) + (2.0 * half).powi(2));
                let izz = m / 2.0 * (r * r + r_in * r_in);
                format!("<inertial pos=\"0 0 0\" mass=\"{m}\" diaginertia=\"{ixx} {ixx} {izz}\"/>")
            }
            Shape::Sphere { r } if !matches!(self.law, Law::Rigid) => {
                // a hollow ball
                let i = 2.0 / 3.0 * self.mass * r * r;
                format!("<inertial pos=\"0 0 0\" mass=\"{}\" diaginertia=\"{i} {i} {i}\"/>", self.mass)
            }
            _ => String::new(),
        };
        let geoms = if inertial.is_empty() { geoms } else { geoms.replace(&format!(" mass=\"{}\"", self.mass), "") };
        format!("<body name=\"{name}\" pos=\"{} {} {}\" quat=\"{} {} {} {}\"><freejoint name=\"{name}\"/>{inertial}{geoms}</body>",
            pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn coral_sdf_and_curvature() {
        let lib = library::Library::builtin();
        let c = PieceKind::from_library(lib.piece("coral-2025").unwrap());
        let Shape::Tube { r, half, .. } = c.shape else { panic!() };
        assert!((c.sdf_local([r + 0.01, 0.0, 0.0]) - 0.01).abs() < 1e-9);
        assert!((c.sdf_local([0.0, 0.0, half + 0.02]) - 0.02).abs() < 1e-9);
        assert!(c.sdf_local([0.0, 0.0, 0.0]) < 0.0);
        let g = c.grad_local([r + 0.01, 0.0, 0.0]);
        assert!((g[0] - 1.0).abs() < 1e-6);
        // curvature: 1/r round the pipe, 0 along it
        assert!((c.curvature_local([r, 0.0, 0.0], [0.0, 1.0, 0.0]) - 1.0 / r).abs() < 1e-9);
        assert!(c.curvature_local([r, 0.0, 0.0], [0.0, 0.0, 1.0]).abs() < 1e-9);
    }

    #[test]
    fn fuel_plate_squeeze_follows_the_measured_curve() {
        // FUEL between two plates, 1 in. of total squeeze: k0*xc*(e^1 - 1), about 34 lbf
        let lib = library::Library::builtin();
        let f = PieceKind::from_library(lib.piece("fuel-2026").unwrap());
        let Shape::Sphere { r } = f.shape else { panic!() };
        let d = 0.5 * IN;
        let patch = 2.0 * (2.0 * r * d).sqrt();
        let force = f.force(d, patch, r, f64::INFINITY);
        let want = 19.8 * LBF / IN * IN * (1f64.exp() - 1.0);
        assert!((force / want - 1.0).abs() < 1e-9, "{force} vs {want}");
    }

    #[test]
    fn algae_is_held_up_by_pressure() {
        let lib = library::Library::builtin();
        let a = PieceKind::from_library(lib.piece("algae-2025").unwrap());
        // more squeeze, more patch, more force; about the ball's own weight at a few mm on a 1 in. strip
        let f1 = a.force(0.002, 0.0254, 0.2, f64::INFINITY);
        let f2 = a.force(0.008, 0.0254, 0.2, f64::INFINITY);
        assert!(f2 > f1 && f1 > 0.0);
    }
}
