use std::collections::HashMap;

use bevy_egui::egui::{Color32, RichText, Ui};
use library::Library;
use model::{Id, Pose, Robot, ShapeKind, ShapeRole};
use sim::game::RobotSize;

use super::{AppState, Selection};

const EPS: f64 = 1e-8;
const FACES: [&str; 4] = ["Front", "Rear", "Left", "Right"];

#[derive(Default, Clone, Copy)]
struct Peak {
    metres: f64,
    owner: Option<Id>,
}

impl Peak {
    fn include(&mut self, metres: f64, owner: Option<Id>) {
        if metres > self.metres {
            *self = Self { metres, owner };
        }
    }
}

/// Exact interval on a world axis. In particular, a tilted roller is measured as a
/// cylinder, independently of how many segments the drawing uses for its silhouette.
fn interval(shape: &ShapeKind, pose: Pose, axis: usize) -> [f64; 2] {
    let mut direction = [0.0; 3];
    direction[axis] = 1.0;
    let d = pose.inverse().rotate(direction);
    let c = pose.pos[axis];
    let extent = match shape {
        ShapeKind::Box { size } => (0..3).map(|i| size[i] * d[i].abs() / 2.0).sum(),
        ShapeKind::Sphere { radius } => *radius,
        ShapeKind::Cylinder { radius, length } => radius * d[0].hypot(d[1]) + length * d[2].abs() / 2.0,
        ShapeKind::Hull { points } => {
            return points.iter().fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], &p| {
                let v = c + model::math::dot(d, p);
                [lo.min(v), hi.max(v)]
            });
        }
    };
    [c - extent, c + extent]
}

#[derive(Default)]
struct Measurements {
    perimeter: f64,
    top: Peak,
    below_floor: Peak,
    sides: [Peak; 4],
    reach: Peak,
}

// Distance to the nearest point of the chassis footprint. Beyond a corner the
// extension limit is a circular arc (REEFSCAPE figure 8-2).
fn reach(shape: &ShapeKind, pose: Pose, half: [f64; 2], over: [f64; 2]) -> f64 {
    if over[0] <= 0.0 || over[1] <= 0.0 {
        return over[0].max(over[1]).max(0.0);
    }
    let dist = |p: model::V3| (p[0].abs() - half[0]).max(0.0).hypot((p[1].abs() - half[1]).max(0.0));
    let mut best = over[0].max(over[1]);
    match shape {
        ShapeKind::Hull { points } => {
            for p in points {
                best = best.max(dist(pose.apply(*p)));
            }
        }
        ShapeKind::Box { size } => {
            for x in [-0.5, 0.5] {
                for y in [-0.5, 0.5] {
                    for z in [-0.5, 0.5] {
                        best = best.max(dist(pose.apply([x * size[0], y * size[1], z * size[2]])));
                    }
                }
            }
        }
        ShapeKind::Sphere { radius } => {
            if dist(pose.pos) > 0.0 {
                best = best.max(dist(pose.pos) + radius);
            }
        }
        ShapeKind::Cylinder { radius, length } => {
            // The maximum on a solid cylinder occurs on an end circle. On each arc,
            // the chord's farthest endpoint plus the sagitta bounds the entire curve.
            for z in [-length / 2.0, length / 2.0] {
                let sample = |angle: f64| {
                    let (s, c) = angle.sin_cos();
                    dist(pose.apply([radius * c, radius * s, z]))
                };
                let upper = dist(pose.apply([0.0, 0.0, z])) + radius;
                let mut arcs = Vec::with_capacity(24);
                for i in 0..4 {
                    let a = f64::from(i) * std::f64::consts::FRAC_PI_2;
                    let b = a + std::f64::consts::FRAC_PI_2;
                    arcs.push((a, b, sample(a), sample(b)));
                }
                while let Some((a, b, da, db)) = arcs.pop() {
                    best = best.max(da).max(db);
                    let error = radius * (1.0 - ((b - a) * 0.5).cos());
                    if (da.max(db) + error).min(upper) <= best + EPS {
                        continue;
                    }
                    let mid = (a + b) * 0.5;
                    let dm = sample(mid);
                    arcs.push((a, mid, da, dm));
                    arcs.push((mid, b, dm, db));
                }
            }
        }
    }
    best
}

impl Measurements {
    fn measure(r: &Robot, lib: &Library, poses: &HashMap<Id, Pose>) -> Self {
        let f = &r.frame;
        let mut m = Self { perimeter: 2.0 * (f.width + f.length), ..Default::default() };
        let mut include = |shape: &ShapeKind, pose: Pose, role: ShapeRole, owner| {
            if role == ShapeRole::Ghost {
                return;
            }
            let [lo, hi] = interval(shape, pose, 2);
            m.top.include(hi, owner);
            m.below_floor.include(-lo, owner);
            if role == ShapeRole::Bumper {
                return;
            }
            let [x0, x1] = interval(shape, pose, 0);
            let [y0, y1] = interval(shape, pose, 1);
            let half = [f.length / 2.0, f.width / 2.0];
            let over = [x1.max(-x0) - half[0], y1.max(-y0) - half[1]];
            m.reach.include(reach(shape, pose, half, over), owner);
            for (side, overhang) in m.sides.iter_mut().zip([
                x1 - f.length / 2.0,
                -x0 - f.length / 2.0,
                y1 - f.width / 2.0,
                -y0 - f.width / 2.0,
            ]) {
                side.include(overhang, owner);
            }
        };
        for link in &r.links {
            let Some(pose) = poses.get(&link.id) else { continue };
            let owner = r.mechanism_of_link(link.id).map(|m| m.id);
            for s in &link.shapes {
                include(&s.kind, pose.mul(&s.pose), s.role, owner);
            }
        }
        for roller in &r.rollers {
            let Some(shaft) = r.shaft(roller.shaft) else { continue };
            let Some(pose) = poses.get(&shaft.link) else { continue };
            let shape = ShapeKind::Cylinder { radius: roller.radius(lib), length: roller.width };
            include(
                &shape,
                pose.mul(&Pose::along(roller.center, roller.axis)),
                ShapeRole::Body,
                r.mechanism_of_link(shaft.link).map(|m| m.id),
            );
        }
        m
    }

    fn height(&self) -> f64 {
        self.top.metres + self.below_floor.metres
    }

    fn extended(&self) -> usize {
        self.sides.iter().filter(|s| s.metres > EPS).count()
    }
}

#[derive(Clone, Copy)]
pub(super) struct Limits {
    perimeter: f64,
    pub height: Option<f64>,
    pub extension: f64,
    pub one_side: bool,
}

impl Limits {
    fn for_pose(profile: &RobotSize, start: bool) -> Self {
        Self {
            perimeter: profile.perimeter,
            height: if start { Some(profile.start_height) } else { profile.operating_height },
            extension: if start { profile.start_extension } else { profile.extension },
            one_side: profile.one_side,
        }
    }

    fn fits(self, m: &Measurements) -> bool {
        m.perimeter <= self.perimeter + EPS
            && self.height.is_none_or(|h| m.height() <= h + EPS)
            && m.reach.metres <= self.extension + EPS
            && (!self.one_side || m.extended() <= 1)
    }
}

pub(super) struct Guides {
    pub limits: Limits,
    pub floor: f64,
}

pub(super) fn panel(ui: &mut Ui, st: &mut AppState, poses: &HashMap<Id, Pose>, start: bool) -> Option<Guides> {
    let profile = match st.game(false) {
        Ok(game) => {
            game.pack.robot_size.clone().ok_or_else(|| "This game pack has no robot size limits.".to_owned())
        }
        Err(e) => Err(e.clone()),
    };
    let m = Measurements::measure(&st.project().robot, &st.lib, poses);
    let limits = profile.as_ref().ok().map(|p| Limits::for_pose(p, start));
    let mode = if start { "Starting limits" } else { "Operating limits" };
    let warning = Color32::from_rgb(255, 180, 102);
    let (label, color) = match limits {
        Some(l) if l.fits(&m) => (format!("{mode} · Within modeled limits"), ui.visuals().text_color()),
        Some(_) => (format!("{mode} · Over limit"), warning),
        None => ("Size checks · Limits unavailable".into(), ui.visuals().weak_text_color()),
    };
    let open = st.assembly.show_limits;
    let action = if open { "Hide details" } else { "Show details" };
    if ui.button(RichText::new(format!("{label}    ·    {action}")).color(color)).clicked() {
        st.assembly.show_limits = !open;
    }
    if !st.assembly.show_limits {
        return None;
    }
    let unit = st.unit(library::units::Kind::Length);
    let format = |v| library::units::format(v, unit);
    let mut select = None;
    ui.group(|ui| {
        let mut row = |ui: &mut Ui, name: &str, measured: f64, max: Option<f64>, owner: Option<Id>| {
            let mut text = format!("{name}   {}", format(measured));
            if let Some(max) = max {
                text.push_str(&format!(" / {}", format(max)));
                if measured > max + EPS {
                    let part =
                        owner.and_then(|id| st.project().robot.mechanism(id)).map_or("Chassis", |m| &m.name);
                    ui.horizontal_wrapped(|ui| {
                        ui.colored_label(warning, format!("{text}   ({} over)", format(measured - max)));
                        if ui.link(part).on_hover_text("Select in the drawing").clicked() {
                            select = Some(owner.map_or(Selection::Robot, Selection::Mechanism));
                        }
                    });
                    return;
                }
            }
            ui.label(text);
        };
        row(ui, "Chassis perimeter", m.perimeter, limits.map(|l| l.perimeter), None);
        row(ui, "Height", m.height(), limits.and_then(|l| l.height), m.top.owner.or(m.below_floor.owner));
        if m.extended() == 0 {
            row(ui, "Side extension", 0.0, limits.map(|l| l.extension), None);
        } else {
            for (face, p) in FACES.iter().zip(&m.sides).filter(|(_, p)| p.metres > EPS) {
                row(ui, &format!("{face} extension"), p.metres, limits.map(|l| l.extension), p.owner);
            }
            let longest_side = m.sides.iter().map(|s| s.metres).fold(0.0, f64::max);
            if m.reach.metres > longest_side + EPS {
                row(ui, "Corner reach", m.reach.metres, limits.map(|l| l.extension), m.reach.owner);
            }
        }
        if limits.is_some_and(|l| l.one_side) && m.extended() > 1 {
            ui.horizontal_wrapped(|ui| {
                ui.colored_label(warning, format!("{} sides extend at once. This game allows one.", m.extended()));
                for (face, p) in FACES.iter().zip(&m.sides).filter(|(_, p)| p.metres > EPS) {
                    let part =
                        p.owner.and_then(|id| st.project().robot.mechanism(id)).map_or("Chassis", |m| &m.name);
                    if ui.link(format!("{face} · {part}")).clicked() {
                        select = Some(p.owner.map_or(Selection::Robot, Selection::Mechanism));
                    }
                }
            });
        }
        ui.horizontal_wrapped(|ui| {
            if limits.is_some() {
                ui.weak("Dashed lines show the limits. Reach excludes bumpers.");
            } else {
                ui.weak("Reach excludes bumpers.");
            }
            match &profile {
                Ok(p) if !p.source.is_empty() => {
                    ui.hyperlink_to("Season manual", &p.source);
                }
                Ok(_) => {}
                Err(e) => {
                    ui.label(e);
                }
            }
        });
    });
    if let Some(sel) = select {
        st.sel = sel;
    }
    limits.map(|limits| Guides { limits, floor: -m.below_floor.metres })
}

#[cfg(test)]
mod tests {
    use super::*;
    use model::{CHASSIS, Shape};

    #[test]
    fn tilted_rollers_boxes_and_asymmetric_hulls_have_exact_extents() {
        let pose = Pose::along([2.0, 0.0, 0.0], [1.0, 0.0, 1.0]);
        let [lo, hi] = interval(&ShapeKind::Cylinder { radius: 0.25, length: 2.0 }, pose, 0);
        let half = 1.25 / 2f64.sqrt();
        assert!((lo - (2.0 - half)).abs() < EPS && (hi - (2.0 + half)).abs() < EPS);
        let [_, hi] = interval(&ShapeKind::Box { size: [0.5, 0.2, 2.0] }, pose, 0);
        assert!((hi - (2.0 + half)).abs() < EPS);
        let hull = ShapeKind::Hull { points: vec![[-1.0, 2.0, 3.0], [4.0, -2.0, 1.0]] };
        assert_eq!(interval(&hull, Pose::at([0.5; 3]), 0), [-0.5, 4.5]);
    }

    #[test]
    fn same_geometry_uses_the_selected_season_and_pose() {
        let mut r = Robot::new("test");
        let lib = Library::builtin();
        let chassis = HashMap::from([(CHASSIS, Pose::default())]);
        let rules = |game: &str, start| {
            Limits::for_pose(&sim::game::GamePack::builtin(game).unwrap().robot_size.unwrap(), start)
        };
        let initial = Measurements::measure(&r, &lib, &chassis);
        assert!(rules("rebuilt-2026", true).fits(&initial));
        assert_eq!(initial.extended(), 0, "exclude bumpers");
        let size = r.frame.length;
        let mut shape = Shape {
            name: "test".into(),
            kind: ShapeKind::Box { size: [size + 0.2, 0.05, 0.05] },
            pose: Pose::at([0.0, 0.0, 0.5]),
            color: None,
            role: ShapeRole::Guide,
        };
        r.link_mut(CHASSIS).unwrap().shapes.push(shape.clone());
        shape.role = ShapeRole::Ghost;
        shape.pose = Pose::at([3.0; 3]);
        r.link_mut(CHASSIS).unwrap().shapes.push(shape);
        let m = Measurements::measure(&r, &lib, &chassis);
        assert!((m.sides[0].metres - 0.1).abs() < EPS);
        assert!((m.height() - 0.525).abs() < EPS);
        assert_eq!(m.extended(), 2);
        assert!(rules("reefscape-2025", false).fits(&m));
        assert!(!rules("reefscape-2025", true).fits(&m));
        assert!(!rules("rebuilt-2026", false).fits(&m), "front and rear at the same time");
        let at_limit =
            Measurements { top: Peak { metres: 0.762, owner: None }, perimeter: 2.794, ..Default::default() };
        assert!(rules("rebuilt-2026", true).fits(&at_limit));
        let raised = Measurements { below_floor: Peak { metres: 0.001, owner: None }, ..at_limit };
        assert!(!rules("rebuilt-2026", false).fits(&raised));
    }

    #[test]
    fn a_diagonal_can_exceed_the_limit_inside_both_side_guides() {
        let pose = Pose::at([0.7, 0.7, 0.0]);
        let half = [0.4, 0.4];
        let expected = 0.3f64.hypot(0.3) + 0.05;
        for shape in [ShapeKind::Sphere { radius: 0.05 }, ShapeKind::Cylinder { radius: 0.05, length: 0.1 }] {
            let actual = reach(&shape, pose, half, [0.35, 0.35]);
            assert!((actual - expected).abs() < EPS);
            assert!(actual > 18.0 * library::IN);
        }
    }
}
