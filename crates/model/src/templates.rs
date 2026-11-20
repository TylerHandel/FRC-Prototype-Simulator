//! Templates carry the hard parts (interface rule 4): a few numbers build a mechanism's links,
//! joints, drives, rollers, pickup and holders. Changing the numbers rebuilds it in place, keeping
//! the ids of what it made so presets and bindings still point at the same drives.
use crate::math::{self, axis_angle, norm, scale, V3};
use crate::*;
use serde::{Deserialize, Serialize};

const IN: f64 = 0.0254;
const DEG: f64 = std::f64::consts::PI / 180.0;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Template {
    pub mount: Mount,
    pub params: Params,
}

/// Where a mechanism attaches: to the chassis (`on` None) at `offset` in the robot frame, or to
/// another mechanism's tip, `offset` from it.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct Mount {
    #[serde(default)]
    pub on: Option<Id>,
    pub offset: V3,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum Rigging { Cascade, Continuous }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Params {
    Elevator { stages: u32, travel: f64, height: f64, width: f64, rigging: Rigging, spool_diameter: f64, motor: String, motors: u32, ratio: f64, carriage_mass: f64, stage_mass: f64 },
    Arm { length: f64, width: f64, mass: f64, motor: String, motors: u32, ratio: f64, min_deg: f64, max_deg: f64, start_deg: f64 },
    Wrist { length: f64, width: f64, mass: f64, motor: String, motors: u32, ratio: f64, min_deg: f64, max_deg: f64, start_deg: f64 },
    Telescope { travel: f64, length: f64, mass: f64, motor: String, motors: u32, ratio: f64, spool_diameter: f64 },
    Intake { width: f64, height: f64, ramp_deg: f64, rollers: u32, wheel: String, wheels_per_roller: u32, motor: String, ratio: f64, mass: f64, deploy: bool, deploy_ratio: f64 },
    Claw { width: f64, gap: f64, depth: f64, wheel: String, motor: String, ratio: f64, mass: f64 },
    Shooter { wheel: String, width: f64, motor: String, motors: u32, ratio: f64, hood_deg: f64, mass: f64, efficiency: f64 },
    Turret { radius: f64, motor: String, ratio: f64, mass: f64, range_deg: f64 },
    Climber { length: f64, motor: String, motors: u32, ratio: f64, mass: f64 },
}

impl Params {
    pub fn kind(&self) -> MechKind {
        match self {
            Params::Elevator { .. } => MechKind::Elevator, Params::Arm { .. } => MechKind::Arm, Params::Wrist { .. } => MechKind::Wrist,
            Params::Telescope { .. } => MechKind::Telescope, Params::Intake { .. } => MechKind::Intake, Params::Claw { .. } => MechKind::Claw,
            Params::Shooter { .. } => MechKind::Shooter, Params::Turret { .. } => MechKind::Turret, Params::Climber { .. } => MechKind::Climber,
        }
    }
}

impl Template {
    /// a 2-stage cascade elevator at the robot's centre, for REEFSCAPE L4
    pub fn elevator_default() -> Self {
        Template { mount: Mount { on: None, offset: [-0.05, 0.0, 0.09] }, params: Params::Elevator {
            stages: 2, travel: 1.3, height: 0.95, width: 0.3, rigging: Rigging::Cascade, spool_diameter: 1.75 * IN,
            motor: "kraken-x60-foc".into(), motors: 2, ratio: 5.0, carriage_mass: 3.0, stage_mass: 2.5 } }
    }
    pub fn arm_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.5] }, params: Params::Arm {
            length: 0.5, width: 0.1, mass: 3.0, motor: "kraken-x60-foc".into(), motors: 1, ratio: 60.0, min_deg: -60.0, max_deg: 120.0, start_deg: 90.0 } }
    }
    pub fn wrist_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.0] }, params: Params::Wrist {
            length: 0.18, width: 0.15, mass: 1.5, motor: "kraken-x44-foc".into(), motors: 1, ratio: 30.0, min_deg: -90.0, max_deg: 135.0, start_deg: 90.0 } }
    }
    pub fn telescope_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.0] }, params: Params::Telescope {
            travel: 0.5, length: 0.6, mass: 2.0, motor: "kraken-x60-foc".into(), motors: 1, ratio: 9.0, spool_diameter: 1.5 * IN } }
    }
    /// an over-the-bumper CORAL floor intake: one roller pinching the pipe against the carpet
    pub fn intake_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.0] }, params: Params::Intake {
            width: 0.5, height: 6.1 * IN, ramp_deg: 30.0, rollers: 2, wheel: "am-compliant-4in-35a".into(), wheels_per_roller: 4,
            motor: "kraken-x60-foc".into(), ratio: 3.0, mass: 3.0, deploy: false, deploy_ratio: 40.0 } }
    }
    pub fn claw_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.0] }, params: Params::Claw {
            width: 0.32, gap: 3.9 * IN, depth: 0.2, wheel: "am-compliant-2in-35a".into(), motor: "kraken-x44-foc".into(), ratio: 3.0, mass: 2.0 } }
    }
    pub fn shooter_default() -> Self {
        Template { mount: Mount { on: None, offset: [-0.1, 0.0, 0.45] }, params: Params::Shooter {
            wheel: "colson-4in".into(), width: 0.3, motor: "kraken-x60-foc".into(), motors: 2, ratio: 1.0, hood_deg: 55.0, mass: 4.0, efficiency: 0.45 } }
    }
    pub fn turret_default() -> Self {
        Template { mount: Mount { on: None, offset: [0.0, 0.0, 0.25] }, params: Params::Turret {
            radius: 0.15, motor: "kraken-x44-foc".into(), ratio: 40.0, mass: 2.0, range_deg: 200.0 } }
    }
    pub fn climber_default() -> Self {
        Template { mount: Mount { on: None, offset: [-0.2, 0.0, 0.3] }, params: Params::Climber {
            length: 0.45, motor: "kraken-x60-foc".into(), motors: 1, ratio: 125.0, mass: 2.5 } }
    }
    pub fn default_for(kind: MechKind) -> Option<Self> {
        Some(match kind {
            MechKind::Elevator => Self::elevator_default(), MechKind::Arm => Self::arm_default(), MechKind::Wrist => Self::wrist_default(),
            MechKind::Telescope => Self::telescope_default(), MechKind::Intake => Self::intake_default(), MechKind::Claw => Self::claw_default(),
            MechKind::Shooter => Self::shooter_default(), MechKind::Turret => Self::turret_default(), MechKind::Climber => Self::climber_default(),
            MechKind::Custom => return None,
        })
    }
}

/// The point a mechanism mounted on this one attaches to, robot frame, design pose.
pub fn tip(r: &Robot, m: &Mechanism) -> V3 {
    let Some(t) = &m.template else {
        return m.links.last().and_then(|l| r.link(*l)).and_then(|l| l.shapes.first()).map_or([0.0; 3], |s| s.pose.pos);
    };
    let base = mount_point(r, &t.mount);
    match &t.params {
        Params::Elevator { height, .. } => math::add(base, [0.12, 0.0, height * 0.35]),
        Params::Arm { length, .. } | Params::Wrist { length, .. } => math::add(base, [*length, 0.0, 0.0]),
        Params::Telescope { length, .. } => math::add(base, [*length, 0.0, 0.0]),
        Params::Turret { .. } => math::add(base, [0.0, 0.0, 0.08]),
        Params::Climber { length, .. } => math::add(base, [0.0, 0.0, *length]),
        Params::Intake { .. } | Params::Claw { .. } | Params::Shooter { .. } => base,
    }
}

pub fn mount_point(r: &Robot, mount: &Mount) -> V3 {
    match mount.on.and_then(|id| r.mechanism(id)) {
        Some(parent) => math::add(tip(r, parent), mount.offset),
        None => mount.offset,
    }
}

fn mount_link(r: &Robot, mount: &Mount) -> Id {
    mount.on.and_then(|id| r.mechanism(id)).and_then(|m| m.end_link()).unwrap_or(CHASSIS)
}

/// Adds a mechanism built from a template; returns its id.
pub fn add(r: &mut Robot, t: Template) -> anyhow::Result<Id> {
    let id = r.id();
    build(r, id, t, &mut Ids::default())?;
    Ok(id)
}

/// Rebuilds a template mechanism after its numbers changed, keeping the ids of what it made.
pub fn rebuild(r: &mut Robot, id: Id, t: Template) -> anyhow::Result<()> {
    let old = r.mechanism(id).cloned().ok_or_else(|| anyhow::anyhow!("no mechanism {id}"))?;
    let mut ids = Ids::from_mechanism(r, &old);
    let (name, fidelity, color) = (old.name.clone(), old.fidelity, old.color.clone());
    // mechanisms mounted on this one keep their mount
    let children: Vec<Id> = r.mechanisms.iter().filter(|m| m.template.as_ref().is_some_and(|t| t.mount.on == Some(id))).map(|m| m.id).collect();
    let presets = r.controls.presets.clone();
    let bindings = r.controls.bindings.clone();
    remove_owned(r, &old);
    let at = r.mechanisms.iter().position(|m| m.id == id).unwrap_or(r.mechanisms.len());
    r.mechanisms.retain(|m| m.id != id);
    build(r, id, t, &mut ids)?;
    // put it back where it was, with its own name, fidelity and color
    if let Some(i) = r.mechanisms.iter().position(|m| m.id == id) {
        let mut m = r.mechanisms.remove(i);
        m.name = name; m.fidelity = fidelity; m.color = color;
        r.mechanisms.insert(at.min(r.mechanisms.len()), m);
    }
    let drives: Vec<Id> = r.drives.iter().map(|d| d.id).collect();
    r.controls.presets = presets.into_iter().map(|mut p| { p.targets.retain(|t| drives.contains(&t.drive)); p }).collect();
    r.controls.bindings = bindings.into_iter().filter(|b| match &b.action { Action::Run { drive, .. } => drives.contains(drive), _ => true }).collect();
    for c in children {
        if let Some(t) = r.mechanism(c).and_then(|m| m.template.clone()) { rebuild(r, c, t)?; }
    }
    Ok(())
}

fn remove_owned(r: &mut Robot, m: &Mechanism) {
    let links = m.links.clone();
    let joints: Vec<Id> = r.joints.iter().filter(|j| links.contains(&j.child)).map(|j| j.id).collect();
    let shafts: Vec<Id> = r.shafts.iter().filter(|s| links.contains(&s.link)).map(|s| s.id).collect();
    r.links.retain(|l| !links.contains(&l.id));
    r.joints.retain(|j| !joints.contains(&j.id));
    r.rollers.retain(|x| !shafts.contains(&x.shaft));
    r.shafts.retain(|s| !shafts.contains(&s.id));
    r.drives.retain(|d| match d.target {
        DriveTarget::Joint { joint, .. } => !joints.contains(&joint),
        DriveTarget::Shaft { shaft } => !shafts.contains(&shaft),
    });
}

/// Ids handed out in the order a template makes things, reusing a mechanism's old ones first.
#[derive(Default)]
struct Ids { links: Vec<Id>, joints: Vec<Id>, drives: Vec<Id>, shafts: Vec<Id>, rollers: Vec<Id> }

impl Ids {
    fn from_mechanism(r: &Robot, m: &Mechanism) -> Self {
        let joints = r.joints.iter().filter(|j| m.links.contains(&j.child)).map(|j| j.id).collect::<Vec<_>>();
        let shafts = r.shafts.iter().filter(|s| m.links.contains(&s.link)).map(|s| s.id).collect::<Vec<_>>();
        Ids {
            links: m.links.clone(),
            drives: r.drives.iter().filter(|d| match d.target { DriveTarget::Joint { joint, .. } => joints.contains(&joint), DriveTarget::Shaft { shaft } => shafts.contains(&shaft) }).map(|d| d.id).collect(),
            rollers: r.rollers.iter().filter(|x| shafts.contains(&x.shaft)).map(|x| x.id).collect(),
            joints, shafts,
        }
    }
    fn take(v: &mut Vec<Id>, r: &mut Robot) -> Id { if v.is_empty() { r.id() } else { v.remove(0) } }
}

fn bx(name: &str, center: V3, size: V3, color: &str) -> Shape {
    Shape { name: name.into(), kind: ShapeKind::Box { size }, pose: Pose::at(center), color: Some(color.into()), role: ShapeRole::Body }
}

fn link(r: &mut Robot, ids: &mut Ids, name: &str, shapes: Vec<Shape>, mass: f64, color: &str) -> Id {
    let id = Ids::take(&mut ids.links, r);
    r.links.push(Link { id, name: name.into(), shapes, parts: vec![], mass: Some(mass), color: color.into(), collision: Collision::Simple, material: None });
    id
}

#[allow(clippy::too_many_arguments)]
fn joint(r: &mut Robot, ids: &mut Ids, name: &str, kind: JointKind, parent: Id, child: Id, anchor: V3, axis: V3, range: Option<[f64; 2]>, initial: f64) -> Id {
    let id = Ids::take(&mut ids.joints, r);
    r.joints.push(Joint { id, name: name.into(), kind, parent, child, anchor, axis, range, initial, damping: 0.0, friction: 0.0, follows: None, spring: None, source: "template".into() });
    id
}

#[allow(clippy::too_many_arguments)]
fn drive(r: &mut Robot, ids: &mut Ids, name: &str, motor: &str, count: u32, ratio: f64, target: DriveTarget, control: Control, stator: f64) -> Id {
    let id = Ids::take(&mut ids.drives, r);
    r.drives.push(Drive { id, name: name.into(), motor: motor.into(), count, ratio, efficiency: 0.9, stator_limit: stator, supply_limit: 60.0, target, control, brake: true });
    id
}

fn shaft(r: &mut Robot, ids: &mut Ids, name: &str, on: Id, inertia: f64) -> Id {
    let id = Ids::take(&mut ids.shafts, r);
    r.shafts.push(Shaft { id, name: name.into(), link: on, inertia, drag: 2e-4 });
    id
}

/// rollers across a width: one per wheel, evenly spaced, on one shaft
#[allow(clippy::too_many_arguments)]
fn rollers(r: &mut Robot, ids: &mut Ids, name: &str, sh: Id, center: V3, axis: V3, width: f64, wheel: &str, n: u32, wheel_width: f64) {
    let n = n.max(1);
    let a = norm(axis);
    for i in 0..n {
        let f = if n == 1 { 0.0 } else { (i as f64 / (n - 1) as f64 - 0.5) * (width - wheel_width) };
        let id = Ids::take(&mut ids.rollers, r);
        r.rollers.push(Roller { id, name: if n == 1 { name.into() } else { format!("{name} {}", i + 1) }, shaft: sh, ratio: 1.0,
            center: math::add(center, scale(a, f)), axis: a, width: if n == 1 { width } else { wheel_width }, wheel: wheel.into(), diameter: None,
            kind: RollerKind::Plain, part: None });
    }
}

fn wheel_size(wheel: &str) -> (f64, f64) {
    let lib = library::Library::builtin();
    lib.wheel(wheel).map_or((0.0254, 0.0254), |w| (w.radius(), w.width()))
}

fn build(r: &mut Robot, id: Id, t: Template, ids: &mut Ids) -> anyhow::Result<()> {
    let base = mount_point(r, &t.mount);
    let parent = mount_link(r, &t.mount);
    let kind = t.params.kind();
    let mut m = Mechanism { id, name: kind.label().into(), kind, fidelity: Fidelity::Concept, links: vec![], template: None,
        pickup: None, holders: vec![], color: String::new() };
    let mut new_presets: Vec<(String, Id, f64)> = vec![];
    let mut new_bindings: Vec<Binding> = vec![];
    match &t.params {
        Params::Elevator { stages, travel, height, width, spool_diameter, motor, motors, ratio, carriage_mass, stage_mass, .. } => {
            m.color = "#e2a03f".into();
            let n = (*stages).clamp(1, 4);
            let per = travel / n as f64;
            let tube = 0.04;
            let upr = |name: &str, x: f64, z0: f64, h: f64, w: f64, c: &str| vec![
                bx(&format!("{name} left"), math::add(base, [x, w / 2.0, z0 + h / 2.0]), [tube, tube, h], c),
                bx(&format!("{name} right"), math::add(base, [x, -w / 2.0, z0 + h / 2.0]), [tube, tube, h], c),
                bx(&format!("{name} top"), math::add(base, [x, 0.0, z0 + h - tube / 2.0]), [tube, w, tube], c),
            ];
            let base_link = link(r, ids, "Elevator base", upr("fixed stage", 0.0, 0.0, *height, *width, "#b9bec4"), *stage_mass, "#b9bec4");
            joint(r, ids, "Elevator mount", JointKind::Fixed, parent, base_link, base, [0.0, 0.0, 1.0], None, 0.0);
            m.links.push(base_link);
            let mut prev = base_link;
            let mut first_joint = 0;
            for k in 1..=n {
                let last = k == n;
                let x = 0.045 * k as f64;
                let w = width - 0.03 * k as f64;
                let (shapes, mass, name) = if last {
                    (vec![bx("carriage", math::add(base, [x + 0.04, 0.0, height * 0.35]), [0.08, w, 0.22], "#e2a03f")], *carriage_mass, "Carriage".to_string())
                } else {
                    (upr(&format!("stage {k}"), x, 0.03 * k as f64, height - 0.03 * k as f64, w, "#cfd4da"), *stage_mass, format!("Stage {k}"))
                };
                let l = link(r, ids, &name, shapes, mass, "#cfd4da");
                let j = joint(r, ids, &format!("{name} slide"), JointKind::Slide, prev, l, math::add(base, [x, 0.0, 0.0]), [0.0, 0.0, 1.0], Some([0.0, per]), 0.0);
                if k == 1 { first_joint = j; } else if let Some(jj) = r.joint_mut(j) { jj.follows = Some(Follow { joint: first_joint, ratio: 1.0 }); }
                m.links.push(l);
                prev = l;
            }
            let d = drive(r, ids, "Elevator motors", motor, *motors, *ratio, DriveTarget::Joint { joint: first_joint, spool_radius: spool_diameter / 2.0 }, Control::Ideal, 60.0);
            new_presets.push(("Elevator down".into(), d, 0.0));
            new_presets.push(("Elevator mid".into(), d, per * 0.5));
            new_presets.push(("Elevator up".into(), d, per));
        }
        Params::Arm { length, width, mass, motor, motors, ratio, min_deg, max_deg, start_deg }
        | Params::Wrist { length, width, mass, motor, motors, ratio, min_deg, max_deg, start_deg } => {
            let wrist = matches!(t.params, Params::Wrist { .. });
            let name = if wrist { "Wrist" } else { "Arm" };
            m.color = if wrist { "#7fb35a".into() } else { "#4f8fd6".into() };
            let l = link(r, ids, name, vec![bx(&format!("{} body", name.to_lowercase()), math::add(base, [length / 2.0, 0.0, 0.0]), [*length, *width, 0.05], &m.color)], *mass, &m.color);
            // about -y, so a positive angle lifts the tip
            let j = joint(r, ids, &format!("{name} pivot"), JointKind::Hinge, parent, l, base, [0.0, -1.0, 0.0], Some([min_deg * DEG, max_deg * DEG]), start_deg * DEG);
            m.links.push(l);
            let d = drive(r, ids, &format!("{name} motor"), motor, *motors, *ratio, DriveTarget::Joint { joint: j, spool_radius: 0.0 }, Control::Ideal, 60.0);
            new_presets.push((format!("{name} stow"), d, start_deg * DEG));
            new_presets.push((format!("{name} level"), d, 0.0));
            new_presets.push((format!("{name} up"), d, (max_deg * 0.8).min(90.0) * DEG));
        }
        Params::Telescope { travel, length, mass, motor, motors, ratio, spool_diameter } => {
            m.color = "#9b6fd0".into();
            let l = link(r, ids, "Telescope", vec![bx("inner tube", math::add(base, [length / 2.0, 0.0, 0.0]), [*length, 0.05, 0.05], "#9b6fd0")], *mass, "#9b6fd0");
            let j = joint(r, ids, "Telescope slide", JointKind::Slide, parent, l, base, [1.0, 0.0, 0.0], Some([0.0, *travel]), 0.0);
            m.links.push(l);
            let d = drive(r, ids, "Telescope motor", motor, *motors, *ratio, DriveTarget::Joint { joint: j, spool_radius: spool_diameter / 2.0 }, Control::Ideal, 60.0);
            new_presets.push(("Telescope in".into(), d, 0.0));
            new_presets.push(("Telescope out".into(), d, *travel));
        }
        Params::Intake { width, height, ramp_deg, rollers: nr, wheel, wheels_per_roller, motor, ratio, mass, deploy, deploy_ratio } => {
            m.color = "#3fae6a".into();
            let (rr, ww) = wheel_size(wheel);
            // An over-the-bumper ground intake. A ramp rises at `ramp_deg` from the carpet in front
            // of the bumper to just over the bumper's top; the rollers sit `height` (centre to
            // ramp) above it, so the squeeze on a piece is the same all the way up. The front
            // roller grabs a piece off the carpet; the back one pushes it over the top.
            let f = &r.frame;
            let front = f.length / 2.0 + f.bumper_thickness;
            let bumper_top = f.bumper_bottom + f.bumper_height;
            let base = if t.mount.on.is_some() { base } else { math::add(base, [front, 0.0, 0.0]) };
            let th = ramp_deg.clamp(10.0, 60.0) * DEG;
            let r1 = math::add(base, [-0.06, 0.0, bumper_top + 0.02]);
            let run = (r1[2] - base[2] - 0.004) / th.tan();
            let r0 = [r1[0] + run, r1[1], base[2] + 0.004];
            let along = math::sub(r1, r0);
            let ramp_len = math::len(along);
            let d = norm(along);
            // the ramp's upward normal: up and a little forward
            let up = [th.sin(), 0.0, th.cos()];
            let c0 = math::add(math::lerp(r0, r1, (0.6 * rr / ramp_len).min(0.3)), scale(up, *height));
            let ramp = Shape { name: "ramp".into(), kind: ShapeKind::Box { size: [ramp_len, *width, 0.006] },
                pose: Pose::new(math::sub(math::lerp(r0, r1, 0.5), scale(up, 0.003)), math::quat_from_to([1.0, 0.0, 0.0], d)), color: Some("#9fc3e6".into()), role: ShapeRole::Guide };
            let side = |y: f64| bx("side plate", [ (c0[0] + r1[0]) / 2.0, y, (c0[2] + r1[2]) / 2.0 + 0.02 ], [c0[0] - r1[0] + 2.0 * rr + 0.04, 0.006, 2.0 * rr + 0.12], "#3fae6a");
            let shapes = vec![ramp, side(width / 2.0 + 0.01), side(-width / 2.0 - 0.01)];
            let l = link(r, ids, "Intake", shapes, *mass, "#3fae6a");
            let pivot = math::add(base, [-0.04, 0.0, bumper_top + 0.06]);
            if *deploy {
                let j = joint(r, ids, "Intake deploy", JointKind::Hinge, parent, l, pivot, [0.0, -1.0, 0.0], Some([0.0, 100.0 * DEG]), 0.0);
                let dd = drive(r, ids, "Intake deploy motor", motor, 1, *deploy_ratio, DriveTarget::Joint { joint: j, spool_radius: 0.0 }, Control::Ideal, 40.0);
                new_presets.push(("Intake out".into(), dd, 0.0));
                new_presets.push(("Intake in".into(), dd, 90.0 * DEG));
            } else {
                joint(r, ids, "Intake mount", JointKind::Fixed, parent, l, pivot, [0.0, 0.0, 1.0], None, 0.0);
            }
            m.links.push(l);
            let sh = shaft(r, ids, "Intake rollers", l, 3e-4);
            rollers(r, ids, "Front roller", sh, c0, [0.0, 1.0, 0.0], *width, wheel, *wheels_per_roller, ww);
            if *nr >= 2 {
                // near the ramp's top
                let at = math::add(math::lerp(r0, r1, 0.8), scale(up, *height));
                rollers(r, ids, "Back roller", sh, at, [0.0, 1.0, 0.0], *width, wheel, *wheels_per_roller, ww);
            }
            // +y axis: positive speed turns a roller's underside backward, pulling pieces in
            let dd = drive(r, ids, "Intake rollers", motor, 1, *ratio, DriveTarget::Shaft { shaft: sh }, Control::Voltage, 40.0);
            new_bindings.push(Binding { button: Button::RT, action: Action::Run { drive: dd, value: 10.0 } });
            new_bindings.push(Binding { button: Button::LB, action: Action::Run { drive: dd, value: -8.0 } });
            m.pickup = Some(Pickup { link: l, center: math::add(c0, [0.06, 0.0, -height / 2.0 + 0.06]), size: [0.24, *width, 0.2], acquire_time: 0.15, drive: Some(dd), pieces: vec![] });
            m.holders.push(Holder { name: "In the intake".into(), link: l, pose: Pose::at(math::add(r1, [-0.08, 0.0, 0.05])), release: Release::Place { eject: 1.5 } });
        }
        Params::Claw { width, gap, depth, wheel, motor, ratio, mass } => {
            m.color = "#d65a8a".into();
            let (rr, ww) = wheel_size(wheel);
            let shapes = vec![
                bx("left plate", math::add(base, [depth / 2.0, width / 2.0 + 0.01, 0.0]), [*depth, 0.006, gap + 2.0 * rr + 0.04], "#d65a8a"),
                bx("right plate", math::add(base, [depth / 2.0, -width / 2.0 - 0.01, 0.0]), [*depth, 0.006, gap + 2.0 * rr + 0.04], "#d65a8a"),
                bx("back plate", math::add(base, [0.005, 0.0, 0.0]), [0.01, *width, gap + 2.0 * rr], "#8e3b5c"),
            ];
            let l = link(r, ids, "Claw", shapes, *mass, "#d65a8a");
            joint(r, ids, "Claw mount", JointKind::Fixed, parent, l, base, [0.0, 0.0, 1.0], None, 0.0);
            m.links.push(l);
            let sh = shaft(r, ids, "Claw rollers", l, 2e-4);
            let mouth = math::add(base, [depth * 0.75, 0.0, 0.0]);
            let n = ((width / (ww + 0.02)).floor() as u32).clamp(1, 6);
            rollers(r, ids, "Top roller", sh, math::add(mouth, [0.0, 0.0, gap / 2.0 + rr]), [0.0, 1.0, 0.0], *width, wheel, n, ww);
            let bottom_start = r.rollers.len();
            rollers(r, ids, "Bottom roller", sh, math::add(mouth, [0.0, 0.0, -gap / 2.0 - rr]), [0.0, 1.0, 0.0], *width, wheel, n, ww);
            // the bottom roller turns the other way on the same belt
            for x in &mut r.rollers[bottom_start..] { x.ratio = -1.0; }
            let d = drive(r, ids, "Claw rollers", motor, 1, *ratio, DriveTarget::Shaft { shaft: sh }, Control::Voltage, 40.0);
            new_bindings.push(Binding { button: Button::RB, action: Action::Run { drive: d, value: 8.0 } });
            new_bindings.push(Binding { button: Button::LT, action: Action::Run { drive: d, value: -10.0 } });
            m.pickup = Some(Pickup { link: l, center: math::add(base, [depth + 0.05, 0.0, 0.0]), size: [0.15, *width, *gap + 0.04], acquire_time: 0.1, drive: Some(d), pieces: vec![] });
            m.holders.push(Holder { name: "In the claw".into(), link: l, pose: Pose::at(math::add(base, [depth * 0.5, 0.0, 0.0])), release: Release::Place { eject: 1.0 } });
        }
        Params::Shooter { wheel, width, motor, motors, ratio, hood_deg, mass, efficiency } => {
            m.color = "#c94f4f".into();
            let (rr, ww) = wheel_size(wheel);
            let a = hood_deg * DEG;
            let hood = Shape { name: "hood".into(), kind: ShapeKind::Box { size: [0.25, *width, 0.006] },
                pose: Pose::new(math::add(base, [0.0, 0.0, rr + 0.03]), axis_angle([0.0, -1.0, 0.0], a)), color: Some("#6b6b6b".into()), role: ShapeRole::Guide };
            let l = link(r, ids, "Shooter", vec![hood,
                bx("left plate", math::add(base, [0.0, width / 2.0 + 0.01, 0.0]), [0.25, 0.006, 0.25], "#c94f4f"),
                bx("right plate", math::add(base, [0.0, -width / 2.0 - 0.01, 0.0]), [0.25, 0.006, 0.25], "#c94f4f")], *mass, "#c94f4f");
            joint(r, ids, "Shooter mount", JointKind::Fixed, parent, l, base, [0.0, 0.0, 1.0], None, 0.0);
            m.links.push(l);
            let sh = shaft(r, ids, "Flywheel", l, 2.5e-3);
            let n = ((width / (ww + 0.01)).floor() as u32).clamp(1, 6);
            rollers(r, ids, "Flywheel", sh, base, [0.0, 1.0, 0.0], *width, wheel, n, ww);
            let d = drive(r, ids, "Flywheel motors", motor, *motors, *ratio, DriveTarget::Shaft { shaft: sh }, Control::Velocity { kp: 0.05, ki: 0.0, ks: 0.15, kv: 0.0 }, 60.0);
            new_bindings.push(Binding { button: Button::RT, action: Action::Run { drive: d, value: 400.0 } });
            m.holders.push(Holder { name: "Ready to shoot".into(), link: l,
                pose: Pose::new(math::add(base, [-0.06, 0.0, rr + 0.02]), axis_angle([0.0, -1.0, 0.0], a)), release: Release::Launch { shaft: sh, efficiency: *efficiency } });
        }
        Params::Turret { radius, motor, ratio, mass, range_deg } => {
            m.color = "#5aa7b3".into();
            let ring = Shape { name: "turntable".into(), kind: ShapeKind::Cylinder { radius: *radius, length: 0.03 }, pose: Pose::at(math::add(base, [0.0, 0.0, 0.015])), color: Some("#5aa7b3".into()), role: ShapeRole::Body };
            let l = link(r, ids, "Turret", vec![ring], *mass, "#5aa7b3");
            let half = range_deg / 2.0 * DEG;
            let j = joint(r, ids, "Turret turn", JointKind::Hinge, parent, l, base, [0.0, 0.0, 1.0], Some([-half, half]), 0.0);
            m.links.push(l);
            let d = drive(r, ids, "Turret motor", motor, 1, *ratio, DriveTarget::Joint { joint: j, spool_radius: 0.0 }, Control::Ideal, 40.0);
            new_presets.push(("Turret forward".into(), d, 0.0));
        }
        Params::Climber { length, motor, motors, ratio, mass } => {
            m.color = "#8c7a5b".into();
            let l = link(r, ids, "Climber", vec![bx("climber arm", math::add(base, [0.0, 0.0, length / 2.0]), [0.05, 0.05, *length], "#8c7a5b")], *mass, "#8c7a5b");
            let j = joint(r, ids, "Climber pivot", JointKind::Hinge, parent, l, base, [0.0, -1.0, 0.0], Some([-100.0 * DEG, 10.0 * DEG]), 0.0);
            m.links.push(l);
            let d = drive(r, ids, "Climber winch", motor, *motors, *ratio, DriveTarget::Joint { joint: j, spool_radius: 0.0 }, Control::Ideal, 80.0);
            new_presets.push(("Climber stow".into(), d, 0.0));
            new_presets.push(("Climber out".into(), d, -90.0 * DEG));
        }
    }
    m.template = Some(t);
    r.mechanisms.push(m);
    // presets and bindings are added only the first time (a rebuild keeps the owner's own)
    let existing: Vec<String> = r.controls.presets.iter().map(|p| p.name.clone()).collect();
    for (name, d, v) in new_presets {
        if existing.contains(&name) { continue; }
        let pid = r.id();
        r.controls.presets.push(Preset { id: pid, name, targets: vec![PresetTarget { drive: d, value: v }] });
    }
    for b in new_bindings {
        if !r.controls.bindings.iter().any(|x| x.button == b.button) { r.controls.bindings.push(b); }
    }
    assign_preset_buttons(r);
    Ok(())
}

/// Gives the first presets the face buttons and the D-pad, where those are free.
pub fn assign_preset_buttons(r: &mut Robot) {
    let order = [Button::A, Button::B, Button::X, Button::Y, Button::Up, Button::Down, Button::Left, Button::Right];
    for p in r.controls.presets.clone() {
        if r.controls.bindings.iter().any(|b| matches!(b.action, Action::Preset { preset } if preset == p.id)) { continue; }
        if let Some(btn) = order.iter().find(|b| !r.controls.bindings.iter().any(|x| x.button == **b)) {
            r.controls.bindings.push(Binding { button: *btn, action: Action::Preset { preset: p.id } });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elevator_with_claw_rebuilds_keeping_ids() {
        let mut r = Robot::new("t");
        let e = add(&mut r, Template::elevator_default()).unwrap();
        let mut claw = Template::claw_default();
        claw.mount.on = Some(e);
        let c = add(&mut r, claw).unwrap();
        let claw_links = r.mechanism(c).unwrap().links.clone();
        let drives_before: Vec<Id> = r.drives.iter().map(|d| d.id).collect();
        let n_presets = r.controls.presets.len();
        // three stages instead of two: one more link, the same drive id
        let mut t = r.mechanism(e).unwrap().template.clone().unwrap();
        if let Params::Elevator { stages, .. } = &mut t.params { *stages = 3; }
        rebuild(&mut r, e, t).unwrap();
        assert_eq!(r.mechanism(e).unwrap().links.len(), 4);
        assert_eq!(r.mechanism(c).unwrap().links, claw_links);
        for d in drives_before { assert!(r.drive(d).is_some(), "drive {d} kept"); }
        assert_eq!(r.controls.presets.len(), n_presets);
        // the claw rides the new carriage
        let carriage = *r.mechanism(e).unwrap().links.last().unwrap();
        assert_eq!(r.joint_of(claw_links[0]).unwrap().parent, carriage);
        // stage joints follow the first
        let followers = r.joints.iter().filter(|j| j.follows.is_some()).count();
        assert_eq!(followers, 2);
        assert!(crate::validate::issues(&r, &library::Library::builtin()).iter().all(|i| i.severity != crate::validate::Severity::Error));
    }

    #[test]
    fn remove_takes_everything() {
        let mut r = Robot::new("t");
        let i = add(&mut r, Template::intake_default()).unwrap();
        assert!(!r.rollers.is_empty());
        r.remove_mechanism(i);
        assert!(r.rollers.is_empty() && r.shafts.is_empty() && r.drives.is_empty() && r.links.len() == 1);
    }
}
