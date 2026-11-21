//! What a robot *is* (PLAN.md "The robot description"): links, joints, drives, rollers,
//! mechanisms, sensors, electrical and controls, plus the CAD parts they are made of.
//!
//! Frames: everything is stated in the robot's frame at its design pose (all joints at zero), the
//! way CAD states it: x forward, y left, z up, metres, the origin on the floor under the frame's
//! centre. The simulator builds its own frames from that.
//!
//! A project is a folder (D10): `robot.json` holds a `Project`; `cad/` the cadpack; `runs/` recordings.
pub mod math;
pub mod templates;
pub mod validate;
pub mod history;

use serde::{Deserialize, Serialize};

pub use math::{Pose, Quat, V3};

pub type Id = u32;
pub const SCHEMA: u32 = 1;
/// The chassis link every robot has.
pub const CHASSIS: Id = 1;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Project {
    pub schema: u32,
    pub name: String,
    pub robot: Robot,
    #[serde(default)]
    pub bench: Bench,
    /// game pack id (assets/games/<id>)
    #[serde(default = "default_game")]
    pub game: String,
    /// forces every mechanism to one fidelity when set
    #[serde(default)]
    pub force_fidelity: Option<Fidelity>,
    #[serde(default)]
    pub notes: String,
}

fn default_game() -> String { "reefscape-2025".into() }

impl Project {
    pub fn new(name: &str) -> Self {
        Project { schema: SCHEMA, name: name.into(), robot: Robot::new(name), bench: Bench::default(), game: default_game(), force_fidelity: None, notes: String::new() }
    }
    pub fn to_json(&self) -> String { serde_json::to_string_pretty(self).expect("project to json") }
    pub fn from_json(s: &str) -> anyhow::Result<Self> {
        let mut v: serde_json::Value = serde_json::from_str(s)?;
        migrate(&mut v)?;
        Ok(serde_json::from_value(v)?)
    }
    pub fn fidelity(&self, m: &Mechanism) -> Fidelity { self.force_fidelity.unwrap_or(m.fidelity) }
}

/// Brings an older file up to the current schema (D10). Schema 1 is the first release.
pub fn migrate(v: &mut serde_json::Value) -> anyhow::Result<()> {
    let schema = v.get("schema").and_then(|s| s.as_u64()).unwrap_or(1);
    if schema > SCHEMA as u64 {
        anyhow::bail!("this project was saved by a newer version of the app (schema {schema}); update the app to open it");
    }
    Ok(())
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Robot {
    pub name: String,
    pub next_id: Id,
    pub frame: Frame,
    #[serde(default)]
    pub drivetrain: Option<Drivetrain>,
    pub links: Vec<Link>,
    #[serde(default)]
    pub joints: Vec<Joint>,
    #[serde(default)]
    pub drives: Vec<Drive>,
    #[serde(default)]
    pub shafts: Vec<Shaft>,
    #[serde(default)]
    pub rollers: Vec<Roller>,
    #[serde(default)]
    pub mechanisms: Vec<Mechanism>,
    #[serde(default)]
    pub sensors: Vec<Sensor>,
    #[serde(default)]
    pub electrical: Electrical,
    #[serde(default)]
    pub controls: Controls,
    /// the CAD's parts, when the robot came from CAD
    #[serde(default)]
    pub parts: Vec<Part>,
    #[serde(default)]
    pub cad: Option<CadSource>,
    /// the weighed total, kg: link masses are scaled to it when set
    #[serde(default)]
    pub weighed_mass: Option<f64>,
}

impl Robot {
    /// A drivable starting point: a 27 in. square frame with bumpers and swerve.
    pub fn new(name: &str) -> Self {
        let frame = Frame::default();
        let mut r = Robot {
            name: name.into(), next_id: 2, frame: frame.clone(), drivetrain: Some(Drivetrain::default()),
            links: vec![], joints: vec![], drives: vec![], shafts: vec![], rollers: vec![], mechanisms: vec![],
            sensors: vec![], electrical: Electrical::default(), controls: Controls::default(), parts: vec![], cad: None,
            weighed_mass: None,
        };
        r.links.push(chassis_link(&frame));
        r
    }
    pub fn id(&mut self) -> Id { let i = self.next_id; self.next_id += 1; i }
    pub fn link(&self, id: Id) -> Option<&Link> { self.links.iter().find(|l| l.id == id) }
    pub fn link_mut(&mut self, id: Id) -> Option<&mut Link> { self.links.iter_mut().find(|l| l.id == id) }
    pub fn joint(&self, id: Id) -> Option<&Joint> { self.joints.iter().find(|j| j.id == id) }
    pub fn joint_mut(&mut self, id: Id) -> Option<&mut Joint> { self.joints.iter_mut().find(|j| j.id == id) }
    pub fn drive(&self, id: Id) -> Option<&Drive> { self.drives.iter().find(|d| d.id == id) }
    pub fn drive_mut(&mut self, id: Id) -> Option<&mut Drive> { self.drives.iter_mut().find(|d| d.id == id) }
    pub fn shaft(&self, id: Id) -> Option<&Shaft> { self.shafts.iter().find(|s| s.id == id) }
    pub fn roller(&self, id: Id) -> Option<&Roller> { self.rollers.iter().find(|r| r.id == id) }
    pub fn roller_mut(&mut self, id: Id) -> Option<&mut Roller> { self.rollers.iter_mut().find(|r| r.id == id) }
    pub fn mechanism(&self, id: Id) -> Option<&Mechanism> { self.mechanisms.iter().find(|m| m.id == id) }
    pub fn mechanism_mut(&mut self, id: Id) -> Option<&mut Mechanism> { self.mechanisms.iter_mut().find(|m| m.id == id) }
    /// the joint that moves a link against its parent
    pub fn joint_of(&self, link: Id) -> Option<&Joint> { self.joints.iter().find(|j| j.child == link) }
    /// the mechanism a link belongs to
    pub fn mechanism_of_link(&self, link: Id) -> Option<&Mechanism> { self.mechanisms.iter().find(|m| m.links.contains(&link)) }
    /// links in an order where every parent comes before its children (chassis first)
    pub fn link_order(&self) -> Vec<Id> {
        let mut out = vec![CHASSIS];
        let mut i = 0;
        while i < out.len() {
            let p = out[i];
            for j in self.joints.iter().filter(|j| j.parent == p) {
                if !out.contains(&j.child) { out.push(j.child); }
            }
            i += 1;
        }
        // links no joint reaches hang on the chassis (validate reports them)
        for l in &self.links { if !out.contains(&l.id) { out.push(l.id); } }
        out
    }
    /// the parent link of a link (None for the chassis or an unattached link)
    pub fn parent_of(&self, link: Id) -> Option<Id> { self.joint_of(link).map(|j| j.parent) }
    /// Removes a mechanism and everything it made.
    pub fn remove_mechanism(&mut self, id: Id) {
        let Some(m) = self.mechanism(id).cloned() else { return };
        let links: Vec<Id> = m.links.clone();
        let joints: Vec<Id> = self.joints.iter().filter(|j| links.contains(&j.child)).map(|j| j.id).collect();
        let shafts: Vec<Id> = self.shafts.iter().filter(|s| links.contains(&s.link)).map(|s| s.id).collect();
        self.links.retain(|l| !links.contains(&l.id));
        self.joints.retain(|j| !joints.contains(&j.id));
        self.rollers.retain(|r| !shafts.contains(&r.shaft));
        self.shafts.retain(|s| !shafts.contains(&s.id));
        self.drives.retain(|d| match d.target {
            DriveTarget::Joint { joint, .. } => !joints.contains(&joint),
            DriveTarget::Shaft { shaft } => !shafts.contains(&shaft),
        });
        let drives: Vec<Id> = self.drives.iter().map(|d| d.id).collect();
        for p in &mut self.controls.presets { p.targets.retain(|t| drives.contains(&t.drive)); }
        self.controls.bindings.retain(|b| match &b.action { Action::Run { drive, .. } => drives.contains(drive), _ => true });
        self.sensors.retain(|s| !links.contains(&s.link()));
        self.mechanisms.retain(|m| m.id != id);
        // mechanisms mounted on this one fall back to the chassis
        for m in &mut self.mechanisms {
            if let Some(t) = &mut m.template { if t.mount.on == Some(id) { t.mount.on = None; } }
        }
    }
    /// total mass of the description (links as stated or estimated from their shapes), kg
    pub fn mass(&self) -> f64 { self.links.iter().map(|l| l.mass_or_estimate()).sum() }
}

pub fn chassis_link(f: &Frame) -> Link {
    let (l, w) = (f.length, f.width);
    let bt = f.bumper_thickness;
    let bz = f.bumper_bottom + f.bumper_height / 2.0;
    let bumper = |name: &str, x: f64, y: f64, sx: f64, sy: f64| Shape {
        name: name.into(), kind: ShapeKind::Box { size: [sx, sy, f.bumper_height] },
        pose: Pose::at([x, y, bz]), color: Some("#b8312f".into()), role: ShapeRole::Bumper,
    };
    Link {
        id: CHASSIS, name: "Chassis".into(),
        shapes: vec![
            Shape { name: "frame".into(), kind: ShapeKind::Box { size: [l, w, 0.05] }, pose: Pose::at([0.0, 0.0, 0.065]), color: Some("#9aa3ad".into()), role: ShapeRole::Body },
            Shape { name: "belly pan".into(), kind: ShapeKind::Box { size: [l - 0.1, w - 0.1, 0.03] }, pose: Pose::at([0.0, 0.0, 0.05]), color: Some("#4b5560".into()), role: ShapeRole::Body },
            bumper("front bumper", l / 2.0 + bt / 2.0, 0.0, bt, w + 2.0 * bt),
            bumper("back bumper", -l / 2.0 - bt / 2.0, 0.0, bt, w + 2.0 * bt),
            bumper("left bumper", 0.0, w / 2.0 + bt / 2.0, l, bt),
            bumper("right bumper", 0.0, -w / 2.0 - bt / 2.0, l, bt),
        ],
        parts: vec![], mass: Some(f.chassis_mass), color: "#9aa3ad".into(), collision: Collision::Box, material: None,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Frame {
    /// frame perimeter, without bumpers, m
    pub length: f64,
    pub width: f64,
    pub bumper_thickness: f64,
    pub bumper_bottom: f64,
    pub bumper_height: f64,
    /// drivetrain, battery and electronics, kg
    pub chassis_mass: f64,
    /// height of the chassis's centre of mass above the floor, m
    pub chassis_com_height: f64,
}

impl Default for Frame {
    fn default() -> Self {
        Frame { length: 27.0 * 0.0254, width: 27.0 * 0.0254, bumper_thickness: 3.25 * 0.0254, bumper_bottom: 0.025, bumper_height: 5.0 * 0.0254,
            chassis_mass: 32.0, chassis_com_height: 0.09 }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DriveKind { Swerve, Tank, Mecanum }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Drivetrain {
    pub kind: DriveKind,
    /// library module id (swerve)
    pub module: String,
    pub motor: String,
    /// motor turns per wheel turn
    pub ratio: f64,
    /// library wheel id
    pub wheel: String,
    pub stator_limit: f64,
    pub supply_limit: f64,
    /// wheel-to-carpet friction
    pub mu: f64,
    /// module (or wheel) positions in the robot frame, x y, m; empty: at the frame's corners
    #[serde(default)]
    pub positions: Vec<[f64; 2]>,
    /// limits the driver can ask for
    pub max_speed: f64,
    pub max_turn: f64,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Drivetrain { kind: DriveKind::Swerve, module: "sds-mk4i-l2".into(), motor: "kraken-x60-foc".into(), ratio: 6.75,
            wheel: "nitrile-tread-4in".into(), stator_limit: 80.0, supply_limit: 70.0, mu: 1.2, positions: vec![],
            max_speed: 5.0, max_turn: 10.0 }
    }
}

impl Drivetrain {
    pub fn module_positions(&self, f: &Frame) -> Vec<[f64; 2]> {
        if !self.positions.is_empty() { return self.positions.clone(); }
        let (x, y) = (f.length / 2.0 - 0.0635, f.width / 2.0 - 0.0635);
        vec![[x, y], [x, -y], [-x, y], [-x, -y]]
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum Collision {
    /// one box round the link
    #[default]
    Box,
    /// a few fitted boxes and cylinders, or the link's own blocks
    Simple,
    /// convex pieces of the parts flagged as touching game pieces
    Detailed,
    /// no collision (wiring, things inside the frame)
    None,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Link {
    pub id: Id,
    pub name: String,
    /// blocks: what a Concept link is made of, or primitives fitted to its CAD
    #[serde(default)]
    pub shapes: Vec<Shape>,
    /// CAD parts (stable part ids) that move with this link
    #[serde(default)]
    pub parts: Vec<String>,
    /// stated mass, kg; None: from the parts or the blocks
    #[serde(default)]
    pub mass: Option<f64>,
    #[serde(default)]
    pub color: String,
    #[serde(default)]
    pub collision: Collision,
    #[serde(default)]
    pub material: Option<String>,
}

impl Link {
    /// mass as stated, or from the blocks at aluminium-structure density (a light frame of tube)
    pub fn mass_or_estimate(&self) -> f64 {
        self.mass.unwrap_or_else(|| self.shapes.iter().filter(|s| s.role != ShapeRole::Ghost).map(|s| s.kind.volume() * 350.0).sum::<f64>().max(0.05))
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum ShapeRole {
    /// structure: collides and has mass
    #[default]
    Body,
    /// bumpers: collide with low friction
    Bumper,
    /// a surface that game pieces ride on (a plate, a hood): rigid contact with pieces
    Guide,
    /// drawn only
    Ghost,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Shape {
    #[serde(default)]
    pub name: String,
    pub kind: ShapeKind,
    /// in the robot frame at the design pose
    pub pose: Pose,
    #[serde(default)]
    pub color: Option<String>,
    #[serde(default)]
    pub role: ShapeRole,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum ShapeKind {
    /// full sizes, m
    Box { size: [f64; 3] },
    /// along the shape's z
    Cylinder { radius: f64, length: f64 },
    Sphere { radius: f64 },
    /// the convex hull of these points (from CAD), centred on the shape's pose
    Hull { points: Vec<V3> },
}

impl ShapeKind {
    pub fn volume(&self) -> f64 {
        match self {
            ShapeKind::Box { size } => size[0] * size[1] * size[2],
            ShapeKind::Cylinder { radius, length } => std::f64::consts::PI * radius * radius * length,
            ShapeKind::Sphere { radius } => 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3),
            // about half its box: a hull is rarely a full box
            ShapeKind::Hull { .. } => { let e = self.extents(); 0.5 * e[0] * e[1] * e[2] }
        }
    }
    /// the size of the box round it, in its own frame
    pub fn extents(&self) -> V3 {
        match self {
            ShapeKind::Box { size } => *size,
            ShapeKind::Cylinder { radius, length } => [2.0 * radius, 2.0 * radius, *length],
            ShapeKind::Sphere { radius } => [2.0 * radius; 3],
            ShapeKind::Hull { points } => {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for p in points { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
                if points.is_empty() { [0.0; 3] } else { [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]] }
            }
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum JointKind { Hinge, Slide, Fixed }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Joint {
    pub id: Id,
    pub name: String,
    pub kind: JointKind,
    pub parent: Id,
    pub child: Id,
    /// a point on the axis, robot frame, design pose
    pub anchor: V3,
    /// unit axis, robot frame, design pose (a hinge turns right-handed about it)
    pub axis: V3,
    /// travel, rad or m, from the design pose; None: free
    #[serde(default)]
    pub range: Option<[f64; 2]>,
    /// where it starts, rad or m
    #[serde(default)]
    pub initial: f64,
    /// viscous damping, N*m*s or N*s
    #[serde(default)]
    pub damping: f64,
    /// dry friction, N*m or N
    #[serde(default)]
    pub friction: f64,
    /// moves as another joint does, times a ratio (the stages of a cascade elevator)
    #[serde(default)]
    pub follows: Option<Follow>,
    /// a spring or gas spring: force = preload + stiffness * travel
    #[serde(default)]
    pub spring: Option<Spring>,
    /// where this joint came from (a mate's name, "two poses", a template)
    #[serde(default)]
    pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Follow { pub joint: Id, pub ratio: f64 }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Spring { pub preload: f64, pub stiffness: f64 }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Drive {
    pub id: Id,
    pub name: String,
    pub motor: String,
    pub count: u32,
    /// motor turns per turn of what it drives (a joint's output, or a shaft)
    pub ratio: f64,
    pub efficiency: f64,
    pub stator_limit: f64,
    pub supply_limit: f64,
    pub target: DriveTarget,
    #[serde(default)]
    pub control: Control,
    /// brake (true) or coast when commanded to zero
    #[serde(default = "yes")]
    pub brake: bool,
}

fn yes() -> bool { true }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum DriveTarget {
    /// a joint; a slide is driven through a spool or pulley of this radius (m of travel per rad)
    Joint { joint: Id, #[serde(default)] spool_radius: f64 },
    /// a shaft of rollers
    Shaft { shaft: Id },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Control {
    /// fixed volts while running (rollers)
    Voltage,
    /// output speed, rad/s; gains on output units (volts per rad/s)
    Velocity { kp: f64, ki: f64, ks: f64, kv: f64 },
    /// output position, rad or m; a trapezoid profile when cruise and accel are set
    Position { kp: f64, ki: f64, kd: f64, ks: f64, kg: f64, kv: f64, cruise: f64, accel: f64 },
    /// the fastest the motor allows, to the target (Concept)
    Ideal,
}

impl Default for Control { fn default() -> Self { Control::Voltage } }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Shaft {
    pub id: Id,
    pub name: String,
    /// the link that carries its bearings
    pub link: Id,
    /// its own inertia (rollers' hubs and shaft), kg*m^2
    pub inertia: f64,
    /// bearing drag, N*m*s
    pub drag: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum RollerKind { #[default] Plain, Omni, Mecanum }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Roller {
    pub id: Id,
    pub name: String,
    pub shaft: Id,
    /// roller turns per shaft turn (a belt to a different-size pulley)
    #[serde(default = "one")]
    pub ratio: f64,
    /// centre and axis, robot frame, design pose
    pub center: V3,
    pub axis: V3,
    /// the width it covers along its axis, m (several wheels on a shaft: one roller each)
    pub width: f64,
    /// library wheel id: what is on it (D19)
    pub wheel: String,
    /// a diameter of its own, m (the CAD's), instead of the wheel's
    #[serde(default)]
    pub diameter: Option<f64>,
    #[serde(default)]
    pub kind: RollerKind,
    /// the CAD part it came from
    #[serde(default)]
    pub part: Option<String>,
}

fn one() -> f64 { 1.0 }

impl Roller {
    pub fn radius(&self, lib: &library::Library) -> f64 {
        self.diameter.map(|d| d / 2.0).or_else(|| lib.wheel(&self.wheel).map(|w| w.radius())).unwrap_or(0.0254)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
#[serde(rename_all = "lowercase")]
pub enum Fidelity {
    /// pickup zones and holders: in the zone with the intake running, the piece is yours
    #[default]
    Concept,
    /// contact: squash, friction, slip, jams
    Full,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum MechKind { Elevator, Arm, Wrist, Telescope, Intake, Claw, Shooter, Turret, Climber, Custom }

impl MechKind {
    pub fn label(self) -> &'static str {
        match self {
            MechKind::Elevator => "Elevator", MechKind::Arm => "Arm", MechKind::Wrist => "Wrist",
            MechKind::Telescope => "Telescope", MechKind::Intake => "Roller intake", MechKind::Claw => "Claw / end effector",
            MechKind::Shooter => "Shooter", MechKind::Turret => "Turret", MechKind::Climber => "Climber", MechKind::Custom => "Custom",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Mechanism {
    pub id: Id,
    pub name: String,
    pub kind: MechKind,
    pub fidelity: Fidelity,
    /// the links it owns
    pub links: Vec<Id>,
    /// the template it was built from, if any: changing its numbers rebuilds the mechanism
    #[serde(default)]
    pub template: Option<templates::Template>,
    #[serde(default)]
    pub pickup: Option<Pickup>,
    #[serde(default)]
    pub holders: Vec<Holder>,
    #[serde(default)]
    pub color: String,
}

impl Mechanism {
    /// the link a mechanism mounted on this one attaches to: its last link
    pub fn end_link(&self) -> Option<Id> { self.links.last().copied() }
}

/// Concept pickup (D3): a box on a link; a piece whose centre is in it while `drive` runs inward is
/// acquired after `acquire_time` and held at the mechanism's first free holder.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Pickup {
    pub link: Id,
    /// centre and full size, robot frame at design pose
    pub center: V3,
    pub size: V3,
    pub acquire_time: f64,
    /// the drive that must run (in the sense that pulls pieces in) for pickup
    #[serde(default)]
    pub drive: Option<Id>,
    /// piece ids it takes; empty: any
    #[serde(default)]
    pub pieces: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Holder {
    pub name: String,
    pub link: Id,
    /// where a held piece sits, robot frame at design pose
    pub pose: Pose,
    pub release: Release,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Release {
    /// let go with the holder's own velocity plus `eject` m/s along the holder's x
    Place { eject: f64 },
    /// thrown at the surface speed of `shaft`'s rollers times `efficiency`, along the holder's x
    Launch { shaft: Id, efficiency: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Sensor {
    /// a ray between two points on a link; reads true while a piece crosses it
    BeamBreak { name: String, link: Id, from: V3, to: V3 },
    /// true when a joint is within `band` of `at`
    LimitSwitch { name: String, joint: Id, at: f64, band: f64, link: Id },
}

impl Sensor {
    pub fn name(&self) -> &str { match self { Sensor::BeamBreak { name, .. } | Sensor::LimitSwitch { name, .. } => name } }
    pub fn link(&self) -> Id { match self { Sensor::BeamBreak { link, .. } | Sensor::LimitSwitch { link, .. } => *link } }
}

/// One electrical model for the whole robot (D6).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Electrical {
    /// open-circuit voltage, full charge
    pub battery_voltage: f64,
    /// battery internal resistance, ohm
    pub battery_resistance: f64,
    /// main wiring, breaker and PDH, ohm
    pub wiring_resistance: f64,
    /// controller, radio, cameras, LEDs, A
    pub fixed_load: f64,
    pub main_breaker: f64,
    /// the controller browns out below this, V (roboRIO 6.75 V; Systemcore's is not known yet)
    pub brownout: f64,
    /// trip the main breaker in simulation (it only warns otherwise)
    #[serde(default)]
    pub trip_breakers: bool,
}

impl Default for Electrical {
    fn default() -> Self {
        Electrical { battery_voltage: 12.8, battery_resistance: 0.012, wiring_resistance: 0.008, fixed_load: 6.0, main_breaker: 120.0, brownout: 6.75, trip_breakers: false }
    }
}

/// Controls without code (D12): presets and what the gamepad does.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct Controls {
    pub presets: Vec<Preset>,
    pub bindings: Vec<Binding>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Preset {
    pub id: Id,
    pub name: String,
    pub targets: Vec<PresetTarget>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PresetTarget { pub drive: Id, pub value: f64 }

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Button { A, B, X, Y, LB, RB, LT, RT, Up, Down, Left, Right, Start, Back, LStick, RStick }

impl Button {
    pub const ALL: [Button; 16] = [Button::A, Button::B, Button::X, Button::Y, Button::LB, Button::RB, Button::LT, Button::RT,
        Button::Up, Button::Down, Button::Left, Button::Right, Button::Start, Button::Back, Button::LStick, Button::RStick];
    /// the keyboard key that stands in for it
    pub fn key(self) -> &'static str {
        match self {
            Button::A => "J", Button::B => "K", Button::X => "U", Button::Y => "I", Button::LB => "Q", Button::RB => "E",
            Button::LT => "Space", Button::RT => "F", Button::Up => "1", Button::Down => "2", Button::Left => "3",
            Button::Right => "4", Button::Start => "Enter", Button::Back => "Backspace", Button::LStick => "Z", Button::RStick => "X",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Binding { pub button: Button, pub action: Action }

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum Action {
    /// go to a preset
    Preset { preset: Id },
    /// run a drive at `value` (volts, or its velocity target) while held; with `until`, it
    /// stops as soon as that sensor (by name) reads true (an intake that stops when it has a piece)
    Run { drive: Id, value: f64, #[serde(default, skip_serializing_if = "Option::is_none")] until: Option<String> },
    /// let go of what a mechanism holds (Concept)
    Release { mechanism: Id },
}

/// A CAD part, with an id that survives re-import (D9).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Part {
    /// the name path joined with '/', then '#' and the instance number among same-named siblings
    pub id: String,
    pub name: String,
    pub path: Vec<String>,
    pub fingerprint: Fingerprint,
    #[serde(default)]
    pub material: Option<String>,
    /// from the BOM, kg
    #[serde(default)]
    pub mass: Option<f64>,
    /// fasteners and other parts left out of collision and drawing
    #[serde(default)]
    pub hardware: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct Fingerprint {
    /// m^3
    pub volume: f64,
    /// centre of mass, robot frame
    pub com: V3,
    pub bbox: [V3; 2],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CadSource {
    pub kind: CadKind,
    /// the cadpack folder, relative to the project
    pub cadpack: String,
    pub imported: String,
    /// CAD frame to robot frame (the importer's "which way is forward" and "where is the floor")
    #[serde(default)]
    pub to_robot: Pose,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum CadKind {
    Onshape { url: String, document: String, workspace: String, element: String },
    Step { path: String },
}

/// The Bench (M1): one mechanism on a stand and a feeder throwing pieces at it.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Bench {
    pub piece: String,
    /// where pieces start, ahead of the robot's front bumper, m
    pub distance: f64,
    /// how fast they come at the robot (the robot driving onto them), m/s
    pub speed: f64,
    /// the piece's turn about the vertical as it arrives, deg, and how much that varies
    pub yaw: f64,
    pub yaw_spread: f64,
    /// sideways offset from the robot's centre line, m, and its spread
    pub offset: f64,
    pub offset_spread: f64,
    /// a new piece every this many seconds (0: one piece)
    pub interval: f64,
    /// how long one run lasts, s
    pub duration: f64,
    /// what to run while it does (drive id and value)
    pub run: Vec<PresetTarget>,
    /// wheels to compare side by side on the same rollers (D19), up to four
    pub compare: Vec<String>,
    pub seed: u64,
}

impl Default for Bench {
    fn default() -> Self {
        Bench { piece: "coral-2025".into(), distance: 0.35, speed: 1.0, yaw: 0.0, yaw_spread: 0.0, offset: 0.0, offset_spread: 0.0,
            interval: 0.0, duration: 1.5, run: vec![], compare: vec![], seed: 1 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_and_rejects_newer() {
        let mut p = Project::new("Test");
        templates::add(&mut p.robot, templates::Template::elevator_default()).unwrap();
        let s = p.to_json();
        let q = Project::from_json(&s).unwrap();
        assert_eq!(p, q);
        let newer = s.replacen("\"schema\": 1", "\"schema\": 99", 1);
        assert!(Project::from_json(&newer).is_err());
    }
}
