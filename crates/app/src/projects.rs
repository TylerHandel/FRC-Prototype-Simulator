//! Projects are folders (PLAN.md D10): robot.json, cad/ (the cadpack), runs/ (recordings).
//! Also the starting points offered on the Home screen.
use model::templates::{self, Template};
use model::{Fidelity, Project};
use std::path::{Path, PathBuf};

pub const FILE: &str = "robot.json";

pub fn load(dir: &Path) -> anyhow::Result<Project> {
    let s = std::fs::read_to_string(dir.join(FILE))?;
    Project::from_json(&s)
}

/// Saves by writing a new file and renaming it over the old, so a crash never leaves half a file.
pub fn save(dir: &Path, p: &Project) -> anyhow::Result<()> {
    std::fs::create_dir_all(dir)?;
    let tmp = dir.join("robot.json.tmp");
    std::fs::write(&tmp, p.to_json())?;
    std::fs::rename(&tmp, dir.join(FILE))?;
    Ok(())
}

/// A folder name that isn't taken yet, under `parent`.
pub fn fresh_dir(parent: &Path, name: &str) -> PathBuf {
    let clean: String = name.chars().map(|c| if c.is_alphanumeric() || c == ' ' || c == '-' || c == '_' { c } else { '_' }).collect();
    let mut d = parent.join(clean.trim());
    let mut i = 2;
    while d.exists() { d = parent.join(format!("{} {i}", clean.trim())); i += 1; }
    d
}

pub fn elevator_example() -> Project {
    let mut p = Project::new("Elevator and claw");
    // forward enough that the claw reaches over the REEF's base to an L4 branch
    let mut el = Template::elevator_default();
    el.mount.offset[0] = 0.2;
    let e = templates::add(&mut p.robot, el).unwrap();
    let mut c = Template::claw_default();
    c.mount.on = Some(e);
    templates::add(&mut p.robot, c).unwrap();
    templates::add(&mut p.robot, Template::intake_default()).unwrap();
    p
}

/// The starting points on the Home screen: (title, what it is, the project).
pub fn starters() -> Vec<(&'static str, &'static str, Project)> {
    let mut v = vec![("Elevator, claw and ground intake", "A REEFSCAPE archetype: a two-stage elevator with a roller claw on its carriage, and an over-the-bumper CORAL intake.", elevator_example())];
    let mut p = Project::new("Arm and wrist");
    let a = templates::add(&mut p.robot, Template::arm_default()).unwrap();
    let mut w = Template::wrist_default();
    w.mount.on = Some(a);
    let wid = templates::add(&mut p.robot, w).unwrap();
    let mut c = Template::claw_default();
    c.mount.on = Some(wid);
    templates::add(&mut p.robot, c).unwrap();
    v.push(("Arm, wrist and claw", "A pivoting arm with a wrist and a roller claw, like the champions' long-reach designs.", p));
    let mut p = Project::new("Intake test");
    p.robot.drivetrain = None;
    let i = templates::add(&mut p.robot, Template::intake_default()).unwrap();
    if let Some(m) = p.robot.mechanism_mut(i) { m.fidelity = Fidelity::Full; }
    v.push(("Intake on the Bench", "One roller intake on a stand, for tuning squeeze, wheels and current against a CORAL.", p));
    v.push(("Just a drivetrain", "A 27 in. swerve base with bumpers. Add mechanisms from the palette.", Project::new("My robot")));
    v
}
