//! The list of what is missing (interface rule 6): everything that stops or weakens the
//! simulation, each pointing at what to fix.
use crate::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity { Error, Warning, Info }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target { Robot, Link(Id), Joint(Id), Drive(Id), Roller(Id), Mechanism(Id), Electrical, Drivetrain }

#[derive(Debug, Clone)]
pub struct Issue { pub severity: Severity, pub message: String, pub target: Target }

fn issue(v: &mut Vec<Issue>, severity: Severity, target: Target, message: String) { v.push(Issue { severity, message, target }); }

pub fn issues(r: &Robot, lib: &library::Library) -> Vec<Issue> {
    let mut v = Vec::new();
    if r.link(CHASSIS).is_none() { issue(&mut v, Severity::Error, Target::Robot, "The robot has no chassis".into()); }
    // links
    for l in &r.links {
        if l.id != CHASSIS && r.joint_of(l.id).is_none() {
            issue(&mut v, Severity::Warning, Target::Link(l.id), format!("{} is not attached to anything: it rides on the chassis", l.name));
        }
        if l.shapes.is_empty() && l.parts.is_empty() {
            issue(&mut v, Severity::Info, Target::Link(l.id), format!("{} has no shape or CAD parts: it has mass but nothing to hit", l.name));
        }
    }
    // joints
    let driven: Vec<Id> = r.drives.iter().filter_map(|d| match d.target { DriveTarget::Joint { joint, .. } => Some(joint), _ => None }).collect();
    for j in &r.joints {
        if r.link(j.parent).is_none() || r.link(j.child).is_none() {
            issue(&mut v, Severity::Error, Target::Joint(j.id), format!("{} joins a link that no longer exists", j.name));
        }
        if j.kind != JointKind::Fixed && !driven.contains(&j.id) && j.follows.is_none() {
            issue(&mut v, Severity::Warning, Target::Joint(j.id), format!("{} has no motor: it will swing or slide freely", j.name));
        }
        if j.kind != JointKind::Fixed && j.range.is_none() {
            issue(&mut v, Severity::Info, Target::Joint(j.id), format!("{} has no travel limits", j.name));
        }
        if let Some(f) = &j.follows { if r.joint(f.joint).is_none() { issue(&mut v, Severity::Error, Target::Joint(j.id), format!("{} follows a joint that no longer exists", j.name)); } }
        if (math::len(j.axis) - 1.0).abs() > 1e-3 { issue(&mut v, Severity::Error, Target::Joint(j.id), format!("{}'s axis is not a direction", j.name)); }
    }
    // a link with two parents
    for l in &r.links {
        let n = r.joints.iter().filter(|j| j.child == l.id).count();
        if n > 1 { issue(&mut v, Severity::Error, Target::Link(l.id), format!("{} is the child of {n} joints; a link can only ride on one", l.name)); }
    }
    // drives
    for d in &r.drives {
        if lib.motor(&d.motor).is_none() { issue(&mut v, Severity::Error, Target::Drive(d.id), format!("{}: no motor called '{}' in the library", d.name, d.motor)); }
        if d.ratio <= 0.0 { issue(&mut v, Severity::Error, Target::Drive(d.id), format!("{}: the gear ratio must be above zero", d.name)); }
        match d.target {
            DriveTarget::Joint { joint, spool_radius } => match r.joint(joint) {
                None => issue(&mut v, Severity::Error, Target::Drive(d.id), format!("{} drives a joint that no longer exists", d.name)),
                Some(j) if j.kind == JointKind::Slide && spool_radius <= 0.0 => issue(&mut v, Severity::Error, Target::Drive(d.id), format!("{} drives a slide: give its spool or pulley diameter", d.name)),
                _ => {}
            },
            DriveTarget::Shaft { shaft } => {
                if r.shaft(shaft).is_none() { issue(&mut v, Severity::Error, Target::Drive(d.id), format!("{} drives a shaft that no longer exists", d.name)); }
            }
        }
    }
    // rollers
    for x in &r.rollers {
        match lib.wheel(&x.wheel) {
            None => issue(&mut v, Severity::Warning, Target::Roller(x.id), format!("{}: no wheel '{}' in the library; using a 60A estimate", x.name, x.wheel)),
            Some(w) if !w.measured => issue(&mut v, Severity::Info, Target::Roller(x.id), format!("{}: {} stiffness is estimated from its {}A durometer", x.name, w.name, w.durometer_a)),
            _ => {}
        }
        if !r.drives.iter().any(|d| d.target == DriveTarget::Shaft { shaft: x.shaft }) {
            issue(&mut v, Severity::Warning, Target::Roller(x.id), format!("{} has no motor", x.name));
        }
    }
    // the drivetrain
    if let Some(dt) = &r.drivetrain {
        if lib.motor(&dt.motor).is_none() { issue(&mut v, Severity::Error, Target::Drivetrain, format!("Drivetrain: no motor '{}'", dt.motor)); }
        if lib.wheel(&dt.wheel).is_none() { issue(&mut v, Severity::Error, Target::Drivetrain, format!("Drivetrain: no wheel '{}'", dt.wheel)); }
    } else {
        issue(&mut v, Severity::Info, Target::Drivetrain, "No drivetrain: the robot can't drive (fine on the Bench)".into());
    }
    // weight
    let m = r.weighed_mass.unwrap_or_else(|| r.mass());
    if m > 68.0 { issue(&mut v, Severity::Warning, Target::Robot, format!("The robot weighs {:.1} lb with bumpers and battery; check the season's limit", m / library::LB)); }
    // electrical
    if r.electrical.battery_resistance <= 0.0 { issue(&mut v, Severity::Error, Target::Electrical, "The battery needs an internal resistance above zero".into()); }
    // mechanisms
    for mech in &r.mechanisms {
        if let Some(p) = &mech.pickup {
            if p.drive.is_some_and(|d| r.drive(d).is_none()) { issue(&mut v, Severity::Warning, Target::Mechanism(mech.id), format!("{}: its pickup waits on a drive that no longer exists", mech.name)); }
            if mech.holders.is_empty() { issue(&mut v, Severity::Warning, Target::Mechanism(mech.id), format!("{} picks up but has nowhere to hold a piece", mech.name)); }
        }
    }
    v.sort_by_key(|i| i.severity);
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finds_an_undriven_joint() {
        let mut r = Robot::new("t");
        let a = templates::add(&mut r, templates::Template::arm_default()).unwrap();
        let lib = library::Library::builtin();
        assert!(!issues(&r, &lib).iter().any(|i| i.severity == Severity::Error));
        let d = r.drives[0].id;
        r.drives.retain(|x| x.id != d);
        let found = issues(&r, &lib);
        assert!(found.iter().any(|i| i.message.contains("no motor") && matches!(i.target, Target::Joint(_))), "{found:?}");
        let _ = a;
    }
}
