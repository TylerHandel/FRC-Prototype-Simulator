//! The importer on a small generated robot (tools/cadread/make_test_step.py, read by cadread): the
//! part tree gives the links, the compliant wheels become rollers on one shaft, the robot simulates.
use cad::cadpack::{self, Cadpack, Dir};
use cad::propose;
use library::Library;
use std::collections::HashMap;
use std::path::Path;

fn pack() -> Cadpack { Cadpack::load(&Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/test-robot")).unwrap() }

#[test]
fn proposes_links_and_rollers_and_simulates() {
    let cp = pack();
    let lib = Library::builtin();
    assert_eq!(cp.parts.len(), 6);
    assert!(cp.mesh(cp.parts[0].shape).is_some_and(|m| !m.indices.is_empty()));
    let to = cadpack::to_robot(&cp, Dir::PX, Dir::PZ);
    let placed = propose::place(&cp, &to);
    // the lowest part sits on the floor
    let low = placed.iter().filter(|p| !p.hardware).map(|p| p.bbox[0][2]).fold(f64::INFINITY, f64::min);
    assert!(low.abs() < 1e-6, "{low}");
    let links = propose::links_from_tree(&placed);
    let names: Vec<&str> = links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, vec!["Chassis", "20 - Intake", "30 - Arm"]);
    let p = propose::propose(&cp, &to, &lib, &HashMap::new(), links, vec![], vec![], "part tree");
    assert_eq!(p.rollers.len(), 3, "{:?}", p.notes);
    assert!(p.rollers.iter().all(|r| r.wheel == "am-compliant-4in-35a" && r.shaft == p.rollers[0].shaft));
    assert!(p.rollers.iter().all(|r| (r.diameter - 4.0 * library::IN).abs() < 1e-4 && (r.width - library::IN).abs() < 1e-4));
    // into a robot, and running
    let mut proj = model::Project::new("cad");
    cad::propose::apply(&mut proj.robot, &cp, &to, &p, &lib);
    assert_eq!(proj.robot.links.len(), 3);
    assert_eq!(proj.robot.rollers.len(), 3);
    assert!(proj.robot.drives.iter().any(|d| matches!(d.target, model::DriveTarget::Shaft { .. })));
    let mut w = sim::World::new(&proj, &lib, sim::Scene::carpet()).unwrap();
    for _ in 0..500 { w.step(); }
    assert!(w.engine.finite());
}

#[test]
fn reimport_keeps_markup_when_names_change() {
    let cp = pack();
    let lib = Library::builtin();
    let to = cadpack::to_robot(&cp, Dir::PX, Dir::PZ);
    let placed = propose::place(&cp, &to);
    let p = propose::propose(&cp, &to, &lib, &HashMap::new(), propose::links_from_tree(&placed), vec![], vec![], "part tree");
    let mut r = model::Project::new("cad").robot;
    propose::apply(&mut r, &cp, &to, &p, &lib);
    // the same CAD with the arm's tube renamed: found again by its fingerprint
    let mut parts = r.parts.clone();
    for x in &mut parts { if x.name == "Arm Tube" { x.id = x.id.replace("Arm Tube", "Arm Tube v2"); x.name = "Arm Tube v2".into(); } }
    let arm = r.links.iter().find(|l| l.name == "30 - Arm").unwrap().id;
    let lost = propose::reattach(&mut r, &parts);
    assert!(lost.is_empty(), "{lost:?}");
    assert!(r.link(arm).unwrap().parts.iter().any(|p| p.contains("Arm Tube v2")));
    // a part that's gone is listed
    let fewer: Vec<model::Part> = parts.iter().filter(|x| !x.name.contains("Arm Tube")).cloned().collect();
    let lost = propose::reattach(&mut r, &fewer);
    assert_eq!(lost.len(), 1);
}
