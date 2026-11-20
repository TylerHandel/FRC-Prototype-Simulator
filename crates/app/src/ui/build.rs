//! Build: Concept robots from a palette (PLAN.md "The interface"). Each mechanism is a template
//! with a few numbers; it drives from the first block.
use bevy_egui::egui::{self, RichText, Ui};
use model::templates::Template;
use model::validate::{self, Severity, Target};
use model::MechKind;

use super::widgets::*;
use crate::simrun::Sim;
use crate::state::{AppState, Selection};

const PALETTE: [(MechKind, &str); 9] = [
    (MechKind::Intake, "Over-the-bumper ground intake: rollers and a ramp"),
    (MechKind::Elevator, "Stages on slides, cascade rigged, a spool and motors"),
    (MechKind::Arm, "A pivoting arm on a geared motor"),
    (MechKind::Wrist, "A small pivot, usually on an arm or a carriage"),
    (MechKind::Telescope, "An extending arm on a slide"),
    (MechKind::Claw, "Two rows of rollers that grab and hold a piece"),
    (MechKind::Shooter, "A flywheel and a hood that throws pieces"),
    (MechKind::Turret, "Turns what's on it about the vertical"),
    (MechKind::Climber, "A winch-driven hook arm"),
];

pub fn add_mechanism(st: &mut AppState, kind: MechKind) {
    let Some(mut t) = Template::default_for(kind) else { return };
    // things that ride on others go on the last mechanism that can carry them
    if matches!(kind, MechKind::Wrist | MechKind::Claw | MechKind::Telescope) {
        let carrier = st.project().robot.mechanisms.iter().rev().find(|m| matches!(m.kind, MechKind::Elevator | MechKind::Arm | MechKind::Wrist | MechKind::Telescope)).map(|m| m.id);
        if carrier.is_some() { t.mount.on = carrier; t.mount.offset = [0.0; 3]; } else { t.mount.offset = [0.1, 0.0, 0.4]; }
    }
    let id = st.doc.edit(&format!("Add {}", kind.label()), |p| model::templates::add(&mut p.robot, t).ok());
    if let Some(id) = id { st.sel = Selection::Mechanism(id); }
}

pub fn left(ui: &mut Ui, st: &mut AppState, _sim: &Sim) {
    heading(ui, "Add a mechanism");
    hint(ui, "Click to add. Its numbers are on the right; change them while it runs.");
    egui::Grid::new("palette").num_columns(2).spacing([6.0, 4.0]).show(ui, |ui| {
        for (i, (k, about)) in PALETTE.iter().enumerate() {
            if ui.add_sized([150.0, 26.0], egui::Button::new(k.label())).on_hover_text(*about).clicked() { add_mechanism(st, *k); }
            if i % 2 == 1 { ui.end_row(); }
        }
    });
    ui.add_space(6.0);
    heading(ui, "Robot");
    tree(ui, st);
}

/// The robot as a list: chassis, drivetrain, electrical, controls, then each mechanism with what it carries.
pub fn tree(ui: &mut Ui, st: &mut AppState) {
    let r = st.project().robot.clone();
    let pick = |ui: &mut Ui, sel: Selection, label: String, st: &mut AppState| {
        if ui.selectable_label(st.sel == sel, label).clicked() { st.sel = sel; }
    };
    pick(ui, Selection::Robot, format!("{} (frame and weight)", r.name), st);
    pick(ui, Selection::Drivetrain, format!("Drivetrain: {}", r.drivetrain.as_ref().map_or("none".into(), |d| st.lib.module(&d.module).map_or(d.module.clone(), |m| m.name.clone()))), st);
    pick(ui, Selection::Electrical, "Battery and wiring".into(), st);
    pick(ui, Selection::Controls, format!("Controls ({} presets)", r.controls.presets.len()), st);
    // mechanisms nested by what they ride on
    fn rec(ui: &mut Ui, st: &mut AppState, r: &model::Robot, on: Option<model::Id>, depth: usize) {
        for m in r.mechanisms.iter().filter(|m| m.template.as_ref().map_or(None, |t| t.mount.on) == on) {
            ui.horizontal(|ui| {
                ui.add_space(12.0 * depth as f32);
                let tag = match m.fidelity { model::Fidelity::Concept => "concept", model::Fidelity::Full => "full physics" };
                let sel = Selection::Mechanism(m.id);
                if ui.selectable_label(st.sel == sel, format!("{}  ", m.name)).clicked() { st.sel = sel; }
                ui.label(RichText::new(tag).color(MUTED).size(11.5));
            });
            rec(ui, st, r, Some(m.id), depth + 1);
        }
    }
    rec(ui, st, &r, None, 0);
    // custom mechanisms (from CAD) and loose links
    for m in r.mechanisms.iter().filter(|m| m.template.is_none()) {
        let sel = Selection::Mechanism(m.id);
        if ui.selectable_label(st.sel == sel, format!("{} (from CAD)", m.name)).clicked() { st.sel = sel; }
    }
    let mass = r.weighed_mass.unwrap_or_else(|| r.mass());
    ui.add_space(4.0);
    hint(ui, &format!("Weight {} (with bumpers and battery)", library::units::format(mass, st.unit(library::units::Kind::Mass))));
}

/// What's missing (rule 6), and live numbers.
pub fn bottom(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    ui.columns(2, |cols| {
        let ui = &mut cols[0];
        heading(ui, "What's missing");
        let issues = validate::issues(&st.project().robot, &st.lib);
        egui::ScrollArea::vertical().id_salt("issues").max_height(140.0).show(ui, |ui| {
            if issues.iter().all(|i| i.severity == Severity::Info) { ui.label(RichText::new("Nothing stops this robot from running.").color(GOOD)); }
            for i in issues.iter().filter(|i| i.severity != Severity::Info).chain(issues.iter().filter(|i| i.severity == Severity::Info).take(6)) {
                let c = match i.severity { Severity::Error => BAD, Severity::Warning => WARN, Severity::Info => MUTED };
                if ui.add(egui::Label::new(RichText::new(&i.message).color(c)).sense(egui::Sense::click())).on_hover_text("Click to go to it").clicked() {
                    st.sel = match i.target {
                        Target::Link(l) => Selection::Link(l), Target::Joint(j) => Selection::Joint(j), Target::Drive(d) => Selection::Drive(d),
                        Target::Roller(r) => Selection::Roller(r), Target::Mechanism(m) => Selection::Mechanism(m), Target::Electrical => Selection::Electrical,
                        Target::Drivetrain => Selection::Drivetrain, Target::Robot => Selection::Robot,
                    };
                }
            }
        });
        let ui = &mut cols[1];
        heading(ui, "Battery, last 10 s");
        super::analyze::mini_plot(ui, st, sim, 130.0);
    });
}
