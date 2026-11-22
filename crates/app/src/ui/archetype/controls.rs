use bevy_egui::egui::{self, Ui};
use model::{Action, Binding, Button, DriveTarget, Id, JointKind, Preset, PresetTarget, Robot};

use super::{AppState, Selection, heading, hint};

pub(super) fn selected(st: &AppState) -> Option<&Preset> {
    let presets = &st.project().robot.controls.presets;
    if st.assembly.operator_action.is_some() {
        return None;
    }
    st.assembly.preset.and_then(|id| presets.iter().find(|p| p.id == id)).or_else(|| presets.first())
}

fn action_name(r: &Robot, action: &Action) -> String {
    match action {
        Action::Preset { preset } => r.controls.presets.iter().find(|p| p.id == *preset).map(|p| p.name.clone()),
        Action::Run { drive, value, .. } => r.drive(*drive).map(|d| {
            let name = match d.target {
                DriveTarget::Shaft { shaft } => {
                    r.shaft(shaft).and_then(|s| r.mechanism_of_link(s.link)).map(|m| m.name.as_str())
                }
                _ => None,
            }
            .unwrap_or(&d.name);
            format!("{name} {}", if *value < 0.0 { "reverse" } else { "forward" })
        }),
        Action::Release { mechanism } => r.mechanism(*mechanism).map(|m| format!("Release from {}", m.name)),
    }
    .unwrap_or_else(|| "Unknown action".into())
}

fn stock_actions(r: &Robot) -> impl Iterator<Item = Action> + '_ {
    r.drives.iter().filter_map(|d| {
        let DriveTarget::Shaft { shaft } = d.target else { return None };
        let mechanism = r.mechanism_of_link(r.shaft(shaft)?.link)?;
        Some(model::templates::roller_bindings(mechanism.kind, d.id))
    }).flatten().map(|b| b.action)
}

// Stock templates can compete for the same button. Keep unassigned roller actions discoverable.
fn mechanism_actions(r: &Robot) -> Vec<Action> {
    let mut actions: Vec<_> = r
        .controls
        .bindings
        .iter()
        .filter(|b| !matches!(b.action, Action::Preset { .. }))
        .map(|b| b.action.clone())
        .collect();
    for action in stock_actions(r) {
        let Action::Run { drive, value, .. } = action else { continue };
        if !actions.iter().any(|a| matches!(a, Action::Run { drive: d, value: v, .. } if *d == drive && v.signum() == value.signum())) {
            actions.push(action);
        }
    }
    actions
}

fn position_drives(r: &Robot) -> impl Iterator<Item = (Id, &model::Joint, &model::Mechanism)> {
    r.drives.iter().filter_map(|d| {
        let DriveTarget::Joint { joint, .. } = d.target else { return None };
        let j = r.joint(joint)?;
        j.range?;
        Some((d.id, j, r.mechanism_of_link(j.child)?))
    })
}

pub(super) fn travel_factor(r: &Robot, joint: &model::Joint) -> f64 {
    if joint.kind != JointKind::Slide {
        return 1.0;
    }
    1.0 + r
        .joints
        .iter()
        .filter_map(|j| j.follows.as_ref())
        .filter(|f| f.joint == joint.id)
        .map(|f| f.ratio)
        .sum::<f64>()
}

pub(super) fn pose_value(r: &Robot, preset: Option<&Preset>, drive: Id, joint: &model::Joint) -> f64 {
    let target = |p: &Preset| p.targets.iter().find(|t| t.drive == drive).map(|t| t.value);
    let value = preset.and_then(target).or_else(|| r.starting_preset().and_then(target)).unwrap_or(joint.initial);
    joint.range.map_or(value, |[a, b]| value.clamp(a.min(b), a.max(b)))
}

pub(super) fn set_value(st: &mut AppState, preset: Id, drive: Id, v: f64) {
    st.doc.edit(&format!("~Teach position {preset}/{drive}"), |p| {
        let Some(pose) = p.robot.controls.presets.iter_mut().find(|p| p.id == preset) else { return };
        if let Some(t) = pose.targets.iter_mut().find(|t| t.drive == drive) {
            t.value = v;
        } else {
            pose.targets.push(PresetTarget { drive, value: v });
        }
    });
}

fn new_position(st: &mut AppState) {
    let r = &st.project().robot;
    let visible = selected(st);
    let targets = position_drives(r)
        .map(|(drive, j, _)| PresetTarget { drive, value: pose_value(r, visible, drive, j) })
        .collect();
    let mut n = 1;
    while r.controls.presets.iter().any(|p| p.name == format!("Position {n}")) {
        n += 1;
    }
    let id = st.doc.edit("New position", |p| {
        let id = p.robot.id();
        p.robot.controls.presets.push(Preset { reef_gap: model::default_reef_gap(), id, name: format!("Position {n}"), targets });
        id
    });
    st.assembly.operator_action = None;
    st.assembly.preset = Some(id);
}

pub(super) fn list(ui: &mut Ui, st: &mut AppState) {
    if has_reef(st) {
        heading(ui, "Reef auto-align");
        ui.label("LB  ·  Left branch       RB  ·  Right branch");
        hint(ui, "Hold a bumper to align. Select a saved position to set its bumper clearance.");
        ui.separator();
    }
    heading(ui, "Saved positions");
    let current = selected(st).map(|p| p.id);
    let mut clicked = None;
    for p in &st.project().robot.controls.presets {
        let label = if st.project().robot.controls.start_position == Some(p.id) {
            format!("{} (start)", p.name)
        } else {
            p.name.clone()
        };
        if ui.selectable_label(current == Some(p.id), label).clicked() {
            clicked = Some(p.id);
        }
    }
    if let Some(id) = clicked {
        st.assembly.preset = Some(id);
        st.assembly.operator_action = None;
        if let Some((owner, turret)) = selected(st).and_then(|p| {
            position_drives(&st.project().robot)
                .find(|(drive, _, _)| p.targets.iter().any(|t| t.drive == *drive))
                .map(|(_, _, m)| (m.id, m.kind == model::MechKind::Turret))
        }) {
            st.sel = Selection::Mechanism(owner);
            st.assembly.view = if turret { super::View::Top } else { super::View::Side };
        }
    }
    if position_drives(&st.project().robot).next().is_some() {
        if ui.button("+ New position").on_hover_text("Start with the position shown in the drawing").clicked() {
            new_position(st);
        }
    } else {
        hint(ui, "Add a moving mechanism in Layout to teach a position.");
    }
    let actions = mechanism_actions(&st.project().robot);
    if !actions.is_empty() {
        ui.separator();
        heading(ui, "Mechanism actions");
        for action in actions {
            if ui
                .selectable_label(
                    st.assembly.operator_action.as_ref() == Some(&action),
                    action_name(&st.project().robot, &action),
                )
                .clicked()
            {
                st.assembly.operator_action = Some(action);
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Assignment { Move, Share, Swap, Conflict }

// A press can command any number of mechanisms, but a drive can only follow one target.
fn conflicts(r: &Robot, a: &Action, b: &Action) -> bool {
    let pose = |id| r.controls.presets.iter().find(|p| p.id == id);
    match (a, b) {
        (Action::Run { drive: a, .. }, Action::Run { drive: b, .. }) => a == b,
        (Action::Preset { preset: a }, Action::Preset { preset: b }) => pose(*a).zip(pose(*b)).is_some_and(|(a, b)| {
            a.targets.iter().any(|x| b.targets.iter().any(|y| x.drive == y.drive && x.value != y.value))
        }),
        (Action::Run { drive, .. }, Action::Preset { preset }) | (Action::Preset { preset }, Action::Run { drive, .. }) =>
            pose(*preset).is_some_and(|p| p.targets.iter().any(|t| t.drive == *drive)),
        _ => false,
    }
}

fn button_assignment(r: &Robot, action: &Action, old: Option<Button>, new: Button) -> Assignment {
    let c = &r.controls;
    let count = |button| c.bindings.iter().filter(|b| b.button == button).count();
    let single_source = old.is_some_and(|b| count(b) == 1);
    if old == Some(new) || single_source && count(new) == 0 { return Assignment::Move; }
    // A button's keyboard shortcut can also be shared with other gamepad buttons.
    let collides = c.bindings.iter().any(|b| &b.action != action && c.key(b.button) == c.key(new) && conflicts(r, action, &b.action));
    if collides {
        if single_source && count(new) == 1 { Assignment::Swap } else { Assignment::Conflict }
    } else if count(new) > 0 { Assignment::Share } else { Assignment::Move }
}

fn key_assignment(r: &Robot, button: Button, key: &str) -> Assignment {
    let c = &r.controls;
    if key == c.key(button) { return Assignment::Move; }
    let others: Vec<_> = c.bindings.iter().filter(|b| b.button != button && c.key(b.button) == key).collect();
    let Some(first) = others.first() else { return Assignment::Move };
    let collides = c.bindings.iter().filter(|b| b.button == button)
        .any(|own| others.iter().any(|other| conflicts(r, &own.action, &other.action)));
    if !collides { return Assignment::Share; }
    let old_shared = c.bindings.iter().any(|b| b.button != button && c.key(b.button) == c.key(button));
    if !old_shared && others.iter().all(|b| b.button == first.button) { Assignment::Swap } else { Assignment::Conflict }
}

fn option_label(control: &str, assignment: Assignment, members: &[String]) -> String {
    let names = members.join(", ");
    match assignment {
        Assignment::Move if names.is_empty() => control.to_owned(),
        Assignment::Move => format!("{control}  ·  {names}"),
        Assignment::Share => format!("{control}  ·  share with {names}"),
        Assignment::Swap => format!("{control}  ·  swap with {names}"),
        Assignment::Conflict => format!("{control}  ·  conflicting drive commands"),
    }
}

fn binding(ui: &mut Ui, st: &mut AppState, action: &Action) {
    let r = &st.project().robot;
    let c = &r.controls;
    let assigned = c.bindings.iter().find(|b| &b.action == action).map(|b| b.button);
    let mut choice = assigned;
    ui.label("Gamepad button");
    egui::ComboBox::from_id_salt("operator_button")
        .selected_text(assigned.map_or("Unassigned".into(), |b| pad_label(st, b)))
        .show_ui(ui, |ui| {
            if matches!(action, Action::Preset { .. }) || stock_actions(r).any(|a| &a == action) {
                ui.selectable_value(&mut choice, None, "Unassigned");
            }
            for physical in Button::ALL {
                let b = if has_reef(st) {
                    if matches!(physical, Button::RB | Button::LB) { continue; }
                    c.bindings.iter().find(|x| c.reef_pad(x.button) == Some(physical)).map_or(physical, |x| x.button)
                } else { physical };
                let assignment = button_assignment(r, action, assigned, b);
                let members = c.bindings.iter().filter(|x| x.button == b && &x.action != action)
                    .map(|x| action_name(r, &x.action)).collect::<Vec<_>>();
                ui.add_enabled_ui(assignment != Assignment::Conflict, |ui| {
                    ui.selectable_value(&mut choice, Some(b), option_label(&format!("{physical:?}"), assignment, &members))
                });
            }
        });
    if choice != assigned {
        assign_button(st, action, assigned, choice);
    }
    let Some(button) = choice else {
        hint(ui, "Choose a button to enable its keyboard shortcut.");
        return;
    };
    let r = &st.project().robot;
    let c = &r.controls;
    let others: Vec<_> = c.bindings.iter().filter(|b| b.button == button && &b.action != action)
        .map(|b| action_name(r, &b.action)).collect();
    if !others.is_empty() {
        hint(ui, &format!("Also on {} and its shortcut · {}", pad_label(st, button), others.join(", ")));
    }
    let old = c.key(button).to_owned();
    let mut key = old.as_str();
    ui.label("Keyboard shortcut");
    egui::ComboBox::from_id_salt("operator_key").selected_text(&old).show_ui(ui, |ui| {
        for &(name, _) in crate::input::OPERATOR_KEYS {
            let assignment = key_assignment(r, button, name);
            let members = c.bindings.iter().filter(|b| b.button != button && c.key(b.button) == name)
                .map(|b| action_name(r, &b.action)).collect::<Vec<_>>();
            ui.add_enabled_ui(assignment != Assignment::Conflict, |ui| {
                ui.selectable_value(&mut key, name, option_label(name, assignment, &members))
            });
        }
    });
    if old != key {
        assign_key(st, button, key);
    } else {
        let peers: Vec<_> = c.bindings.iter().filter(|b| b.button != button && c.key(b.button) == key)
            .map(|b| action_name(r, &b.action)).collect();
        if !peers.is_empty() { hint(ui, &format!("Also on {key} · {}", peers.join(", "))); }
    }
}

fn has_reef(st: &AppState) -> bool {
    st.game.as_ref().and_then(|(_, g)| g.as_ref().ok()).is_some_and(|g| !g.pack.reef_faces.is_empty())
}

fn pad_label(st: &AppState, b: Button) -> String {
    if has_reef(st) && !(st.assembly.driving() && st.assembly.open_carpet) {
        return st.project().robot.controls.reef_pad(b).map_or_else(|| "Keyboard only".into(), |p| format!("{p:?}"));
    }
    format!("{b:?}")
}

fn assign_button(st: &mut AppState, action: &Action, old: Option<Button>, new: Option<Button>) {
    let assignment = new.map(|b| button_assignment(&st.project().robot, action, old, b));
    if old == new || assignment == Some(Assignment::Conflict) { return; }
    st.doc.edit("Assign control", |p| {
        let c = &mut p.robot.controls;
        match (old, new) {
            (Some(a), Some(b)) => {
                let swap = assignment == Some(Assignment::Swap);
                // A standalone control keeps its keyboard equivalent when moved to an empty slot.
                if swap || c.bindings.iter().all(|x| x.button != b)
                    && c.bindings.iter().filter(|x| x.button == a).count() == 1 {
                    let (key_a, key_b) = (c.key(a).to_owned(), c.key(b).to_owned());
                    c.keys.insert(a, key_b);
                    c.keys.insert(b, key_a);
                }
                for binding in &mut c.bindings {
                    if binding.button == a && &binding.action == action {
                        binding.button = b;
                    } else if swap && binding.button == b {
                        binding.button = a;
                    }
                }
            }
            (None, Some(b)) => c.bindings.push(Binding { button: b, action: action.clone() }),
            (Some(a), None) => c.bindings.retain(|b| b.button != a || &b.action != action),
            _ => {}
        }
    });
}

fn assign_key(st: &mut AppState, button: Button, key: &str) {
    let assignment = key_assignment(&st.project().robot, button, key);
    if assignment == Assignment::Conflict { return; }
    st.doc.edit("Set keyboard shortcut", |p| {
        let c = &mut p.robot.controls;
        if assignment != Assignment::Share {
            let old = c.key(button).to_owned();
            for other in Button::ALL {
                if other != button && c.key(other) == key {
                    c.keys.insert(other, old.clone());
                }
            }
        }
        c.keys.insert(button, key.to_owned());
    });
}

pub(super) fn inspector(ui: &mut Ui, st: &mut AppState) {
    if let Some(action) = st.assembly.operator_action.clone() {
        if mechanism_actions(&st.project().robot).contains(&action) {
            heading(ui, &action_name(&st.project().robot, &action));
            hint(
                ui,
                if matches!(action, Action::Run { .. }) {
                    "Hold the assigned control to run."
                } else {
                    "Press the assigned control to release the piece."
                },
            );
            binding(ui, st, &action);
            if let Action::Run { drive, .. } = action {
                roller_hold_editor(ui, st, drive);
            }
            return;
        }
        st.assembly.operator_action = None;
    }
    let Some(pose) = selected(st).cloned() else {
        heading(ui, "Controls");
        hint(ui, "Choose a position or a mechanism action on the left.");
        return;
    };
    ui.push_id(pose.id, |ui| position_editor(ui, st, &pose));
}

fn roller_hold_editor(ui: &mut Ui, st: &mut AppState, drive: Id) {
    let robot = &st.project().robot;
    let Some(d) = robot.drive(drive) else { return };
    if !matches!(d.target, DriveTarget::Shaft { .. }) { return; }
    let old = robot.controls.roller_hold_amps.get(&drive).copied().unwrap_or(0.0);
    let mut amps = old.abs();
    let mut reverse = old < 0.0;
    heading(ui, "Roller hold");
    hint(ui, "Applies when neither roller action is active. Set 0 A to turn off.");
    ui.horizontal(|ui| {
        ui.label("Current per motor");
        ui.add(egui::DragValue::new(&mut amps).speed(0.25).range(0.0..=d.stator_limit).suffix(" A"));
    });
    ui.add_enabled_ui(amps > 0.0, |ui| { ui.horizontal(|ui| {
        ui.selectable_value(&mut reverse, false, "Forward");
        ui.selectable_value(&mut reverse, true, "Reverse");
    }); });
    let value = if reverse { -amps } else { amps };
    if value != old {
        st.doc.edit(&format!("~Roller hold {drive}"), |p| {
            let currents = &mut p.robot.controls.roller_hold_amps;
            if value == 0.0 { currents.remove(&drive); } else { currents.insert(drive, value); }
        });
    }
}

fn position_editor(ui: &mut Ui, st: &mut AppState, pose: &Preset) {
    heading(ui, "Position name");
    let mut name = pose.name.clone();
    let edit = ui.text_edit_singleline(&mut name);
    if edit.changed() {
        st.doc.edit(&format!("~Rename position {}", pose.id), |p| {
            p.robot.controls.presets.iter_mut().find(|p| p.id == pose.id).unwrap().name = name;
        });
    }
    if edit.lost_focus() {
        if st
            .project()
            .robot
            .controls
            .presets
            .iter()
            .find(|p| p.id == pose.id)
            .is_some_and(|p| p.name.trim().is_empty())
        {
            st.doc.edit(&format!("~Rename position {}", pose.id), |p| {
                p.robot.controls.presets.iter_mut().find(|p| p.id == pose.id).unwrap().name =
                    format!("Position {}", pose.id);
            });
        }
        st.doc.seal();
    }
    let mut starting = st.project().robot.controls.start_position == Some(pose.id);
    if ui
        .checkbox(&mut starting, "Use as starting position")
        .on_hover_text("Use this pose for starting size checks and each fresh Test drive.")
        .changed()
    {
        st.doc.edit("Set starting position", |p| p.robot.controls.start_position = starting.then_some(pose.id));
    }
    heading(ui, "When pressed");
    let rows: Vec<_> = position_drives(&st.project().robot)
        .map(|(drive, j, m)| {
            let factor = travel_factor(&st.project().robot, j);
            (
                drive,
                m.id,
                m.name.clone(),
                j.kind,
                j.range.unwrap(),
                pose_value(&st.project().robot, Some(pose), drive, j),
                factor,
            )
        })
        .collect();
    for (drive, mechanism, name, kind, [lo, hi], v, factor) in rows {
        ui.push_id((pose.id, drive), |ui| {
            let mut included = pose.targets.iter().any(|t| t.drive == drive);
            if ui.checkbox(&mut included, &name).on_hover_text("Include this mechanism in the position").changed()
            {
                if included {
                    set_value(st, pose.id, drive, v);
                } else {
                    st.doc.edit("Leave mechanism in place", |p| {
                        p.robot
                            .controls
                            .presets
                            .iter_mut()
                            .find(|p| p.id == pose.id)
                            .unwrap()
                            .targets
                            .retain(|t| t.drive != drive)
                    });
                }
            }
            if included {
                let unit = st.unit(if kind == JointKind::Slide {
                    library::units::Kind::Length
                } else {
                    library::units::Kind::Angle
                });
                let f = factor / unit.scale;
                let mut display = v * f;
                let resp =
                    ui.add(egui::Slider::new(&mut display, lo * f..=hi * f).suffix(format!(" {}", unit.label)));
                if resp.drag_started() {
                    st.doc.seal();
                }
                if resp.changed() {
                    st.sel = Selection::Mechanism(mechanism);
                    set_value(st, pose.id, drive, display / f);
                }
                if resp.drag_stopped() {
                    st.doc.seal();
                }
            }
        });
    }
    if has_reef(st) {
        heading(ui, "Reef auto-align");
        let mut gap = pose.reef_gap;
        if crate::ui::widgets::num_row(ui, "Bumper clearance", "Distance from the front bumper to the reef face at this position.",
            &mut gap, library::units::Kind::Length, st.unit(library::units::Kind::Length)) {
            st.doc.edit(&format!("~Reef distance {}", pose.id), |p| {
                if let Some(preset) = p.robot.controls.presets.iter_mut().find(|p| p.id == pose.id) {
                    preset.reef_gap = gap.max(0.);
                }
            });
        }
    }
    heading(ui, "Activate position");
    binding(ui, st, &Action::Preset { preset: pose.id });
    ui.separator();
    if ui.small_button("Delete position").clicked() {
        st.doc.edit("Delete position", |p| {
            p.robot.controls.presets.retain(|p| p.id != pose.id);
            if p.robot.controls.start_position == Some(pose.id) {
                p.robot.controls.start_position = None;
            }
            p.robot.controls.bindings.retain(|b| b.action != (Action::Preset { preset: pose.id }));
        });
        st.assembly.preset = None;
    }
}

pub(super) fn drive_card(ui: &mut Ui, st: &mut AppState, sim: &crate::simrun::Sim) {
    heading(ui, "Practice area");
    let previous = st.assembly.open_carpet;
    let available = st.game(false).is_ok();
    ui.horizontal_wrapped(|ui| {
        ui.add_enabled_ui(available, |ui| ui.selectable_value(&mut st.assembly.open_carpet, false, "Season field"));
        ui.selectable_value(&mut st.assembly.open_carpet, true, "Open carpet");
    });
    if !available {
        st.assembly.open_carpet = true;
        hint(ui, "The selected game pack could not be loaded.");
    }
    if previous != st.assembly.open_carpet { st.sim_rev = u64::MAX; }
    if !st.assembly.open_carpet { ui.checkbox(&mut st.assembly.overview, "Show whole field"); }
    ui.separator();
    heading(ui, "Game piece");
    if st.lib.pieces.is_empty() {
        hint(ui, "The piece library is empty.");
    } else {
        let (ready, held, touching, scored, score) = sim
            .shared
            .lock()
            .map(|s| {
                let ready = s.test_drive_ready();
                let held = s.snap.as_ref().is_some_and(|snap| snap.pieces.iter().any(|p| p.held));
                let snap = s.snap.as_ref();
                (ready, held, snap.is_some_and(|s| s.contacts.iter().any(|c| c.roller != 0 && c.force > 0.1)),
                    snap.is_some_and(|s| s.pieces.iter().any(|p| p.scored)), snap.map_or(0., |s| s.score))
            })
            .unwrap_or_default();
        ui.add_enabled_ui(ready, |ui| {
            let mut changed = super::piece_pick(ui, "practice_piece", &st.lib, &mut st.assembly.fit_piece);
            if st.lib.piece(&st.assembly.fit_piece).is_some_and(|p| matches!(p.shape, library::PieceShape::Tube { .. })) {
                use sim::world::TubeOrientation::*;
                ui.label("Orientation at placement");
                ui.horizontal_wrapped(|ui| {
                    for (orientation, label) in [(Crosswise,"Crosswise"), (EndOn,"End-on"), (Upright,"Upright")] {
                        changed |= ui.selectable_value(&mut st.assembly.tube, orientation, label).changed();
                    }
                });
            }
            let place = ui
                .button("Place in front of robot")
                .on_hover_text("Bring this piece back to the carpet ahead of the robot.")
                .clicked();
            let mut claw = None;
            let robot = &st.project().robot;
            let claws = robot.mechanisms.iter().filter(|m| m.kind == model::MechKind::Claw);
            let single = claws.clone().count() == 1;
            for m in claws {
                let button = if single { "Feed into claw".into() } else { format!("Feed into {}", m.name) };
                if ui.button(button).on_hover_text("Present a free piece to the rollers at their current height and angle.").clicked() {
                    claw = Some(m.id);
                }
                if let Some(binding) = m.pickup.as_ref().and_then(|p| p.drive).and_then(|d| robot.controls.bindings.iter().find(
                    |b| matches!(b.action, Action::Run { drive, value, .. } if drive == d && value > 0.))) {
                    hint(ui, &format!("Hold {} or key {} while feeding.", pad_label(st, binding.button), robot.controls.key(binding.button)));
                }
            }
            if changed || place || claw.is_some() {
                sim.send(crate::simrun::Msg::PlacePracticePiece { id: st.assembly.fit_piece.clone(), tube: st.assembly.tube, claw });
            }
        });
        if ready {
            let r = &st.project().robot;
            if scored {
                ui.label(format!("Scored  ·  {score:.0} points total"));
            } else if touching {
                ui.label("Rollers in contact");
            } else if held {
                ui.label("Robot is holding the piece");
            } else if let Some((m, b)) = r
                .mechanisms
                .iter()
                .filter_map(|m| {
                    let d = m.pickup.as_ref()?.drive?;
                    let binding = r.controls.bindings.iter().find(
                        |b| matches!(b.action, Action::Run { drive, value, .. } if drive == d && value > 0.0),
                    )?;
                    Some((m, binding))
                })
                .min_by_key(|(m, _)| m.kind != model::MechKind::Intake)
            {
                hint(
                    ui,
                    &format!(
                        "Hold key {} or gamepad {} to run {}.",
                        r.controls.key(b.button),
                        pad_label(st, b.button),
                        m.name
                    ),
                );
            }
        }
    }
    ui.separator();
    heading(ui, "Field-centric drive");
    if has_reef(st) && !st.assembly.open_carpet {
        ui.label("Hold LB / [  ·  Align left");
        ui.label("Hold RB / ]  ·  Align right");
        let state = sim.shared.lock().ok().and_then(|s| s.snap.as_ref().map(|s| (s.active_position,s.reef_alignment)));
        if let Some((position, alignment)) = state {
            let c = &st.project().robot.controls;
            let name = c.presets.iter().find(|p| Some(p.id) == position).map_or("Default", |p| p.name.as_str());
            let unit = st.unit(library::units::Kind::Length);
            ui.label(format!("{name} · {} bumper clearance", library::units::format(c.reef_gap(position),unit)));
            if let Some(a) = alignment {
                let side = if a.right { "right" } else { "left" };
                if a.distance < 0.025 && a.heading_error.abs() < 3_f64.to_radians() {
                    ui.label(format!("Aligned with {side} branch"));
                } else {
                    ui.label(format!("Aligning {side} · {} to target",library::units::format(a.distance,unit)));
                }
            }
        }
    }
    if let Some(p) = st.project().robot.starting_preset() {
        ui.label(format!("Starting position · {}", p.name));
    }
    ui.label("W A S D / left stick    Move");
    ui.label("Left and right arrows / right stick    Turn");
    hint(ui, "Forward always drives away from the blue driver station.");
    let r = &st.project().robot;
    ui.separator();
    heading(ui, "Operator controls");
    for button in Button::ALL {
        let mut bindings = r.controls.bindings.iter().filter(|b| b.button == button).peekable();
        if bindings.peek().is_none() { continue; }
        ui.label(format!("{}  /  Key {}", pad_label(st, button), r.controls.key(button)));
        for b in bindings {
            let verb = if matches!(b.action, Action::Run { .. }) { "Hold" } else { "Press" };
            hint(ui, &format!("{verb} · {}", action_name(r, &b.action)));
        }
    }
    if r.controls.bindings.is_empty() {
        hint(ui, "Assign an action in Controls to try it here.");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reef_driver_bumpers_leave_both_operator_actions_reachable() {
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, model::MechKind::Claw);
        super::super::add_suggested(&mut st, model::MechKind::Intake);
        let c = &st.project().robot.controls;
        let used: std::collections::BTreeSet<_> = c.bindings.iter().map(|b| b.button).collect();
        let actual: std::collections::BTreeSet<_> = used.iter().map(|b| c.reef_pad(*b).unwrap()).collect();
        assert_eq!(used.len(), actual.len());
        assert!(!actual.contains(&Button::RB) && !actual.contains(&Button::LB));
        assert_eq!((c.key(Button::LB),c.key(Button::RB)), ("Q","E"));
    }

    #[test]
    fn shared_position_and_roller_survive_reopen_and_release_independently() {
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, model::MechKind::Elevator);
        super::super::add_suggested(&mut st, model::MechKind::Claw);
        let r = &st.project().robot;
        let action_on = |button| r.controls.bindings.iter().find(|b| b.button == button).unwrap().action.clone();
        let position = action_on(Button::B);
        let roller = action_on(Button::RB);
        let Action::Preset { preset } = position else { panic!() };
        let Action::Run { drive, value, .. } = roller else { panic!() };
        let target = r.controls.presets.iter().find(|p| p.id == preset).unwrap().targets[0].clone();
        let original = st.project().clone();

        assert!(button_assignment(r, &roller, Some(Button::RB), Button::B) == Assignment::Share);
        assign_button(&mut st, &roller, Some(Button::RB), Some(Button::B));
        let saved = model::Project::from_json(&st.project().to_json()).unwrap();
        let c = &saved.robot.controls;
        assert_eq!(c.key(Button::B), original.robot.controls.key(Button::B));
        assert_eq!(c.bindings.iter().filter(|b| b.button == Button::B).count(), 2);
        assert!(c.bindings.iter().all(|b| b.button != Button::RB));
        let mut world = sim::world::World::test_drive(&saved, &st.lib, "coral-2025").unwrap();
        world.button(Button::B, true);
        assert_eq!(world.cmd.targets[&target.drive], target.value);
        assert_eq!(world.cmd.run[&drive], value);
        for _ in 0..125 { world.step(); }
        assert!(world.snapshot().drives.iter().any(|d| d.id == target.drive && d.position > 0.01));
        world.button(Button::B, false);
        assert!(!world.cmd.run.contains_key(&drive));
        assert_eq!(world.cmd.targets[&target.drive], target.value);

        assign_button(&mut st, &roller, Some(Button::B), None);
        assert!(mechanism_actions(&st.project().robot).contains(&roller));
        assert!(st.project().robot.controls.bindings.iter().any(|b| b.button == Button::B && b.action == position));
        assign_button(&mut st, &roller, None, Some(Button::B));
        assign_button(&mut st, &position, Some(Button::B), Some(Button::RB));
        assert!(st.project().robot.controls.bindings.iter().any(|b| b.button == Button::B && b.action == roller));
        st.doc.undo();
        st.doc.undo();
        st.doc.undo();
        assert_eq!(st.project(), &saved);
    }

    #[test]
    fn a_keyboard_shortcut_can_be_shared_without_moving_the_gamepad_buttons() {
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, model::MechKind::Elevator);
        super::super::add_suggested(&mut st, model::MechKind::Claw);
        let original = st.project().clone();
        let r = &original.robot;
        let key = r.controls.key(Button::A);
        assert!(key_assignment(r, Button::LT, key) == Assignment::Share);
        assign_key(&mut st, Button::LT, key);
        let c = &st.project().robot.controls;
        assert_eq!(c.bindings, r.controls.bindings);
        assert_eq!(c.key(Button::LT), key);
        assert_eq!(c.key(Button::A), key);
        assign_key(&mut st, Button::LT, "G");
        assert_eq!(st.project().robot.controls.key(Button::A), key);
        assert_eq!(st.project().robot.controls.key(Button::LT), "G");
        st.doc.undo();
        st.doc.undo();
        assert_eq!(st.project(), &original);
    }

    #[test]
    fn swapped_and_unassigned_controls_survive_new_parts_and_reopen() {
        let mut st = AppState::new(Default::default());
        super::super::add_suggested(&mut st, model::MechKind::Elevator);
        let first = st.project().robot.controls.presets[0].id;
        let a = Action::Preset { preset: first };
        let b =
            st.project().robot.controls.bindings.iter().find(|b| b.button == Button::B).unwrap().action.clone();
        assign_button(&mut st, &a, Some(Button::A), Some(Button::B));
        assert_eq!(st.project().robot.controls.key(Button::B), "J");
        assign_key(&mut st, Button::B, "K");
        let saved = model::Project::from_json(&st.project().to_json()).unwrap();
        assert_eq!(saved.robot.controls.key(Button::B), "K");
        assert_eq!(saved.robot.controls.key(Button::A), "J");
        let mut world = sim::world::World::new(&saved, &st.lib, sim::world::Scene::carpet()).unwrap();
        world.button(Button::B, true);
        assert_eq!(
            world.cmd.targets[&saved.robot.controls.presets[0].targets[0].drive],
            saved.robot.controls.presets[0].targets[0].value
        );
        assert_eq!(saved.robot.controls.bindings.iter().find(|x| x.button == Button::A).unwrap().action, b);
        assign_button(&mut st, &a, Some(Button::B), None);
        super::super::add_suggested(&mut st, model::MechKind::Arm);
        assert!(st.project().robot.controls.bindings.iter().all(|x| x.action != a));
        st.doc.undo();
        st.doc.undo();
        assert_eq!(st.project(), &saved);
    }
}
