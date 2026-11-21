//! The interface (PLAN.md "The interface"): five workspaces across the top, the 3D view always in
//! the middle, a panel on each side and one below. Rules it keeps: click the thing; nothing
//! required is blank; edit while it runs; a list of what's missing; undo everything; a command
//! palette (Ctrl+K).
pub mod analyze;
pub mod bench;
pub mod build;
pub mod field;
pub mod home;
pub mod inspect;
pub mod rig;
pub mod widgets;

use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{self, RichText};
use bevy_egui::{EguiContexts, EguiPrimaryContextPass};

use crate::input::Held;
use crate::simrun::{Msg, SceneKind, Sim};
use crate::state::{AppState, Workspace};
use crate::update::{Countdown, UpdateStatus};
use crate::view::{MainCam, Orbit, PointerOverUi};
use widgets::*;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, ui)
            .add_systems(Update, (sync_sim, autosave, rig::poll_import, bench::poll))
            .add_systems(Startup, |mut st: ResMut<AppState>| {
                // reopen what was open before an update restarted the app
                if let Some(last) = st.settings.recent.first().cloned() {
                    if std::env::var("FPS_FRESH").is_err() && last.join(crate::projects::FILE).exists() && st.settings.workspace != "home" {
                        let ws = Workspace::from_key(&st.settings.workspace.clone());
                        st.open(last);
                        st.ws = ws;
                    }
                }
            });
    }
}

fn scene_for(ws: Workspace, current: SceneKind) -> SceneKind {
    match ws {
        Workspace::Bench => SceneKind::Bench,
        Workspace::Field => SceneKind::Field,
        Workspace::Build | Workspace::Rig | Workspace::Home => SceneKind::Carpet,
        Workspace::Analyze => current,
    }
}

/// Keeps the simulation on the project: rebuilds it after an edit (carrying its state across) or a
/// change of workspace.
fn sync_sim(mut st: ResMut<AppState>, sim: Res<Sim>, mut last_ws: Local<Option<Workspace>>, mut last_kind: Local<Option<SceneKind>>) {
    let current = sim.shared.lock().map(|s| s.kind).unwrap_or(SceneKind::Carpet);
    let kind = scene_for(st.ws, last_kind.unwrap_or(current));
    let changed_ws = *last_ws != Some(st.ws);
    if st.sim_rev != st.doc.revision || changed_ws && *last_kind != Some(kind) {
        let keep = !changed_ws || *last_kind == Some(kind);
        let game = if kind == SceneKind::Field { st.game(false).ok().cloned().map(Box::new) } else { None };
        sim.send(Msg::Load { project: Box::new(st.doc.project.clone()), kind, keep, game });
        st.sim_rev = st.doc.revision;
        *last_kind = Some(kind);
    }
    *last_ws = Some(st.ws);
}

fn autosave(mut st: ResMut<AppState>, time: Res<Time>) {
    st.autosave_t += time.delta_secs_f64();
    if st.doc.dirty && st.autosave_t > 3.0 && st.dir.is_some() { st.save(); st.autosave_t = 0.0; }
    for t in &mut st.toasts { t.1 -= time.delta_secs_f64(); }
    st.toasts.retain(|t| t.1 > 0.0);
}

#[allow(clippy::too_many_arguments)]
fn ui(
    mut contexts: EguiContexts, mut st: ResMut<AppState>, sim: Res<Sim>, mut cam: Single<&mut Camera, With<MainCam>>,
    window: Single<&Window, With<PrimaryWindow>>, mut over: ResMut<PointerOverUi>, mut held: ResMut<Held>,
    updates: Res<UpdateStatus>, mut countdown: ResMut<Countdown>, time: Res<Time>, mut orbit: ResMut<Orbit>,
    mut cad: Option<ResMut<rig::CadMeshes>>, mut exit: MessageWriter<AppExit>,
) -> Result {
    let ctx = contexts.ctx_mut()?.clone();
    static THEMED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
    if !THEMED.swap(true, std::sync::atomic::Ordering::Relaxed) { theme(&ctx); }
    let mut root = egui::Ui::new(ctx.clone(), "root".into(), egui::UiBuilder::new().layer_id(egui::LayerId::background()).max_rect(ctx.viewport_rect()));
    let st = &mut *st;

    shortcuts(&ctx, st, &sim);

    // ---- the top bar: workspaces, undo, the simulation's controls, updates ----
    let top = egui::Panel::top("top").exact_size(42.0).show(&mut root, |ui| {
        ui.horizontal_centered(|ui| {
            ui.label(RichText::new("FRC Prototype Simulator").strong().color(ACCENT));
            ui.separator();
            for w in Workspace::ALL {
                let on = st.ws == w;
                if ui.selectable_label(on, RichText::new(w.label()).size(15.0)).on_hover_text(w.hint()).clicked() && !on {
                    st.ws = w;
                    st.settings.workspace = w.key().into();
                    st.settings.save();
                }
            }
            ui.separator();
            if ui.add_enabled(st.doc.can_undo(), egui::Button::new("Undo")).on_hover_text(format!("Ctrl+Z: {}", st.doc.undo_name().unwrap_or("nothing to undo"))).clicked() { st.doc.undo(); }
            if ui.add_enabled(st.doc.can_redo(), egui::Button::new("Redo")).on_hover_text(format!("Ctrl+Y: {}", st.doc.redo_name().unwrap_or("nothing to redo"))).clicked() { st.doc.redo(); }
            ui.separator();
            let (playing, speed, rt, err) = sim.shared.lock().map(|s| (s.playing, s.speed, s.realtime, s.error.clone())).unwrap_or((true, 1.0, 1.0, None));
            if ui.button(if playing { "Pause" } else { "Play" }).on_hover_text("P").clicked() { sim.send(Msg::Play(!playing)); }
            if ui.button("Step").on_hover_text("One step (paused)").clicked() { sim.send(Msg::StepOnce); }
            if ui.button("Reset").on_hover_text("R: put everything back where it started").clicked() { sim.send(Msg::Reset); }
            egui::ComboBox::from_id_salt("speed").selected_text(format!("{speed}x")).width(60.0).show_ui(ui, |ui| {
                for s in [0.05, 0.1, 0.25, 0.5, 1.0, 2.0] { if ui.selectable_label(speed == s, format!("{s}x")).clicked() { sim.send(Msg::Speed(s)); } }
            }).response.on_hover_text("Slow motion");
            if playing && rt < 0.9 * speed { ui.label(RichText::new(format!("{:.0}% of real time", rt * 100.0)).color(WARN)).on_hover_text("This computer can't keep up at this speed; the simulation runs slower than the clock, but stays correct."); }
            if let Some(e) = err { ui.label(RichText::new("Simulation error").color(BAD)).on_hover_text(e); }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                update_notice(ui, st, &updates, &mut countdown, &time, &sim);
                if ui.button("Ctrl+K").on_hover_text("Find any action by name").clicked() { st.palette_open = true; }
                let name = st.doc.project.name.clone();
                let saved = if st.doc.dirty { " (saving...)" } else { "" };
                ui.label(RichText::new(format!("{name}{saved}")).color(MUTED));
            });
        });
    }).response.rect;

    // ---- the workspace's own panels ----
    let mut left = 0.0;
    let mut right = 0.0;
    let mut bottom = 0.0;
    match st.ws {
        Workspace::Home => {
            egui::CentralPanel::default().show(&mut root, |ui| home::show(ui, st));
        }
        ws => {
            left = egui::Panel::left("left").default_size(330.0).resizable(true).show(&mut root, |ui| {
                egui::ScrollArea::vertical().id_salt("left_scroll").show(ui, |ui| match ws {
                    Workspace::Build => build::left(ui, st, &sim),
                    Workspace::Rig => rig::left(ui, st, &sim, cad.as_deref_mut()),
                    Workspace::Bench => bench::left(ui, st, &sim),
                    Workspace::Field => field::left(ui, st, &sim),
                    Workspace::Analyze => analyze::left(ui, st, &sim),
                    Workspace::Home => {}
                });
            }).response.rect.width();
            right = egui::Panel::right("right").default_size(360.0).resizable(true).show(&mut root, |ui| {
                egui::ScrollArea::vertical().id_salt("right_scroll").show(ui, |ui| match ws {
                    Workspace::Bench => bench::right(ui, st, &sim),
                    Workspace::Analyze => analyze::right(ui, st, &sim),
                    _ => inspect::show(ui, st, &sim),
                });
            }).response.rect.width();
            bottom = egui::Panel::bottom("bottom").default_size(210.0).min_size(170.0).resizable(true).show(&mut root, |ui| match ws {
                Workspace::Bench => bench::bottom(ui, st, &sim),
                Workspace::Analyze => analyze::bottom(ui, st, &sim),
                Workspace::Field => field::bottom(ui, st, &sim),
                _ => build::bottom(ui, st, &sim),
            }).response.rect.height();
            // overlays on the view: what's selected, the view switches, help
            let view_rect = egui::Rect::from_min_max(egui::pos2(left, top.bottom()), egui::pos2(ctx.viewport_rect().right() - right, ctx.viewport_rect().bottom() - bottom));
            view_overlay(&ctx, st, &sim, view_rect, &mut orbit);
        }
    }
    toasts(&ctx, st);
    palette(&ctx, st, &sim, &mut exit);

    // the 3D view gets what the panels leave
    let scale = window.scale_factor();
    let top_h = top.height();
    let pos = UVec2::new((left * scale) as u32, (top_h * scale) as u32);
    let full = UVec2::new(window.physical_width(), window.physical_height());
    let used = UVec2::new(((left + right) * scale) as u32, ((top_h + bottom) * scale) as u32);
    let size = UVec2::new(full.x.saturating_sub(used.x).max(1), full.y.saturating_sub(used.y).max(1));
    if st.ws == Workspace::Home {
        cam.viewport = Some(Viewport { physical_position: UVec2::ZERO, physical_size: UVec2::ONE, ..default() });
    } else {
        cam.viewport = Some(Viewport { physical_position: pos, physical_size: size, ..default() });
    }
    // the camera leaves the pointer alone over a panel, and the keys alone while typing
    over.0 = window.cursor_position().is_none_or(|p| p.x < left || p.y < top_h || p.x > window.width() - right || p.y > window.height() - bottom);
    held.typing = ctx.egui_wants_keyboard_input();
    let _ = cad;
    Ok(())
}

fn shortcuts(ctx: &egui::Context, st: &mut AppState, sim: &Sim) {
    let typing = ctx.egui_wants_keyboard_input();
    ctx.input_mut(|i| {
        if i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z) { st.doc.undo(); }
        if i.consume_key(egui::Modifiers::COMMAND, egui::Key::Y) || i.consume_key(egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, egui::Key::Z) { st.doc.redo(); }
        if i.consume_key(egui::Modifiers::COMMAND, egui::Key::S) { st.save(); st.toast("Saved"); }
        if i.consume_key(egui::Modifiers::COMMAND, egui::Key::K) { st.palette_open = true; st.palette_query.clear(); }
        if !typing {
            if i.consume_key(egui::Modifiers::NONE, egui::Key::P) {
                let playing = sim.shared.lock().map(|s| s.playing).unwrap_or(true);
                sim.send(Msg::Play(!playing));
            }
            if i.consume_key(egui::Modifiers::NONE, egui::Key::R) { sim.send(Msg::Reset); }
            if i.key_pressed(egui::Key::Delete) {
                if let crate::state::Selection::Mechanism(m) = st.sel.clone() {
                    st.doc.edit("Remove mechanism", |p| p.robot.remove_mechanism(m));
                    st.sel = crate::state::Selection::Robot;
                }
            }
        }
    });
}

fn update_notice(ui: &mut egui::Ui, st: &mut AppState, up: &UpdateStatus, countdown: &mut Countdown, time: &Time, sim: &Sim) {
    let Some(v) = up.ready_version() else {
        ui.label(RichText::new(up.text()).color(MUTED).size(11.5));
        return;
    };
    // safe to restart by itself: saved, no sweep running, no match on
    let in_match = sim.shared.lock().map(|s| s.kind == SceneKind::Field && s.playing).unwrap_or(false) && st.ws == Workspace::Field;
    let safe = !st.doc.dirty && st.bench.running.is_none() && st.bench.sweep_running.is_none() && !in_match && st.import.busy.is_none();
    let restart = |st: &mut AppState| {
        st.save();
        st.settings.workspace = st.ws.key().into();
        st.settings.save();
        crate::update::apply(up);
    };
    if safe {
        let t = countdown.0.get_or_insert(crate::update::NOTICE);
        *t -= time.delta_secs();
        if *t <= 0.0 { restart(st); }
        if ui.button(RichText::new(format!("Updating to v{v} in {:.0}s (click to restart now)", t.ceil())).color(GOOD)).clicked() { restart(st); }
    } else {
        countdown.0 = None;
        if ui.button(RichText::new(format!("Restart to update to v{v}")).color(GOOD)).on_hover_text("Saves your project and reopens it after").clicked() { restart(st); }
    }
}

fn view_overlay(ctx: &egui::Context, st: &mut AppState, sim: &Sim, rect: egui::Rect, orbit: &mut Orbit) {
    egui::Area::new("view_switches".into()).fixed_pos(rect.left_top() + egui::vec2(8.0, 8.0)).show(ctx, |ui| {
        egui::Frame::popup(ui.style()).show(ui, |ui| ui.horizontal(|ui| {
            ui.checkbox(&mut st.view.com, "Centre of mass").on_hover_text("The robot's centre of mass and its wheels' footprint: red when it's close to tipping");
            ui.checkbox(&mut st.view.contacts, "Contacts").on_hover_text("Roller contacts: green gripping, orange slipping; longer is harder");
            ui.checkbox(&mut st.view.zones, "Pickup zones");
            ui.checkbox(&mut st.view.follow, "Follow robot");
            if ui.button("Frame robot").clicked() { *orbit = Orbit::default(); }
        }));
    });
    let snap = sim.snap();
    egui::Area::new("view_readout".into()).anchor(egui::Align2::RIGHT_TOP, egui::vec2(-(ctx.viewport_rect().right() - rect.right()) - 8.0, rect.top() + 8.0)).show(ctx, |ui| {
        if let Some(s) = &snap {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_min_width(170.0);
                ui.label(format!("t {:.2} s", s.time));
                let vc = if s.brownout { BAD } else if s.bus_voltage < 9.0 { WARN } else { GOOD };
                ui.label(RichText::new(format!("Battery {:.2} V  {:.0} A", s.bus_voltage, s.bus_current)).color(vc));
                if !s.support.is_empty() {
                    let unit = st.unit(library::units::Kind::Speed);
                    ui.label(format!("Speed {}", library::units::format(s.speed, unit)));
                    let tc = if s.tip_margin > 0.08 { GOOD } else if s.tip_margin > 0.0 { WARN } else { BAD };
                    ui.label(RichText::new(format!("Tip margin {}", library::units::format(s.tip_margin, st.unit(library::units::Kind::Length)))).color(tc))
                        .on_hover_text("How far the centre of mass is inside the wheels' footprint. Less is closer to tipping.");
                }
                if s.brownout { ui.label(RichText::new("BROWNOUT").color(BAD).strong()); }
            });
        }
    });
    let warnings = sim.shared.lock().map(|s| s.warnings.clone()).unwrap_or_default();
    let err = sim.shared.lock().ok().and_then(|s| s.error.clone());
    if err.is_some() || !warnings.is_empty() {
        egui::Area::new("view_warn".into()).fixed_pos(rect.left_bottom() + egui::vec2(8.0, -60.0)).show(ctx, |ui| {
            if let Some(e) = &err { ui.label(RichText::new(format!("Can't simulate: {e}")).color(BAD)); }
            for w in warnings.iter().take(3) { ui.label(RichText::new(w).color(WARN).size(12.0)); }
        });
    }
    if matches!(st.ws, Workspace::Build | Workspace::Field) {
        egui::Area::new("view_help".into()).fixed_pos(rect.left_bottom() + egui::vec2(8.0, -34.0)).show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.label(RichText::new("Drive: W A S D, turn: arrow keys (or a gamepad). Camera: right-drag turns, wheel zooms, middle-drag pans. Mechanism keys: Controls.").size(12.0));
            });
        });
    }
}

fn toasts(ctx: &egui::Context, st: &AppState) {
    if st.toasts.is_empty() { return; }
    egui::Area::new("toasts".into()).anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -210.0)).show(ctx, |ui| {
        for (t, _) in &st.toasts {
            egui::Frame::popup(ui.style()).show(ui, |ui| { ui.label(t); });
        }
    });
}

/// Every action by name (rule 10).
fn palette(ctx: &egui::Context, st: &mut AppState, sim: &Sim, exit: &mut MessageWriter<AppExit>) {
    if !st.palette_open { return; }
    let mut open = true;
    let mut chosen: Option<String> = None;
    egui::Window::new("Find an action").open(&mut open).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 80.0)).show(ctx, |ui| {
        let r = ui.text_edit_singleline(&mut st.palette_query);
        r.request_focus();
        let q = st.palette_query.to_lowercase();
        let mut actions: Vec<String> = Workspace::ALL.iter().map(|w| format!("Go to {}", w.label())).collect();
        for k in [model::MechKind::Elevator, model::MechKind::Arm, model::MechKind::Wrist, model::MechKind::Telescope, model::MechKind::Intake, model::MechKind::Claw, model::MechKind::Shooter, model::MechKind::Turret, model::MechKind::Climber] {
            actions.push(format!("Add {}", k.label()));
        }
        actions.extend(["Save", "Undo", "Redo", "Reset simulation", "Pause or play", "Start a match", "Export the run as CSV", "Switch units (in/mm)", "Make every mechanism Full physics", "Make every mechanism Concept", "Quit"].map(String::from));
        let hits: Vec<&String> = actions.iter().filter(|a| q.split_whitespace().all(|w| a.to_lowercase().contains(w))).collect();
        for a in hits.iter().take(12) { if ui.selectable_label(false, a.as_str()).clicked() { chosen = Some((*a).clone()); } }
        if ui.input(|i| i.key_pressed(egui::Key::Enter)) { chosen = hits.first().map(|s| (*s).clone()); }
    });
    if let Some(a) = chosen {
        st.palette_open = false;
        if let Some(w) = Workspace::ALL.iter().find(|w| a == format!("Go to {}", w.label())) { st.ws = *w; return; }
        if let Some(k) = a.strip_prefix("Add ") {
            for kind in [model::MechKind::Elevator, model::MechKind::Arm, model::MechKind::Wrist, model::MechKind::Telescope, model::MechKind::Intake, model::MechKind::Claw, model::MechKind::Shooter, model::MechKind::Turret, model::MechKind::Climber] {
                if kind.label() == k { build::add_mechanism(st, kind); st.ws = Workspace::Build; }
            }
            return;
        }
        match a.as_str() {
            "Save" => { st.save(); st.toast("Saved"); }
            "Undo" => st.doc.undo(),
            "Redo" => st.doc.redo(),
            "Reset simulation" => sim.send(Msg::Reset),
            "Pause or play" => { let p = sim.shared.lock().map(|s| s.playing).unwrap_or(true); sim.send(Msg::Play(!p)); }
            "Start a match" => { st.ws = Workspace::Field; sim.send(Msg::StartMatch); }
            "Export the run as CSV" => analyze::export_csv(st, sim),
            "Switch units (in/mm)" => { st.settings.units = if st.settings.imperial() { "metric".into() } else { "imperial".into() }; st.settings.save(); }
            "Make every mechanism Full physics" => st.doc.edit("All Full physics", |p| for m in &mut p.robot.mechanisms { m.fidelity = model::Fidelity::Full; }),
            "Make every mechanism Concept" => st.doc.edit("All Concept", |p| for m in &mut p.robot.mechanisms { m.fidelity = model::Fidelity::Concept; }),
            "Quit" => { st.save(); exit.write(AppExit::Success); }
            _ => {}
        }
    }
    if !open { st.palette_open = false; }
    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) { st.palette_open = false; }
}
