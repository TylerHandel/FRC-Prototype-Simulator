//! The Field (M4): drive on REEFSCAPE, timed cycles and matches, with a log of what scored when.
use bevy_egui::egui::{self, RichText, Ui};

use super::widgets::*;
use crate::simrun::{Msg, Sim};
use crate::state::AppState;

pub fn left(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    game_picker(ui, st);
    let g = st.game(false).ok().map(|f| f.pack.clone());
    let Some(g) = g else {
        if let Err(e) = st.game(false) { ui.colored_label(BAD, e.clone()); }
        return;
    };
    let g = Some(g);
    heading(ui, g.as_ref().map_or("Field", |g| g.name.as_str()));
    if let Some(g) = &g { hint(ui, &g.about); }
    let snap = sim.snap();
    if let Some(s) = &snap {
        ui.label(RichText::new(format!("{}  {:.0} points", s.phase, s.score)).size(22.0).strong());
        if let Some(g) = &g {
            let m = sim::game::Match { running: s.phase != "Practice", time: s.match_time, ..Default::default() };
            if s.phase != "Practice" { ui.label(RichText::new(format!("{:.0} s left", m.remaining(&g.timing))).size(18.0)); }
        }
    }
    ui.horizontal(|ui| {
        let tip = g.as_ref().map_or(String::new(), |g| format!("{:.0} s autonomous, then {:.0} s teleop. Points count from now.", g.timing.auto, g.timing.teleop));
        if ui.button(RichText::new("Start a match").strong()).on_hover_text(tip).clicked() { sim.send(Msg::Reset); sim.send(Msg::StartMatch); }
        if ui.button("Back to the start").clicked() { sim.send(Msg::Reset); }
    });
    ui.checkbox(&mut st.settings.field_relative, "Field-relative driving").on_hover_text("W drives away from the driver station, whichever way the robot faces");
    heading(ui, "How to play");
    hint(ui, "Drive with W A S D (or the left stick), turn with the arrow keys (or the right stick). Hold Shift to go slow.");
    hint(ui, "Drive up to a human player station: it hands you a piece. Run the intake over pieces on the carpet to pick them up.");
    hint(ui, "In Concept, let go of a piece close to where it scores and it's seated and counted (the scoring assist). In Full, it has to really land.");
    heading(ui, "Buttons");
    let r = st.project().robot.clone();
    for b in &r.controls.bindings {
        let what = match &b.action {
            model::Action::Preset { preset } => r.controls.presets.iter().find(|p| p.id == *preset).map_or("?".into(), |p| p.name.clone()),
            model::Action::Run { drive, value, .. } => format!("{} {}", if *value >= 0.0 { "run" } else { "reverse" }, r.drive(*drive).map_or("?".into(), |d| d.name.clone())),
            model::Action::Release { mechanism } => format!("let go ({})", r.mechanism(*mechanism).map_or("?".into(), |m| m.name.clone())),
        };
        ui.label(format!("{:?} / {}: {what}", b.button, b.button.key()));
    }
}

pub fn bottom(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    ui.columns(2, |cols| {
        let ui = &mut cols[0];
        heading(ui, "Cycle log");
        let log = sim.shared.lock().map(|s| s.log.clone()).unwrap_or_default();
        if log.is_empty() { hint(ui, "Picks and scores show up here, with the time between scores."); }
        let mut last_score: Option<f64> = None;
        let scores: Vec<f64> = log.iter().filter(|e| e.points > 0.0).map(|e| e.time).collect();
        egui::ScrollArea::vertical().id_salt("log").max_height(150.0).stick_to_bottom(true).show(ui, |ui| {
            for e in &log {
                let gap = if e.points > 0.0 { let g = last_score.map(|l| e.time - l); last_score = Some(e.time); g } else { None };
                let extra = gap.map_or(String::new(), |g| format!("  ({g:.1} s cycle)"));
                ui.label(format!("{:6.1} s  {}{}{}", e.time, e.what, if e.points > 0.0 { format!("  +{}", e.points) } else { String::new() }, extra));
            }
        });
        if scores.len() >= 2 {
            let mean = (scores[scores.len() - 1] - scores[0]) / (scores.len() - 1) as f64;
            ui.label(RichText::new(format!("{} scored, a cycle every {:.1} s on average", scores.len(), mean)).strong());
        }
        let ui = &mut cols[1];
        heading(ui, "Battery, last 10 s");
        super::analyze::mini_plot(ui, st, sim, 140.0);
    });
}

/// Which game: the packs that come with the app and any in the game packs folder, each with its
/// problems; the folder opens from here, and Reload picks up edits.
fn game_picker(ui: &mut Ui, st: &mut AppState) {
    if st.games.is_empty() { st.list_games(); }
    let current = st.project().game.clone();
    let shown = st.games.iter().find(|g| g.0 == current).map_or(current.clone(), |g| g.1.clone());
    let mut pick = None;
    ui.horizontal(|ui| {
        ui.label("Game");
        egui::ComboBox::from_id_salt("game").selected_text(shown).width(200.0).show_ui(ui, |ui| {
            for (id, name, problems) in &st.games {
                let label = if problems.is_empty() { name.clone() } else { format!("{name}  ({} to fix)", problems.len()) };
                if ui.selectable_label(*id == current, label).clicked() { pick = Some(id.clone()); }
            }
        });
    });
    ui.horizontal(|ui| {
        if ui.small_button("Open the game packs folder").on_hover_text("Put a game pack's folder here (see the kickoff guide)").clicked() {
            let d = st.games_dir();
            let _ = std::fs::create_dir_all(&d);
            open_folder(&d);
        }
        if ui.small_button("New game pack").on_hover_text("A small working example to start the new game from (the kickoff guide's step 1)").clicked() {
            let dir = (0..).map(|i| st.games_dir().join(if i == 0 { "new-game".to_string() } else { format!("new-game-{i}") })).find(|d| !d.exists()).unwrap();
            let id = dir.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
            let made = std::fs::create_dir_all(&dir)
                .and_then(|_| std::fs::write(dir.join("game.json"), TEMPLATE_GAME.replace("\"id\": \"new-game\"", &format!("\"id\": \"{id}\""))))
                .and_then(|_| std::fs::write(dir.join("pieces.json"), TEMPLATE_PIECES));
            match made {
                Ok(()) => {
                    st.list_games();
                    st.doc.edit("Game", |p| p.game = id.clone());
                    let _ = st.game(true);
                    open_folder(&dir);
                    st.toast(format!("Made {}: edit game.json, then Reload", dir.display()));
                }
                Err(e) => st.toast(format!("Could not make the pack: {e}")),
            }
        }
        if ui.small_button("Reload").on_hover_text("Read the packs again after editing one").clicked() {
            st.list_games();
            let _ = st.game(true);
            st.sim_rev = u64::MAX;
        }
    });
    if let Some(id) = pick {
        st.doc.edit("Game", |p| p.game = id);
        let _ = st.game(true);
    }
    if let Some((_, _, problems)) = st.games.iter().find(|g| g.0 == current) {
        if !problems.is_empty() {
            egui::CollapsingHeader::new(RichText::new(format!("{} things to fix in this game pack", problems.len())).color(WARN)).id_salt("game_problems").show(ui, |ui| {
                for p in problems { hint(ui, p); }
            });
        }
    }
}

fn open_folder(d: &std::path::Path) {
    let cmd = if cfg!(windows) { "explorer" } else if cfg!(target_os = "macos") { "open" } else { "xdg-open" };
    let _ = std::process::Command::new(cmd).arg(d).spawn();
}

/// The starting point for a new game pack (docs/KICKOFF.md step 1)
const TEMPLATE_GAME: &str = include_str!("../../../../assets/games/template/game.json");
const TEMPLATE_PIECES: &str = include_str!("../../../../assets/games/template/pieces.json");
