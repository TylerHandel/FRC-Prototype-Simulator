//! The Field (M4): drive on REEFSCAPE, timed cycles and matches, with a log of what scored when.
use bevy_egui::egui::{self, RichText, Ui};

use super::widgets::*;
use crate::simrun::{Msg, Sim};
use crate::state::AppState;

pub fn left(ui: &mut Ui, st: &mut AppState, sim: &Sim) {
    let g = sim::game::GamePack::builtin(&st.project().game);
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
        if ui.button(RichText::new("Start a match").strong()).on_hover_text("15 s autonomous, then 2:15 teleop. Points count from now.").clicked() { sim.send(Msg::Reset); sim.send(Msg::StartMatch); }
        if ui.button("Back to the start").clicked() { sim.send(Msg::Reset); }
    });
    ui.checkbox(&mut st.settings.field_relative, "Field-relative driving").on_hover_text("W drives away from the driver station, whichever way the robot faces");
    heading(ui, "How to play");
    hint(ui, "Drive with W A S D (or the left stick), turn with the arrow keys (or the right stick). Hold Shift to go slow.");
    hint(ui, "Drive up to a CORAL STATION: it hands you a CORAL. Run the intake over pieces on the carpet to pick them up.");
    hint(ui, "In Concept, let go of a CORAL close to a branch and it's seated and scored (the scoring assist). In Full, it has to really land.");
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
