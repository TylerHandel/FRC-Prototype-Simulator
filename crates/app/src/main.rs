#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
//! FRC Prototype Simulator: a design tool for FRC robots (PLAN.md). The simulation runs in the
//! `sim` crate on its own thread (simrun); this draws it (view), takes input (input), and holds the
//! interface (ui) and the project (state).

mod input;
mod projects;
mod settings;
mod shot;
mod simrun;
mod state;
mod ui;
mod update;
mod view;

use bevy::prelude::*;
use bevy::window::PresentMode;

fn main() {
    // Velopack's install and update hooks run first and may exit (on install, update, uninstall).
    velopack::VelopackApp::build().run();
    // `--self-test`: build every starter robot and run it for a second, then exit (0 when all ran).
    // Release CI runs it on the packaged app, so a missing DLL or a broken library fails the build.
    if std::env::args().any(|a| a == "--self-test") { std::process::exit(self_test()); }
    let settings = settings::Settings::load();
    let shot = std::env::var("FPS_SHOT").ok();
    App::new()
        .register_asset_source("gamepacks", bevy::asset::io::AssetSourceBuilder::platform_default(
            &settings.projects_dir.join("Game packs").to_string_lossy(), None))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("FRC Prototype Simulator {}", env!("CARGO_PKG_VERSION")),
                present_mode: if shot.is_some() { PresentMode::AutoNoVsync } else { PresentMode::AutoVsync },
                resolution: (1600u32, 950u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(bevy_egui::EguiPlugin::default())
        .insert_resource(ClearColor(Color::srgb(0.11, 0.12, 0.14)))
        .insert_resource(state::AppState::new(settings))
        .add_plugins((simrun::SimPlugin, view::ViewPlugin, ui::UiPlugin, input::InputPlugin, update::UpdatePlugin, shot::ShotPlugin))
        .run();
}

fn self_test() -> i32 {
    use std::io::Write;
    let mut out = String::new();
    let lib = library::Library::builtin();
    let mut ok = true;
    for (name, _, project) in projects::starters() {
        match sim::world::World::new(&project, &lib, sim::world::Scene::carpet()) {
            Ok(mut w) => {
                let n = (1.0 / w.dt).round() as usize;
                for _ in 0..n { w.step(); }
                let s = w.snapshot();
                let fine = s.links.iter().all(|l| l.pose.pos.iter().all(|x| x.is_finite()));
                ok &= fine;
                out += &format!("{name}: {:.2} s, {:.2} V{}\n", s.time, s.bus_voltage, if fine { "" } else { " NOT FINITE" });
            }
            Err(e) => { ok = false; out += &format!("{name}: {e:#}\n"); }
        }
    }
    out += &format!("FRC Prototype Simulator {}: {}\n", env!("CARGO_PKG_VERSION"), if ok { "ok" } else { "FAILED" });
    // a windowed app on Windows has no console, so write a file too
    let _ = std::fs::write(std::env::temp_dir().join("frc-prototype-sim-self-test.txt"), &out);
    let _ = std::io::stdout().write_all(out.as_bytes());
    if ok { 0 } else { 1 }
}
