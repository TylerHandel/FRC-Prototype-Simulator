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
    let settings = settings::Settings::load();
    let shot = std::env::var("FPS_SHOT").ok();
    App::new()
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
