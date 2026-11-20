//! For checking a build without a person at it: with FPS_SHOT=<file.png> the app opens a starting
//! project (FPS_SHOT_STARTER=<n>, the Home screen's order) in a temporary folder, goes to a
//! workspace (FPS_SHOT_WS=build|rig|bench|field|analyze|home), waits FPS_SHOT_WAIT frames (240),
//! saves a screenshot and exits. FPS_SHOT_BENCH=1 runs the Bench's wheel comparison first;
//! FPS_SHOT_KEYS=KeyW,KeyF holds those keys down the whole time.
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use crate::state::{AppState, Workspace};

pub struct ShotPlugin;

impl Plugin for ShotPlugin {
    fn build(&self, app: &mut App) {
        if let Ok(path) = std::env::var("FPS_SHOT") {
            app.insert_resource(Shot { path, frames: 0, taken: false })
                .add_systems(Startup, open)
                .add_systems(PreUpdate, hold_keys.after(bevy::input::InputSystems))
                .add_systems(Update, shoot);
        }
    }
}

#[derive(Resource)]
struct Shot { path: String, frames: u32, taken: bool }

fn open(mut st: ResMut<AppState>) {
    let n: usize = std::env::var("FPS_SHOT_STARTER").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let starters = crate::projects::starters();
    let (_, _, p) = starters.into_iter().nth(n).expect("no such starter");
    st.settings.projects_dir = std::env::temp_dir().join("fps-shot");
    st.start_new(p);
    st.ws = std::env::var("FPS_SHOT_WS").map(|w| Workspace::from_key(&w)).unwrap_or(Workspace::Build);
    if std::env::var("FPS_SHOT_WS").as_deref() == Ok("home") { st.ws = Workspace::Home; }
    if std::env::var("FPS_SHOT_BENCH").is_ok() {
        let mut p = st.project().clone();
        p.bench.compare = vec!["am-compliant-4in-35a".into(), "am-compliant-4in-60a".into(), "wcp-flex-4in-45a".into()];
        let lib = st.lib.clone();
        st.doc.edit("Compare", |x| x.bench.compare = p.bench.compare.clone());
        let out = sim::bench::compare(&p, &lib, &p.bench.compare).into_iter().zip(p.bench.compare.iter()).map(|(r, w)| (lib.wheel(w).map_or(w.clone(), |x| x.name.clone()), r.map_err(|e| e.to_string()))).collect();
        st.bench.results = out;
    }
}

fn hold_keys(mut keys: ResMut<ButtonInput<KeyCode>>) {
    let Ok(list) = std::env::var("FPS_SHOT_KEYS") else { return };
    for k in list.split(',') {
        let code = match k.trim() { "KeyW" => KeyCode::KeyW, "KeyA" => KeyCode::KeyA, "KeyD" => KeyCode::KeyD, "KeyS" => KeyCode::KeyS, "KeyF" => KeyCode::KeyF, "KeyJ" => KeyCode::KeyJ, "KeyK" => KeyCode::KeyK, "ArrowLeft" => KeyCode::ArrowLeft, _ => continue };
        keys.press(code);
    }
}

fn shoot(mut commands: Commands, mut shot: ResMut<Shot>, mut exit: MessageWriter<AppExit>) {
    shot.frames += 1;
    let wait: u32 = std::env::var("FPS_SHOT_WAIT").ok().and_then(|s| s.parse().ok()).unwrap_or(240);
    if !shot.taken && shot.frames >= wait {
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(shot.path.clone()));
        shot.taken = true;
    }
    if shot.taken && shot.frames >= wait + 30 { exit.write(AppExit::Success); }
}
