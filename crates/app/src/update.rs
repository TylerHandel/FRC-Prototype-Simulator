//! Updates (PLAN.md D21), as rebuilt-sim-native's crates/app/src/update.rs at da6b231: a background
//! thread asks GitHub Releases for a newer version when the app starts and every minute after, and
//! downloads it. Once one is downloaded, the app restarts into it by itself after a few seconds'
//! notice when nothing would be lost (the project saved, no sweep or match running); otherwise a
//! button waits. Closing the app installs it too (Velopack applies a downloaded update at the next
//! start). The project is saved first and reopened after.
use bevy::prelude::*;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const REPO: &str = "https://github.com/TylerHandel/FRC-Prototype-Simulator";
const RECHECK: Duration = Duration::from_secs(60);
pub const NOTICE: f32 = 5.0;

#[derive(Default)]
struct State { text: String, ready: Option<velopack::UpdateInfo> }

#[derive(Resource, Clone, Default)]
pub struct UpdateStatus(Arc<Mutex<State>>);

impl UpdateStatus {
    pub fn text(&self) -> String { self.0.lock().map(|s| s.text.clone()).unwrap_or_default() }
    fn set(&self, s: impl Into<String>) { if let Ok(mut g) = self.0.lock() { g.text = s.into(); } }
    pub fn ready_version(&self) -> Option<String> { self.0.lock().ok()?.ready.as_ref().map(|i| i.TargetFullRelease.Version.clone()) }
}

/// The countdown to restarting by itself, while it's safe.
#[derive(Resource, Default)]
pub struct Countdown(pub Option<f32>);

pub struct UpdatePlugin;

impl Plugin for UpdatePlugin {
    fn build(&self, app: &mut App) {
        let status = UpdateStatus::default();
        let s = status.clone();
        std::thread::spawn(move || watch(&s));
        app.insert_resource(status).init_resource::<Countdown>();
    }
}

/// The latest release's files, straight from its download links: no GitHub API calls, so checking
/// every minute stays clear of the API's hourly limit.
fn manager() -> Option<velopack::UpdateManager> {
    let source = velopack::sources::HttpSource::new(format!("{REPO}/releases/latest/download"));
    velopack::UpdateManager::new(source, None, None).ok()
}

pub fn current_version() -> String {
    manager().map(|m| m.get_current_version_as_string()).unwrap_or_else(|| format!("{} (development build)", env!("CARGO_PKG_VERSION")))
}

fn watch(status: &UpdateStatus) {
    use velopack::UpdateCheck;
    // not installed (a `cargo run`): nothing to update. FPS_FAKE_UPDATE=<version> pretends one is
    // ready, to see the notice.
    let Some(um) = manager() else {
        if let Ok(v) = std::env::var("FPS_FAKE_UPDATE") {
            let mut info = velopack::UpdateInfo::default();
            info.TargetFullRelease.Version = v;
            if let Ok(mut g) = status.0.lock() { g.ready = Some(info); }
        }
        return status.set("development build (no updates)");
    };
    let v = um.get_current_version_as_string();
    loop {
        status.set(format!("v{v} | checking for updates..."));
        match um.check_for_updates() {
            Ok(UpdateCheck::UpdateAvailable(info)) => {
                let to = info.TargetFullRelease.Version.clone();
                status.set(format!("v{v} | downloading v{to}..."));
                match um.download_updates(&info, None) {
                    Ok(()) => {
                        status.set(format!("v{v} | v{to} is ready"));
                        if let Ok(mut g) = status.0.lock() { g.ready = Some(*info); }
                        return;
                    }
                    Err(e) => status.set(format!("v{v} | download failed: {e}")),
                }
            }
            Ok(_) => status.set(format!("v{v} | up to date")),
            Err(e) => status.set(format!("v{v} | update check failed: {e}")),
        }
        std::thread::sleep(RECHECK);
    }
}

/// Exits, applies the update and starts the new version (returns only if that fails).
pub fn apply(status: &UpdateStatus) {
    let info = status.0.lock().ok().and_then(|g| g.ready.clone());
    if let (Some(info), Some(um)) = (info, manager()) {
        status.set("restarting into the update...");
        if let Err(e) = um.apply_updates_and_restart(&info.TargetFullRelease) {
            status.set(format!("update failed: {e}"));
        }
    }
}
