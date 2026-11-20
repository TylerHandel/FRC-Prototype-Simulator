//! The user's settings, kept between runs in settings.json in their app data folder.
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// "imperial" (in, lb) or "metric" (mm, kg)
    pub units: String,
    /// projects opened, most recent first
    pub recent: Vec<PathBuf>,
    /// where new projects go
    pub projects_dir: PathBuf,
    /// drive field-relative (from the driver's view) or robot-relative
    pub field_relative: bool,
    /// the last workspace, reopened after an update restarts the app
    pub workspace: String,
    /// shown the first-run tips
    pub seen_tips: bool,
}

impl Default for Settings {
    fn default() -> Self {
        let docs = dirs::document_dir().or_else(dirs::home_dir).unwrap_or_else(|| PathBuf::from("."));
        Settings { units: "imperial".into(), recent: vec![], projects_dir: docs.join("FRC Prototype Simulator"), field_relative: true, workspace: "build".into(), seen_tips: false }
    }
}

pub fn app_dir() -> PathBuf {
    dirs::config_dir().unwrap_or_else(|| PathBuf::from(".")).join("FRC Prototype Simulator")
}

impl Settings {
    pub fn load() -> Self {
        std::fs::read_to_string(app_dir().join("settings.json")).ok().and_then(|s| serde_json::from_str(&s).ok()).unwrap_or_default()
    }
    pub fn save(&self) {
        // screenshot runs (FPS_SHOT) leave the user's settings alone
        if std::env::var("FPS_SHOT").is_ok() { return; }
        let d = app_dir();
        let _ = std::fs::create_dir_all(&d);
        let _ = std::fs::write(d.join("settings.json"), serde_json::to_string_pretty(self).unwrap_or_default());
    }
    pub fn remember(&mut self, p: &std::path::Path) {
        self.recent.retain(|x| x != p);
        self.recent.insert(0, p.to_path_buf());
        self.recent.truncate(12);
        self.save();
    }
    pub fn imperial(&self) -> bool { self.units != "metric" }
}
