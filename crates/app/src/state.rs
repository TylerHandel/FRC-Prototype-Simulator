//! Everything the interface works on: the open project (with undo), the library, what's selected,
//! the Bench's results and the import in progress.
use bevy::prelude::*;
use library::Library;
use model::history::Doc;
use model::{Id, Project};
use std::path::PathBuf;
use std::sync::mpsc::Receiver;

/// A channel's receiving end that a Bevy resource can hold (resources must be shareable between threads).
pub type Rx<T> = std::sync::Mutex<Receiver<T>>;

use crate::settings::Settings;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Workspace { Home, Build, Rig, Bench, Field, Analyze }

impl Workspace {
    pub const ALL: [Workspace; 6] = [Workspace::Home, Workspace::Build, Workspace::Rig, Workspace::Bench, Workspace::Field, Workspace::Analyze];
    pub fn label(self) -> &'static str {
        match self { Workspace::Home => "Home", Workspace::Build => "Build", Workspace::Rig => "Rig CAD", Workspace::Bench => "Bench", Workspace::Field => "Field", Workspace::Analyze => "Analyze" }
    }
    pub fn hint(self) -> &'static str {
        match self {
            Workspace::Home => "Open or start a project",
            Workspace::Build => "Build a robot from blocks: drivetrain, elevator, arm, intake, claw...",
            Workspace::Rig => "Bring in CAD from Onshape or a STEP file and mark what moves",
            Workspace::Bench => "One mechanism on a stand: send game pieces at it, swap wheels, compare",
            Workspace::Field => "Drive it on the REEFSCAPE field and time your cycles",
            Workspace::Analyze => "Plots, power budget and CSV export of the last run",
        }
    }
    pub fn key(self) -> &'static str {
        match self { Workspace::Home => "home", Workspace::Build => "build", Workspace::Rig => "rig", Workspace::Bench => "bench", Workspace::Field => "field", Workspace::Analyze => "analyze" }
    }
    pub fn from_key(k: &str) -> Workspace { Workspace::ALL.into_iter().find(|w| w.key() == k).unwrap_or(Workspace::Build) }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Selection { Robot, Drivetrain, Electrical, Controls, Mechanism(Id), Link(Id), Joint(Id), Drive(Id), Roller(Id), Part(String) }

/// The Bench's comparison runs, worked out off the interface thread.
#[derive(Default)]
pub struct BenchUi {
    pub results: Vec<(String, Result<sim::bench::Trial, String>)>,
    pub running: Option<Rx<Vec<(String, Result<sim::bench::Trial, String>)>>>,
    pub sweep: Vec<sim::bench::Cell>,
    pub sweep_running: Option<Rx<Vec<sim::bench::Cell>>>,
    /// what to sweep: up to three rows, each a kind (see `ui::bench::KINDS`) and its values
    pub rows: Vec<(usize, String)>,
    /// the axes of the sweep shown, for the table, the heat map and the CSV
    pub sweep_axes: Vec<sim::bench::Axis>,
    /// the outcome the heat map shows
    pub outcome: usize,
    /// the bottom panel shows the sweep (else the comparison's plots)
    pub show_sweep: bool,
    pub throws: usize,
}

#[derive(Default, Clone, Copy, PartialEq)]
pub struct ViewOpts { pub com: bool, pub contacts: bool, pub zones: bool, pub axes: bool, pub follow: bool }

#[derive(Resource)]
pub struct AppState {
    pub doc: Doc,
    pub lib: Library,
    pub dir: Option<PathBuf>,
    pub ws: Workspace,
    pub sel: Selection,
    pub settings: Settings,
    pub toasts: Vec<(String, f64)>,
    pub bench: BenchUi,
    pub view: ViewOpts,
    /// the project changed since the simulation last loaded it
    pub sim_rev: u64,
    pub new_name: String,
    pub palette_open: bool,
    pub palette_query: String,
    pub import: crate::ui::rig::ImportUi,
    /// seconds since the last autosave
    pub autosave_t: f64,
    pub plot_signals: Vec<String>,
    /// the project's game pack, found (cached by id; `None` until looked up)
    pub game: Option<(String, Result<sim::game::Found, String>)>,
    /// every pack the Field's picker offers, and each one's problems (refreshed on Reload)
    pub games: Vec<(String, String, Vec<String>)>,
    /// runs kept to compare (Analyze), saved with the project
    pub kept: Vec<crate::ui::analyze::Kept>,
}

impl AppState {
    pub fn new(settings: Settings) -> Self {
        let mut lib = Library::builtin();
        let _ = lib.load_overlay(&crate::settings::app_dir().join("library"));
        let ws = Workspace::Home;
        AppState {
            doc: Doc::new(Project::new("My robot")), lib, dir: None, ws, sel: Selection::Robot, settings, toasts: vec![], bench: BenchUi { throws: 5, rows: vec![(0, "0.5, 1, 1.5, 2".into())], ..Default::default() },
            view: ViewOpts { com: true, contacts: true, zones: true, axes: true, follow: false }, sim_rev: u64::MAX, new_name: "My robot".into(),
            palette_open: false, palette_query: String::new(), import: Default::default(), autosave_t: 0.0,
            plot_signals: vec!["Battery voltage".into(), "Battery current".into()], game: None, games: vec![], kept: vec![],
        }
    }
    pub fn project(&self) -> &Project { &self.doc.project }

    /// Where the user's game packs live: "Game packs" beside the projects.
    pub fn games_dir(&self) -> PathBuf { self.settings.projects_dir.join("Game packs") }

    /// The project's game pack, looked up again when the project names another (or `reload`):
    /// its pieces join the library.
    pub fn game(&mut self, reload: bool) -> Result<&sim::game::Found, &String> {
        let id = self.doc.project.game.clone();
        if reload || self.game.as_ref().is_none_or(|g| g.0 != id) {
            let r = sim::game::find(&id, &[self.games_dir()]);
            if let Ok(Some(d)) = r.as_ref().map(|f| f.dir.clone()) { let _ = self.lib.load_overlay(&d); }
            self.game = Some((id, r));
        }
        self.game.as_ref().unwrap().1.as_ref()
    }

    /// Every pack the app can see, with its problems, for the Field's picker.
    pub fn list_games(&mut self) {
        let dirs = [self.games_dir()];
        let lib = self.lib.clone();
        self.games = sim::game::list(&dirs).into_iter().map(|(id, dir, r)| match r {
            Ok(g) => {
                let mut l = lib.clone();
                if let Some(d) = &dir { if let Err(e) = l.load_overlay(d) { return (id, g.name.clone(), vec![format!("pieces.json: {e}")]); } }
                let name = if dir.is_some() { format!("{} (your folder)", g.name) } else { g.name.clone() };
                (id, name, g.check(&l))
            }
            Err(e) => (id.clone(), format!("{id} (can't be read)"), vec![e]),
        }).collect();
    }
    pub fn toast(&mut self, s: impl Into<String>) { self.toasts.push((s.into(), 5.0)); }

    pub fn open(&mut self, dir: PathBuf) {
        match crate::projects::load(&dir) {
            Ok(p) => {
                self.doc = Doc::new(p);
                self.kept = crate::ui::analyze::load_kept(&dir);
                self.settings.remember(&dir);
                self.dir = Some(dir);
                self.sel = Selection::Robot;
                self.ws = Workspace::Build;
                self.toast(format!("Opened {}", self.doc.project.name));
            }
            Err(e) => self.toast(format!("Could not open {}: {e}", dir.display())),
        }
    }
    pub fn start_new(&mut self, p: Project) {
        let dir = crate::projects::fresh_dir(&self.settings.projects_dir, &p.name);
        self.doc = Doc::new(p);
        self.dir = Some(dir.clone());
        self.save();
        self.settings.remember(&dir);
        self.sel = Selection::Robot;
        self.ws = Workspace::Build;
    }
    pub fn save(&mut self) {
        let Some(d) = self.dir.clone() else { return };
        match crate::projects::save(&d, &self.doc.project) {
            Ok(()) => { self.doc.dirty = false; }
            Err(e) => self.toast(format!("Could not save: {e}")),
        }
    }
    /// The units a field shows: inches and pounds, or millimetres and kilograms.
    pub fn unit(&self, k: library::units::Kind) -> &'static library::units::Unit {
        use library::units::{unit, Kind};
        let imp = self.settings.imperial();
        let label = match k {
            Kind::Length => if imp { "in" } else { "mm" },
            Kind::Mass => if imp { "lb" } else { "kg" },
            Kind::Angle => "deg",
            Kind::AngularSpeed => "rpm",
            Kind::Speed => if imp { "ft/s" } else { "m/s" },
            Kind::Force => if imp { "lbf" } else { "N" },
            Kind::Torque => if imp { "in*lbf" } else { "N*m" },
            Kind::Current => "A",
            Kind::Voltage => "V",
            Kind::Pressure => "psi",
            Kind::Time => "s",
            Kind::Inertia => if imp { "lb*in^2" } else { "kg*m^2" },
            Kind::Ratio => ":1",
        };
        unit(k, label).unwrap()
    }
}
