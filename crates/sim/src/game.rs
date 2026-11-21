//! Game packs (PLAN.md D11): the field's static shapes, where pieces start, feeders, scoring zones,
//! points and match timing, as data in `assets/games/<id>/game.json`.
use model::math::*;
use model::V3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GamePack {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub about: String,
    pub field: FieldSize,
    pub statics: Vec<Static>,
    #[serde(default)]
    pub pieces: Vec<Spawn>,
    #[serde(default)]
    pub feeders: Vec<Feeder>,
    #[serde(default)]
    pub zones: Vec<Zone>,
    pub timing: Timing,
    pub start: Start,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldSize { pub length: f64, pub width: f64, pub carpet_friction: f64 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Static {
    pub name: String,
    pub shape: StaticShape,
    pub pos: V3,
    #[serde(default)]
    pub euler_deg: V3,
    #[serde(default)]
    pub color: String,
    /// a hollow piece slides over it (a REEF branch)
    #[serde(default)]
    pub bore: bool,
    /// drawn but not solid (lines, marks)
    #[serde(default)]
    pub ghost: bool,
    /// also make the other alliance's copy, turned half round the field's centre
    #[serde(default, skip_serializing_if = "is_false")]
    pub mirror: bool,
    /// the copy's colour (else the same)
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub mirror_color: String,
}

fn is_false(b: &bool) -> bool { !*b }

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StaticShape { Box { size: V3 }, Cylinder { radius: f64, length: f64 } }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Spawn {
    pub piece: String,
    pub pos: V3,
    /// turn about the vertical, deg; and whether it stands on end (a tube)
    #[serde(default)]
    pub yaw_deg: f64,
    #[serde(default)]
    pub upright: bool,
    /// a block of pieces: this many along x and y, `spacing` apart (m), `pos` the first's place
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grid: Option<[u32; 2]>,
    #[serde(default)]
    pub spacing: [f64; 2],
    #[serde(default, skip_serializing_if = "is_false")]
    pub mirror: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Feeder {
    pub name: String,
    pub piece: String,
    pub alliance: String,
    /// where it drops a piece and which way it sends it
    pub pos: V3,
    pub dir: V3,
    pub speed: f64,
    /// the soonest it gives another, s
    pub interval: f64,
    /// how far a robot has to be for it to give one, m
    pub reach: f64,
    /// how many pieces it holds to start (pieces that leave the field come back to it)
    #[serde(default = "two")]
    pub stock: u32,
    #[serde(default, skip_serializing_if = "is_false")]
    pub mirror: bool,
}

fn two() -> u32 { 2 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Zone {
    pub name: String,
    pub alliance: String,
    pub piece: String,
    pub center: V3,
    pub size: V3,
    #[serde(default)]
    pub euler_deg: V3,
    pub auto_points: f64,
    pub teleop_points: f64,
    /// how many pieces it can count
    #[serde(default = "one")]
    pub capacity: u32,
    /// where a piece released close enough is seated (Concept's scoring assist), and how close
    #[serde(default)]
    pub seat: Option<Seat>,
    /// a goal that gives scored pieces back (a HUB): each is counted, then sent out from here
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exit: Option<Exit>,
    #[serde(default, skip_serializing_if = "is_false")]
    pub mirror: bool,
}

/// Where a goal sends its pieces back out: from `pos`, along `dir`, at `speed` m/s.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Exit { pub pos: V3, pub dir: V3, pub speed: f64 }

fn one() -> u32 { 1 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Seat { pub pos: V3, pub euler_deg: V3, pub tolerance: f64 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timing { pub auto: f64, pub teleop: f64 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Start { pub pos: [f64; 2], pub yaw_deg: f64 }

impl Zone {
    pub fn contains(&self, p: V3) -> bool {
        let q = qconj(euler_to_quat(self.euler_deg.map(f64::to_radians)));
        let l = qrot(q, sub(p, self.center));
        (0..3).all(|i| l[i].abs() <= self.size[i] / 2.0)
    }
}

/// "Blue" for "Red" and the other way round; "(red)" added when the name says neither.
fn other_side(s: &str) -> String {
    if s.contains("Blue") { s.replace("Blue", "Red") } else if s.contains("Red") { s.replace("Red", "Blue") }
    else if s.contains("blue") { s.replace("blue", "red") } else if s.contains("red") { s.replace("red", "blue") }
    else if s.is_empty() { s.into() } else { format!("{s} (red)") }
}

/// The packs that come with the app: (id, game.json)
const BUILTIN: [(&str, &str); 2] = [
    ("reefscape-2025", include_str!("../../../assets/games/reefscape-2025/game.json")),
    ("rebuilt-2026", include_str!("../../../assets/games/rebuilt-2026/game.json")),
];

/// A game pack and where it came from: the app (no folder) or a folder of the user's, which may
/// also hold the game's own pieces (`pieces.json`, the library's format).
#[derive(Debug, Clone)]
pub struct Found { pub pack: GamePack, pub dir: Option<std::path::PathBuf> }

impl GamePack {
    pub fn builtin(id: &str) -> Option<GamePack> {
        let s = BUILTIN.iter().find(|b| b.0 == id)?.1;
        GamePack::parse(s).map_err(|e| eprintln!("game pack {id}: {e}")).ok()
    }
    /// From game.json's text, with where it went wrong in words a person can act on.
    pub fn parse(s: &str) -> Result<GamePack, String> {
        let mut g: GamePack = serde_json::from_str(s).map_err(|e| format!("game.json line {}, column {}: {}", e.line(), e.column(), e))?;
        g.expand();
        Ok(g)
    }

    /// Makes what the pack asks to be made: blocks of pieces from `grid`, and the other alliance's
    /// copy of everything marked `mirror` (turned half round the field's centre, Blue named Red).
    pub fn expand(&mut self) {
        let mut spawns = vec![];
        for p in self.pieces.drain(..) {
            let [nx, ny] = p.grid.unwrap_or([1, 1]);
            for i in 0..nx.max(1) { for j in 0..ny.max(1) {
                spawns.push(Spawn { pos: [p.pos[0] + i as f64 * p.spacing[0], p.pos[1] + j as f64 * p.spacing[1], p.pos[2]], grid: None, ..p.clone() });
            } }
        }
        let turn = |v: V3| [-v[0], -v[1], v[2]];
        let yaw = |e: V3| [e[0], e[1], e[2] + 180.0];
        let mut extra = vec![];
        for p in spawns.iter().filter(|p| p.mirror) { extra.push(Spawn { pos: turn(p.pos), yaw_deg: p.yaw_deg + 180.0, mirror: false, ..p.clone() }); }
        spawns.extend(extra);
        self.pieces = spawns;
        let statics: Vec<Static> = self.statics.iter().filter(|x| x.mirror).map(|x| Static {
            name: other_side(&x.name), pos: turn(x.pos), euler_deg: yaw(x.euler_deg), mirror: false,
            color: if x.mirror_color.is_empty() { x.color.clone() } else { x.mirror_color.clone() }, mirror_color: String::new(), ..x.clone()
        }).collect();
        self.statics.extend(statics);
        let feeders: Vec<Feeder> = self.feeders.iter().filter(|x| x.mirror).map(|x| Feeder {
            name: other_side(&x.name), alliance: other_side(&x.alliance), pos: turn(x.pos), dir: turn(x.dir), mirror: false, ..x.clone()
        }).collect();
        self.feeders.extend(feeders);
        let zones: Vec<Zone> = self.zones.iter().filter(|x| x.mirror).map(|x| Zone {
            name: other_side(&x.name), alliance: other_side(&x.alliance), center: turn(x.center), euler_deg: yaw(x.euler_deg), mirror: false,
            seat: x.seat.as_ref().map(|s| Seat { pos: turn(s.pos), euler_deg: yaw(s.euler_deg), tolerance: s.tolerance }),
            exit: x.exit.as_ref().map(|e| Exit { pos: turn(e.pos), dir: turn(e.dir), speed: e.speed }),
            ..x.clone()
        }).collect();
        self.zones.extend(zones);
        for x in &mut self.statics { x.mirror = false; }
        for x in &mut self.feeders { x.mirror = false; }
        for x in &mut self.zones { x.mirror = false; }
        for x in &mut self.pieces { x.mirror = false; }
    }
    pub fn load(dir: &std::path::Path) -> Result<GamePack, String> {
        let s = std::fs::read_to_string(dir.join("game.json")).map_err(|e| format!("{}: {e}", dir.join("game.json").display()))?;
        GamePack::parse(&s)
    }
    pub fn ids() -> Vec<&'static str> { BUILTIN.iter().map(|b| b.0).collect() }

    /// Things in the pack that will go wrong or surprise, each a sentence saying what and where.
    /// `lib` must already hold the pack's own pieces.
    pub fn check(&self, lib: &library::Library) -> Vec<String> {
        let mut out = vec![];
        let (hx, hy) = (self.field.length / 2.0, self.field.width / 2.0);
        let off = |p: V3| p[0].abs() > hx + 1.5 || p[1].abs() > hy + 1.5;
        let piece = |id: &str, what: &str, out: &mut Vec<String>| if lib.piece(id).is_none() { out.push(format!("{what}: no game piece '{id}' (add it to the pack's pieces.json, or use one in the library)")); };
        if self.field.length <= 0.0 || self.field.width <= 0.0 { out.push("field: length and width must be more than 0".into()); }
        for s in &self.statics {
            let bad = match &s.shape { StaticShape::Box { size } => size.iter().any(|x| *x <= 0.0), StaticShape::Cylinder { radius, length } => *radius <= 0.0 || *length <= 0.0 };
            if bad { out.push(format!("'{}': every size must be more than 0", s.name)); }
            if off(s.pos) { out.push(format!("'{}' is off the field, at {:?}", s.name, s.pos)); }
        }
        for (i, p) in self.pieces.iter().enumerate() {
            piece(&p.piece, &format!("pieces[{i}]"), &mut out);
            if off(p.pos) { out.push(format!("pieces[{i}] ({}) is off the field", p.piece)); }
        }
        for f in &self.feeders {
            piece(&f.piece, &format!("feeder '{}'", f.name), &mut out);
            if len(f.dir) < 1e-6 { out.push(format!("feeder '{}': dir can't be 0, 0, 0", f.name)); }
            if f.interval <= 0.0 || f.reach <= 0.0 { out.push(format!("feeder '{}': interval and reach must be more than 0", f.name)); }
        }
        let mut names = std::collections::HashSet::new();
        for z in &self.zones {
            piece(&z.piece, &format!("zone '{}'", z.name), &mut out);
            if z.size.iter().any(|x| *x <= 0.0) { out.push(format!("zone '{}': every size must be more than 0", z.name)); }
            if z.capacity == 0 { out.push(format!("zone '{}': capacity 0 can never score", z.name)); }
            if z.seat.as_ref().is_some_and(|s| s.tolerance <= 0.0) { out.push(format!("zone '{}': the seat's tolerance must be more than 0", z.name)); }
            if !names.insert(z.name.clone()) { out.push(format!("zone '{}' is named twice: the cycle log won't tell them apart", z.name)); }
            if off(z.center) { out.push(format!("zone '{}' is off the field", z.name)); }
        }
        if self.zones.is_empty() { out.push("no scoring zones: nothing will score".into()); }
        if self.start.pos[0].abs() > hx || self.start.pos[1].abs() > hy { out.push("start: the robot starts off the field".into()); }
        if self.timing.auto < 0.0 || self.timing.teleop <= 0.0 { out.push("timing: auto can't be negative and teleop must be more than 0".into()); }
        out
    }
}

/// The pack named `id`: one that comes with the app, else a folder under one of `dirs` whose
/// game.json has that id.
pub fn find(id: &str, dirs: &[std::path::PathBuf]) -> Result<Found, String> {
    for (pid, dir, r) in list(dirs) {
        if pid == id { return r.map(|pack| Found { pack, dir }); }
    }
    Err(format!("no game pack '{id}': put its folder in {}", dirs.first().map_or("the game packs folder".into(), |d| d.display().to_string())))
}

/// Every pack the app can see: (id, folder, the pack or what's wrong with it). Folders whose
/// game.json won't read are listed by folder name with the reason.
pub fn list(dirs: &[std::path::PathBuf]) -> Vec<(String, Option<std::path::PathBuf>, Result<GamePack, String>)> {
    let mut out: Vec<(String, Option<std::path::PathBuf>, Result<GamePack, String>)> = BUILTIN.iter().map(|b| (b.0.to_string(), None, GamePack::parse(b.1))).collect();
    for d in dirs {
        let Ok(rd) = std::fs::read_dir(d) else { continue };
        let mut folders: Vec<std::path::PathBuf> = rd.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.join("game.json").is_file()).collect();
        folders.sort();
        for f in folders {
            let r = GamePack::load(&f);
            let id = r.as_ref().map(|g| g.id.clone()).unwrap_or_else(|_| f.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default());
            // a folder of the user's with the same id as a built-in one replaces it
            out.retain(|x| x.0 != id);
            out.push((id, Some(f), r));
        }
    }
    out
}

/// The match: phase, clock, points and what each scoring zone holds.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Match {
    pub running: bool,
    pub time: f64,
    pub score: f64,
    pub filled: Vec<u32>,
    pub log: Vec<Event>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event { pub time: f64, pub what: String, pub points: f64 }

impl Match {
    pub fn phase(&self, t: &Timing) -> &'static str {
        if !self.running { "Practice" } else if self.time < t.auto { "Autonomous" } else if self.time < t.auto + t.teleop { "Teleop" } else { "Over" }
    }
    pub fn remaining(&self, t: &Timing) -> f64 {
        if self.time < t.auto { t.auto - self.time } else { (t.auto + t.teleop - self.time).max(0.0) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir(name: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("fps-games-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn the_built_in_packs_read_and_have_nothing_to_fix() {
        let lib = library::Library::builtin();
        for id in GamePack::ids() {
            let g = GamePack::builtin(id).unwrap_or_else(|| panic!("{id} doesn't read"));
            assert!(g.check(&lib).is_empty(), "{id}: {:?}", g.check(&lib));
        }
    }

    #[test]
    fn a_pack_from_a_folder_with_its_own_piece() {
        let root = dir("own");
        let pack = root.join("my-game");
        std::fs::create_dir_all(&pack).unwrap();
        std::fs::write(pack.join("pieces.json"), r##"{ "pieces": [ { "id": "widget-2027", "name": "WIDGET", "game": "2027", "shape": { "sphere": { "d_in": 7.0 } }, "mass_kg": 0.3, "surface": "foam", "law": "rigid", "color": "#ff8800", "source": "test", "measured": false } ] }"##).unwrap();
        std::fs::write(pack.join("game.json"), r#"{ "id": "my-game", "name": "My game", "field": { "length": 16, "width": 8, "carpet_friction": 1.0 },
            "statics": [ { "name": "Goal", "shape": { "box": { "size": [1, 1, 1] } }, "pos": [3, 0, 0.5] } ],
            "pieces": [ { "piece": "widget-2027", "pos": [1, 0, 0.1] } ],
            "zones": [ { "name": "Goal", "alliance": "blue", "piece": "widget-2027", "center": [3, 0, 1.2], "size": [1, 1, 0.4], "auto_points": 2, "teleop_points": 1 } ],
            "timing": { "auto": 15, "teleop": 135 }, "start": { "pos": [-6, 0], "yaw_deg": 0 } }"#).unwrap();
        let f = find("my-game", &[root.clone()]).unwrap();
        assert_eq!(f.dir.as_deref(), Some(pack.as_path()));
        let mut lib = library::Library::builtin();
        assert!(!f.pack.check(&lib).is_empty(), "without its pieces the WIDGET is unknown");
        lib.load_overlay(&pack).unwrap();
        assert!(f.pack.check(&lib).is_empty(), "{:?}", f.pack.check(&lib));
        let w = crate::world::World::new(&model::Project::new("t"), &lib, crate::world::Scene::field(&f.pack)).unwrap();
        assert_eq!(w.piece_positions().len(), 1);
        assert!(list(&[root]).iter().any(|x| x.0 == "my-game"));
    }

    #[test]
    fn the_new_game_template_reads_clean() {
        let root = dir("template");
        let t = root.join("new-game");
        std::fs::create_dir_all(&t).unwrap();
        std::fs::write(t.join("game.json"), include_str!("../../../assets/games/template/game.json")).unwrap();
        std::fs::write(t.join("pieces.json"), include_str!("../../../assets/games/template/pieces.json")).unwrap();
        let f = find("new-game", &[root]).unwrap();
        let mut lib = library::Library::builtin();
        lib.load_overlay(&t).unwrap();
        assert!(f.pack.check(&lib).is_empty(), "{:?}", f.pack.check(&lib));
        assert_eq!(f.pack.zones.len(), 2);
        let w = crate::world::World::new(&model::Project::new("t"), &lib, crate::world::Scene::field(&f.pack)).unwrap();
        assert!(w.engine.finite());
    }

    #[test]
    fn a_broken_pack_says_where() {
        let root = dir("broken");
        std::fs::create_dir_all(root.join("oops")).unwrap();
        std::fs::write(root.join("oops/game.json"), "{\n  \"id\": \"oops\",\n  \"name\": \"Oops\"\n  \"field\": {}\n}").unwrap();
        let l = list(&[root]);
        let (_, _, r) = l.iter().find(|x| x.0 == "oops").unwrap();
        let e = r.as_ref().unwrap_err();
        assert!(e.contains("line 4"), "{e}");
    }
}
