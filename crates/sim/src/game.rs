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
}

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
}

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
}

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

impl GamePack {
    pub fn builtin(id: &str) -> Option<GamePack> {
        let s = match id {
            "reefscape-2025" => include_str!("../../../assets/games/reefscape-2025/game.json"),
            _ => return None,
        };
        serde_json::from_str(s).map_err(|e| eprintln!("game pack {id}: {e}")).ok()
    }
    pub fn load(dir: &std::path::Path) -> anyhow::Result<GamePack> {
        Ok(serde_json::from_str(&std::fs::read_to_string(dir.join("game.json"))?)?)
    }
    pub fn ids() -> Vec<&'static str> { vec!["reefscape-2025"] }
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
