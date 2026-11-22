use crate::Wheel;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, f64::consts::TAU, sync::LazyLock};

pub const SECTORS: usize = 180;
type Bands = Vec<[f64; 2]>;
static CAD: LazyLock<HashMap<String, Vec<Bands>>> = LazyLock::new(|| {
    serde_json::from_str(include_str!("../../../assets/library/wheel-sections.json")).unwrap()
});

static ELASTIC: LazyLock<HashMap<String, Vec<f64>>> = LazyLock::new(|| {
    serde_json::from_str(include_str!("../../../assets/library/wheel-stiffness.json")).unwrap()
});

/// A set of full-width notches. Distances are in metres at the original wheel OD.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RimCut {
    pub depth: f64,
    pub width: f64,
    pub angle_deg: f64,
    pub repeat: u32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct WheelEdits {
    pub cuts: Vec<RimCut>,
    /// Flat plate load at 10% radial compression (one wheel, newtons).
    pub press_force: Option<f64>,
    pub pvc_friction: Option<[f64; 2]>,
}

#[derive(Clone, Debug)]
pub struct Section {
    pub bands: Vec<Bands>,
    pub irregular: bool,
    pub cad: Option<&'static str>,
    /// Local assembled support estimate, relative to solid stock.
    pub support: Vec<f64>,
}

fn cad_section(w: &Wheel) -> Option<&'static str> {
    match w.id.split('-').collect::<Vec<_>>().as_slice() {
            ["am", "compliant", "2in", _] => Some("am-3462"),
            ["am", "compliant", "3in", _] => Some("am-3945"),
            ["am", "compliant", "4in", _] => Some("am-3480"),
            ["am", "star", "3in", _] => Some("am-5740"),
            ["am", "star", "5in", _] => Some("am-5123"),
            _ => None,
    }
}

pub(crate) fn stock_stiffness(w: &Wheel) -> Option<f64> {
    let factors = ELASTIC.get(cad_section(w)?)?;
    Some(crate::stiffness::modulus_from_shore_a(w.durometer_a) / w.radius()
        * factors.iter().sum::<f64>() / factors.len() as f64)
}

/// Through-cuts can separate a tread segment completely from its spokes.
fn discard_detached(bands: &mut [Bands], bore: &[f64]) {
    use std::collections::VecDeque;
    let n = bands.len();
    let mut attached: Vec<Vec<bool>> = bands.iter().map(|b| vec![false; b.len()]).collect();
    let mut queue = VecDeque::new();
    for i in 0..n {
        if bands[i].first().is_some_and(|b| b[0] <= bore[i] + 1e-6) {
            attached[i][0] = true;
            queue.push_back((i,0));
        }
    }
    while let Some((i,j)) = queue.pop_front() {
        let [lo,hi] = bands[i][j];
        for next in [(i+1)%n,(i+n-1)%n] {
            for (k,&[a,b]) in bands[next].iter().enumerate() {
                if !attached[next][k] && a < hi && lo < b {
                    attached[next][k] = true;
                    queue.push_back((next,k));
                }
            }
        }
    }
    for (band, flags) in bands.iter_mut().zip(attached) {
        let mut flags = flags.into_iter();
        band.retain(|_| flags.next().unwrap());
    }
}

impl Section {
    pub fn new(w: &Wheel, edits: &WheelEdits) -> Self {
        let cad = cad_section(w);
        let mut bands = cad.map(|k| CAD[k].clone()).unwrap_or_else(|| vec![vec![[0.00635 / w.radius(), 1.]]; SECTORS]);
        let original: Vec<f64> = bands.iter().map(|band| band.iter().map(|b| b[1]-b[0]).sum()).collect();
        let bore: Vec<f64> = bands.iter().map(|band| band[0][0]).collect();
        let mut support = vec![1.; SECTORS];
        for (i, band) in bands.iter_mut().enumerate() {
            let a = TAU * (i as f64 + 0.5) / SECTORS as f64;
            let mut edge: f64 = 1.;
            for cut in &edits.cuts {
                let period = TAU / f64::from(cut.repeat.clamp(1, 32));
                let offset = ((a - cut.angle_deg.to_radians() + period / 2.).rem_euclid(period) - period / 2.).abs();
                let half_width = (cut.width / (2. * w.radius())).max(1e-6);
                let depth = (cut.depth / w.radius()).clamp(0., 0.75);
                edge = edge.min(1. - depth * (1. - offset / half_width).max(0.));
            }
            for b in band.iter_mut() { b[1] = b[1].min(edge); }
            band.retain(|b| b[0] < b[1]);
        }
        if !edits.cuts.is_empty() { discard_detached(&mut bands, &bore); }
        for (i, band) in bands.iter().enumerate() {
            let solid: f64 = band.iter().map(|b| b[1]-b[0]).sum();
            support[i] = if cad.is_some() {
                (solid / original[i].max(0.05)).clamp(0.05, 1.).powi(3)
            } else { (solid / (1.-bore[i]).max(0.05)).clamp(0.05, 1.).powi(3) };
        }
        // The connected rim spreads a contact load onto neighbouring spokes.
        let raw = support.clone();
        for (i, k) in support.iter_mut().enumerate() {
            *k = (0..15).map(|j| raw[(i+j+SECTORS-7)%SECTORS]).sum::<f64>() / 15.;
        }
        if let Some(key) = cad {
            for (k, fe) in support.iter_mut().zip(&ELASTIC[key]) { *k *= fe; }
        }
        let irregular = w.id.starts_with("am-star-") || !edits.cuts.is_empty();
        Self { bands, support, irregular, cad }
    }

    pub fn index(angle: f64) -> usize { ((angle.rem_euclid(TAU)/TAU * SECTORS as f64) as usize).min(SECTORS-1) }
    pub fn edge(&self, i: usize) -> f64 { self.bands[i].last().map_or(0., |b| b[1]) }
    pub fn mean_support(&self) -> f64 { self.support.iter().sum::<f64>() / SECTORS as f64 }
    pub fn stiffness(&self, w: &Wheel, edits: &WheelEdits) -> f64 {
        if let Some(f) = edits.press_force.filter(|f| f.is_finite() && *f > 0.) {
            return crate::stiffness::k_from_press(f, w.radius()*0.1, w.radius(), w.width());
        }
        if self.cad.is_some() && w.k_area.is_none() {
            // Plane-stress section solve, scaled from unit modulus to the stock elastomer.
            crate::stiffness::modulus_from_shore_a(w.durometer_a) / w.radius() * self.mean_support()
        } else { w.k_area() }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::Library;

    #[test]
    fn cad_pockets_and_cuts_change_the_profile_and_press_response() {
        let lib = Library::builtin();
        let stock = lib.wheel("am-compliant-4in-35a").unwrap();
        let original = Section::new(stock, &WheelEdits::default());
        assert!(original.bands.iter().any(|bands| bands.len() >= 2));
        let mut edits = WheelEdits { cuts: vec![RimCut { depth: stock.radius()*0.35,
            width: stock.radius()*0.4, repeat: 6, angle_deg: 0. }], ..Default::default() };
        let cut = Section::new(stock, &edits);
        assert!(cut.edge(0) < 0.75 && cut.edge(SECTORS/12) > 0.99);
        assert!(cut.stiffness(stock,&edits) < original.stiffness(stock,&WheelEdits::default()));
        edits.press_force = Some(23.);
        let k = cut.stiffness(stock, &edits);
        assert!((crate::stiffness::plate_force(k, stock.radius(), stock.width(), stock.radius()*0.1) - 23.).abs() < 1e-10);
        for size in [3,5] {
            let star = lib.wheel(&format!("am-star-{size}in-35a")).unwrap();
            let section = Section::new(star, &Default::default());
            assert!(section.irregular && section.cad.is_some());
            let radii = (0..SECTORS).map(|i| section.edge(i));
            assert!(radii.clone().fold(1.,f64::min) < 0.65 && radii.fold(0., f64::max) > 0.97);
        }
    }
}
