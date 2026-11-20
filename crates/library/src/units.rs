//! Number fields that accept units and arithmetic (PLAN.md D18): `1.75in`, `30/11`, `6000rpm`,
//! `2ft + 3in`, `12 * 0.0254`. Everything inside the simulator is SI; a field knows its kind and the
//! unit to show it in, and a bare number is read in that unit.
use crate::{IN, LB, LBF, PSI, RPM};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Kind {
    Length,
    Mass,
    Angle,
    AngularSpeed,
    Speed,
    Force,
    Torque,
    Current,
    Voltage,
    Pressure,
    Time,
    Inertia,
    Ratio,
}

/// One way of showing a kind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Unit {
    pub kind: Kind,
    pub label: &'static str,
    /// SI per one of this unit
    pub scale: f64,
}

pub const UNITS: &[Unit] = &[
    Unit { kind: Kind::Length, label: "m", scale: 1.0 },
    Unit { kind: Kind::Length, label: "mm", scale: 1e-3 },
    Unit { kind: Kind::Length, label: "cm", scale: 1e-2 },
    Unit { kind: Kind::Length, label: "in", scale: IN },
    Unit { kind: Kind::Length, label: "\"", scale: IN },
    Unit { kind: Kind::Length, label: "ft", scale: 12.0 * IN },
    Unit { kind: Kind::Length, label: "'", scale: 12.0 * IN },
    Unit { kind: Kind::Mass, label: "kg", scale: 1.0 },
    Unit { kind: Kind::Mass, label: "g", scale: 1e-3 },
    Unit { kind: Kind::Mass, label: "lb", scale: LB },
    Unit { kind: Kind::Mass, label: "lbs", scale: LB },
    Unit { kind: Kind::Mass, label: "oz", scale: LB / 16.0 },
    Unit { kind: Kind::Angle, label: "deg", scale: std::f64::consts::PI / 180.0 },
    Unit { kind: Kind::Angle, label: "°", scale: std::f64::consts::PI / 180.0 },
    Unit { kind: Kind::Angle, label: "rad", scale: 1.0 },
    Unit { kind: Kind::Angle, label: "rot", scale: std::f64::consts::TAU },
    Unit { kind: Kind::AngularSpeed, label: "rpm", scale: RPM },
    Unit { kind: Kind::AngularSpeed, label: "rad/s", scale: 1.0 },
    Unit { kind: Kind::AngularSpeed, label: "deg/s", scale: std::f64::consts::PI / 180.0 },
    Unit { kind: Kind::AngularSpeed, label: "rps", scale: std::f64::consts::TAU },
    Unit { kind: Kind::Speed, label: "m/s", scale: 1.0 },
    Unit { kind: Kind::Speed, label: "ft/s", scale: 12.0 * IN },
    Unit { kind: Kind::Speed, label: "in/s", scale: IN },
    Unit { kind: Kind::Speed, label: "mph", scale: 0.44704 },
    Unit { kind: Kind::Force, label: "N", scale: 1.0 },
    Unit { kind: Kind::Force, label: "lbf", scale: LBF },
    Unit { kind: Kind::Torque, label: "N*m", scale: 1.0 },
    Unit { kind: Kind::Torque, label: "Nm", scale: 1.0 },
    Unit { kind: Kind::Torque, label: "in*lbf", scale: IN * LBF },
    Unit { kind: Kind::Torque, label: "ft*lbf", scale: 12.0 * IN * LBF },
    Unit { kind: Kind::Current, label: "A", scale: 1.0 },
    Unit { kind: Kind::Voltage, label: "V", scale: 1.0 },
    Unit { kind: Kind::Pressure, label: "psi", scale: PSI },
    Unit { kind: Kind::Pressure, label: "kPa", scale: 1e3 },
    Unit { kind: Kind::Pressure, label: "Pa", scale: 1.0 },
    Unit { kind: Kind::Time, label: "s", scale: 1.0 },
    Unit { kind: Kind::Time, label: "ms", scale: 1e-3 },
    Unit { kind: Kind::Inertia, label: "kg*m^2", scale: 1.0 },
    Unit { kind: Kind::Inertia, label: "lb*in^2", scale: LB * IN * IN },
    Unit { kind: Kind::Ratio, label: ":1", scale: 1.0 },
    Unit { kind: Kind::Ratio, label: "x", scale: 1.0 },
];

/// The units of one kind, in the order a picker shows them.
pub fn units_of(kind: Kind) -> impl Iterator<Item = &'static Unit> {
    UNITS.iter().filter(move |u| u.kind == kind && !matches!(u.label, "\"" | "'" | "lbs" | "°" | "Nm" | "x"))
}

pub fn unit(kind: Kind, label: &str) -> Option<&'static Unit> {
    UNITS.iter().find(|u| u.kind == kind && u.label.eq_ignore_ascii_case(label))
        .or_else(|| UNITS.iter().find(|u| u.kind == kind && u.label == label))
}

/// A value in SI shown in `unit`, trimmed to a sensible number of digits.
pub fn format(si: f64, unit: &Unit) -> String {
    let v = si / unit.scale;
    let label = if unit.label == ":1" { ":1" } else { unit.label };
    let sep = if matches!(label, ":1" | "\"" | "'" | "°") { "" } else { " " };
    format!("{}{}{}", trim(v), sep, label)
}

/// A number with up to 4 significant digits after the point, no trailing zeros.
pub fn trim(v: f64) -> String {
    if !v.is_finite() { return format!("{v}"); }
    let a = v.abs();
    let decimals = if a >= 1000.0 { 1 } else if a >= 100.0 { 2 } else if a >= 1.0 { 3 } else if a >= 0.01 { 4 } else { 6 };
    let s = format!("{v:.decimals$}");
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    if s == "-0" { "0".into() } else { s }
}

/// Reads what a person typed into a field of `kind`, a bare number taken in `default`.
pub fn parse(input: &str, kind: Kind, default: &Unit) -> Result<f64, String> {
    let toks = tokenize(input, kind)?;
    if toks.is_empty() { return Err("empty".into()); }
    let mut p = Parser { toks, i: 0, kind, default: default.scale };
    let v = p.expr()?;
    if p.i != p.toks.len() { return Err(format!("could not read '{input}'")); }
    if !v.is_finite() { return Err("not a finite number".into()); }
    Ok(v)
}

#[derive(Debug, Clone, PartialEq)]
enum Tok { Num(f64, Option<f64>), Op(char), Open, Close }

fn tokenize(s: &str, kind: Kind) -> Result<Vec<Tok>, String> {
    let c: Vec<char> = s.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < c.len() {
        let ch = c[i];
        if ch.is_whitespace() { i += 1; continue; }
        if ch.is_ascii_digit() || ch == '.' {
            let start = i;
            while i < c.len() && (c[i].is_ascii_digit() || c[i] == '.' || ((c[i] == 'e' || c[i] == 'E') && i + 1 < c.len() && (c[i + 1].is_ascii_digit() || c[i + 1] == '-'))) {
                if c[i] == 'e' || c[i] == 'E' { i += 1; }
                i += 1;
            }
            let num: String = c[start..i].iter().collect();
            let v: f64 = num.parse().map_err(|_| format!("'{num}' is not a number"))?;
            // a unit straight after the number (spaces allowed), longest label first
            let mut j = i;
            while j < c.len() && c[j] == ' ' { j += 1; }
            let rest: String = c[j..].iter().collect();
            let mut best: Option<&Unit> = None;
            for u in UNITS.iter().filter(|u| u.kind == kind) {
                if rest.to_lowercase().starts_with(&u.label.to_lowercase()) && best.is_none_or(|b| u.label.len() > b.label.len()) {
                    // a unit's label must end the word (so "m" doesn't eat "mm")
                    let after = rest.chars().nth(u.label.chars().count());
                    if after.is_none_or(|a| !a.is_alphabetic()) { best = Some(u); }
                }
            }
            match best {
                Some(u) => { out.push(Tok::Num(v, Some(u.scale))); i = j + u.label.chars().count(); }
                None => out.push(Tok::Num(v, None)),
            }
            continue;
        }
        match ch {
            '+' | '-' | '*' | '/' => out.push(Tok::Op(ch)),
            '(' => out.push(Tok::Open),
            ')' => out.push(Tok::Close),
            ':' => { // "30:1" is a ratio: read as 30 / 1
                out.push(Tok::Op('/'));
            }
            _ => {
                let word: String = c[i..].iter().take_while(|x| x.is_alphanumeric() || **x == '/' || **x == '*' || **x == '^').collect();
                return Err(format!("unknown unit '{word}' for a {}", kind_name(kind)));
            }
        }
        i += 1;
    }
    Ok(out)
}

struct Parser { toks: Vec<Tok>, i: usize, kind: Kind, default: f64 }

/// a value and whether it carries a unit (a ratio between two lengths has none)
type V = (f64, bool);

impl Parser {
    fn expr(&mut self) -> Result<f64, String> {
        let (v, has) = self.sum()?;
        Ok(if has { v } else { v * self.default })
    }
    fn sum(&mut self) -> Result<V, String> {
        let mut a = self.product()?;
        while let Some(Tok::Op(op)) = self.toks.get(self.i).cloned() {
            if op != '+' && op != '-' { break; }
            self.i += 1;
            let b = self.product()?;
            // mixing a bare number with a unit: the bare one is in the default unit
            let (x, y) = match (a.1, b.1) {
                (true, false) => (a.0, b.0 * self.default),
                (false, true) => (a.0 * self.default, b.0),
                _ => (a.0, b.0),
            };
            a = (if op == '+' { x + y } else { x - y }, a.1 || b.1);
        }
        Ok(a)
    }
    fn product(&mut self) -> Result<V, String> {
        let mut a = self.unary()?;
        while let Some(Tok::Op(op)) = self.toks.get(self.i).cloned() {
            if op != '*' && op != '/' { break; }
            self.i += 1;
            let b = self.unary()?;
            a = if op == '*' { (a.0 * b.0, a.1 != b.1 || (a.1 && b.1 && self.kind == Kind::Ratio)) }
                else { if b.0 == 0.0 { return Err("divide by zero".into()); } (a.0 / b.0, a.1 && !b.1) };
        }
        Ok(a)
    }
    fn unary(&mut self) -> Result<V, String> {
        match self.toks.get(self.i).cloned() {
            Some(Tok::Op('-')) => { self.i += 1; let v = self.unary()?; Ok((-v.0, v.1)) }
            Some(Tok::Op('+')) => { self.i += 1; self.unary() }
            Some(Tok::Num(v, u)) => { self.i += 1; Ok(match u { Some(s) => (v * s, true), None => (v, false) }) }
            Some(Tok::Open) => {
                self.i += 1;
                let v = self.sum()?;
                if self.toks.get(self.i) != Some(&Tok::Close) { return Err("missing ')'".into()); }
                self.i += 1;
                Ok(v)
            }
            _ => Err("expected a number".into()),
        }
    }
}

pub fn kind_name(k: Kind) -> &'static str {
    match k {
        Kind::Length => "length", Kind::Mass => "mass", Kind::Angle => "angle", Kind::AngularSpeed => "speed",
        Kind::Speed => "speed", Kind::Force => "force", Kind::Torque => "torque", Kind::Current => "current",
        Kind::Voltage => "voltage", Kind::Pressure => "pressure", Kind::Time => "time", Kind::Inertia => "inertia",
        Kind::Ratio => "ratio",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn len(s: &str) -> f64 { parse(s, Kind::Length, unit(Kind::Length, "in").unwrap()).unwrap() }

    #[test]
    fn units_and_arithmetic() {
        assert!((len("1.75in") - 1.75 * IN).abs() < 1e-12);
        assert!((len("1.75") - 1.75 * IN).abs() < 1e-12);
        assert!((len("2ft + 3in") - 27.0 * IN).abs() < 1e-12);
        assert!((len("30mm") - 0.03).abs() < 1e-12);
        assert!((len("1 m") - 1.0).abs() < 1e-12);
        assert!((len("2 * 3in") - 6.0 * IN).abs() < 1e-12);
        assert!((len("(1+1)/4 * 12in") - 6.0 * IN).abs() < 1e-9);
        let ratio = |s: &str| parse(s, Kind::Ratio, unit(Kind::Ratio, ":1").unwrap()).unwrap();
        assert!((ratio("30/11") - 30.0 / 11.0).abs() < 1e-12);
        assert!((ratio("5*4") - 20.0).abs() < 1e-12);
        assert!((ratio("30:1") - 30.0).abs() < 1e-12);
        let rpm = parse("6000rpm", Kind::AngularSpeed, unit(Kind::AngularSpeed, "rad/s").unwrap()).unwrap();
        assert!((rpm - 6000.0 * RPM).abs() < 1e-9);
        assert!(parse("3 parsecs", Kind::Length, unit(Kind::Length, "m").unwrap()).is_err());
    }

    #[test]
    fn formats() {
        assert_eq!(format(0.0254 * 1.75, unit(Kind::Length, "in").unwrap()), "1.75 in");
        assert_eq!(format(2.5, unit(Kind::Ratio, ":1").unwrap()), "2.5:1");
        assert_eq!(trim(-0.0), "0");
    }
}
