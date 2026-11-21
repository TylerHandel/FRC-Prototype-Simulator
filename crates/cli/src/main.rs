//! fps: the simulator with no window (PLAN.md layout, `cli`). For checking a project, running the
//! Bench and sweeps on a server, and importing CAD from a script.
//!
//!   fps check  <project-dir>                     what's missing, then two seconds of driving
//!   fps bench  <project-dir> [wheel-id ...]      one throw per wheel (the project's own if none)
//!   fps sweep  <project-dir> speed|volts|limit|yaw <v1,v2,...> [throws]
//!   fps import <onshape-link|file.step> <project-dir> [--forward +X] [--up +Z]
//!   fps wheels                                   the wheel library
use library::Library;
use model::Project;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

fn load(dir: &Path) -> anyhow::Result<Project> { Project::from_json(&std::fs::read_to_string(dir.join("robot.json"))?) }

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let lib = Library::builtin();
    match args.first().map(String::as_str) {
        Some("check") => {
            let p = load(Path::new(&args[1]))?;
            for i in model::validate::issues(&p.robot, &lib) { println!("{:?}: {}", i.severity, i.message); }
            let mut w = sim::World::new(&p, &lib, sim::Scene::carpet())?;
            w.cmd.drive = [2.0, 0.0, 0.0];
            while w.time < 2.0 { w.step(); }
            let s = w.snapshot();
            println!("after 2 s: {:.2} m/s, battery {:.2} V, tip margin {:.3} m, {}", s.speed, s.bus_voltage, s.tip_margin, if w.engine.finite() { "stable" } else { "BLEW UP" });
        }
        Some("bench") => {
            let p = load(Path::new(&args[1]))?;
            let wheels: Vec<String> = args[2..].to_vec();
            let trials = if wheels.is_empty() { vec![sim::bench::trial(&p, &lib, None, p.bench.seed)] } else { sim::bench::compare(&p, &lib, &wheels) };
            for t in trials {
                let t = t?;
                println!("{:<45} {:<6} t_in {:>6} mean {:5.1} A peak {:5.1} A  squeeze {:4.0} N (peak {:4.0})  rpm {:4.0}->{:4.0}  slip {:3.0}%",
                    t.label, if t.acquired { "IN" } else if t.rejected { "OUT" } else { "STUCK" }, t.time_in.map_or("-".into(), |x| format!("{x:.2}s")),
                    t.mean_current, t.peak_current, t.mean_squeeze, t.peak_squeeze, t.start_roller_rpm, t.min_roller_rpm, t.slip * 100.0);
            }
        }
        Some("sweep") => {
            let p = load(Path::new(&args[1]))?;
            let vals: Vec<f64> = args[3].split(',').filter_map(|s| s.trim().parse().ok()).collect();
            let axis = match args[2].as_str() {
                "speed" => sim::bench::Axis::Speed(vals), "volts" => sim::bench::Axis::Volts(vals),
                "limit" => sim::bench::Axis::StatorLimit(vals), "yaw" => sim::bench::Axis::Yaw(vals),
                other => anyhow::bail!("no sweep axis '{other}'"),
            };
            let throws = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(5);
            let t0 = std::time::Instant::now();
            for c in sim::bench::sweep(&p, &lib, &[axis], throws) {
                println!("{:<14} in {}/{}  t {:>5}  peak {:5.1} A  squeeze {:4.0} N", c.labels.join(","), c.acquired, c.trials, if c.mean_time.is_finite() { format!("{:.2}", c.mean_time) } else { "-".into() }, c.mean_peak_current, c.mean_peak_squeeze);
            }
            println!("({:.1} s)", t0.elapsed().as_secs_f64());
        }
        Some("import") => {
            let src = &args[1];
            let dir = PathBuf::from(&args[2]);
            let get = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
            let dir_of = |s: Option<String>, d| s.and_then(|s| cad::cadpack::Dir::ALL.into_iter().find(|x| x.label().eq_ignore_ascii_case(&s))).unwrap_or(d);
            let forward = dir_of(get("--forward"), cad::cadpack::Dir::PX);
            let up = dir_of(get("--up"), cad::cadpack::Dir::PZ);
            let source = if src.starts_with("http") {
                let access = std::env::var("ONSHAPE_ACCESS_KEY").map_err(|_| anyhow::anyhow!("set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY"))?;
                let secret = std::env::var("ONSHAPE_SECRET_KEY")?;
                cad::import::Source::Onshape { url: src.clone(), access, secret, second_pose: args.iter().any(|a| a == "--second-pose") }
            } else { cad::import::Source::Step { path: src.into() } };
            let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/cadread/cadread.py");
            let python = std::env::var("FPS_CADREAD_PYTHON").unwrap_or_else(|_| "python3".into());
            let sc = cad::import::Sidecar { program: python.into(), args: vec!["-I".into(), script.display().to_string()] };
            let budget = dirs::config_dir().unwrap_or_default().join("FRC Prototype Simulator").join("onshape-calls.json");
            let t0 = std::time::Instant::now();
            let im = cad::import::run(&source, &dir, &sc, budget, &|p| match p {
                cad::import::Progress::Step(s) => eprintln!("{s}"),
                cad::import::Progress::Fraction(f, s) => if (f * 100.0) as i64 % 10 == 0 { eprintln!("{:3.0}% {s}", f * 100.0) },
            })?;
            let (to, prop) = cad::import::proposals(&im, &lib, forward, up);
            println!("{} parts read in {:.0} s; links from {}", im.cadpack.parts.len(), t0.elapsed().as_secs_f64(), prop.links_from);
            for l in &prop.links { println!("  link {:<40} {} parts", l.name, l.parts.len()); }
            for j in &prop.joints { println!("  joint {:<50} {:?} axis [{:.2} {:.2} {:.2}] at [{:.3} {:.3} {:.3}] range {:?}", j.name, j.kind, j.axis[0], j.axis[1], j.axis[2], j.anchor[0], j.anchor[1], j.anchor[2], j.range); }
            for r in &prop.rollers { println!("  roller {} on link {} shaft {} ({:.2} in, {:.2} in wide)", r.name, r.link, r.shaft, r.diameter / library::IN, r.width / library::IN); }
            println!("  module {:?}, {} drive wheels, {} motors", prop.module, prop.module_positions.len(), prop.motors.len());
            for n in &prop.notes { println!("  note: {n}"); }
            let mut p = if dir.join("robot.json").exists() { load(&dir)? } else { Project::new(dir.file_name().map_or("robot".into(), |s| s.to_string_lossy().to_string()).as_str()) };
            cad::propose::apply(&mut p.robot, &im.cadpack, &to, &prop, &lib);
            p.robot.cad = Some(model::CadSource { kind: im.source.clone(), cadpack: "cad/pack".into(), imported: "cli".into(), to_robot: to });
            std::fs::write(dir.join("robot.json"), p.to_json())?;
            println!("saved {}", dir.join("robot.json").display());
        }
        Some("propose") => {
            // fps propose <cadpack-dir> [--bom onshape-bom.json] [--second <cadpack-dir>] [--mates assembly.json]
            let get = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
            let cp = cad::Cadpack::load(Path::new(&args[1]))?;
            let bom = get("--bom").map(|f| cad::onshape::parse_bom(&serde_json::from_str(&std::fs::read_to_string(f).unwrap()).unwrap())).unwrap_or_default();
            let second = get("--second").map(|d| cad::Cadpack::load(Path::new(&d))).transpose()?;
            let assembly = get("--mates").map(|f| serde_json::from_str(&std::fs::read_to_string(f).unwrap()).unwrap());
            let im = cad::import::Imported { dir: PathBuf::from(&args[1]), cadpack: cp, second, assembly, bom, source: model::CadKind::Step { path: args[1].clone() }, notes: vec![], calls: 0 };
            let t0 = std::time::Instant::now();
            let fwd = get("--forward").and_then(|s| cad::cadpack::Dir::ALL.into_iter().find(|x| x.label().eq_ignore_ascii_case(&s))).unwrap_or(cad::cadpack::Dir::PX);
            let (to, prop) = cad::import::proposals(&im, &lib, fwd, cad::cadpack::Dir::PZ);
            println!("proposed in {:.2} s from {}", t0.elapsed().as_secs_f64(), prop.links_from);
            for l in &prop.links { println!("  link {:<45} {:>4} parts", l.name, l.parts.len()); }
            for j in &prop.joints { println!("  joint {:<45} {:?} parent {} axis [{:.2} {:.2} {:.2}] at [{:.3} {:.3} {:.3}] range {:?}", j.name, j.kind, j.parent, j.axis[0], j.axis[1], j.axis[2], j.anchor[0], j.anchor[1], j.anchor[2], j.range.map(|r| [(r[0] * 1000.0).round() / 1000.0, (r[1] * 1000.0).round() / 1000.0])); }
            for r in &prop.rollers { println!("  roller {:<40} link {} shaft {} {:.2} in x {:.2} in at [{:.3} {:.3} {:.3}]", r.name, r.link, r.shaft, r.diameter / library::IN, r.width / library::IN, r.center[0], r.center[1], r.center[2]); }
            println!("  module {:?}, drive wheels {:?}", prop.module, prop.module_positions);
            let mut m: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
            for (_, id) in &prop.motors { *m.entry(id.clone()).or_default() += 1; }
            println!("  motors {m:?}");
            let total: f64 = prop.masses.values().map(|x| x.1).sum();
            println!("  total mass {:.1} kg ({:.1} lb)", total, total / library::LB);
            if std::env::var("FPS_HEAVY").is_ok() {
                let names: HashMap<&str, &str> = im.cadpack.parts.iter().map(|p| (p.id.as_str(), p.name.as_str())).collect();
                let mut h: Vec<_> = prop.masses.iter().collect();
                h.sort_by(|a, b| b.1.1.total_cmp(&a.1.1));
                for (id, (mat, kg)) in h.iter().take(25) { println!("    {:8.2} kg  {:?}  {}", kg, mat, names.get(id.as_str()).unwrap_or(&"?")); }
            }
            for n in &prop.notes { println!("  note: {n}"); }
            // --save <project-dir>: a project from it, with the cadpack copied in
            if let Some(out) = get("--save") {
                let out = PathBuf::from(out);
                std::fs::create_dir_all(out.join("cad/pack"))?;
                for f in ["parts.json", "meshes.bin"] { std::fs::copy(Path::new(&args[1]).join(f), out.join("cad/pack").join(f))?; }
                let mut p = Project::new(out.file_name().map_or("robot".into(), |s| s.to_string_lossy().to_string()).as_str());
                cad::propose::apply(&mut p.robot, &im.cadpack, &to, &prop, &lib);
                p.robot.cad = Some(model::CadSource { kind: model::CadKind::Step { path: args[1].clone() }, cadpack: "cad/pack".into(), imported: "cli".into(), to_robot: to });
                std::fs::write(out.join("robot.json"), p.to_json())?;
                println!("saved {}", out.display());
            }
        }
        Some("wheels") => {
            for w in &lib.wheels { println!("{:<28} {:<55} {:>4.0}A  k {:.2e} Pa/m", w.id, w.name, w.durometer_a, w.k_area()); }
        }
        _ => { eprintln!("{}", include_str!("main.rs").lines().skip(1).take(9).map(|l| l.trim_start_matches("//!")).collect::<Vec<_>>().join("\n")); }
    }
    Ok(())
}
