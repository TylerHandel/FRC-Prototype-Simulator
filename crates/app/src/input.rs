//! Driving and the operator's buttons: keyboard or gamepad to the simulation (D12). The left stick
//! (or W A S D) moves, the right stick (or the left and right arrows) turns; each mechanism's buttons are
//! what its bindings say (Controls), with a key standing in for each gamepad button.
use bevy::prelude::*;
use model::Button;

use crate::simrun::{Msg, Sim};
use crate::state::{AppState, Workspace};

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) { app.init_resource::<Held>().add_systems(Update, drive); }
}

/// Keyboards and egui: set by the interface when a text field has the keyboard.
#[derive(Resource, Default)]
pub struct Held { pub typing: bool, last: [f64; 3], down: Vec<Button>, aligning: Option<bool>, frames: u32, generation: u64 }

pub const OPERATOR_KEYS: &[(&str, KeyCode)] = &[
    ("B", KeyCode::KeyB),
    ("C", KeyCode::KeyC),
    ("E", KeyCode::KeyE),
    ("F", KeyCode::KeyF),
    ("G", KeyCode::KeyG),
    ("H", KeyCode::KeyH),
    ("I", KeyCode::KeyI),
    ("J", KeyCode::KeyJ),
    ("K", KeyCode::KeyK),
    ("L", KeyCode::KeyL),
    ("M", KeyCode::KeyM),
    ("N", KeyCode::KeyN),
    ("O", KeyCode::KeyO),
    ("Q", KeyCode::KeyQ),
    ("T", KeyCode::KeyT),
    ("U", KeyCode::KeyU),
    ("V", KeyCode::KeyV),
    ("X", KeyCode::KeyX),
    ("Y", KeyCode::KeyY),
    ("Z", KeyCode::KeyZ),
    ("0", KeyCode::Digit0),
    ("1", KeyCode::Digit1),
    ("2", KeyCode::Digit2),
    ("3", KeyCode::Digit3),
    ("4", KeyCode::Digit4),
    ("5", KeyCode::Digit5),
    ("6", KeyCode::Digit6),
    ("7", KeyCode::Digit7),
    ("8", KeyCode::Digit8),
    ("9", KeyCode::Digit9),
    ("Space", KeyCode::Space),
    ("Enter", KeyCode::Enter),
    ("Backspace", KeyCode::Backspace),
];

fn pad_of(b: Button) -> GamepadButton {
    match b {
        Button::A => GamepadButton::South, Button::B => GamepadButton::East, Button::X => GamepadButton::West, Button::Y => GamepadButton::North,
        Button::LB => GamepadButton::LeftTrigger, Button::RB => GamepadButton::RightTrigger, Button::LT => GamepadButton::LeftTrigger2, Button::RT => GamepadButton::RightTrigger2,
        Button::Up => GamepadButton::DPadUp, Button::Down => GamepadButton::DPadDown, Button::Left => GamepadButton::DPadLeft, Button::Right => GamepadButton::DPadRight,
        Button::Start => GamepadButton::Start, Button::Back => GamepadButton::Select, Button::LStick => GamepadButton::LeftThumb, Button::RStick => GamepadButton::RightThumb,
    }
}

fn deadband(x: f32) -> f64 { if x.abs() < 0.1 { 0.0 } else { ((x.abs() - 0.1) / 0.9 * x.signum()) as f64 } }

fn drive(keys: Res<ButtonInput<KeyCode>>, pads: Query<&Gamepad>, st: Res<AppState>, sim: Res<Sim>, mut held: ResMut<Held>) {
    let practice = st.ws == Workspace::Archetype && st.assembly.driving();
    let (practice_ready, reef, generation) = if practice {
        sim.shared.lock().map(|s| (s.test_drive_ready(), s.field.is_some() && st.game.as_ref()
            .and_then(|(_,g)| g.as_ref().ok()).is_some_and(|g| !g.pack.reef_faces.is_empty()), s.generation)).unwrap_or_default()
    } else { (false, false, 0) };
    if held.generation != generation {
        held.down.clear();
        held.aligning = None;
        held.generation = generation;
    }
    if !matches!(st.ws, Workspace::Build | Workspace::Field | Workspace::Rig) && !practice_ready {
        if let Some(right) = held.aligning.take() { sim.send(Msg::ReefAlign { right, down: false }); }
        for b in held.down.drain(..) { sim.send(Msg::Button(b, false)); }
        if held.last != [0.0; 3] { held.last = [0.0; 3]; sim.send(Msg::Drive { v: held.last, field_relative: false }); }
        return;
    }
    let typing = held.typing;
    let shortcut = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
    let k = |c: KeyCode| !typing && !shortcut && keys.pressed(c);
    let (mut x, mut y, mut w) = (0.0f64, 0.0f64, 0.0f64);
    if k(KeyCode::KeyW) { x += 1.0; }
    if k(KeyCode::KeyS) { x -= 1.0; }
    if k(KeyCode::KeyA) { y += 1.0; }
    if k(KeyCode::KeyD) { y -= 1.0; }
    if k(KeyCode::ArrowLeft) { w += 1.0; }
    if k(KeyCode::ArrowRight) { w -= 1.0; }
    let slow = k(KeyCode::ShiftLeft);
    for pad in &pads {
        x += deadband(pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0));
        y -= deadband(pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0));
        w -= deadband(pad.get(GamepadAxis::RightStickX).unwrap_or(0.0));
    }
    let (vmax, wmax) = st.project().robot.drivetrain.as_ref().map_or((0.0, 0.0), |d| (d.max_speed, d.max_turn.min(8.0)));
    let s = if slow { 0.35 } else { 1.0 };
    let v = [x.clamp(-1.0, 1.0) * vmax * s, y.clamp(-1.0, 1.0) * vmax * s, w.clamp(-1.0, 1.0) * wmax * s];
    // sent when it changes, and every few frames anyway (a world rebuilt after an edit starts still)
    held.frames += 1;
    if v != held.last || (v != [0.0; 3] && held.frames % 10 == 0) {
        held.last = v;
        sim.send(Msg::Drive { v, field_relative: st.settings.field_relative && st.ws == Workspace::Field });
    }
    for b in Button::ALL {
        let c = &st.project().robot.controls;
        let pad = if reef { c.reef_pad(b) } else { Some(b) };
        let down = OPERATOR_KEYS.iter().find(|(name, _)| *name == c.key(b)).is_some_and(|(_, code)| k(*code))
            || pad.is_some_and(|b| pads.iter().any(|p| p.pressed(pad_of(b))));
        let was = held.down.contains(&b);
        if down != was {
            if down { held.down.push(b); } else { held.down.retain(|x| *x != b); }
            sim.send(Msg::Button(b, down));
        }
    }
    let left = reef && (k(KeyCode::BracketLeft) || pads.iter().any(|p| p.pressed(pad_of(Button::LB))));
    let right = reef && (k(KeyCode::BracketRight) || pads.iter().any(|p| p.pressed(pad_of(Button::RB))));
    let side = match (left, right) { (true, false) => Some(false), (false, true) => Some(true), _ => None };
    if held.aligning != side {
        if let Some(right) = held.aligning { sim.send(Msg::ReefAlign { right, down: false }); }
        if let Some(right) = side { sim.send(Msg::ReefAlign { right, down: true }); }
        held.aligning = side;
    }
}
