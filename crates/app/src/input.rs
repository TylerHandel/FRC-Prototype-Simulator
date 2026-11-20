//! Driving and the operator's buttons: keyboard or gamepad to the simulation (D12). The left stick
//! (or W A S D) moves, the right stick (or Q E, the arrow keys) turns; each mechanism's buttons are
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
pub struct Held { pub typing: bool, last: [f64; 3], down: Vec<Button>, frames: u32 }

pub fn key_of(b: Button) -> KeyCode {
    match b {
        Button::A => KeyCode::KeyJ, Button::B => KeyCode::KeyK, Button::X => KeyCode::KeyU, Button::Y => KeyCode::KeyI,
        Button::LB => KeyCode::KeyQ, Button::RB => KeyCode::KeyE, Button::LT => KeyCode::Space, Button::RT => KeyCode::KeyF,
        Button::Up => KeyCode::Digit1, Button::Down => KeyCode::Digit2, Button::Left => KeyCode::Digit3, Button::Right => KeyCode::Digit4,
        Button::Start => KeyCode::Enter, Button::Back => KeyCode::Backspace, Button::LStick => KeyCode::KeyZ, Button::RStick => KeyCode::KeyX,
    }
}

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
    if !matches!(st.ws, Workspace::Build | Workspace::Field | Workspace::Rig) { return; }
    let typing = held.typing;
    let k = |c: KeyCode| !typing && keys.pressed(c);
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
        let down = (!typing && keys.pressed(key_of(b))) || pads.iter().any(|p| p.pressed(pad_of(b)));
        let was = held.down.contains(&b);
        if down != was {
            if down { held.down.push(b); } else { held.down.retain(|x| *x != b); }
            sim.send(Msg::Button(b, down));
        }
    }
}
