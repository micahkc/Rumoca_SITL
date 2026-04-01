//! Quadrotor SIL (Software-in-the-Loop) plant simulator.
//!
//! Receives motor commands via UDP flatbuffer, steps 6-DOF quadrotor
//! physics, sends back sensor data, and streams visualization to a
//! Three.js browser viewer.
//!
//! The flatbuffer protocol is driven by a TOML config + .bfbs schema
//! reflection — no hand-coded pack/unpack. See `sil_config.toml`.
//!
//! Usage:
//!   cargo run                        # uses ./sil_config.toml
//!   cargo run -- path/to/config.toml # custom config

mod bfbs;
mod codec;
mod config;

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, UdpSocket};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use gilrs::{Axis, Button, Gilrs};
use rumoca_sim::{SimStepper, StepperOptions};
use tungstenite::{accept, Message};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

const HTTP_PORT: u16 = 8080;
const WS_PORT: u16 = 8081;
const MAX_SUB_DT: f64 = 0.002;

const MODEL_SOURCE: &str = include_str!("../models/QuadrotorSIL.mo");
const THREE_JS: &str = include_str!("../web/three.min.js");

// Physical constants matching the Modelica model
const MASS: f64 = 2.0;
const G: f64 = 9.80665;
const CT: f64 = 8.5e-6;

fn hover_omega() -> f64 {
    (MASS * G / (4.0 * CT)).sqrt()
}


// ===========================================================================
// Input → RC channels (gamepad, keyboard, or test sequence)
// ===========================================================================

const RC_CENTER: i32 = 1500;
const RC_MIN: i32 = 1000;
const RC_MAX: i32 = 2000;

/// Unified input source for RC channels.
struct InputState {
    gilrs: Gilrs,
    rc: [i32; 16],
    armed: bool,
    arm_prev: bool,
    arm_last_toggle: Instant,
    throttle: f64,
    last_poll: Instant,
    mode: InputMode,
    // Keyboard: track which keys are held
    kb_roll: f64,
    kb_pitch: f64,
    kb_yaw: f64,
    kb_throttle_input: f64,
    // Test sequence state
    test_start: Instant,
    // Reset request (gamepad A / keyboard R)
    reset_requested: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum InputMode {
    Gamepad,
    Keyboard,
    Test,
}

impl std::fmt::Display for InputMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InputMode::Gamepad => write!(f, "gamepad"),
            InputMode::Keyboard => write!(f, "keyboard"),
            InputMode::Test => write!(f, "test"),
        }
    }
}

// Test sequence: time-based scripted flight
// Each step: (start_time_sec, throttle_0_to_1, roll, pitch, yaw, armed)
const TEST_SEQUENCE: &[(f64, f64, f64, f64, f64, bool)] = &[
    //  time  throttle  roll  pitch  yaw   armed
    (0.0,    0.0,      0.0,  0.0,   0.0,  false),  // ground, disarmed
    (2.0,    0.0,      0.0,  0.0,   0.0,  true),   // arm (throttle low)
    (3.0,    0.50,     0.0,  0.0,   0.0,  true),   // ramp to hover
    (5.0,    0.52,     0.0,  0.0,   0.0,  true),   // steady hover
    (8.0,    0.52,     0.0,  0.15,  0.0,  true),   // gentle pitch forward
    (10.0,   0.52,     0.0,  0.0,   0.0,  true),   // level out
    (12.0,   0.52,     0.15, 0.0,   0.0,  true),   // gentle roll right
    (14.0,   0.52,     0.0,  0.0,   0.0,  true),   // level out
    (16.0,   0.52,     0.0,  0.0,   0.2,  true),   // yaw right
    (18.0,   0.52,     0.0,  0.0,   0.0,  true),   // stop yaw
    (20.0,   0.30,     0.0,  0.0,   0.0,  true),   // descend
    (23.0,   0.0,      0.0,  0.0,   0.0,  true),   // cut throttle
    (25.0,   0.0,      0.0,  0.0,   0.0,  false),  // disarm
    (30.0,   0.0,      0.0,  0.0,   0.0,  false),  // hold (sequence restarts)
];

impl InputState {
    fn new(mode: InputMode) -> Self {
        let gilrs = Gilrs::new().expect("Failed to initialize gilrs");
        for (_id, gamepad) in gilrs.gamepads() {
            eprintln!("Gamepad found: {} ({})", gamepad.name(), gamepad.os_name());
        }

        let effective_mode = match mode {
            InputMode::Test => {
                eprintln!("[input] Test sequence mode");
                InputMode::Test
            }
            InputMode::Gamepad | InputMode::Keyboard => {
                if gilrs.gamepads().count() > 0 {
                    eprintln!("[input] Using gamepad");
                    InputMode::Gamepad
                } else {
                    eprintln!("[input] No gamepad detected — using keyboard");
                    eprintln!("[input] Keyboard controls:");
                    eprintln!("  Up/Down    — throttle up/down");
                    eprintln!("  Left/Right — yaw left/right");
                    eprintln!("  W/S        — pitch forward/back");
                    eprintln!("  A/D        — roll left/right");
                    eprintln!("  Space      — arm/disarm toggle");
                    eprintln!("  Q          — quit");
                    if crossterm::terminal::enable_raw_mode().is_err() {
                        eprintln!("[input] Warning: could not enable raw mode (not a tty?)");
                    }
                    InputMode::Keyboard
                }
            }
        };

        let mut rc = [RC_CENTER; 16];
        rc[2] = RC_MIN;
        rc[4] = RC_MIN;
        Self {
            gilrs,
            rc,
            armed: false,
            arm_prev: false,
            arm_last_toggle: Instant::now() - Duration::from_secs(10),
            throttle: 0.0,
            last_poll: Instant::now(),
            mode: effective_mode,
            kb_roll: 0.0,
            kb_pitch: 0.0,
            kb_yaw: 0.0,
            kb_throttle_input: 0.0,
            test_start: Instant::now(),
            reset_requested: false,
        }
    }

    fn poll(&mut self) {
        match self.mode {
            InputMode::Gamepad => self.poll_gamepad(),
            InputMode::Keyboard => self.poll_keyboard(),
            InputMode::Test => self.poll_test(),
        }
    }

    fn poll_gamepad(&mut self) {
        while self.gilrs.next_event().is_some() {}

        let Some((_id, gamepad)) = self.gilrs.gamepads().next() else {
            return;
        };

        // Debug: print raw axis values once per second
        if self.last_poll.elapsed() > Duration::from_secs(1) {
            eprintln!(
                "\r[gp] LX={:.2} LY={:.2} RX={:.2} RY={:.2} armed={} throttle={:.2}    ",
                gamepad.value(Axis::LeftStickX),
                gamepad.value(Axis::LeftStickY),
                gamepad.value(Axis::RightStickX),
                gamepad.value(Axis::RightStickY),
                self.armed,
                self.throttle,
            );
        }

        self.rc[0] = axis_to_rc(gamepad.value(Axis::RightStickX), RC_CENTER, 500);
        self.rc[1] = axis_to_rc(gamepad.value(Axis::RightStickY), RC_CENTER, -500);
        self.rc[3] = axis_to_rc(gamepad.value(Axis::LeftStickX), RC_CENTER, 500);

        let dt = self.last_poll.elapsed().as_secs_f64();
        self.last_poll = Instant::now();
        let stick_y = gamepad.value(Axis::LeftStickY) as f64;
        let deadzone = 0.1;
        let input = if stick_y.abs() < deadzone { 0.0 } else { stick_y };
        self.throttle = (self.throttle + input * 0.7 * dt).clamp(0.0, 1.0);
        self.rc[2] = (RC_MIN as f64 + self.throttle * (RC_MAX - RC_MIN) as f64).round() as i32;

        let arm_btn = gamepad.is_pressed(Button::Start);
        if arm_btn && !self.arm_prev && self.arm_last_toggle.elapsed() > Duration::from_millis(500) {
            if self.rc[2] <= 1050 {
                self.armed = !self.armed;
                self.arm_last_toggle = Instant::now();
                eprintln!(
                    "\r[gamepad] {}",
                    if self.armed { "ARMED" } else { "DISARMED" }
                );
            }
        }
        self.arm_prev = arm_btn;
        self.rc[4] = if self.armed { RC_MAX } else { RC_MIN };

        // Reset: A button (South)
        if gamepad.is_pressed(Button::South) {
            self.reset_requested = true;
        }
    }

    fn poll_keyboard(&mut self) {
        let dt = self.last_poll.elapsed().as_secs_f64();
        self.last_poll = Instant::now();

        // Decay axes toward zero when keys are released
        let decay = 0.85_f64.powf(dt / 0.016);
        self.kb_roll *= decay;
        self.kb_pitch *= decay;
        self.kb_yaw *= decay;
        self.kb_throttle_input *= decay;

        // Drain all pending key events (only act on Press/Repeat, ignore Release)
        while event::poll(Duration::ZERO).unwrap_or(false) {
            match event::read() {
                Ok(Event::Key(KeyEvent { code, modifiers, kind, .. })) => {
                    if kind == crossterm::event::KeyEventKind::Release {
                        continue;
                    }
                    let ctrl = modifiers.contains(KeyModifiers::CONTROL);
                    match code {
                        KeyCode::Char('c') if ctrl => {
                            crossterm::terminal::disable_raw_mode().ok();
                            std::process::exit(0);
                        }
                        KeyCode::Char('q') => {
                            crossterm::terminal::disable_raw_mode().ok();
                            std::process::exit(0);
                        }
                        // Arrow keys: throttle and yaw
                        KeyCode::Up => self.kb_throttle_input = 1.0,
                        KeyCode::Down => self.kb_throttle_input = -1.0,
                        KeyCode::Left => self.kb_yaw = -0.6,
                        KeyCode::Right => self.kb_yaw = 0.6,
                        // WASD: pitch and roll
                        KeyCode::Char('w') => self.kb_pitch = -0.6,
                        KeyCode::Char('s') => self.kb_pitch = 0.6,
                        KeyCode::Char('a') => self.kb_roll = -0.6,
                        KeyCode::Char('d') => self.kb_roll = 0.6,
                        // Arm/disarm toggle (500ms debounce to prevent double-toggle)
                        KeyCode::Char(' ') => {
                            if self.arm_last_toggle.elapsed() > Duration::from_millis(500) {
                                self.armed = !self.armed;
                                self.arm_last_toggle = Instant::now();
                                eprint!(
                                    "\r[keyboard] {}                    \r",
                                    if self.armed { "ARMED" } else { "DISARMED" }
                                );
                            }
                        }
                        KeyCode::Char('r') => self.reset_requested = true,
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        // Apply throttle ramp
        self.throttle = (self.throttle + self.kb_throttle_input * 0.7 * dt).clamp(0.0, 1.0);
        self.rc[2] = (RC_MIN as f64 + self.throttle * (RC_MAX - RC_MIN) as f64).round() as i32;

        // Apply axes
        self.rc[0] = (RC_CENTER as f64 + self.kb_roll * 500.0).round().clamp(RC_MIN as f64, RC_MAX as f64) as i32;
        self.rc[1] = (RC_CENTER as f64 + self.kb_pitch * 500.0).round().clamp(RC_MIN as f64, RC_MAX as f64) as i32;
        self.rc[3] = (RC_CENTER as f64 + self.kb_yaw * 500.0).round().clamp(RC_MIN as f64, RC_MAX as f64) as i32;

        self.rc[4] = if self.armed { RC_MAX } else { RC_MIN };
    }

    fn poll_test(&mut self) {
        let t = self.test_start.elapsed().as_secs_f64();
        let total_duration = TEST_SEQUENCE.last().unwrap().0;

        // Loop the sequence
        let t = t % total_duration;

        // Find the current and next keyframe
        let mut cur = &TEST_SEQUENCE[0];
        let mut next = &TEST_SEQUENCE[0];
        for i in 0..TEST_SEQUENCE.len() - 1 {
            if t >= TEST_SEQUENCE[i].0 && t < TEST_SEQUENCE[i + 1].0 {
                cur = &TEST_SEQUENCE[i];
                next = &TEST_SEQUENCE[i + 1];
                break;
            }
        }

        // Linearly interpolate between keyframes
        let span = next.0 - cur.0;
        let frac = if span > 0.0 { (t - cur.0) / span } else { 0.0 };
        let lerp = |a: f64, b: f64| a + (b - a) * frac;

        let throttle = lerp(cur.1, next.1);
        let roll = lerp(cur.2, next.2);
        let pitch = lerp(cur.3, next.3);
        let yaw = lerp(cur.4, next.4);
        self.armed = cur.5;

        self.throttle = throttle;
        self.rc[2] = (RC_MIN as f64 + throttle * (RC_MAX - RC_MIN) as f64).round() as i32;
        self.rc[0] = (RC_CENTER as f64 + roll * 500.0).round() as i32;
        self.rc[1] = (RC_CENTER as f64 + pitch * 500.0).round() as i32;
        self.rc[3] = (RC_CENTER as f64 + yaw * 500.0).round() as i32;
        self.rc[4] = if self.armed { RC_MAX } else { RC_MIN };
    }

    fn is_connected(&self) -> bool {
        match self.mode {
            InputMode::Gamepad => self.gilrs.gamepads().count() > 0,
            InputMode::Keyboard | InputMode::Test => true,
        }
    }

    fn mode_name(&self) -> &str {
        match self.mode {
            InputMode::Gamepad => "gamepad",
            InputMode::Keyboard => "keyboard",
            InputMode::Test => "test",
        }
    }
}

impl Drop for InputState {
    fn drop(&mut self) {
        if self.mode == InputMode::Keyboard {
            crossterm::terminal::disable_raw_mode().ok();
        }
    }
}

fn axis_to_rc(value: f32, center: i32, half_range: i32) -> i32 {
    ((center as f32) + value * (half_range as f32))
        .round()
        .clamp(RC_MIN as f32, RC_MAX as f32) as i32
}

// ===========================================================================
// SIL API
// ===========================================================================

#[derive(Debug, Clone, Copy)]
pub struct SensorOutput {
    pub clock_sec: f64,
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
    pub mag: [f64; 3],
    pub position_ned: [f64; 3],
    pub velocity_ned: [f64; 3],
    pub quaternion: [f64; 4],
}

pub struct QuadrotorSil {
    stepper: SimStepper,
    model_source: String,
}

impl QuadrotorSil {
    pub fn new() -> anyhow::Result<Self> {
        let source = MODEL_SOURCE.to_string();
        let compiler = rumoca::Compiler::new().model("QuadrotorSIL");
        let result = compiler.compile_str(&source, "QuadrotorSIL.mo")?;
        let stepper = SimStepper::new(
            &result.dae,
            StepperOptions {
                rtol: 1e-3,
                atol: 1e-3,
                ..Default::default()
            },
        )?;
        Ok(Self {
            stepper,
            model_source: source,
        })
    }

    /// Step physics with motor angular velocities [rad/s] to target clock.
    pub fn receive_motors(
        &mut self,
        motor_rpms: [f64; 4],
        clock_sec: f64,
    ) -> anyhow::Result<SensorOutput> {
        if let Err(e) = self.stepper.set_input("omega_m1", motor_rpms[0]) {
            eprintln!("[sil] set_input omega_m1 failed: {e}");
        }
        if let Err(e) = self.stepper.set_input("omega_m2", motor_rpms[1]) {
            eprintln!("[sil] set_input omega_m2 failed: {e}");
        }
        if let Err(e) = self.stepper.set_input("omega_m3", motor_rpms[2]) {
            eprintln!("[sil] set_input omega_m3 failed: {e}");
        }
        if let Err(e) = self.stepper.set_input("omega_m4", motor_rpms[3]) {
            eprintln!("[sil] set_input omega_m4 failed: {e}");
        }

        let current_time = self.stepper.time();
        let dt = clock_sec - current_time;
        if dt > 0.0 {
            let n_steps = ((dt / MAX_SUB_DT).ceil() as usize).max(1);
            let sub_dt = dt / n_steps as f64;
            for i in 0..n_steps {
                if let Err(e) = self.stepper.step(sub_dt) {
                    eprintln!("[sil] step {}/{} failed (sub_dt={:.4}s): {e}", i+1, n_steps, sub_dt);
                }
            }
        }
        Ok(self.read_sensors())
    }

    pub fn read_sensors(&self) -> SensorOutput {
        let get = |name: &str| self.stepper.get(name).unwrap_or(0.0);
        SensorOutput {
            clock_sec: self.stepper.time(),
            accel: [get("accel_x"), get("accel_y"), get("accel_z")],
            gyro: [get("gyro_x"), get("gyro_y"), get("gyro_z")],
            mag: [get("mag_x"), get("mag_y"), get("mag_z")],
            position_ned: [get("px"), get("py"), get("pz")],
            velocity_ned: [get("vx"), get("vy"), get("vz")],
            quaternion: [
                self.stepper.get("q0").unwrap_or(1.0),
                get("q1"),
                get("q2"),
                get("q3"),
            ],
        }
    }

    pub fn state_json(&self) -> String {
        let get = |name: &str| self.stepper.get(name).unwrap_or(0.0);
        serde_json::json!({
            "t": self.stepper.time(),
            "px": get("px"), "py": get("py"), "pz": get("pz"),
            "vx": get("vx"), "vy": get("vy"), "vz": get("vz"),
            "q0": self.stepper.get("q0").unwrap_or(1.0),
            "q1": get("q1"), "q2": get("q2"), "q3": get("q3"),
            "R11": get("R11"), "R12": get("R12"), "R13": get("R13"),
            "R21": get("R21"), "R22": get("R22"), "R23": get("R23"),
            "R31": get("R31"), "R32": get("R32"), "R33": get("R33"),
            "omega_m1": get("omega_m1"), "omega_m2": get("omega_m2"),
            "omega_m3": get("omega_m3"), "omega_m4": get("omega_m4"),
            "T": get("T"),
            "accel_x": get("accel_x"), "accel_y": get("accel_y"), "accel_z": get("accel_z"),
            "gyro_x": get("gyro_x"), "gyro_y": get("gyro_y"), "gyro_z": get("gyro_z"),
            "mag_x": get("mag_x"), "mag_y": get("mag_y"), "mag_z": get("mag_z"),
        })
        .to_string()
    }

    pub fn reset(&mut self) -> anyhow::Result<()> {
        let compiler = rumoca::Compiler::new().model("QuadrotorSIL");
        let result = compiler.compile_str(&self.model_source, "QuadrotorSIL.mo")?;
        self.stepper = SimStepper::new(
            &result.dae,
            StepperOptions {
                rtol: 1e-3,
                atol: 1e-3,
                ..Default::default()
            },
        )?;
        Ok(())
    }

    pub fn time(&self) -> f64 {
        self.stepper.time()
    }

}

// ===========================================================================
// HTML / Three.js viewer
// ===========================================================================

const HTML_PAGE: &str = r##"<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Quadrotor SIL Simulator</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #111; color: #eee; font-family: monospace; overflow: hidden; }
  #container { width: 100vw; height: 100vh; }
  #hud {
    position: fixed; top: 10px; left: 10px; z-index: 10;
    background: rgba(30,20,10,0.75); padding: 10px 14px; border-radius: 6px;
    font-size: 13px; line-height: 1.6; backdrop-filter: blur(4px);
    border: 1px solid rgba(200,160,80,0.2);
  }
  #hud .label { color: #a89070; }
  #hud .val { color: #e8c840; font-weight: bold; }
  #hud .sensor { color: #70b8e0; }
  #status {
    position: fixed; top: 10px; right: 10px; z-index: 10;
    background: rgba(30,20,10,0.75); padding: 6px 12px; border-radius: 6px;
    font-size: 12px; backdrop-filter: blur(4px);
    border: 1px solid rgba(200,160,80,0.2);
  }
  .connected { color: #4c4; }
  .disconnected { color: #c44; }
  #mode {
    position: fixed; bottom: 10px; left: 10px; z-index: 10;
    background: rgba(30,20,10,0.75); padding: 8px 14px; border-radius: 6px;
    font-size: 12px; line-height: 1.5; backdrop-filter: blur(4px);
    border: 1px solid rgba(200,160,80,0.2);
  }
  #controls {
    position: fixed; bottom: 10px; right: 10px; z-index: 10;
    background: rgba(30,20,10,0.75); padding: 8px 14px; border-radius: 6px;
    font-size: 11px; line-height: 1.6; backdrop-filter: blur(4px);
    border: 1px solid rgba(200,160,80,0.2); max-width: 240px;
  }
  #controls b { color: #e8c840; }
  #controls .key { color: #70b8e0; display: inline-block; min-width: 70px; }
  #controls .action { color: #a89070; }
</style>
</head>
<body>
<div id="container"></div>
<div id="hud">
  <b>SIL Plant Simulator</b><br>
  <span class="label">t:</span> <span class="val" id="v-t">0.00</span>s<br>
  <span class="label">pos NED:</span> <span class="val" id="v-pos">0, 0, 0</span><br>
  <span class="label">alt:</span> <span class="val" id="v-alt">0.0</span>m<br>
  <span class="label">vel:</span> <span class="val" id="v-vel">0.0</span> m/s<br>
  <span class="label">roll:</span> <span class="val" id="v-roll">0.0</span>&deg;
  <span class="label">pitch:</span> <span class="val" id="v-pitch">0.0</span>&deg;<br>
  <hr style="border-color:#554430; margin:4px 0">
  <span class="label">motors:</span> <span class="sensor" id="v-motors">0, 0, 0, 0</span> rad/s<br>
  <span class="label">thrust:</span> <span class="sensor" id="v-thrust">0.0</span> N<br>
  <span class="label">accel:</span> <span class="sensor" id="v-accel">0, 0, 0</span><br>
  <span class="label">gyro:</span> <span class="sensor" id="v-gyro">0, 0, 0</span><br>
  <span class="label">mag:</span> <span class="sensor" id="v-mag">0, 0, 0</span>
</div>
<div id="mode">
  <b>Mode:</b> <span id="v-mode-text">Self-test (hover)</span>
</div>
<div id="status"><span class="disconnected" id="ws-status">Connecting...</span></div>
<div id="controls"></div>

<script>
__THREE_JS__
</script>
<script>
function nedToThree(px, py, pz) { return [py, -pz, px]; }

const ARM_LEN = 0.28;
const TRAIL_MAX = 800;

const container = document.getElementById("container");
const scene = new THREE.Scene();
scene.background = null;
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 500);
camera.position.set(2, 3, 5);
camera.lookAt(0, 0, 0);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
container.appendChild(renderer.domElement);
window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

let camAngle = 0.8, camElev = 0.5, camDist = 4;
let camTarget = new THREE.Vector3(0, 1, 0);
container.addEventListener("wheel", (e) => { camDist = Math.max(1, Math.min(20, camDist + e.deltaY * 0.005)); });
let dragging = false, lastMX = 0, lastMY = 0;
container.addEventListener("mousedown", (e) => { dragging = true; lastMX = e.clientX; lastMY = e.clientY; });
container.addEventListener("mouseup", () => { dragging = false; });
container.addEventListener("mousemove", (e) => {
  if (!dragging) return;
  camAngle -= (e.clientX - lastMX) * 0.005;
  camElev = Math.max(-1.2, Math.min(1.5, camElev + (e.clientY - lastMY) * 0.005));
  lastMX = e.clientX; lastMY = e.clientY;
});

// ═══ DESERT SKY ═══
const skyGeo = new THREE.SphereGeometry(200, 32, 16);
const skyColors = [];
const posAttr = skyGeo.getAttribute("position");
for (let i = 0; i < posAttr.count; i++) {
  const y = posAttr.getY(i);
  const t = Math.max(0, Math.min(1, (y / 200 + 1) * 0.5));
  const r = 0.94 + (0.29 - 0.94) * Math.pow(t, 0.6);
  const g = 0.85 + (0.56 - 0.85) * Math.pow(t, 0.6);
  const b = 0.69 + (0.78 - 0.69) * Math.pow(t, 0.6);
  skyColors.push(r, g, b);
}
skyGeo.setAttribute("color", new THREE.Float32BufferAttribute(skyColors, 3));
const sky = new THREE.Mesh(skyGeo, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide }));
scene.add(sky);

// ═══ DESERT LIGHTING ═══
const sun = new THREE.DirectionalLight(0xfff0d0, 1.8);
sun.position.set(8, 12, 4); scene.add(sun);
const fill = new THREE.DirectionalLight(0xd4a060, 0.4);
fill.position.set(-5, 3, -4); scene.add(fill);
const rim = new THREE.DirectionalLight(0xffeebb, 0.25);
rim.position.set(0, -1, -6); scene.add(rim);
scene.add(new THREE.HemisphereLight(0x87ceeb, 0xc2956b, 0.5));

// ═══ DESERT GROUND ═══
const sandMat = new THREE.MeshStandardMaterial({ color: 0xd4a860, roughness: 0.95, metalness: 0.02 });
const GROUND_Y = -0.071;  // below landing gear feet (y=-0.068 minus foot radius)
const floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), sandMat);
floor.rotation.x = -Math.PI / 2; floor.position.y = GROUND_Y; scene.add(floor);
const grid = new THREE.GridHelper(50, 50, 0xc49850, 0xc49850);
grid.position.y = GROUND_Y;
grid.material.transparent = true; grid.material.opacity = 0.12;
scene.add(grid);

// ═══ "CogniPilot" IN THE SAND ═══
{
  const canvas = document.createElement("canvas");
  canvas.width = 1024; canvas.height = 128;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "rgba(0,0,0,0)";
  ctx.fillRect(0, 0, 1024, 128);
  ctx.font = "bold 90px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#b8944a";
  ctx.fillText("CogniPilot", 512, 64);
  const tex = new THREE.CanvasTexture(canvas);
  tex.anisotropy = 4;
  const mat = new THREE.MeshStandardMaterial({
    map: tex, transparent: true, alphaTest: 0.1,
    roughness: 0.95, metalness: 0.0,
    depthWrite: false,
  });
  const plane = new THREE.Mesh(new THREE.PlaneGeometry(8, 1), mat);
  plane.rotation.x = -Math.PI / 2;
  plane.position.set(0, GROUND_Y + 0.012, 1.5);
  scene.add(plane);
}

// ═══ SAND DUNES ═══
const duneMat = new THREE.MeshStandardMaterial({ color: 0xd9b06a, roughness: 0.9 });
const duneDarkMat = new THREE.MeshStandardMaterial({ color: 0xc49850, roughness: 0.95 });
[
  { x:-18,z:20,sx:12,sy:1.5,sz:5,ry:0.3 }, { x:22,z:15,sx:15,sy:2.0,sz:6,ry:-0.2 },
  { x:-25,z:-10,sx:10,sy:1.2,sz:4,ry:0.5 }, { x:15,z:-22,sx:18,sy:2.5,sz:7,ry:0.1 },
  { x:-10,z:-25,sx:14,sy:1.8,sz:5,ry:-0.4 }, { x:30,z:-5,sx:8,sy:1.0,sz:4,ry:0.6 },
  { x:-30,z:5,sx:11,sy:1.3,sz:5,ry:-0.1 }, { x:0,z:30,sx:20,sy:2.2,sz:6,ry:0.15 },
].forEach((d) => {
  const dune = new THREE.Mesh(
    new THREE.SphereGeometry(1, 16, 8, 0, Math.PI*2, 0, Math.PI/2),
    Math.random() > 0.5 ? duneMat : duneDarkMat
  );
  dune.scale.set(d.sx, d.sy, d.sz);
  dune.position.set(d.x, GROUND_Y, d.z);
  dune.rotation.y = d.ry;
  scene.add(dune);
});

// ═══ DESERT ROCKS ═══
const rockMat = new THREE.MeshStandardMaterial({ color: 0x8b7355, roughness: 0.85, metalness: 0.05 });
const rockDarkMat = new THREE.MeshStandardMaterial({ color: 0x6b5740, roughness: 0.9, metalness: 0.05 });
[
  {x:5,z:8,s:0.3},{x:-7,z:6,s:0.5},{x:9,z:-4,s:0.2},{x:-4,z:-8,s:0.4},
  {x:12,z:3,s:0.35},{x:-11,z:-3,s:0.25},{x:3,z:-10,s:0.45},{x:-8,z:10,s:0.3},
].forEach((r) => {
  const rock = new THREE.Mesh(
    new THREE.DodecahedronGeometry(r.s, 1),
    Math.random() > 0.5 ? rockMat : rockDarkMat
  );
  rock.position.set(r.x, r.s*0.3, r.z);
  rock.scale.set(1+Math.random()*0.5, 0.5+Math.random()*0.4, 1+Math.random()*0.3);
  rock.rotation.set(Math.random()*0.3, Math.random()*Math.PI, Math.random()*0.2);
  scene.add(rock);
});

// ═══ CACTI ═══
const cactusMat = new THREE.MeshStandardMaterial({ color: 0x3a6b35, roughness: 0.8, metalness: 0.05 });
[{x:-6,z:12},{x:14,z:7},{x:-13,z:-6},{x:8,z:-12},{x:-3,z:-15}].forEach((c) => {
  const h = 0.8 + Math.random() * 1.2;
  const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.15, h, 8), cactusMat);
  trunk.position.set(c.x, h/2, c.z); scene.add(trunk);
  const top = new THREE.Mesh(new THREE.SphereGeometry(0.12, 8, 6), cactusMat);
  top.position.set(c.x, h, c.z); scene.add(top);
  if (Math.random() > 0.3) {
    const armH = 0.4 + Math.random()*0.4;
    const armY = h*0.4 + Math.random()*h*0.3;
    const dir = Math.random() > 0.5 ? 1 : -1;
    const aH = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.3, 6), cactusMat);
    aH.rotation.z = dir*Math.PI/2; aH.position.set(c.x+dir*0.2, armY, c.z); scene.add(aH);
    const aV = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.07, armH, 6), cactusMat);
    aV.position.set(c.x+dir*0.35, armY+armH/2, c.z); scene.add(aV);
    const aT = new THREE.Mesh(new THREE.SphereGeometry(0.06, 6, 4), cactusMat);
    aT.position.set(c.x+dir*0.35, armY+armH, c.z); scene.add(aT);
  }
});

// ═══════════════════════════════════════════════════
//  HIGH-FIDELITY QUADROTOR
// ═══════════════════════════════════════════════════
const quad = new THREE.Group(); quad.name = "quadrotor";

// --- Material palette ---
const matBody = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, metalness: 0.4, roughness: 0.45 });
const matBodyAccent = new THREE.MeshStandardMaterial({ color: 0x1a1a1e, metalness: 0.5, roughness: 0.3 });
const matCarbon = new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.3, roughness: 0.6 });
const matMotor = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.85, roughness: 0.15 });
const matMotorBell = new THREE.MeshStandardMaterial({ color: 0x333338, metalness: 0.9, roughness: 0.1 });
const matLens = new THREE.MeshStandardMaterial({ color: 0x111122, metalness: 0.9, roughness: 0.05 });
const matGimbal = new THREE.MeshStandardMaterial({ color: 0xaaaaaa, metalness: 0.7, roughness: 0.2 });
const matBattery = new THREE.MeshStandardMaterial({ color: 0x2a2a30, metalness: 0.3, roughness: 0.6 });
const matGuard = new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.2, roughness: 0.7, transparent: true, opacity: 0.6 });
const matSkid = new THREE.MeshStandardMaterial({ color: 0x444444, metalness: 0.3, roughness: 0.5 });

// ── CENTRAL FUSELAGE ──
const bodyTop = new THREE.Mesh(
  new THREE.SphereGeometry(0.09, 24, 12, 0, Math.PI*2, 0, Math.PI/2), matBody
);
bodyTop.scale.set(1.3, 0.5, 1.0); bodyTop.position.y = 0.005; quad.add(bodyTop);
const bodyBot = new THREE.Mesh(
  new THREE.SphereGeometry(0.09, 24, 12, 0, Math.PI*2, Math.PI/2, Math.PI/2), matBodyAccent
);
bodyBot.scale.set(1.3, 0.35, 1.0); bodyBot.position.y = 0.005; quad.add(bodyBot);
for (const sz of [-1, 1]) {
  const strip = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.012, 0.004), matCarbon);
  strip.position.set(0, 0.005, sz*0.085); quad.add(strip);
}
const cover = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.07, 0.008, 20), matBodyAccent);
cover.position.y = 0.05; quad.add(cover);

// ── GPS / ANTENNA MAST ──
const antBase = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.014, 0.015, 8), matCarbon);
antBase.position.set(0, 0.058, -0.02); quad.add(antBase);
const antMast = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.035, 6), matGimbal);
antMast.position.set(0, 0.082, -0.02); quad.add(antMast);
const antTop = new THREE.Mesh(new THREE.SphereGeometry(0.006, 8, 6), matGimbal);
antTop.position.set(0, 0.1, -0.02); quad.add(antTop);

// ── BATTERY PACK ──
const batt = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.022, 0.045), matBattery);
batt.position.set(0, 0.04, 0.01); quad.add(batt);
const battStripe = new THREE.Mesh(
  new THREE.BoxGeometry(0.04, 0.023, 0.002),
  new THREE.MeshStandardMaterial({ color: 0xddaa00, metalness: 0.3, roughness: 0.5 })
);
battStripe.position.set(0, 0.04, 0.034); quad.add(battStripe);

// ── CAMERA GIMBAL ──
const gimbalMount = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.012, 0.03), matGimbal);
gimbalMount.position.set(0, -0.025, 0.065); quad.add(gimbalMount);
const gimbalYoke = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.02, 0.025), matGimbal);
gimbalYoke.position.set(0, -0.038, 0.065); quad.add(gimbalYoke);
const camHousing = new THREE.Mesh(new THREE.BoxGeometry(0.028, 0.018, 0.022), matBody);
camHousing.position.set(0, -0.038, 0.065); quad.add(camHousing);
const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.008, 0.008, 12), matLens);
lens.rotation.x = Math.PI/2; lens.position.set(0, -0.038, 0.078); quad.add(lens);
const lensGlass = new THREE.Mesh(
  new THREE.CircleGeometry(0.006, 12),
  new THREE.MeshStandardMaterial({ color: 0x223355, metalness: 1.0, roughness: 0.0 })
);
lensGlass.position.set(0, -0.038, 0.0825); quad.add(lensGlass);

// ── ARMS + MOTORS + PROPELLERS ──
const armConfigs = [
  { angle: Math.PI/4,      cw: false, ledColor: 0x00ff44 },
  { angle: -Math.PI/4,     cw: true,  ledColor: 0xff2200 },
  { angle: 3*Math.PI/4,    cw: true,  ledColor: 0xffffff },
  { angle: -3*Math.PI/4,   cw: false, ledColor: 0xffffff },
];
const propGroups = [];

function createPropBlade(cw) {
  const blade = new THREE.Group();
  const bladeLen = 0.11, segments = 6;
  for (let i = 0; i < segments; i++) {
    const t = (i + 0.5) / segments;
    const segLen = bladeLen / segments;
    const chord = 0.022 * (1.0 - 0.5*t);
    const thickness = 0.003 * (1.0 - 0.6*t);
    const seg = new THREE.Mesh(
      new THREE.BoxGeometry(segLen, thickness, chord),
      new THREE.MeshStandardMaterial({ color: 0x1a1a1a, metalness: 0.15, roughness: 0.55, side: THREE.DoubleSide })
    );
    const r = 0.012 + bladeLen * t;
    seg.position.x = (cw ? 1 : -1) * r;
    seg.rotation.x = (cw ? 1 : -1) * (0.35 - 0.25*t);
    seg.position.y = -t*t*0.005;
    blade.add(seg);
  }
  const tipCap = new THREE.Mesh(
    new THREE.SphereGeometry(0.006, 6, 4),
    new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.5 })
  );
  tipCap.scale.set(0.5, 0.3, 1.0);
  tipCap.position.x = (cw ? 1 : -1) * (0.012 + bladeLen);
  blade.add(tipCap);
  return blade;
}

armConfigs.forEach((cfg) => {
  const dx = Math.sin(cfg.angle) * ARM_LEN;
  const dz = Math.cos(cfg.angle) * ARM_LEN;

  // Arm: tapered carbon tube
  const armGeo = new THREE.CylinderGeometry(0.009, 0.012, ARM_LEN, 8);
  const arm = new THREE.Mesh(armGeo, matCarbon);
  arm.rotation.z = Math.PI/2;
  arm.rotation.y = -cfg.angle + Math.PI/2;
  arm.position.set(dx/2, 0.005, dz/2); quad.add(arm);

  // Arm reinforcement rib
  const rib = new THREE.Mesh(new THREE.BoxGeometry(ARM_LEN*0.7, 0.018, 0.004), matCarbon);
  rib.rotation.y = -cfg.angle + Math.PI/2;
  rib.position.set(dx*0.55, 0.005, dz*0.55); quad.add(rib);

  // Motor base
  const motorBase = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.024, 0.008, 12), matCarbon);
  motorBase.position.set(dx, 0.0, dz); quad.add(motorBase);

  // Motor stator
  const stator = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.018, 10), matMotor);
  stator.position.set(dx, 0.013, dz); quad.add(stator);

  // Motor bell
  const bellTop = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.020, 0.012, 12), matMotorBell);
  bellTop.position.set(dx, 0.024, dz); quad.add(bellTop);
  const bellRing = new THREE.Mesh(new THREE.TorusGeometry(0.019, 0.002, 6, 16), matMotorBell);
  bellRing.rotation.x = Math.PI/2; bellRing.position.set(dx, 0.018, dz); quad.add(bellRing);
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.01, 6), matMotor);
  shaft.position.set(dx, 0.035, dz); quad.add(shaft);

  // Propeller group (rotates)
  const propGroup = new THREE.Group();
  propGroup.position.set(dx, 0.038, dz);
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.005, 8), matCarbon);
  propGroup.add(hub);
  for (let b = 0; b < 2; b++) {
    const blade = createPropBlade(cfg.cw);
    blade.rotation.y = b * Math.PI;
    propGroup.add(blade);
  }
  quad.add(propGroup);
  propGroups.push({ group: propGroup, cw: cfg.cw });

  // Prop guard
  const guard = new THREE.Mesh(new THREE.TorusGeometry(0.13, 0.004, 6, 32), matGuard);
  guard.rotation.x = Math.PI/2; guard.position.set(dx, 0.032, dz); quad.add(guard);
  for (const strutAngle of [cfg.angle - 0.3, cfg.angle + 0.3]) {
    const sx = Math.sin(strutAngle)*0.12, sz = Math.cos(strutAngle)*0.12;
    const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.002, 0.002, 0.04, 4), matCarbon);
    strut.position.set(dx+sx*0.3, 0.025, dz+sz*0.3);
    strut.rotation.z = (sx > 0 ? -1 : 1)*0.4;
    strut.rotation.x = (sz > 0 ? 1 : -1)*0.4;
    quad.add(strut);
  }

  // Navigation LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(0.005, 8, 6),
    new THREE.MeshStandardMaterial({ color: cfg.ledColor, emissive: cfg.ledColor, emissiveIntensity: 2.0 })
  );
  led.position.set(dx*0.85, -0.015, dz*0.85); quad.add(led);
  const ledLight = new THREE.PointLight(cfg.ledColor, 0.15, 0.3);
  ledLight.position.copy(led.position); quad.add(ledLight);
});

// ── LANDING GEAR ──
for (const side of [-1, 1]) {
  const skidZ = side * 0.055;
  const rail = new THREE.Mesh(new THREE.CapsuleGeometry(0.005, 0.22, 4, 8), matSkid);
  rail.rotation.z = Math.PI/2; rail.position.set(0, -0.065, skidZ); quad.add(rail);
  for (const fb of [-0.06, 0.06]) {
    const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.005, 0.045, 6), matSkid);
    strut.position.set(fb, -0.042, skidZ); strut.rotation.z = side*0.08; quad.add(strut);
  }
  const brace = new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.12, 4), matSkid);
  brace.rotation.z = Math.PI/2; brace.position.set(0, -0.048, skidZ); quad.add(brace);
  for (const xOff of [-0.11, 0.11]) {
    const foot = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 6, 4),
      new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.9 })
    );
    foot.position.set(xOff, -0.068, skidZ); foot.scale.y = 0.5; quad.add(foot);
  }
}

// ── REAR STATUS LED BAR ──
const ledBar = new THREE.Mesh(
  new THREE.BoxGeometry(0.06, 0.006, 0.004),
  new THREE.MeshStandardMaterial({ color: 0x00aaff, emissive: 0x0066ff, emissiveIntensity: 1.5 })
);
ledBar.position.set(0, 0.01, -0.09); quad.add(ledBar);

// ── FRONT HEADLIGHT ──
const headlight = new THREE.Mesh(
  new THREE.SphereGeometry(0.004, 6, 4),
  new THREE.MeshStandardMaterial({ color: 0xffffee, emissive: 0xffffcc, emissiveIntensity: 3.0 })
);
headlight.position.set(0, -0.01, 0.092); quad.add(headlight);
const headlightBeam = new THREE.SpotLight(0xffffdd, 0.4, 2, 0.5, 0.8);
headlightBeam.position.set(0, -0.01, 0.092);
headlightBeam.target.position.set(0, -0.5, 0.5);
quad.add(headlightBeam); quad.add(headlightBeam.target);

scene.add(quad);

// ═══ FLIGHT TRAIL ═══
const trailGeo = new THREE.BufferGeometry();
const trailPos = new Float32Array(TRAIL_MAX * 3);
trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
trailGeo.setDrawRange(0, 0);
const trail = new THREE.Line(trailGeo,
  new THREE.LineBasicMaterial({ color: 0x1155cc, transparent: true, opacity: 0.6 })
);
scene.add(trail); let trailCount = 0;

// ═══ GROUND SHADOW ═══
const shadow = new THREE.Mesh(
  new THREE.CircleGeometry(0.2, 20),
  new THREE.MeshBasicMaterial({ color: 0x3a2a10, transparent: true, opacity: 0.3 })
);
shadow.rotation.x = -Math.PI/2; shadow.position.y = 0.003; scene.add(shadow);

let ws = null, latestState = null;
function connectWS() {
  ws = new WebSocket("ws://localhost:__WS_PORT__");
  ws.onopen = () => { document.getElementById("ws-status").textContent = "Connected"; document.getElementById("ws-status").className = "connected"; };
  ws.onclose = () => { document.getElementById("ws-status").textContent = "Disconnected"; document.getElementById("ws-status").className = "disconnected"; setTimeout(connectWS, 2000); };
  ws.onerror = () => { ws.close(); };
  ws.onmessage = (e) => { try { latestState = JSON.parse(e.data); } catch(err) {} };
}
connectWS();

const controlsMap = {
  keyboard: [
    ["<b>Keyboard Controls</b>"],
    ["Up / Down", "Throttle"],
    ["Left / Right", "Yaw"],
    ["W / S", "Pitch fwd/back"],
    ["A / D", "Roll left/right"],
    ["Space", "Arm / Disarm"],
    ["Q", "Quit"],
  ],
  gamepad: [
    ["<b>Gamepad Controls</b>"],
    ["Left Stick Y", "Throttle (ramp)"],
    ["Left Stick X", "Yaw"],
    ["Right Stick Y", "Pitch"],
    ["Right Stick X", "Roll"],
    ["Start", "Arm / Disarm"],
  ],
  test: [
    ["<b>Test Sequence</b>"],
    ["", "Scripted flight pattern"],
    ["", "Arms, hovers, maneuvers"],
    ["", "Loops every 30s"],
  ],
};

let currentInputMode = "";
function updateControls(mode) {
  if (mode === currentInputMode) return;
  currentInputMode = mode;
  const entries = controlsMap[mode] || controlsMap["keyboard"];
  const el = document.getElementById("controls");
  el.innerHTML = entries.map((e, i) => {
    if (i === 0) return e[0];
    if (!e[0]) return '<span class="action">' + e[1] + '</span>';
    return '<span class="key">' + e[0] + '</span> <span class="action">' + e[1] + '</span>';
  }).join("<br>");
}
updateControls("keyboard");

function animate() {
  requestAnimationFrame(animate);
  if (latestState) {
    const s = latestState;
    const px = s.px||0, py = s.py||0, pz = s.pz||0;
    const [tx, ty, tz] = nedToThree(px, py, pz);
    quad.position.set(tx, ty, tz);
    const R11=s.R11||1,R12=s.R12||0,R13=s.R13||0,R21=s.R21||0,R22=s.R22||1,R23=s.R23||0,R31=s.R31||0,R32=s.R32||0,R33=s.R33||1;
    const m=quad.matrix, e=m.elements;
    e[0]=R22;e[1]=-R32;e[2]=R12;e[3]=0; e[4]=-R23;e[5]=R33;e[6]=-R13;e[7]=0;
    e[8]=R21;e[9]=-R31;e[10]=R11;e[11]=0; e[12]=tx;e[13]=ty;e[14]=tz;e[15]=1;
    quad.matrixAutoUpdate=false; quad.matrixWorldNeedsUpdate=true;

    // Spin propellers proportional to motor speed
    const omegas=[s.omega_m1||0,s.omega_m2||0,s.omega_m3||0,s.omega_m4||0];
    const avgOmega = (omegas[0]+omegas[1]+omegas[2]+omegas[3])/4;
    propGroups.forEach((p, i) => {
      const dir = p.cw ? 1 : -1;
      p.group.rotation.y += dir * (0.3 + Math.min(omegas[i]/500, 2.0) * 0.5);
    });

    shadow.position.set(tx,0.003,tz);
    const alt=Math.max(ty,0.01),sc=Math.max(0.4,1.2-alt*0.04);
    shadow.scale.set(sc,sc,1); shadow.material.opacity=Math.max(0.03,0.25-alt*0.015);
    const idx2=trailCount%TRAIL_MAX;
    trailPos[idx2*3]=tx;trailPos[idx2*3+1]=ty;trailPos[idx2*3+2]=tz;trailCount++;
    trail.geometry.attributes.position.needsUpdate=true;trail.geometry.setDrawRange(0,Math.min(trailCount,TRAIL_MAX));
    camTarget.lerp(new THREE.Vector3(tx,ty,tz),0.05);
    document.getElementById("v-t").textContent=(s.t||0).toFixed(2);
    document.getElementById("v-pos").textContent=`${px.toFixed(2)}, ${py.toFixed(2)}, ${pz.toFixed(2)}`;
    document.getElementById("v-alt").textContent=(-pz).toFixed(2);
    document.getElementById("v-vel").textContent=Math.sqrt((s.vx||0)**2+(s.vy||0)**2+(s.vz||0)**2).toFixed(2);
    const q0=s.q0||1,q1=s.q1||0,q2=s.q2||0,q3=s.q3||0;
    document.getElementById("v-roll").textContent=(Math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*180/Math.PI).toFixed(1);
    document.getElementById("v-pitch").textContent=(Math.asin(Math.max(-1,Math.min(1,2*(q0*q2-q3*q1))))*180/Math.PI).toFixed(1);
    document.getElementById("v-motors").textContent=omegas.map(o=>o.toFixed(0)).join(", ");
    document.getElementById("v-thrust").textContent=(s.T||0).toFixed(2);
    document.getElementById("v-accel").textContent=`${(s.accel_x||0).toFixed(2)}, ${(s.accel_y||0).toFixed(2)}, ${(s.accel_z||0).toFixed(2)}`;
    document.getElementById("v-gyro").textContent=`${(s.gyro_x||0).toFixed(2)}, ${(s.gyro_y||0).toFixed(2)}, ${(s.gyro_z||0).toFixed(2)}`;
    document.getElementById("v-mag").textContent=`${(s.mag_x||0).toFixed(3)}, ${(s.mag_y||0).toFixed(3)}, ${(s.mag_z||0).toFixed(3)}`;
    if (s.mode) document.getElementById("v-mode-text").textContent = s.mode;
    if (s.input_mode) updateControls(s.input_mode);
  }
  camera.position.set(camTarget.x+camDist*Math.sin(camAngle)*Math.cos(camElev),camTarget.y+camDist*Math.sin(camElev),camTarget.z+camDist*Math.cos(camAngle)*Math.cos(camElev));
  camera.lookAt(camTarget); renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>"##;

// ===========================================================================
// HTTP server
// ===========================================================================

fn serve_http(listener: TcpListener, html: String) {
    for stream in listener.incoming() {
        let Ok(mut stream) = stream else { continue };
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut request_line = String::new();
        let _ = reader.read_line(&mut request_line);
        loop {
            let mut line = String::new();
            let _ = reader.read_line(&mut line);
            if line.trim().is_empty() { break; }
        }
        let response = format!(
            "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            html.len(), html
        );
        let _ = stream.write_all(response.as_bytes());
    }
}

// ===========================================================================
// Main
// ===========================================================================

fn ctrlc_handler() {
    unsafe {
        libc::signal(libc::SIGINT, handle_sigint as libc::sighandler_t);
        libc::signal(libc::SIGTERM, handle_sigint as libc::sighandler_t);
    }
}

extern "C" fn handle_sigint(_: libc::c_int) {
    SHUTDOWN.store(true, Ordering::SeqCst);
    crossterm::terminal::disable_raw_mode().ok();
    std::process::exit(0);
}

/// Build the values HashMap for the pack (send) codec from sensor outputs
/// and input state. Variable names match the TOML routing config.
fn build_send_values(sensors: &SensorOutput, input: &InputState) -> HashMap<String, f64> {
    let mut v = HashMap::with_capacity(24);

    // IMU sensors
    v.insert("gyro_x".into(), sensors.gyro[0]);
    v.insert("gyro_y".into(), sensors.gyro[1]);
    v.insert("gyro_z".into(), sensors.gyro[2]);
    v.insert("accel_x".into(), sensors.accel[0]);
    v.insert("accel_y".into(), sensors.accel[1]);
    v.insert("accel_z".into(), sensors.accel[2]);

    // RC channels
    for i in 0..16 {
        v.insert(format!("rc_{i}"), input.rc[i] as f64);
    }

    // Link status
    let connected = input.is_connected();
    v.insert("rc_link_quality".into(), if connected { 255.0 } else { 0.0 });
    v.insert("rc_valid".into(), if connected { 1.0 } else { 0.0 });
    v.insert("imu_valid".into(), 1.0);

    v
}

fn main() -> anyhow::Result<()> {
    ctrlc_handler();

    // Load TOML config
    let config_path = std::env::args().nth(1).unwrap_or_else(|| "sil_config.toml".into());
    let cfg = config::SilConfig::load(Path::new(&config_path))?;
    eprintln!("Loaded config from {config_path}");

    let dt = cfg.sim.dt;
    let realtime = cfg.sim.realtime;
    let test_mode = cfg.sim.test;

    // Load .bfbs schemas and compile codecs
    let mut schema = bfbs::SchemaSet::new();
    for bfbs_path in &cfg.schema.bfbs {
        eprintln!("Loading schema: {bfbs_path}");
        schema.load_bfbs(Path::new(bfbs_path))?;
    }
    let unpack_codec = codec::UnpackCodec::compile(&schema, &cfg.receive)?;
    let pack_codec = codec::PackCodec::compile(&schema, &cfg.send)?;
    eprintln!(
        "Codecs compiled: receive {} ({} bytes, {} fields), send {} ({} bytes, {} fields)",
        cfg.receive.root_type,
        unpack_codec.expected_size(),
        cfg.receive.route.len(),
        cfg.send.root_type,
        pack_codec.size(),
        cfg.send.route.len(),
    );

    // Compile physics model
    eprintln!("Compiling QuadrotorSIL model...");
    let mut sil = QuadrotorSil::new()?;
    eprintln!("Inputs: {:?}", sil.stepper.input_names());

    let omega_hover = hover_omega();
    eprintln!(
        "Hover omega: {:.1} rad/s ({:.0} RPM)",
        omega_hover,
        omega_hover * 60.0 / (2.0 * std::f64::consts::PI)
    );

    let udp_mode = cfg.udp.is_some();
    if udp_mode {
        let udp = cfg.udp.as_ref().unwrap();
        eprintln!("UDP mode: listen={} send={} dt={}s realtime={}", udp.listen, udp.send, dt, realtime);
    } else {
        eprintln!("Self-test mode (no [udp] in config).");
    }

    // Prepare HTML
    let html = HTML_PAGE
        .replace("__THREE_JS__", THREE_JS)
        .replace("__WS_PORT__", &WS_PORT.to_string());

    // Start HTTP server
    let http_listener = TcpListener::bind(format!("0.0.0.0:{HTTP_PORT}"))?;
    eprintln!("HTTP server: http://localhost:{HTTP_PORT}");
    thread::spawn(move || serve_http(http_listener, html));

    // WebSocket thread for viz
    let (state_tx, state_rx) = mpsc::channel::<String>();
    thread::spawn(move || {
        let ws_listener = TcpListener::bind(format!("0.0.0.0:{WS_PORT}")).unwrap();
        eprintln!("WebSocket: ws://localhost:{WS_PORT}");
        eprintln!("\nOpen http://localhost:{HTTP_PORT} in your browser!\n");
        for stream in ws_listener.incoming() {
            let Ok(stream) = stream else { continue };
            eprintln!("Viewer connected");
            let mut ws = match accept(stream) {
                Ok(ws) => ws,
                Err(e) => { eprintln!("WS error: {e}"); continue; }
            };
            ws.get_ref().set_nonblocking(true).ok();
            ws.get_ref().set_write_timeout(Some(Duration::from_millis(100))).ok();
            let mut alive = true;
            while alive {
                loop {
                    match ws.read() {
                        Ok(Message::Close(_)) => { alive = false; break; }
                        Err(tungstenite::Error::Io(ref e)) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                        Err(_) => { alive = false; break; }
                        _ => {}
                    }
                }
                if !alive { break; }
                if let Ok(json) = state_rx.try_recv() {
                    let mut latest = json;
                    while let Ok(newer) = state_rx.try_recv() { latest = newer; }
                    if ws.send(Message::Text(latest.into())).is_err() { break; }
                }
                thread::sleep(Duration::from_millis(16));
            }
        }
    });

    if let Some(ref udp_cfg) = cfg.udp {
        // ===== UDP mode: receive motor commands, step physics, send sensor data =====
        let socket = UdpSocket::bind(&udp_cfg.listen)?;
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;
        eprintln!("Listening on {}", udp_cfg.listen);
        eprintln!("Sending to {}", udp_cfg.send);

        let input_mode = if test_mode { InputMode::Test } else { InputMode::Gamepad };
        let mut input = InputState::new(input_mode);
        let mut recv_buf = [0u8; 512];
        let mut armed = false;
        let mut pkt_count = 0u64;
        let mut first_nonzero_motors = true;
        let mut motor_rpms = [0.0f64; 4]; // persists across frames

        let recv_expected = unpack_codec.expected_size();
        eprintln!("Expecting {recv_expected}-byte receive packets on {}", udp_cfg.listen);

        loop {
            let frame_start = Instant::now();
            input.poll();

            // Reset: gamepad A or keyboard R
            if input.reset_requested {
                input.reset_requested = false;
                motor_rpms = [0.0; 4];
                armed = false;
                input.armed = false;
                input.throttle = 0.0;
                input.rc[2] = RC_MIN;
                input.rc[4] = RC_MIN;
                if let Err(e) = sil.reset() {
                    eprintln!("[reset] error: {e}");
                } else {
                    eprintln!("[reset] simulation reset");
                }
            }

            // Drain all queued packets, updating motor commands.
            // motor_rpms persists — if no new packet arrives this frame,
            // physics keeps stepping with the last known values.
            socket.set_nonblocking(true).ok();
            loop {
                match socket.recv_from(&mut recv_buf) {
                    Ok((n, _src)) => {
                        pkt_count += 1;
                        if n == recv_expected {
                            let values = unpack_codec.unpack(&recv_buf[..n]);
                            if !values.is_empty() {
                                motor_rpms = [
                                    values.get("omega_m1").copied().unwrap_or(0.0),
                                    values.get("omega_m2").copied().unwrap_or(0.0),
                                    values.get("omega_m3").copied().unwrap_or(0.0),
                                    values.get("omega_m4").copied().unwrap_or(0.0),
                                ];
                                armed = values.get("armed").copied().unwrap_or(0.0) != 0.0;

                                if first_nonzero_motors && motor_rpms.iter().any(|&m| m > 0.01) {
                                    eprintln!("\r[udp] FIRST NON-ZERO MOTORS: [{:.1},{:.1},{:.1},{:.1}] armed={} t={:.4}    ",
                                        motor_rpms[0], motor_rpms[1], motor_rpms[2], motor_rpms[3], armed, sil.time());
                                    first_nonzero_motors = false;
                                }
                            }
                        }
                    }
                    Err(_) => break, // WouldBlock — queue drained
                }
            }
            socket.set_nonblocking(false).ok();

            // Always step physics with latest motor values
            let target_clock = sil.time() + dt;
            if let Err(e) = sil.receive_motors(motor_rpms, target_clock) {
                eprintln!("\r[udp] Step error: {e}");
            }

            // Always send sensor data so cerebri doesn't starve
            let sensors = sil.read_sensors();
            let send_vals = build_send_values(&sensors, &input);
            let buf = pack_codec.pack(&send_vals);
            let _ = socket.send_to(&buf, &udp_cfg.send);

            // Status logging
            if pkt_count > 0 && pkt_count % 250 == 0 {
                eprintln!(
                    "\r[udp] t={:.1}s alt={:.2}m motors=[{:.0},{:.0},{:.0},{:.0}] armed={} rc=[{},{},{},{},arm={}] input={}    ",
                    sensors.clock_sec,
                    -sensors.position_ned[2],
                    motor_rpms[0], motor_rpms[1], motor_rpms[2], motor_rpms[3],
                    armed,
                    input.rc[0], input.rc[1], input.rc[2], input.rc[3], input.rc[4],
                    input.mode_name(),
                );
            }

            // Viz update
            let mode_str = if pkt_count > 0 {
                format!("UDP ({}, {})", if armed { "ARMED" } else { "disarmed" }, input.mode_name())
            } else {
                format!("UDP (waiting, {})", input.mode_name())
            };
            let mut json = sil.state_json();
            json.pop();
            json.push_str(&format!(
                r#","mode":"{}","armed":{},"rc_throttle":{},"input_connected":{},"input_mode":"{}"}}"#,
                mode_str, armed, input.rc[2], input.is_connected(), input.mode_name(),
            ));
            let _ = state_tx.send(json);

            if realtime {
                let elapsed = frame_start.elapsed();
                let target = Duration::from_secs_f64(dt);
                if elapsed < target {
                    thread::sleep(target - elapsed);
                }
            }
        }
    } else {
        // ===== Self-test mode: no UDP =====
        eprintln!("Running self-test: drone on ground (no motors)\n");
        let input_mode = if test_mode { InputMode::Test } else { InputMode::Gamepad };
        let mut input = InputState::new(input_mode);
        let mut frame_count = 0u64;

        loop {
            let frame_start = Instant::now();
            input.poll();

            if input.reset_requested {
                input.reset_requested = false;
                input.armed = false;
                input.throttle = 0.0;
                input.rc[2] = RC_MIN;
                input.rc[4] = RC_MIN;
                if let Err(e) = sil.reset() {
                    eprintln!("[reset] error: {e}");
                } else {
                    eprintln!("[reset] simulation reset");
                }
            }

            let motor_rpms = [0.0; 4];
            let target_clock = sil.time() + dt;
            match sil.receive_motors(motor_rpms, target_clock) {
                Ok(sensors) => {
                    frame_count += 1;
                    if frame_count % 250 == 0 {
                        eprintln!(
                            "\r[sil] t={:.1}s alt={:.3}m accel_z={:.2} gyro=[{:.3},{:.3},{:.3}] input={}    ",
                            sensors.clock_sec,
                            -sensors.position_ned[2],
                            sensors.accel[2],
                            sensors.gyro[0], sensors.gyro[1], sensors.gyro[2],
                            input.mode_name(),
                        );
                    }
                }
                Err(e) => eprintln!("\r[sil] Step error: {e}"),
            }

            let mut json = sil.state_json();
            json.pop();
            json.push_str(&format!(
                r#","mode":"Self-test (ground, {})","input_mode":"{}"}}"#,
                input.mode_name(),
                input.mode_name(),
            ));
            let _ = state_tx.send(json);

            if realtime {
                let elapsed = frame_start.elapsed();
                let target = Duration::from_secs_f64(dt);
                if elapsed < target {
                    thread::sleep(target - elapsed);
                }
            }
        }
    }
}
