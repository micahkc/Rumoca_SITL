# Quadrotor SIL Plant Simulator

Software-in-the-loop plant model for a quadrotor. Communicates with any
flatbuffer-based flight controller via UDP. The protocol is driven by a
TOML config file + `.bfbs` (binary flatbuffer schema) reflection — no
hand-coded pack/unpack. Swap the config and schema to work with a
different controller.

## Quick Start

```bash
# Edit sil_config.toml to set .bfbs paths and UDP addresses, then:
cargo run

# Or specify a custom config:
cargo run -- path/to/my_config.toml
```

Open http://localhost:8080 for the Three.js visualization.

## Configuration

All settings live in `sil_config.toml` (or a custom path passed as arg):

```toml
[sim]
dt = 0.004        # timestep [s]
realtime = true    # wall-clock pacing
test = false       # scripted test flight

[udp]              # omit this section for self-test mode (no UDP)
listen = "0.0.0.0:4243"
send = "127.0.0.1:4242"

[schema]
bfbs = ["path/to/cerebri2_topics.bfbs", "path/to/cerebri2_sil.bfbs"]

[receive]
root_type = "cerebri2.topic.MotorOutput"

[receive.route]
"motors.m0" = { var = "omega_m1", scale = 1100.0 }
# ...

[send]
root_type = "cerebri2.sil.SimInput"

[send.route]
"gyro.x" = { var = "gyro_x" }
# ...
```

See `sil_config.toml` for the full cerebri2 configuration.

## Protocol

### Receive: motor_output (48 bytes)

Cerebri sends this flatbuffer packet containing:
- `motors[4]`: float, normalised 0..1 motor commands
- `raw[4]`: uint16, PWM microseconds
- `armed`: bool
- `test_mode`: bool

The normalised motor values are converted to rad/s:
`omega = motors[i] * OMEGA_MAX` (default OMEGA_MAX = 1100 rad/s)

### Send: flight_snapshot (164 bytes)

Rumoca sends back this flatbuffer packet containing:
- `imu.gyro_rad_s[3]`: body-frame angular velocity [rad/s]
- `imu.accel_m_s2[3]`: body-frame specific force [m/s^2]
- `rc_channels[16]`: zeros (plant sim has no RC)
- `control_status`: armed state, imu_ok=true, timestamps
- `rate_desired[3]`, `rate_cmd[3]`: current body rates

## Data Flow

```
Cerebri                        Rumoca SIL                    Browser
  |                               |                             |
  |-- motor_output (48B, UDP) --> |                             |
  |                               |-- step physics (dt)         |
  |                               |-- compute sensors           |
  |                               |-- stream JSON -----------> | (WS:8081)
  |<- flight_snapshot (164B) ---- |                             |
```

## Conventions

- **World frame:** NED (North-East-Down)
- **Body frame:** FRD (Forward-Right-Down)
- **Quaternion:** `{w, x, y, z}` scalar-first, body-to-world
- **Motor layout:** X-config matching Cerebri MixQuadX
  - Motor 1: front-right, CW
  - Motor 2: rear-right, CCW
  - Motor 3: rear-left, CW
  - Motor 4: front-left, CCW

## Rust API (for custom integration)

```rust
use quadrotor_sil::{QuadrotorSil, SensorOutput};

let mut sil = QuadrotorSil::new()?;

// From your UDP/protocol layer:
let motor_rpms = [omega1, omega2, omega3, omega4]; // rad/s
let sensors = sil.receive_motors(motor_rpms, target_clock_sec)?;

// sensors.accel  — body-frame accelerometer [m/s^2]
// sensors.gyro   — body-frame gyroscope [rad/s]
// sensors.mag    — body-frame magnetometer [Gauss]

// Convert to flatbuffer for sending:
let snap = sil.sensor_to_snapshot(&sensors, armed);
let buf = cerebri_fb::pack_flight_snapshot(&snap);
```

## Physical Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| mass | 2.0 | kg |
| Ixx | 0.020 | kg*m^2 |
| Iyy | 0.020 | kg*m^2 |
| Izz | 0.040 | kg*m^2 |
| Ct (thrust coeff) | 8.5e-6 | N/(rad/s)^2 |
| Cm (torque coeff) | 1.36e-7 | N*m/(rad/s)^2 |
| arm_length | 0.2 | m |
| OMEGA_MAX | 1100 | rad/s |
| hover omega | ~759.5 | rad/s |

## Ports

- HTTP: 8080 (Three.js viewer)
- WebSocket: 8081 (state streaming)
- UDP: configurable via env vars
# Rumoca_SITL
