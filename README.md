# Autonomous Drone Simulation System

A modular, fixed-timestep simulation of a quadrotor drone with full flight dynamics, PID-based control, IMU sensor modeling, complementary filter state estimation, waypoint navigation, and failsafe handling. Written in C++20 with no external dependencies beyond the standard library.

---

## Architecture

The system is structured as a library (`adsim_core`) consumed by a CLI executable (`adsim`). Each module has a clear boundary and communicates through value types rather than shared state.

```
adsim_core
├── math        — Vec3, Quaternion, PID controller
├── state       — DroneState (position, velocity, attitude, angular velocity)
├── dynamics    — FlightModel (Newtonian + gyroscopic integration)
├── control     — FlightController (cascaded PID: position → attitude → rate)
├── sensors     — IMU (accelerometer + gyroscope with noise and dropout)
├── estimation  — ComplementaryFilter (attitude fusion from accel + gyro)
├── navigation  — WaypointNavigator (sequential waypoint tracking)
├── failsafe    — FailsafeMonitor (sensor loss, attitude limits, bounds)
├── simulation  — Simulator (loop orchestration, config binding)
├── logging     — CsvLogger (machine-readable time-series output)
├── config      — Config (INI-style parser, no third-party dependencies)
└── network     — TelemetryServer (TCP streaming, line-delimited JSON)

ui-java         — JavaFX telemetry monitor (2D top-down + 3D orbital view)
```

---

## Module Breakdown

**math**
`Vec3` and `Quaternion` are plain structs with arithmetic operators and no dynamic allocation. `PidController` maintains integral and derivative state across calls with clamped output.

**dynamics**
`FlightModel` integrates the rigid-body equations of motion using a simplified quadrotor model: thrust along the body z-axis, linear aerodynamic drag, gyroscopic torque coupling, and angular drag. The integration uses a semi-implicit Euler scheme (second-order position update, first-order attitude via quaternion axis-angle).

**control**
`FlightController` implements a cascaded structure: outer position loops generate desired pitch/roll angles and a thrust correction; inner attitude loops drive angular rate setpoints; rate loops generate body torques. All loops are independent PID instances with separate gain tuning.

**sensors**
`Imu` transforms true state into body-frame accelerometer and gyroscope readings with additive Gaussian noise and random-walk bias. Dropout events are modeled probabilistically and as forced injections.

**estimation**
`ComplementaryFilter` fuses gyroscope-integrated attitude with accelerometer-derived gravity direction. The accelerometer correction is applied as a small quaternion rotation proportional to the cross-product error. Gyro bias is slowly corrected using the same error signal.

**navigation**
`WaypointNavigator` holds an ordered list of waypoints with per-waypoint acceptance radii. It advances the active waypoint when the drone enters the acceptance sphere and exposes the current target as a `ControlTarget` for the flight controller.

**failsafe**
`FailsafeMonitor` tracks three independent conditions:
- Sensor loss: IMU dropout sustained beyond a configurable timeout → controlled descent
- Attitude violation: roll or pitch exceeding limits for longer than the recovery window → hover hold
- Bounds violation: altitude or position outside envelope, or invalid state → abort (sticky)

The abort state is terminal within a run.

**simulation**
`Simulator` owns all subsystems and runs the fixed-timestep loop. At each step it samples the IMU, updates the estimator, evaluates failsafe, computes the control target, runs the controller, integrates dynamics, logs at a configurable decimated rate, and optionally streams telemetry over TCP.

**network**
`TelemetryServer` listens on a TCP port and accepts one client at a time. A background thread handles accept; the simulation thread calls `try_publish()` which rate-limits output to a configurable Hz and serializes a `TelemetryFrame` as line-delimited JSON. Uses POSIX sockets on Linux/macOS and Winsock2 on Windows with no external dependencies.

**logging**
`CsvLogger` writes one header row and one data row per logged step. All state, IMU readings, estimated attitude, failsafe state, and active waypoint index are recorded.

**config**
`Config` parses a simple INI-style file with `[section]` headers and `key = value` pairs. Comments begin with `#`. Values are accessed with typed getters that return defaults when a key is absent.

---

## Build

Requires CMake 3.20+ and a C++20-capable compiler (GCC 10+, Clang 12+, MSVC 2022+).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

Tests are built by default:

```bash
ctest --test-dir build --output-on-failure
```

To disable tests:

```bash
cmake -S . -B build -DADSIM_BUILD_TESTS=OFF
```

A convenience script is provided for Linux:

```bash
bash scripts/build.sh Release
```

---

## Running

```
adsim -c <config.ini> [-o <output.csv>] [-v] [--stream] [--wait] [--host <addr>] [--port <port>]

Options:
  -c, --config <path>    Configuration file (required)
  -o, --output <path>    Log output path (default: sim_output.csv)
  -v, --verbose          Print step-by-step state
      --stream           Enable TCP telemetry streaming
      --wait             Wait for a client to connect before starting
      --host <addr>      Telemetry bind address (default: 127.0.0.1)
      --port <port>      Telemetry TCP port (default: 5760)
  -h, --help             Show usage
```

Example:

```bash
./build/adsim -c config/default.ini -o logs/run1.csv

# With live telemetry (waits for Java UI to connect first):
./build/adsim -c config/default.ini --wait --port 5760
```

Exit codes:
- `0` — mission completed or simulation ran to time limit normally
- `1` — startup error (bad config, missing file)
- `2` — simulation terminated in abort failsafe state

---

## Configuration

Configuration files use a simple INI format. All parameters have defaults; only the sections relevant to overrides need to be present.

```ini
[simulation]
timestep  = 0.005    # integration step (s)
duration  = 90.0     # max run time (s)
log_rate  = 0.02     # minimum time between log rows (s)

[flight_model]
mass      = 1.5      # kg
gravity   = 9.81     # m/s^2
inertia_x = 0.020    # kg·m^2, roll axis
inertia_y = 0.020    # kg·m^2, pitch axis
inertia_z = 0.040    # kg·m^2, yaw axis

[controller]
alt_kp = 2.5   alt_ki = 0.10   alt_kd = 1.20
roll_kp = 4.0  roll_ki = 0.05  roll_kd = 0.80
# ... (see config/default.ini for all gains)

[imu]
accel_noise_std     = 0.05   # m/s^2
gyro_noise_std      = 0.005  # rad/s
dropout_probability = 0.0    # per-second probability of a dropout event

[estimator]
alpha = 0.02   # complementary filter accelerometer weight [0, 1]

[failsafe]
max_roll_deg           = 50.0
max_pitch_deg          = 50.0
max_altitude           = 150.0
sensor_timeout         = 0.5    # s
attitude_recovery_time = 2.0    # s before hover hold triggers

[waypoint.0]
x = 0.0   y = 0.0   z = 8.0   yaw = 0.0   radius = 0.8

[waypoint.1]
x = 20.0  y = 0.0   z = 8.0   yaw = 0.0   radius = 1.0
```

Waypoints are indexed from `0` sequentially. The navigator stops scanning when it encounters an index with no `x` or `z` key.

---

## Log Format

The CSV output contains one row per logged timestep. Column groups:

| Group | Columns |
|---|---|
| Time | `timestamp` |
| Position | `pos_x`, `pos_y`, `pos_z` |
| Velocity | `vel_x`, `vel_y`, `vel_z` |
| Acceleration | `accel_x`, `accel_y`, `accel_z` |
| True attitude (quaternion) | `att_w`, `att_x`, `att_y`, `att_z` |
| True attitude (Euler) | `roll`, `pitch`, `yaw` |
| Angular velocity | `angvel_x`, `angvel_y`, `angvel_z` |
| IMU accelerometer | `imu_accel_x`, `imu_accel_y`, `imu_accel_z` |
| IMU gyroscope | `imu_gyro_x`, `imu_gyro_y`, `imu_gyro_z` |
| IMU validity | `imu_valid` |
| Estimated attitude (quaternion) | `est_att_w`, ..., `est_att_z` |
| Estimated attitude (Euler) | `est_roll`, `est_pitch`, `est_yaw` |
| Failsafe | `failsafe` (`nominal`, `hover_hold`, `controlled_descent`, `abort`) |
| Navigation | `waypoint_index` |

Plotting with the bundled script (requires `matplotlib`):

```bash
python3 scripts/plot_log.py logs/run1.csv
python3 scripts/plot_log.py logs/run1.csv -o logs/run1.png
python3 scripts/plot_log.py logs/run1.csv --stats
```

---

## Sample Output

```
adsim starting
  Config  : config/default.ini
  Output  : sim_output.csv
  Duration: 90.0 s
  Dt      : 0.005 s
  Waypoints: 5

--- Simulation Summary ---
  Duration simulated : 72.440 s
  Mission complete   : yes
  Waypoints          : 5 / 5
  Final position     : (0.022, 0.017, 0.503)
  Final altitude     : 0.503 m
  Failsafe state     : nominal
  Log records        : 3622
--------------------------
```

---

## Telemetry Protocol

When `--stream` is active the server emits one JSON object per line at up to 30 Hz:

```json
{"t":1.234,"pos":{"x":1.0,"y":2.0,"z":3.0},"vel":{"x":0.1,"y":0.0,"z":0.0},
 "accel":{"x":0.0,"y":0.0,"z":0.0},"att":{"w":1.0,"x":0.0,"y":0.0,"z":0.0},
 "euler":{"x":0.0,"y":0.0,"z":0.0},"angvel":{"x":0.0,"y":0.0,"z":0.0},
 "wp_idx":1,"wp_total":5,"wp_target":{"x":20.0,"y":0.0,"z":8.0},
 "mission_complete":false,"failsafe":"nominal"}
```

Coordinate frame: x = North, y = East, z = Up.

---

## Java UI

A standalone JavaFX visualization client lives in `ui-java/`. Requires Java 21+ and Gradle (wrapper included).

```bash
# Terminal 1 — simulation (waits for client before starting)
./build/adsim -c config/default.ini --wait --port 5760

# Terminal 2 — visualization
cd ui-java
./gradlew run
```

The UI provides:
- **2D view** — top-down canvas with waypoint path, flight trail, and drone indicator; pan with left-drag, zoom with scroll
- **3D view** — SubScene with orbital camera (left-drag to rotate, scroll to zoom), drone body model with live quaternion orientation, waypoint spheres, and trail cubes
- **Telemetry panel** — live readout of position, velocity, attitude, angular rates, waypoint progress, and failsafe state

---

## Limitations

- The flight model is a linearized rigid-body approximation. Aerodynamic effects (blade flapping, induced drag, ground effect) are not modeled.
- Motor dynamics are not simulated; thrust and torque are applied instantaneously.
- The complementary filter assumes gravity is the dominant accelerometer signal. High-acceleration maneuvers will degrade attitude estimates.
- Position estimation is not implemented; the controller uses ground-truth position from the simulation state. A real system would require GPS, visual odometry, or a Kalman filter.
- Wind and atmospheric disturbances are not modeled.

---

## Potential Extensions

- Extended Kalman Filter for full pose estimation from IMU + barometer + GPS
- Motor saturation model with actuator dynamics
- Wind disturbance injection
- Obstacle avoidance integrated into the navigation layer
- ROS 2 interface for hardware-in-the-loop testing
- Configurable mission types (survey grid, orbit, return-to-launch)
