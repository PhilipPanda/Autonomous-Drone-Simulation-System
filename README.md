# Autonomous Drone Simulation System

C++20 quadrotor simulation with cascaded PID flight control, IMU sensor modeling, waypoint navigation, failsafe handling, and a JavaFX telemetry monitor with live 2D/3D visualization.

---

## Build

Requires CMake 3.20+ and a C++20 compiler.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build
```

## Run

```bash
# Basic run with CSV logging
./build/adsim -c config/default.ini -o output.csv

# Stream telemetry and wait for the Java UI to connect before starting
./build/adsim -c config/default.ini --wait --port 5760
```

Options: `-c` config (required), `-o` output (default: sim_output.csv), `--stream`, `--wait`, `--host`, `--port`.

## Java UI

Requires Java 21+ and the included Gradle wrapper.

```bash
cd ui-java
./gradlew run
```

Provides a 2D top-down view and a 3D orbital view with live telemetry. Reconnects automatically if the simulation is restarted.

## Architecture

The simulation is structured as a static library (`adsim_core`) consumed by a CLI executable (`adsim`). Each module has a clear boundary and communicates through value types with no shared mutable state.

```
adsim_core
├── math        — Vec3, Quaternion, and PID controller primitives
├── dynamics    — Rigid-body flight model: thrust, linear/angular drag,
│                 gyroscopic coupling, semi-implicit Euler integration
├── control     — Cascaded PID: position loops → desired attitude angles
│                 → angular rate setpoints → body torques
├── sensors     — IMU with Gaussian noise, random-walk bias instability,
│                 and configurable probabilistic dropout events
├── estimation  — Complementary filter fusing gyroscope integration with
│                 accelerometer gravity direction; slow gyro bias correction
├── navigation  — Sequential waypoint tracker with per-waypoint acceptance
│                 radii; exposes current target as a ControlTarget
├── failsafe    — Three independent monitors: sustained IMU dropout →
│                 controlled descent; attitude limit violation → hover hold;
│                 position/altitude out of bounds → abort (sticky, terminal)
├── simulation  — Fixed-timestep orchestration loop; owns all subsystems;
│                 binds INI config to typed structs at startup
├── logging     — Decimated CSV output: full state, IMU readings, estimated
│                 attitude, failsafe status, and active waypoint index
├── config      — Zero-dependency INI parser with typed getters and defaults
└── network     — TCP telemetry server (PIMPL, cross-platform Winsock/POSIX);
                  background accept thread; 30 Hz rate-limited line-delimited
                  JSON broadcast to a single connected client
```

**ui-java** is a standalone JavaFX application that connects over TCP, parses the JSON stream with Jackson, and renders a 2D top-down canvas and a 3D SubScene at 60 fps using an `AnimationTimer`. The 3D view applies the full quaternion orientation as a JavaFX `Affine` transform on the drone model.

## Configuration

```ini
[simulation]
timestep = 0.005
duration = 90.0

[flight_model]
mass = 1.5
gravity = 9.81

[imu]
accel_noise_std     = 0.05
dropout_probability = 0.0

[waypoint.0]
x = 0.0  y = 0.0  z = 8.0  yaw = 0.0  radius = 0.8
```

See `config/default.ini` and `config/noisy_imu.ini` for full examples.
