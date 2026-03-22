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

```
adsim_core
├── math        — Vec3, Quaternion, PID
├── dynamics    — rigid-body flight model
├── control     — cascaded PID (position → attitude → rate)
├── sensors     — IMU with noise, bias, and dropout
├── estimation  — complementary filter
├── navigation  — waypoint sequencer
├── failsafe    — sensor loss, attitude limits, bounds violation
├── simulation  — fixed-timestep loop
├── logging     — CSV output
├── config      — INI parser
└── network     — TCP telemetry server (line-delimited JSON, 30 Hz)
```

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
