#include "adsim/failsafe/failsafe.hpp"
#include "adsim/state/drone_state.hpp"
#include "adsim/sensors/imu.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace {

adsim::FailsafeParams make_params() {
    adsim::FailsafeParams p;
    p.max_roll_rad = 0.87;
    p.max_pitch_rad = 0.87;
    p.max_altitude = 100.0;
    p.min_altitude = -0.5;
    p.sensor_timeout = 0.3;
    p.max_position_bound = 200.0;
    p.attitude_recovery_time = 1.0;
    return p;
}

adsim::DroneState make_level_state() {
    adsim::DroneState s;
    s.attitude = adsim::Quaternion::identity();
    s.position = {0.0, 0.0, 5.0};
    return s;
}

adsim::ImuReading make_valid_imu() {
    adsim::ImuReading r;
    r.accelerometer = {0.0, 0.0, 9.81};
    r.gyroscope = {0.0, 0.0, 0.0};
    r.valid = true;
    return r;
}

void test_nominal_under_normal_conditions() {
    adsim::FailsafeMonitor monitor(make_params());
    auto state = make_level_state();
    auto imu = make_valid_imu();

    auto result = monitor.evaluate(state, imu, 0.01);
    assert(result == adsim::FailsafeState::Nominal);
    assert(!monitor.is_active());
}

void test_sensor_dropout_triggers_descent() {
    adsim::FailsafeMonitor monitor(make_params());
    auto state = make_level_state();

    adsim::ImuReading invalid;
    invalid.valid = false;

    for (int i = 0; i < 60; ++i) {
        monitor.evaluate(state, invalid, 0.01);
    }

    assert(monitor.state() == adsim::FailsafeState::ControlledDescent);
}

void test_excessive_roll_triggers_hover() {
    adsim::FailsafeMonitor monitor(make_params());

    adsim::DroneState state = make_level_state();
    state.attitude = adsim::Quaternion::from_euler(1.2, 0.0, 0.0);

    auto imu = make_valid_imu();

    adsim::FailsafeState result = adsim::FailsafeState::Nominal;
    for (int i = 0; i < 200; ++i) {
        result = monitor.evaluate(state, imu, 0.01);
    }

    assert(result == adsim::FailsafeState::HoverHold);
}

void test_out_of_bounds_triggers_abort() {
    adsim::FailsafeMonitor monitor(make_params());

    adsim::DroneState state = make_level_state();
    state.position.z = 200.0;

    auto imu = make_valid_imu();
    auto result = monitor.evaluate(state, imu, 0.01);
    assert(result == adsim::FailsafeState::Abort);
}

void test_abort_is_sticky() {
    adsim::FailsafeMonitor monitor(make_params());

    adsim::DroneState bad_state = make_level_state();
    bad_state.position.z = 500.0;

    auto imu = make_valid_imu();
    monitor.evaluate(bad_state, imu, 0.01);

    auto good_state = make_level_state();
    auto result = monitor.evaluate(good_state, imu, 0.01);
    assert(result == adsim::FailsafeState::Abort);
}

void test_recovery_clears_hover() {
    adsim::FailsafeMonitor monitor(make_params());
    auto imu = make_valid_imu();

    adsim::DroneState tilted = make_level_state();
    tilted.attitude = adsim::Quaternion::from_euler(1.2, 0.0, 0.0);

    for (int i = 0; i < 200; ++i) monitor.evaluate(tilted, imu, 0.01);
    assert(monitor.state() == adsim::FailsafeState::HoverHold);

    auto level = make_level_state();
    adsim::FailsafeState result = adsim::FailsafeState::HoverHold;
    for (int i = 0; i < 20; ++i) {
        result = monitor.evaluate(level, imu, 0.01);
    }

    assert(result == adsim::FailsafeState::Nominal);
}

void test_failsafe_target_hover_is_current_position() {
    adsim::FailsafeMonitor monitor(make_params());
    auto state = make_level_state();
    state.position = {3.0, 7.0, 12.0};
    auto imu = make_valid_imu();

    adsim::DroneState tilted = state;
    tilted.attitude = adsim::Quaternion::from_euler(1.2, 0.0, 0.0);
    for (int i = 0; i < 200; ++i) monitor.evaluate(tilted, imu, 0.01);

    auto target = monitor.failsafe_target(state);
    assert(target.position.x == 3.0);
    assert(target.position.y == 7.0);
    assert(target.position.z == 12.0);
}

void run(const std::string& name, void (*fn)()) {
    fn();
    std::cout << "[pass] " << name << "\n";
}

}

int main() {
    run("nominal_under_normal_conditions",      test_nominal_under_normal_conditions);
    run("sensor_dropout_triggers_descent",      test_sensor_dropout_triggers_descent);
    run("excessive_roll_triggers_hover",        test_excessive_roll_triggers_hover);
    run("out_of_bounds_triggers_abort",         test_out_of_bounds_triggers_abort);
    run("abort_is_sticky",                      test_abort_is_sticky);
    run("recovery_clears_hover",                test_recovery_clears_hover);
    run("failsafe_target_hover_position",       test_failsafe_target_hover_is_current_position);
    return 0;
}
