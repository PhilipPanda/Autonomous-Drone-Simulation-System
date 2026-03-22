#include "adsim/estimation/complementary_filter.hpp"
#include "adsim/sensors/imu.hpp"
#include "adsim/state/drone_state.hpp"

#include <cassert>
#include <cmath>
#include <numbers>
#include <iostream>
#include <string>

namespace {

adsim::ImuReading make_level_reading(double timestamp = 0.0) {
    adsim::ImuReading r;
    r.accelerometer = {0.0, 0.0, 9.81};
    r.gyroscope = {0.0, 0.0, 0.0};
    r.timestamp = timestamp;
    r.valid = true;
    return r;
}

void test_converges_from_level() {
    adsim::ComplementaryFilter filter(0.1);

    for (int i = 0; i < 200; ++i) {
        filter.update(make_level_reading(i * 0.01), 0.01);
    }

    auto state = filter.state();
    adsim::Vec3 euler = state.attitude.to_euler();
    assert(std::abs(euler.x) < 0.01);
    assert(std::abs(euler.y) < 0.01);
    assert(state.converged);
}

void test_invalid_reading_preserves_state() {
    adsim::ComplementaryFilter filter(0.1);
    for (int i = 0; i < 50; ++i) filter.update(make_level_reading(), 0.01);

    adsim::FilterState before = filter.state();

    adsim::ImuReading invalid;
    invalid.valid = false;
    filter.update(invalid, 0.01);

    adsim::FilterState after = filter.state();
    adsim::Vec3 euler_before = before.attitude.to_euler();
    adsim::Vec3 euler_after = after.attitude.to_euler();

    assert(std::abs(euler_before.x - euler_after.x) < 1e-9);
    assert(std::abs(euler_before.y - euler_after.y) < 1e-9);
}

void test_attitude_tracks_tilt() {
    adsim::ComplementaryFilter filter(0.5);

    adsim::ImuReading reading;
    reading.accelerometer = {9.81, 0.0, 0.0};
    reading.gyroscope = {0.0, 0.0, 0.0};
    reading.valid = true;

    for (int i = 0; i < 500; ++i) {
        filter.update(reading, 0.01);
    }

    adsim::Vec3 euler = filter.state().attitude.to_euler();
    assert(std::abs(euler.y - (-std::numbers::pi / 2.0)) < 0.05);
}

void test_reset_clears_state() {
    adsim::ComplementaryFilter filter(0.1);
    for (int i = 0; i < 50; ++i) filter.update(make_level_reading(), 0.01);
    filter.reset();

    adsim::FilterState state = filter.state();
    adsim::Vec3 euler = state.attitude.to_euler();
    assert(std::abs(euler.x) < 1e-9);
    assert(std::abs(euler.y) < 1e-9);
    assert(!state.converged);
}

void test_gyro_integration_no_accel() {
    adsim::ComplementaryFilter filter(0.0);

    adsim::ImuReading init;
    init.accelerometer = {0.0, 0.0, 9.81};
    init.gyroscope = {0.0, 0.0, 0.0};
    init.valid = true;
    filter.update(init, 0.01);

    adsim::ImuReading reading;
    reading.accelerometer = {0.0, 0.0, 0.0};
    reading.gyroscope = {0.0, 0.0, 1.0};
    reading.valid = true;

    for (int i = 0; i < 100; ++i) {
        filter.update(reading, 0.01);
    }

    adsim::Vec3 euler = filter.state().attitude.to_euler();
    assert(std::abs(euler.z - 1.0) < 0.05);
}

void run(const std::string& name, void (*fn)()) {
    fn();
    std::cout << "[pass] " << name << "\n";
}

}

int main() {
    run("converges_from_level",         test_converges_from_level);
    run("invalid_reading_preserves",    test_invalid_reading_preserves_state);
    run("attitude_tracks_tilt",         test_attitude_tracks_tilt);
    run("reset_clears_state",           test_reset_clears_state);
    run("gyro_integration_no_accel",    test_gyro_integration_no_accel);
    return 0;
}
