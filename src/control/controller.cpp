#include "adsim/control/controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace adsim {

static constexpr double kMaxTiltRad     = 0.35;
static constexpr double kMaxRateRad     = 3.0;
static constexpr double kMaxThrustScale = 2.5;

FlightController::FlightController(ControllerGains gains, double mass, double gravity)
    : gains_(gains)
    , hover_thrust_(mass * gravity)
    , altitude_pid_(gains.altitude, -hover_thrust_ * 0.8, hover_thrust_ * kMaxThrustScale)
    , north_pid_(gains.north,  -kMaxTiltRad, kMaxTiltRad)
    , east_pid_(gains.east,    -kMaxTiltRad, kMaxTiltRad)
    , roll_pid_(gains.roll,    -kMaxRateRad, kMaxRateRad)
    , pitch_pid_(gains.pitch,  -kMaxRateRad, kMaxRateRad)
    , yaw_pid_(gains.yaw,      -kMaxRateRad, kMaxRateRad)
    , roll_rate_pid_(gains.roll_rate,   -30.0, 30.0)
    , pitch_rate_pid_(gains.pitch_rate, -30.0, 30.0)
    , yaw_rate_pid_(gains.yaw_rate,     -15.0, 15.0)
{}

MotorCommand FlightController::update(const DroneState& state, const ControlTarget& target, double dt) {
    Vec3 euler = state.attitude.to_euler();

    double alt_correction = altitude_pid_.update(target.position.z - state.position.z, dt);
    double cos_tilt = std::cos(euler.x) * std::cos(euler.y);
    double tilt_comp = (cos_tilt > 0.15) ? hover_thrust_ / cos_tilt : hover_thrust_ / 0.15;
    double base_thrust = tilt_comp + alt_correction;

    double raw_north = north_pid_.update(target.position.x - state.position.x, dt);
    double raw_east  = east_pid_.update(target.position.y - state.position.y, dt);

    double yaw = euler.z;
    double desired_pitch = std::clamp( raw_north * std::cos(yaw) + raw_east  * std::sin(yaw), -kMaxTiltRad, kMaxTiltRad);
    double desired_roll  = std::clamp( raw_north * std::sin(yaw) - raw_east  * std::cos(yaw), -kMaxTiltRad, kMaxTiltRad);

    double yaw_error = target.yaw - euler.z;
    while (yaw_error >  std::numbers::pi) yaw_error -= 2.0 * std::numbers::pi;
    while (yaw_error < -std::numbers::pi) yaw_error += 2.0 * std::numbers::pi;

    double desired_roll_rate  = roll_pid_.update(desired_roll  - euler.x, dt);
    double desired_pitch_rate = pitch_pid_.update(desired_pitch - euler.y, dt);
    double desired_yaw_rate   = yaw_pid_.update(yaw_error, dt);

    double roll_torque  = roll_rate_pid_.update(desired_roll_rate  - state.angular_velocity.x, dt);
    double pitch_torque = pitch_rate_pid_.update(desired_pitch_rate - state.angular_velocity.y, dt);
    double yaw_torque   = yaw_rate_pid_.update(desired_yaw_rate    - state.angular_velocity.z, dt);

    MotorCommand cmd;
    cmd.thrust = std::max(0.0, base_thrust);
    cmd.torque = {roll_torque, pitch_torque, yaw_torque};
    return cmd;
}

void FlightController::reset() {
    altitude_pid_.reset();
    north_pid_.reset();
    east_pid_.reset();
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_pid_.reset();
    roll_rate_pid_.reset();
    pitch_rate_pid_.reset();
    yaw_rate_pid_.reset();
}

}
