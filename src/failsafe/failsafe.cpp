#include "adsim/failsafe/failsafe.hpp"

#include <cmath>

namespace adsim {

FailsafeMonitor::FailsafeMonitor(FailsafeParams params) : params_(params) {}

bool FailsafeMonitor::check_sensor_health(const ImuReading& reading, double dt) {
    if (!reading.valid) {
        sensor_loss_elapsed_ += dt;
    } else {
        sensor_loss_elapsed_ = 0.0;
    }
    return sensor_loss_elapsed_ > params_.sensor_timeout;
}

bool FailsafeMonitor::check_attitude(const DroneState& state, double dt) {
    Vec3 euler = state.attitude.to_euler();
    bool violated = std::abs(euler.x) > params_.max_roll_rad
                 || std::abs(euler.y) > params_.max_pitch_rad;

    if (violated) {
        attitude_violation_elapsed_ += dt;
    } else {
        attitude_violation_elapsed_ = 0.0;
    }

    return attitude_violation_elapsed_ > params_.attitude_recovery_time;
}

bool FailsafeMonitor::check_bounds(const DroneState& state) {
    if (!state.is_valid()) return true;

    double bound = params_.max_position_bound;
    if (std::abs(state.position.x) > bound) return true;
    if (std::abs(state.position.y) > bound) return true;
    if (state.position.z > params_.max_altitude) return true;
    if (state.position.z < params_.min_altitude) return true;
    return false;
}

FailsafeState FailsafeMonitor::evaluate(const DroneState& state, const ImuReading& reading, double dt) {
    if (!state.is_valid() || check_bounds(state)) {
        current_state_ = FailsafeState::Abort;
        return current_state_;
    }

    if (current_state_ == FailsafeState::Abort) {
        return current_state_;
    }

    bool sensor_failed = check_sensor_health(reading, dt);
    bool attitude_failed = check_attitude(state, dt);

    if (sensor_failed) {
        current_state_ = FailsafeState::ControlledDescent;
    } else if (attitude_failed) {
        current_state_ = FailsafeState::HoverHold;
    } else {
        current_state_ = FailsafeState::Nominal;
    }

    return current_state_;
}

ControlTarget FailsafeMonitor::failsafe_target(const DroneState& state) const {
    switch (current_state_) {
    case FailsafeState::HoverHold:
        return {state.position, state.attitude.to_euler().z};

    case FailsafeState::ControlledDescent: {
        Vec3 descent_pos = state.position;
        descent_pos.z = 0.0;
        return {descent_pos, 0.0};
    }

    case FailsafeState::Abort:
    case FailsafeState::Nominal:
    default:
        return {state.position, state.attitude.to_euler().z};
    }
}

const char* failsafe_state_name(FailsafeState state) {
    switch (state) {
    case FailsafeState::Nominal:           return "nominal";
    case FailsafeState::HoverHold:         return "hover_hold";
    case FailsafeState::ControlledDescent: return "controlled_descent";
    case FailsafeState::Abort:             return "abort";
    }
    return "unknown";
}

}
