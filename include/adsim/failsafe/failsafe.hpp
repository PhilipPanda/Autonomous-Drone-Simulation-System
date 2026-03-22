#pragma once

#include "adsim/state/drone_state.hpp"
#include "adsim/sensors/imu.hpp"
#include "adsim/control/controller.hpp"

namespace adsim {

enum class FailsafeState {
    Nominal,
    HoverHold,
    ControlledDescent,
    Abort
};

struct FailsafeParams {
    double max_roll_rad{0.87};
    double max_pitch_rad{0.87};
    double max_altitude{150.0};
    double min_altitude{-1.0};
    double sensor_timeout{0.5};
    double max_position_bound{500.0};
    double attitude_recovery_time{2.0};
};

class FailsafeMonitor {
public:
    explicit FailsafeMonitor(FailsafeParams params);

    FailsafeState evaluate(const DroneState& state, const ImuReading& reading, double dt);

    bool is_active() const { return current_state_ != FailsafeState::Nominal; }
    FailsafeState state() const { return current_state_; }

    ControlTarget failsafe_target(const DroneState& state) const;

private:
    FailsafeParams params_;
    FailsafeState current_state_{FailsafeState::Nominal};
    double sensor_loss_elapsed_{0.0};
    double attitude_violation_elapsed_{0.0};

    bool check_sensor_health(const ImuReading& reading, double dt);
    bool check_attitude(const DroneState& state, double dt);
    bool check_bounds(const DroneState& state);
};

const char* failsafe_state_name(FailsafeState state);

}
