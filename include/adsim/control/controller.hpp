#pragma once

#include "adsim/math/pid.hpp"
#include "adsim/math/vec3.hpp"
#include "adsim/state/drone_state.hpp"
#include "adsim/dynamics/flight_model.hpp"

namespace adsim {

struct ControllerGains {
    PidGains altitude{2.5, 0.1, 4.0};
    PidGains roll{4.0, 0.0, 0.0};
    PidGains pitch{4.0, 0.0, 0.0};
    PidGains yaw{2.0, 0.01, 0.5};
    PidGains roll_rate{0.8, 0.0, 0.0};
    PidGains pitch_rate{0.8, 0.0, 0.0};
    PidGains yaw_rate{0.6, 0.0, 0.0};
    PidGains north{0.02, 0.0, 0.08};
    PidGains east{0.02, 0.0, 0.08};
};

struct ControlTarget {
    Vec3 position{};
    double yaw{0.0};
};

class FlightController {
public:
    explicit FlightController(ControllerGains gains, double mass, double gravity);

    MotorCommand update(const DroneState& state, const ControlTarget& target, double dt);
    void reset();

private:
    ControllerGains gains_;
    double hover_thrust_;

    PidController altitude_pid_;
    PidController north_pid_;
    PidController east_pid_;
    PidController roll_pid_;
    PidController pitch_pid_;
    PidController yaw_pid_;
    PidController roll_rate_pid_;
    PidController pitch_rate_pid_;
    PidController yaw_rate_pid_;
};

}
