#pragma once

#include "adsim/state/drone_state.hpp"
#include "adsim/math/vec3.hpp"

namespace adsim {

struct MotorCommand {
    double thrust{0.0};
    Vec3 torque{};
};

class FlightModel {
public:
    struct Params {
        double mass{1.5};
        double gravity{9.81};
        Vec3 drag_linear{0.5, 0.5, 0.4};
        Vec3 drag_angular{0.05, 0.05, 0.08};
        Vec3 inertia{0.02, 0.02, 0.04};
    };

    explicit FlightModel(Params params);

    DroneState step(const DroneState& state, const MotorCommand& cmd, double dt) const;

    const Params& params() const { return params_; }

private:
    Params params_;

    Vec3 compute_acceleration(const DroneState& state, const MotorCommand& cmd) const;
    Vec3 compute_angular_acceleration(const DroneState& state, const MotorCommand& cmd) const;
};

}
