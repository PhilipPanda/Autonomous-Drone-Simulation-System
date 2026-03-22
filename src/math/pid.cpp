#include "adsim/math/pid.hpp"

#include <algorithm>

namespace adsim {

PidController::PidController(PidGains gains, double output_min, double output_max)
    : gains_(gains), output_min_(output_min), output_max_(output_max) {}

double PidController::update(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    integral_ += error * dt;

    double derivative = 0.0;
    if (!first_update_) {
        derivative = (error - prev_error_) / dt;
    }
    first_update_ = false;
    prev_error_ = error;

    double output = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    return std::clamp(output, output_min_, output_max_);
}

void PidController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_update_ = true;
}

}
