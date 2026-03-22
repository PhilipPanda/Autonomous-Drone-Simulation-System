#include "adsim/estimation/complementary_filter.hpp"

#include <cmath>

namespace adsim {

ComplementaryFilter::ComplementaryFilter(double alpha) : alpha_(alpha) {}

FilterState ComplementaryFilter::update(const ImuReading& reading, double dt) {
    if (!reading.valid) {
        return state_;
    }

    if (!initialized_) {
        Vec3 accel = reading.accelerometer;
        double accel_norm = accel.norm();
        if (accel_norm > 0.5 && accel_norm < 20.0) {
            Vec3 a = accel / accel_norm;
            double pitch = std::asin(-a.x);
            double roll = std::atan2(a.y, a.z);
            state_.attitude = Quaternion::from_euler(roll, pitch, 0.0);
            initialized_ = true;
        }
        return state_;
    }

    Vec3 gyro_corrected = reading.gyroscope - state_.gyro_bias;
    Quaternion attitude_pred = state_.attitude.integrated(gyro_corrected, dt);

    Vec3 accel = reading.accelerometer;
    double accel_norm = accel.norm();
    bool accel_usable = accel_norm > 5.0 && accel_norm < 20.0;

    if (accel_usable) {
        Vec3 gravity_estimated = attitude_pred.rotate_inverse({0.0, 0.0, kGravity});
        Vec3 accel_normalized = accel / accel_norm;
        Vec3 gravity_normalized = gravity_estimated / gravity_estimated.norm();

        Vec3 correction = gravity_normalized.cross(accel_normalized);
        double correction_magnitude = correction.norm();

        if (correction_magnitude > 1e-6) {
            Vec3 correction_axis = correction / correction_magnitude;
            double correction_angle = std::asin(std::min(1.0, correction_magnitude));
            Quaternion dq = Quaternion::from_axis_angle(correction_axis, correction_angle * alpha_);
            state_.attitude = (attitude_pred * dq).normalized();

            state_.gyro_bias = state_.gyro_bias + correction * (alpha_ * 0.1);
        } else {
            state_.attitude = attitude_pred;
        }

        state_.converged = true;
    } else {
        state_.attitude = attitude_pred;
    }

    return state_;
}

void ComplementaryFilter::reset() {
    state_ = {};
    initialized_ = false;
}

}
