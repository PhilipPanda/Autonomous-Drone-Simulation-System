#include "adsim/sensors/imu.hpp"

#include <cmath>

namespace adsim {

Imu::Imu(ImuParams params, uint64_t seed)
    : params_(params), rng_(seed) {}

Vec3 Imu::sample_noise(double std_dev) {
    return {
        unit_normal_(rng_) * std_dev,
        unit_normal_(rng_) * std_dev,
        unit_normal_(rng_) * std_dev
    };
}

void Imu::update_bias(double dt) {
    double accel_walk = params_.accel_bias_instability * std::sqrt(dt);
    double gyro_walk = params_.gyro_bias_instability * std::sqrt(dt);
    accel_bias_ += sample_noise(accel_walk);
    gyro_bias_ += sample_noise(gyro_walk);
}

ImuReading Imu::sample(const DroneState& state, double dt) {
    ImuReading reading;
    reading.timestamp = state.timestamp;

    if (dropout_remaining_ > 0.0) {
        dropout_remaining_ -= dt;
        reading.valid = false;
        return reading;
    }

    if (params_.dropout_probability > 0.0) {
        std::uniform_real_distribution<double> uniform(0.0, 1.0);
        if (uniform(rng_) < params_.dropout_probability * dt) {
            dropout_remaining_ = std::max(0.05, 0.1 + unit_normal_(rng_) * 0.05);
            reading.valid = false;
            return reading;
        }
    }

    update_bias(dt);

    Vec3 gravity_world{0.0, 0.0, 9.81};
    Vec3 accel_body = state.attitude.rotate_inverse(state.acceleration + gravity_world);
    reading.accelerometer = accel_body + accel_bias_ + sample_noise(params_.accel_noise_std);
    reading.gyroscope = state.angular_velocity + gyro_bias_ + sample_noise(params_.gyro_noise_std);
    reading.valid = true;

    return reading;
}

void Imu::force_dropout(double duration) {
    dropout_remaining_ = duration;
}

}
