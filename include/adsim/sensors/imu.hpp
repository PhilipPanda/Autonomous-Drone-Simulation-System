#pragma once

#include "adsim/math/vec3.hpp"
#include "adsim/state/drone_state.hpp"

#include <random>
#include <cstdint>

namespace adsim {

struct ImuParams {
    double accel_noise_std{0.05};
    double gyro_noise_std{0.005};
    double accel_bias_instability{0.002};
    double gyro_bias_instability{0.0001};
    double dropout_probability{0.0};
};

struct ImuReading {
    Vec3 accelerometer{};
    Vec3 gyroscope{};
    double timestamp{0.0};
    bool valid{true};
};

class Imu {
public:
    explicit Imu(ImuParams params, uint64_t seed = 42);

    ImuReading sample(const DroneState& state, double dt);

    void force_dropout(double duration);
    bool is_healthy() const { return dropout_remaining_ <= 0.0; }

private:
    ImuParams params_;
    Vec3 accel_bias_{};
    Vec3 gyro_bias_{};
    double dropout_remaining_{0.0};

    std::mt19937_64 rng_;
    std::normal_distribution<double> unit_normal_{0.0, 1.0};

    Vec3 sample_noise(double std_dev);
    void update_bias(double dt);
};

}
