#pragma once

#include "adsim/math/quaternion.hpp"
#include "adsim/math/vec3.hpp"
#include "adsim/sensors/imu.hpp"

namespace adsim {

struct FilterState {
    Quaternion attitude{Quaternion::identity()};
    Vec3 gyro_bias{};
    bool converged{false};
};

class ComplementaryFilter {
public:
    explicit ComplementaryFilter(double alpha = 0.02);

    FilterState update(const ImuReading& reading, double dt);

    const FilterState& state() const { return state_; }
    void reset();

private:
    double alpha_;
    FilterState state_;
    bool initialized_{false};

    static constexpr double kGravity = 9.81;
};

}
