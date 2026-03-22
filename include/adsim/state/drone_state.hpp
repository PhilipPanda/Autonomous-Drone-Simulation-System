#pragma once

#include "adsim/math/vec3.hpp"
#include "adsim/math/quaternion.hpp"

namespace adsim {

struct DroneState {
    double timestamp{0.0};

    Vec3 position{};
    Vec3 velocity{};
    Vec3 acceleration{};

    Quaternion attitude{Quaternion::identity()};
    Vec3 angular_velocity{};

    bool is_valid() const {
        return position.is_finite()
            && velocity.is_finite()
            && acceleration.is_finite()
            && attitude.is_finite()
            && attitude.is_unit()
            && angular_velocity.is_finite();
    }
};

}
