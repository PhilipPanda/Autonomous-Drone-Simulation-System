#pragma once

#include "adsim/math/vec3.hpp"
#include "adsim/control/controller.hpp"

#include <vector>
#include <cstddef>

namespace adsim {

struct Waypoint {
    Vec3 position{};
    double yaw{0.0};
    double acceptance_radius{0.5};
};

class WaypointNavigator {
public:
    explicit WaypointNavigator(std::vector<Waypoint> waypoints);

    bool advance(const Vec3& position);
    bool mission_complete() const;

    ControlTarget current_target() const;
    const Waypoint& current_waypoint() const;
    std::size_t current_index() const { return current_; }
    std::size_t total_waypoints() const { return waypoints_.size(); }

    double distance_to_current(const Vec3& position) const;

private:
    std::vector<Waypoint> waypoints_;
    std::size_t current_{0};
};

}
