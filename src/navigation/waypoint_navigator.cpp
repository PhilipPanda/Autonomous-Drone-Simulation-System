#include "adsim/navigation/waypoint_navigator.hpp"

namespace adsim {

WaypointNavigator::WaypointNavigator(std::vector<Waypoint> waypoints)
    : waypoints_(std::move(waypoints)) {}

bool WaypointNavigator::advance(const Vec3& position) {
    if (mission_complete()) return false;

    double dist = distance_to_current(position);
    if (dist < waypoints_[current_].acceptance_radius) {
        ++current_;
        return true;
    }
    return false;
}

bool WaypointNavigator::mission_complete() const {
    return current_ >= waypoints_.size();
}

ControlTarget WaypointNavigator::current_target() const {
    if (mission_complete()) {
        if (!waypoints_.empty()) {
            const auto& last = waypoints_.back();
            return {last.position, last.yaw};
        }
        return {};
    }
    const auto& wp = waypoints_[current_];
    return {wp.position, wp.yaw};
}

const Waypoint& WaypointNavigator::current_waypoint() const {
    if (waypoints_.empty()) {
        static const Waypoint empty{};
        return empty;
    }
    if (mission_complete()) return waypoints_.back();
    return waypoints_[current_];
}

double WaypointNavigator::distance_to_current(const Vec3& position) const {
    if (mission_complete()) return 0.0;
    return (waypoints_[current_].position - position).norm();
}

}
