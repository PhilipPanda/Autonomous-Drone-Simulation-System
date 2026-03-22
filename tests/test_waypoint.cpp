#include "adsim/navigation/waypoint_navigator.hpp"

#include <cassert>
#include <iostream>
#include <string>
#include <cmath>

namespace {

adsim::Waypoint make_wp(double x, double y, double z, double radius = 0.5) {
    adsim::Waypoint wp;
    wp.position = {x, y, z};
    wp.acceptance_radius = radius;
    return wp;
}

void test_initial_target() {
    adsim::WaypointNavigator nav({make_wp(10.0, 0.0, 5.0)});
    auto target = nav.current_target();
    assert(target.position.x == 10.0);
    assert(target.position.z == 5.0);
    assert(!nav.mission_complete());
}

void test_advance_on_arrival() {
    adsim::WaypointNavigator nav({
        make_wp(5.0, 0.0, 5.0, 1.0),
        make_wp(10.0, 0.0, 5.0, 1.0)
    });

    bool advanced = nav.advance({5.0, 0.0, 5.0});
    assert(advanced);
    assert(nav.current_index() == 1);
    assert(!nav.mission_complete());
}

void test_no_advance_when_far() {
    adsim::WaypointNavigator nav({make_wp(10.0, 0.0, 5.0, 0.5)});
    bool advanced = nav.advance({0.0, 0.0, 0.0});
    assert(!advanced);
    assert(nav.current_index() == 0);
}

void test_mission_complete_after_last() {
    adsim::WaypointNavigator nav({make_wp(0.0, 0.0, 0.0, 1.0)});
    nav.advance({0.0, 0.0, 0.0});
    assert(nav.mission_complete());
}

void test_distance_calculation() {
    adsim::WaypointNavigator nav({make_wp(3.0, 4.0, 0.0)});
    double dist = nav.distance_to_current({0.0, 0.0, 0.0});
    assert(std::abs(dist - 5.0) < 1e-9);
}

void test_sequential_waypoints() {
    std::vector<adsim::Waypoint> wps = {
        make_wp(1.0, 0.0, 0.0, 0.5),
        make_wp(2.0, 0.0, 0.0, 0.5),
        make_wp(3.0, 0.0, 0.0, 0.5)
    };
    adsim::WaypointNavigator nav(wps);

    nav.advance({1.0, 0.0, 0.0});
    assert(nav.current_index() == 1);
    nav.advance({2.0, 0.0, 0.0});
    assert(nav.current_index() == 2);
    nav.advance({3.0, 0.0, 0.0});
    assert(nav.mission_complete());
}

void test_target_holds_at_last_on_complete() {
    adsim::WaypointNavigator nav({make_wp(5.0, 5.0, 10.0, 0.5)});
    nav.advance({5.0, 5.0, 10.0});
    assert(nav.mission_complete());
    auto target = nav.current_target();
    assert(target.position.x == 5.0);
    assert(target.position.z == 10.0);
}

void run(const std::string& name, void (*fn)()) {
    fn();
    std::cout << "[pass] " << name << "\n";
}

}

int main() {
    run("initial_target",               test_initial_target);
    run("advance_on_arrival",           test_advance_on_arrival);
    run("no_advance_when_far",          test_no_advance_when_far);
    run("mission_complete_after_last",  test_mission_complete_after_last);
    run("distance_calculation",         test_distance_calculation);
    run("sequential_waypoints",         test_sequential_waypoints);
    run("target_holds_at_last",         test_target_holds_at_last_on_complete);
    return 0;
}
