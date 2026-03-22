#include "adsim/config/config.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace {

const char* kSample = R"(
[simulation]
timestep = 0.005
duration = 120.0
log_rate = 0.02

[flight_model]
mass = 2.0
gravity = 9.81

[waypoint.0]
x = 10.0
y = 0.0
z = 5.0
radius = 1.0

[waypoint.1]
x = 20.0
y = 10.0
z = 8.0

[flags]
use_estimator = true
debug = false
)";

void test_double_parsing() {
    auto cfg = adsim::Config::from_string(kSample);
    assert(std::abs(cfg.get_double("simulation", "timestep") - 0.005) < 1e-10);
    assert(std::abs(cfg.get_double("flight_model", "mass") - 2.0) < 1e-10);
}

void test_default_values() {
    auto cfg = adsim::Config::from_string(kSample);
    assert(std::abs(cfg.get_double("simulation", "nonexistent", 99.0) - 99.0) < 1e-10);
    assert(cfg.get_int("simulation", "nonexistent", 7) == 7);
    assert(cfg.get_string("simulation", "nonexistent", "hello") == "hello");
}

void test_waypoint_sections() {
    auto cfg = adsim::Config::from_string(kSample);
    assert(cfg.has("waypoint.0", "x"));
    assert(cfg.has("waypoint.1", "x"));
    assert(!cfg.has("waypoint.2", "x"));
    assert(std::abs(cfg.get_double("waypoint.0", "x") - 10.0) < 1e-10);
    assert(std::abs(cfg.get_double("waypoint.1", "y") - 10.0) < 1e-10);
}

void test_bool_parsing() {
    auto cfg = adsim::Config::from_string(kSample);
    assert(cfg.get_bool("flags", "use_estimator") == true);
    assert(cfg.get_bool("flags", "debug") == false);
    assert(cfg.get_bool("flags", "missing", true) == true);
}

void test_comment_stripping() {
    const char* with_comments = R"(
[section]
key = 42  # inline comment
other = hello  # another
)";
    auto cfg = adsim::Config::from_string(with_comments);
    assert(cfg.get_int("section", "key") == 42);
    assert(cfg.get_string("section", "other") == "hello");
}

void test_missing_section_has_check() {
    auto cfg = adsim::Config::from_string(kSample);
    assert(!cfg.has("nonexistent_section", "key"));
}

void test_sections_list() {
    auto cfg = adsim::Config::from_string(kSample);
    auto secs = cfg.sections();
    bool found_sim = false;
    for (const auto& s : secs) {
        if (s == "simulation") found_sim = true;
    }
    assert(found_sim);
}

void run(const std::string& name, void (*fn)()) {
    fn();
    std::cout << "[pass] " << name << "\n";
}

}

int main() {
    run("double_parsing",           test_double_parsing);
    run("default_values",           test_default_values);
    run("waypoint_sections",        test_waypoint_sections);
    run("bool_parsing",             test_bool_parsing);
    run("comment_stripping",        test_comment_stripping);
    run("missing_section_has_check", test_missing_section_has_check);
    run("sections_list",            test_sections_list);
    return 0;
}
