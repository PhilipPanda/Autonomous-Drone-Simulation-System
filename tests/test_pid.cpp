#include "adsim/math/pid.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

namespace {

void test_proportional_response() {
    adsim::PidController pid({1.0, 0.0, 0.0}, -100.0, 100.0);
    double out = pid.update(5.0, 0.01);
    assert(std::abs(out - 5.0) < 1e-9);
}

void test_integral_accumulation() {
    adsim::PidController pid({0.0, 1.0, 0.0}, -100.0, 100.0);
    double dt = 0.1;
    double total = 0.0;
    for (int i = 0; i < 10; ++i) {
        total = pid.update(1.0, dt);
    }
    assert(std::abs(total - 1.0) < 1e-9);
}

void test_derivative_kick() {
    adsim::PidController pid({0.0, 0.0, 1.0}, -100.0, 100.0);
    pid.update(0.0, 0.01);
    double out = pid.update(1.0, 0.1);
    assert(std::abs(out - 10.0) < 1e-9);
}

void test_output_clamping() {
    adsim::PidController pid({100.0, 0.0, 0.0}, -5.0, 5.0);
    double out = pid.update(1.0, 0.01);
    assert(out == 5.0);

    pid.reset();
    out = pid.update(-1.0, 0.01);
    assert(out == -5.0);
}

void test_reset_clears_integral() {
    adsim::PidController pid({0.0, 1.0, 0.0}, -100.0, 100.0);
    for (int i = 0; i < 10; ++i) pid.update(1.0, 0.1);
    pid.reset();
    double out = pid.update(1.0, 0.1);
    assert(std::abs(out - 0.1) < 1e-9);
}

void test_zero_dt_safe() {
    adsim::PidController pid({1.0, 1.0, 1.0}, -100.0, 100.0);
    double out = pid.update(1.0, 0.0);
    assert(out == 0.0);
}

void run(const std::string& name, void (*fn)()) {
    fn();
    std::cout << "[pass] " << name << "\n";
}

}

int main() {
    run("proportional_response",      test_proportional_response);
    run("integral_accumulation",      test_integral_accumulation);
    run("derivative_kick",            test_derivative_kick);
    run("output_clamping",            test_output_clamping);
    run("reset_clears_integral",      test_reset_clears_integral);
    run("zero_dt_safe",               test_zero_dt_safe);
    return 0;
}
