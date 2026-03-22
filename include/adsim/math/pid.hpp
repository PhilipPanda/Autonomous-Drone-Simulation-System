#pragma once

#include <algorithm>
#include <limits>

namespace adsim {

struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

class PidController {
public:
    PidController(PidGains gains, double output_min, double output_max);

    double update(double error, double dt);
    void reset();

    PidGains gains() const { return gains_; }
    double integral() const { return integral_; }

private:
    PidGains gains_;
    double output_min_;
    double output_max_;
    double integral_{0.0};
    double prev_error_{0.0};
    bool first_update_{true};
};

}
