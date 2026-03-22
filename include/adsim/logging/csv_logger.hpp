#pragma once

#include "adsim/state/drone_state.hpp"
#include "adsim/sensors/imu.hpp"
#include "adsim/estimation/complementary_filter.hpp"
#include "adsim/failsafe/failsafe.hpp"

#include <fstream>
#include <string>
#include <cstddef>

namespace adsim {

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path);
    ~CsvLogger();

    void log(
        const DroneState& state,
        const ImuReading& imu,
        const FilterState& filter,
        FailsafeState failsafe,
        std::size_t waypoint_index
    );

    void flush();

    bool is_open() const { return file_.is_open(); }
    std::size_t records_written() const { return records_written_; }

private:
    std::ofstream file_;
    std::size_t records_written_{0};

    void write_header();
};

}
