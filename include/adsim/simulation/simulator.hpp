#pragma once

#include "adsim/dynamics/flight_model.hpp"
#include "adsim/control/controller.hpp"
#include "adsim/sensors/imu.hpp"
#include "adsim/estimation/complementary_filter.hpp"
#include "adsim/navigation/waypoint_navigator.hpp"
#include "adsim/failsafe/failsafe.hpp"
#include "adsim/logging/csv_logger.hpp"
#include "adsim/config/config.hpp"

#include <string>
#include <optional>

namespace adsim {

class TelemetryServer;

struct SimulationConfig {
    double timestep{0.005};
    double duration{60.0};
    double log_rate{0.02};

    FlightModel::Params flight_model{};
    ControllerGains controller{};
    ImuParams imu{};
    double filter_alpha{0.02};
    FailsafeParams failsafe{};
    std::vector<Waypoint> waypoints{};
    DroneState initial_state{};
};

struct SimulationSummary {
    double duration_simulated{0.0};
    std::size_t waypoints_completed{0};
    std::size_t total_waypoints{0};
    bool mission_complete{false};
    FailsafeState final_failsafe_state{FailsafeState::Nominal};
    std::size_t log_records{0};
    double final_altitude{0.0};
    Vec3 final_position{};
};

SimulationConfig load_config(const Config& cfg);

class Simulator {
public:
    explicit Simulator(SimulationConfig config);

    SimulationSummary run(CsvLogger* logger, TelemetryServer* telemetry = nullptr);

private:
    SimulationConfig config_;
    FlightModel flight_model_;
    FlightController controller_;
    Imu imu_;
    ComplementaryFilter filter_;
    WaypointNavigator navigator_;
    FailsafeMonitor failsafe_;
    DroneState state_;
};

}
