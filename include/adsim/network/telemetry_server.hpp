#pragma once

#include "adsim/math/vec3.hpp"
#include "adsim/math/quaternion.hpp"
#include "adsim/failsafe/failsafe.hpp"

#include <string>
#include <memory>
#include <cstdint>

namespace adsim {

struct TelemetryFrame {
    double sim_time{0.0};
    Vec3 position{};
    Vec3 velocity{};
    Vec3 acceleration{};
    Quaternion attitude{};
    Vec3 euler{};
    Vec3 angular_velocity{};
    Vec3 waypoint_target{};
    int waypoint_index{0};
    int total_waypoints{0};
    bool mission_complete{false};
    FailsafeState failsafe_state{FailsafeState::Nominal};
};

struct TelemetryServerConfig {
    std::string host{"127.0.0.1"};
    uint16_t port{5760};
    double rate_hz{30.0};
};

class TelemetryServer {
public:
    explicit TelemetryServer(TelemetryServerConfig config);
    ~TelemetryServer();

    TelemetryServer(const TelemetryServer&) = delete;
    TelemetryServer& operator=(const TelemetryServer&) = delete;

    void start();
    void stop();
    bool is_client_connected() const;
    void try_publish(const TelemetryFrame& frame, double sim_time);
    void publish_now(const TelemetryFrame& frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::string serialize_telemetry(const TelemetryFrame& frame);

}
