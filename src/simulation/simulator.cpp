#include "adsim/simulation/simulator.hpp"
#include "adsim/network/telemetry_server.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>

namespace adsim {

namespace {

std::vector<Waypoint> load_waypoints(const Config& cfg) {
    std::vector<Waypoint> waypoints;
    for (int i = 0; ; ++i) {
        std::string section = "waypoint." + std::to_string(i);
        if (!cfg.has(section, "x") && !cfg.has(section, "z")) break;

        Waypoint wp;
        wp.position.x = cfg.get_double(section, "x", 0.0);
        wp.position.y = cfg.get_double(section, "y", 0.0);
        wp.position.z = cfg.get_double(section, "z", 0.0);
        wp.yaw = cfg.get_double(section, "yaw", 0.0);
        wp.acceptance_radius = cfg.get_double(section, "radius", 0.5);
        waypoints.push_back(wp);
    }
    return waypoints;
}

}

SimulationConfig load_config(const Config& cfg) {
    SimulationConfig sc;

    sc.timestep = cfg.get_double("simulation", "timestep", 0.005);
    sc.duration = cfg.get_double("simulation", "duration", 60.0);
    sc.log_rate = cfg.get_double("simulation", "log_rate", 0.02);

    sc.flight_model.mass = cfg.get_double("flight_model", "mass", 1.5);
    sc.flight_model.gravity = cfg.get_double("flight_model", "gravity", 9.81);
    sc.flight_model.drag_linear.x = cfg.get_double("flight_model", "drag_linear_x", 0.5);
    sc.flight_model.drag_linear.y = cfg.get_double("flight_model", "drag_linear_y", 0.5);
    sc.flight_model.drag_linear.z = cfg.get_double("flight_model", "drag_linear_z", 0.4);
    sc.flight_model.drag_angular.x = cfg.get_double("flight_model", "drag_angular_x", 0.05);
    sc.flight_model.drag_angular.y = cfg.get_double("flight_model", "drag_angular_y", 0.05);
    sc.flight_model.drag_angular.z = cfg.get_double("flight_model", "drag_angular_z", 0.08);
    sc.flight_model.inertia.x = cfg.get_double("flight_model", "inertia_x", 0.02);
    sc.flight_model.inertia.y = cfg.get_double("flight_model", "inertia_y", 0.02);
    sc.flight_model.inertia.z = cfg.get_double("flight_model", "inertia_z", 0.04);

    sc.controller.altitude.kp = cfg.get_double("controller", "alt_kp", 2.5);
    sc.controller.altitude.ki = cfg.get_double("controller", "alt_ki", 0.1);
    sc.controller.altitude.kd = cfg.get_double("controller", "alt_kd", 4.0);
    sc.controller.roll.kp = cfg.get_double("controller", "roll_kp", 4.0);
    sc.controller.roll.ki = cfg.get_double("controller", "roll_ki", 0.0);
    sc.controller.roll.kd = cfg.get_double("controller", "roll_kd", 0.0);
    sc.controller.pitch.kp = cfg.get_double("controller", "pitch_kp", 4.0);
    sc.controller.pitch.ki = cfg.get_double("controller", "pitch_ki", 0.0);
    sc.controller.pitch.kd = cfg.get_double("controller", "pitch_kd", 0.0);
    sc.controller.yaw.kp = cfg.get_double("controller", "yaw_kp", 2.0);
    sc.controller.yaw.ki = cfg.get_double("controller", "yaw_ki", 0.01);
    sc.controller.yaw.kd = cfg.get_double("controller", "yaw_kd", 0.5);
    sc.controller.roll_rate.kp = cfg.get_double("controller", "roll_rate_kp", 0.8);
    sc.controller.roll_rate.kd = cfg.get_double("controller", "roll_rate_kd", 0.0);
    sc.controller.pitch_rate.kp = cfg.get_double("controller", "pitch_rate_kp", 0.8);
    sc.controller.pitch_rate.kd = cfg.get_double("controller", "pitch_rate_kd", 0.0);
    sc.controller.yaw_rate.kp = cfg.get_double("controller", "yaw_rate_kp", 0.6);
    sc.controller.yaw_rate.kd = cfg.get_double("controller", "yaw_rate_kd", 0.0);
    sc.controller.north.kp = cfg.get_double("controller", "north_kp", 0.02);
    sc.controller.north.ki = cfg.get_double("controller", "north_ki", 0.0);
    sc.controller.north.kd = cfg.get_double("controller", "north_kd", 0.08);
    sc.controller.east.kp = cfg.get_double("controller", "east_kp", 0.02);
    sc.controller.east.ki = cfg.get_double("controller", "east_ki", 0.0);
    sc.controller.east.kd = cfg.get_double("controller", "east_kd", 0.08);

    sc.imu.accel_noise_std = cfg.get_double("imu", "accel_noise_std", 0.05);
    sc.imu.gyro_noise_std = cfg.get_double("imu", "gyro_noise_std", 0.005);
    sc.imu.accel_bias_instability = cfg.get_double("imu", "accel_bias_instability", 0.002);
    sc.imu.gyro_bias_instability = cfg.get_double("imu", "gyro_bias_instability", 0.0001);
    sc.imu.dropout_probability = cfg.get_double("imu", "dropout_probability", 0.0);

    sc.filter_alpha = cfg.get_double("estimator", "alpha", 0.02);

    sc.failsafe.max_roll_rad = cfg.get_double("failsafe", "max_roll_deg", 50.0) * std::numbers::pi / 180.0;
    sc.failsafe.max_pitch_rad = cfg.get_double("failsafe", "max_pitch_deg", 50.0) * std::numbers::pi / 180.0;
    sc.failsafe.max_altitude = cfg.get_double("failsafe", "max_altitude", 150.0);
    sc.failsafe.min_altitude = cfg.get_double("failsafe", "min_altitude", -1.0);
    sc.failsafe.sensor_timeout = cfg.get_double("failsafe", "sensor_timeout", 0.5);
    sc.failsafe.max_position_bound = cfg.get_double("failsafe", "max_position_bound", 500.0);
    sc.failsafe.attitude_recovery_time = cfg.get_double("failsafe", "attitude_recovery_time", 2.0);

    sc.initial_state.position.x = cfg.get_double("initial_state", "pos_x", 0.0);
    sc.initial_state.position.y = cfg.get_double("initial_state", "pos_y", 0.0);
    sc.initial_state.position.z = cfg.get_double("initial_state", "pos_z", 0.0);

    sc.waypoints = load_waypoints(cfg);

    return sc;
}

static TelemetryFrame build_frame(const DroneState& state,
                                   const WaypointNavigator& nav,
                                   FailsafeState fs) {
    TelemetryFrame f;
    f.sim_time          = state.timestamp;
    f.position          = state.position;
    f.velocity          = state.velocity;
    f.acceleration      = state.acceleration;
    f.attitude          = state.attitude;
    f.euler             = state.attitude.to_euler();
    f.angular_velocity  = state.angular_velocity;
    f.waypoint_index    = static_cast<int>(nav.current_index());
    f.total_waypoints   = static_cast<int>(nav.total_waypoints());
    f.mission_complete  = nav.mission_complete();
    f.failsafe_state    = fs;
    if (!nav.mission_complete()) {
        f.waypoint_target = nav.current_waypoint().position;
    }
    return f;
}

Simulator::Simulator(SimulationConfig config)
    : config_(std::move(config))
    , flight_model_(config_.flight_model)
    , controller_(config_.controller, config_.flight_model.mass, config_.flight_model.gravity)
    , imu_(config_.imu)
    , filter_(config_.filter_alpha)
    , navigator_(config_.waypoints)
    , failsafe_(config_.failsafe)
    , state_(config_.initial_state)
{}

SimulationSummary Simulator::run(CsvLogger* logger, TelemetryServer* telemetry) {
    double log_accumulator = 0.0;
    double dt = config_.timestep;

    for (double t = 0.0; t < config_.duration; t += dt) {
        state_.timestamp = t;

        ImuReading imu_reading = imu_.sample(state_, dt);
        FilterState filter_state = filter_.update(imu_reading, dt);

        FailsafeState fs = failsafe_.evaluate(state_, imu_reading, dt);

        ControlTarget target;
        if (fs == FailsafeState::Nominal) {
            navigator_.advance(state_.position);
            target = navigator_.current_target();
        } else {
            target = failsafe_.failsafe_target(state_);
        }

        if (fs == FailsafeState::Abort) {
            if (logger) {
                logger->log(state_, imu_reading, filter_state, fs, navigator_.current_index());
            }
            if (telemetry) {
                telemetry->publish_now(build_frame(state_, navigator_, fs));
            }
            break;
        }

        MotorCommand cmd = controller_.update(state_, target, dt);
        state_ = flight_model_.step(state_, cmd, dt);

        if (telemetry) {
            TelemetryFrame frame = build_frame(state_, navigator_, fs);
            telemetry->try_publish(frame, t);
        }

        log_accumulator += dt;
        if (logger && log_accumulator >= config_.log_rate) {
            logger->log(state_, imu_reading, filter_state, fs, navigator_.current_index());
            log_accumulator = 0.0;
        }

        if (fs == FailsafeState::Nominal && navigator_.mission_complete()) {
            if (logger) {
                logger->log(state_, imu_reading, filter_state, fs, navigator_.current_index());
            }
            if (telemetry) {
                telemetry->publish_now(build_frame(state_, navigator_, fs));
            }
            break;
        }
    }

    SimulationSummary summary;
    summary.duration_simulated = state_.timestamp;
    summary.waypoints_completed = std::min(navigator_.current_index(), navigator_.total_waypoints());
    summary.total_waypoints = navigator_.total_waypoints();
    summary.mission_complete = navigator_.mission_complete();
    summary.final_failsafe_state = failsafe_.state();
    summary.log_records = logger ? logger->records_written() : 0;
    summary.final_altitude = state_.position.z;
    summary.final_position = state_.position;
    return summary;
}

}
