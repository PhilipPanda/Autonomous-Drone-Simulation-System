#include "adsim/simulation/simulator.hpp"
#include "adsim/logging/csv_logger.hpp"
#include "adsim/config/config.hpp"
#include "adsim/network/telemetry_server.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <optional>
#include <filesystem>
#include <memory>
#include <thread>
#include <chrono>

namespace {

struct CliArgs {
    std::string config_path;
    std::string log_path;
    bool verbose{false};
    bool stream{false};
    bool wait_for_client{false};
    std::string stream_host{"127.0.0.1"};
    uint16_t stream_port{5760};
};

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " [options]\n"
              << "  -c, --config <path>   Configuration file (required)\n"
              << "  -o, --output <path>   Log output path (default: sim_output.csv)\n"
              << "  -v, --verbose         Print step-by-step state\n"
              << "      --stream          Enable TCP telemetry streaming\n"
              << "      --wait            Wait for a client to connect before starting\n"
              << "      --host <addr>     Telemetry bind address (default: 127.0.0.1)\n"
              << "      --port <port>     Telemetry TCP port (default: 5760)\n"
              << "  -h, --help            Show this message\n";
}

std::optional<CliArgs> parse_args(int argc, char** argv) {
    CliArgs args;
    args.log_path = "sim_output.csv";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            return std::nullopt;
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            args.config_path = argv[++i];
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            args.log_path = argv[++i];
        } else if (arg == "-v" || arg == "--verbose") {
            args.verbose = true;
        } else if (arg == "--stream") {
            args.stream = true;
        } else if (arg == "--wait") {
            args.stream = true;
            args.wait_for_client = true;
        } else if (arg == "--host" && i + 1 < argc) {
            args.stream_host = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            args.stream_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return std::nullopt;
        }
    }

    if (args.config_path.empty()) {
        std::cerr << "Error: config file is required.\n";
        return std::nullopt;
    }

    return args;
}

void print_summary(const adsim::SimulationSummary& summary) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n--- Simulation Summary ---\n";
    std::cout << "  Duration simulated : " << summary.duration_simulated << " s\n";
    std::cout << "  Mission complete   : " << (summary.mission_complete ? "yes" : "no") << "\n";
    std::cout << "  Waypoints          : " << summary.waypoints_completed
              << " / " << summary.total_waypoints << "\n";
    std::cout << "  Final position     : ("
              << summary.final_position.x << ", "
              << summary.final_position.y << ", "
              << summary.final_position.z << ")\n";
    std::cout << "  Final altitude     : " << summary.final_altitude << " m\n";
    std::cout << "  Failsafe state     : " << adsim::failsafe_state_name(summary.final_failsafe_state) << "\n";
    std::cout << "  Log records        : " << summary.log_records << "\n";
    std::cout << "--------------------------\n";
}

}

int main(int argc, char** argv) {
    auto args = parse_args(argc, argv);
    if (!args) {
        print_usage(argv[0]);
        return 1;
    }

    adsim::Config cfg;
    try {
        cfg = adsim::Config::from_file(args->config_path);
    } catch (const adsim::ConfigError& e) {
        std::cerr << "Config error: " << e.what() << "\n";
        return 1;
    }

    adsim::SimulationConfig sim_config;
    try {
        sim_config = adsim::load_config(cfg);
    } catch (const std::exception& e) {
        std::cerr << "Failed to build simulation config: " << e.what() << "\n";
        return 1;
    }

    if (sim_config.waypoints.empty()) {
        std::cerr << "Warning: no waypoints defined. Drone will hover at origin.\n";
        adsim::Waypoint hover;
        hover.position = sim_config.initial_state.position;
        hover.position.z = std::max(sim_config.initial_state.position.z, 5.0);
        sim_config.waypoints.push_back(hover);
    }

    std::cout << "adsim starting\n";
    std::cout << "  Config  : " << args->config_path << "\n";
    std::cout << "  Output  : " << args->log_path << "\n";
    std::cout << "  Duration: " << sim_config.duration << " s\n";
    std::cout << "  Dt      : " << sim_config.timestep << " s\n";
    std::cout << "  Waypoints: " << sim_config.waypoints.size() << "\n";

    std::unique_ptr<adsim::TelemetryServer> telemetry;
    if (args->stream) {
        adsim::TelemetryServerConfig tcfg;
        tcfg.host = args->stream_host;
        tcfg.port = args->stream_port;
        telemetry = std::make_unique<adsim::TelemetryServer>(tcfg);
        telemetry->start();
        std::cout << "  Telemetry: " << tcfg.host << ":" << tcfg.port << "\n";
    }
    std::cout << "\n";

    if (telemetry && args->wait_for_client) {
        std::cout << "Waiting for telemetry client...\n" << std::flush;
        while (!telemetry->is_client_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << "Client connected. Starting simulation.\n\n";
    }

    adsim::SimulationSummary summary;
    try {
        adsim::CsvLogger logger(args->log_path);
        adsim::Simulator sim(sim_config);
        summary = sim.run(&logger, telemetry.get());
    } catch (const std::exception& e) {
        std::cerr << "Simulation error: " << e.what() << "\n";
        return 1;
    }

    print_summary(summary);

    return summary.final_failsafe_state == adsim::FailsafeState::Abort ? 2 : 0;
}
