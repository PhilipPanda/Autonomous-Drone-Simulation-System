#include "adsim/network/telemetry_server.hpp"

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
   using socket_t = SOCKET;
   using socklen_t = int;
   static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
   static void close_socket(socket_t s) { closesocket(s); }
   static bool is_valid(socket_t s) { return s != INVALID_SOCKET; }
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#  include <errno.h>
   using socket_t = int;
   static constexpr socket_t kInvalidSocket = -1;
   static void close_socket(socket_t s) { ::close(s); }
   static bool is_valid(socket_t s) { return s >= 0; }
#endif

#include <atomic>
#include <thread>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <cstring>

namespace adsim {

static std::string json_vec3(const char* key, const Vec3& v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << '"' << key << R"(":{"x":)" << v.x
       << R"(,"y":)" << v.y
       << R"(,"z":)" << v.z << '}';
    return ss.str();
}

static std::string json_quat(const char* key, const Quaternion& q) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << '"' << key << R"(":{"w":)" << q.w
       << R"(,"x":)" << q.x
       << R"(,"y":)" << q.y
       << R"(,"z":)" << q.z << '}';
    return ss.str();
}

static const char* failsafe_json_name(FailsafeState state) {
    switch (state) {
        case FailsafeState::Nominal:           return "nominal";
        case FailsafeState::HoverHold:         return "hover_hold";
        case FailsafeState::ControlledDescent: return "controlled_descent";
        case FailsafeState::Abort:             return "abort";
    }
    return "nominal";
}

std::string serialize_telemetry(const TelemetryFrame& f) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << '{';
    ss << R"("t":)" << f.sim_time << ',';
    ss << json_vec3("pos", f.position) << ',';
    ss << json_vec3("vel", f.velocity) << ',';
    ss << json_vec3("accel", f.acceleration) << ',';
    ss << json_quat("att", f.attitude) << ',';
    ss << json_vec3("euler", f.euler) << ',';
    ss << json_vec3("angvel", f.angular_velocity) << ',';
    ss << R"("wp_idx":)" << f.waypoint_index << ',';
    ss << R"("wp_total":)" << f.total_waypoints << ',';
    ss << json_vec3("wp_target", f.waypoint_target) << ',';
    ss << R"("mission_complete":)" << (f.mission_complete ? "true" : "false") << ',';
    ss << R"("failsafe":")" << failsafe_json_name(f.failsafe_state) << '"';
    ss << "}\n";
    return ss.str();
}

struct TelemetryServer::Impl {
    TelemetryServerConfig config;
    double publish_interval;
    double last_publish_time{-1.0e9};

    socket_t listen_fd{kInvalidSocket};
    socket_t client_fd{kInvalidSocket};
    mutable std::mutex client_mutex;

    std::atomic<bool> running{false};
    std::thread accept_thread;

#ifdef _WIN32
    bool wsa_initialized{false};
#endif

    explicit Impl(TelemetryServerConfig cfg)
        : config(std::move(cfg))
        , publish_interval(config.rate_hz > 0.0 ? 1.0 / config.rate_hz : 0.0)
    {}

    void accept_loop();
    bool send_line(const std::string& data);
    void close_client();
};

void TelemetryServer::Impl::close_client() {
    if (is_valid(client_fd)) {
        close_socket(client_fd);
        client_fd = kInvalidSocket;
    }
}

void TelemetryServer::Impl::accept_loop() {
    while (running.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd, &read_fds);

        struct timeval tv{};
        tv.tv_usec = 100000;

#ifdef _WIN32
        int nfds = 0;
#else
        int nfds = static_cast<int>(listen_fd) + 1;
#endif

        int ret = select(nfds, &read_fds, nullptr, nullptr, &tv);
        if (ret <= 0 || !FD_ISSET(listen_fd, &read_fds)) continue;

        sockaddr_in peer{};
        socklen_t peer_len = sizeof(peer);
        socket_t incoming = ::accept(listen_fd,
                                     reinterpret_cast<sockaddr*>(&peer),
                                     &peer_len);
        if (!is_valid(incoming)) continue;

        std::lock_guard<std::mutex> lock(client_mutex);
        close_client();
        client_fd = incoming;
    }
}

bool TelemetryServer::Impl::send_line(const std::string& data) {
    std::lock_guard<std::mutex> lock(client_mutex);
    if (!is_valid(client_fd)) return false;

    const char* ptr = data.data();
    int remaining = static_cast<int>(data.size());

    while (remaining > 0) {
#ifdef _WIN32
        int sent = ::send(client_fd, ptr, remaining, 0);
        if (sent == SOCKET_ERROR) {
#else
        ssize_t sent = ::send(client_fd, ptr, static_cast<std::size_t>(remaining), MSG_NOSIGNAL);
        if (sent < 0) {
#endif
            close_client();
            return false;
        }
        ptr += sent;
        remaining -= static_cast<int>(sent);
    }
    return true;
}

TelemetryServer::TelemetryServer(TelemetryServerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{}

TelemetryServer::~TelemetryServer() {
    stop();
}

void TelemetryServer::start() {
#ifdef _WIN32
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) == 0) {
        impl_->wsa_initialized = true;
    }
#endif

    impl_->listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (!is_valid(impl_->listen_fd)) return;

    int opt = 1;
#ifdef _WIN32
    setsockopt(impl_->listen_fd, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&opt), sizeof(opt));
#else
    setsockopt(impl_->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(impl_->config.port);
    inet_pton(AF_INET, impl_->config.host.c_str(), &addr.sin_addr);

    if (::bind(impl_->listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close_socket(impl_->listen_fd);
        impl_->listen_fd = kInvalidSocket;
        return;
    }

    ::listen(impl_->listen_fd, 1);

    impl_->running.store(true);
    impl_->accept_thread = std::thread([this] { impl_->accept_loop(); });
}

void TelemetryServer::stop() {
    impl_->running.store(false);

    if (impl_->accept_thread.joinable()) {
        impl_->accept_thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(impl_->client_mutex);
        impl_->close_client();
    }

    if (is_valid(impl_->listen_fd)) {
        close_socket(impl_->listen_fd);
        impl_->listen_fd = kInvalidSocket;
    }

#ifdef _WIN32
    if (impl_->wsa_initialized) {
        WSACleanup();
        impl_->wsa_initialized = false;
    }
#endif
}

bool TelemetryServer::is_client_connected() const {
    std::lock_guard<std::mutex> lock(impl_->client_mutex);
    return is_valid(impl_->client_fd);
}

void TelemetryServer::try_publish(const TelemetryFrame& frame, double sim_time) {
    if (sim_time - impl_->last_publish_time < impl_->publish_interval) return;
    impl_->last_publish_time = sim_time;
    impl_->send_line(serialize_telemetry(frame));
}

void TelemetryServer::publish_now(const TelemetryFrame& frame) {
    impl_->last_publish_time = 1.0e9;
    impl_->send_line(serialize_telemetry(frame));
}

}
