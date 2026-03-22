#include "adsim/dynamics/flight_model.hpp"

namespace adsim {

FlightModel::FlightModel(Params params) : params_(params) {}

Vec3 FlightModel::compute_acceleration(const DroneState& state, const MotorCommand& cmd) const {
    Vec3 thrust_world = state.attitude.rotate({0.0, 0.0, cmd.thrust});
    Vec3 gravity_world{0.0, 0.0, -params_.mass * params_.gravity};

    Vec3 drag{
        -params_.drag_linear.x * state.velocity.x,
        -params_.drag_linear.y * state.velocity.y,
        -params_.drag_linear.z * state.velocity.z
    };

    Vec3 net_force = thrust_world + gravity_world + drag;
    return net_force / params_.mass;
}

Vec3 FlightModel::compute_angular_acceleration(const DroneState& state, const MotorCommand& cmd) const {
    Vec3 w = state.angular_velocity;

    Vec3 gyroscopic{
        (params_.inertia.y - params_.inertia.z) * w.y * w.z,
        (params_.inertia.z - params_.inertia.x) * w.z * w.x,
        (params_.inertia.x - params_.inertia.y) * w.x * w.y
    };

    Vec3 drag{
        -params_.drag_angular.x * w.x,
        -params_.drag_angular.y * w.y,
        -params_.drag_angular.z * w.z
    };

    Vec3 net_torque = cmd.torque + gyroscopic + drag;

    return {
        net_torque.x / params_.inertia.x,
        net_torque.y / params_.inertia.y,
        net_torque.z / params_.inertia.z
    };
}

DroneState FlightModel::step(const DroneState& state, const MotorCommand& cmd, double dt) const {
    Vec3 accel = compute_acceleration(state, cmd);
    Vec3 alpha = compute_angular_acceleration(state, cmd);

    DroneState next;
    next.timestamp = state.timestamp + dt;

    next.velocity = state.velocity + accel * dt;
    next.position = state.position + state.velocity * dt + accel * (0.5 * dt * dt);
    next.acceleration = accel;

    next.angular_velocity = state.angular_velocity + alpha * dt;
    next.attitude = state.attitude.integrated(state.angular_velocity, dt);

    return next;
}

}
