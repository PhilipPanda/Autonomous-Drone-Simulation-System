#pragma once

#include "adsim/math/vec3.hpp"
#include <cmath>
#include <numbers>

namespace adsim {

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Quaternion() = default;
    constexpr Quaternion(double qw, double qx, double qy, double qz) : w(qw), x(qx), y(qy), z(qz) {}

    static Quaternion identity() { return {1.0, 0.0, 0.0, 0.0}; }

    static Quaternion from_euler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    static Quaternion from_axis_angle(const Vec3& axis, double angle) {
        Vec3 n = axis.normalized();
        double s = std::sin(angle * 0.5);
        return {std::cos(angle * 0.5), n.x * s, n.y * s, n.z * s};
    }

    Vec3 to_euler() const {
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (w * y - z * x);
        double pitch = std::abs(sinp) >= 1.0
            ? std::copysign(std::numbers::pi / 2.0, sinp)
            : std::asin(sinp);

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {roll, pitch, yaw};
    }

    Quaternion operator*(const Quaternion& rhs) const {
        return {
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        };
    }

    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quaternion normalized() const {
        double n = norm();
        if (n < 1e-12) return identity();
        return {w / n, x / n, y / n, z / n};
    }

    Vec3 rotate(const Vec3& v) const {
        Quaternion qv{0.0, v.x, v.y, v.z};
        Quaternion result = *this * qv * conjugate();
        return {result.x, result.y, result.z};
    }

    Vec3 rotate_inverse(const Vec3& v) const {
        return conjugate().rotate(v);
    }

    bool is_finite() const {
        return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    bool is_unit(double tol = 1e-6) const {
        return std::abs(norm() - 1.0) < tol;
    }

    Quaternion integrated(const Vec3& angular_velocity, double dt) const {
        double angle = angular_velocity.norm() * dt;
        if (angle < 1e-12) return *this;
        Quaternion dq = from_axis_angle(angular_velocity, angle);
        return (*this * dq).normalized();
    }
};

}
