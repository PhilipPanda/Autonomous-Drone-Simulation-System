#include "adsim/logging/csv_logger.hpp"

#include <iomanip>
#include <stdexcept>

namespace adsim {

CsvLogger::CsvLogger(const std::string& path) {
    file_.open(path);
    if (!file_.is_open()) {
        throw std::runtime_error("Failed to open log file: " + path);
    }
    file_ << std::fixed << std::setprecision(6);
    write_header();
}

CsvLogger::~CsvLogger() {
    if (file_.is_open()) {
        file_.flush();
    }
}

void CsvLogger::write_header() {
    file_ << "timestamp"
          << ",pos_x,pos_y,pos_z"
          << ",vel_x,vel_y,vel_z"
          << ",accel_x,accel_y,accel_z"
          << ",att_w,att_x,att_y,att_z"
          << ",roll,pitch,yaw"
          << ",angvel_x,angvel_y,angvel_z"
          << ",imu_accel_x,imu_accel_y,imu_accel_z"
          << ",imu_gyro_x,imu_gyro_y,imu_gyro_z"
          << ",imu_valid"
          << ",est_att_w,est_att_x,est_att_y,est_att_z"
          << ",est_roll,est_pitch,est_yaw"
          << ",failsafe"
          << ",waypoint_index"
          << "\n";
}

void CsvLogger::log(
    const DroneState& state,
    const ImuReading& imu,
    const FilterState& filter,
    FailsafeState failsafe,
    std::size_t waypoint_index
) {
    Vec3 euler = state.attitude.to_euler();
    Vec3 est_euler = filter.attitude.to_euler();

    file_ << state.timestamp
          << "," << state.position.x << "," << state.position.y << "," << state.position.z
          << "," << state.velocity.x << "," << state.velocity.y << "," << state.velocity.z
          << "," << state.acceleration.x << "," << state.acceleration.y << "," << state.acceleration.z
          << "," << state.attitude.w << "," << state.attitude.x << "," << state.attitude.y << "," << state.attitude.z
          << "," << euler.x << "," << euler.y << "," << euler.z
          << "," << state.angular_velocity.x << "," << state.angular_velocity.y << "," << state.angular_velocity.z
          << "," << imu.accelerometer.x << "," << imu.accelerometer.y << "," << imu.accelerometer.z
          << "," << imu.gyroscope.x << "," << imu.gyroscope.y << "," << imu.gyroscope.z
          << "," << (imu.valid ? 1 : 0)
          << "," << filter.attitude.w << "," << filter.attitude.x << "," << filter.attitude.y << "," << filter.attitude.z
          << "," << est_euler.x << "," << est_euler.y << "," << est_euler.z
          << "," << failsafe_state_name(failsafe)
          << "," << waypoint_index
          << "\n";

    ++records_written_;
}

void CsvLogger::flush() {
    file_.flush();
}

}
