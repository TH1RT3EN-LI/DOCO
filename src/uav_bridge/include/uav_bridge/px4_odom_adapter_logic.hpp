#pragma once

#include <array>
#include <optional>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace uav_bridge
{

struct Px4OdomResolvedState
{
  bool position_valid{false};
  bool orientation_valid{false};
  bool linear_velocity_valid{false};

  bool used_odometry_position{false};
  bool used_odometry_orientation{false};
  bool used_odometry_velocity{false};
  bool used_heading_fallback{false};

  std::array<double, 3> position_enu_m{};
  std::array<double, 3> linear_velocity_enu_mps{};
  std::array<double, 3> position_variance_enu_m2{};
  std::array<double, 3> orientation_variance_rad2{};
  std::array<double, 3> velocity_variance_enu_m2ps2{};
  tf2::Quaternion orientation_enu_flu{0.0, 0.0, 0.0, 1.0};
};

double normalizeAngle(double angle_rad);
double yawNedToEnu(double yaw_ned_rad);
double yawFromQuaternionEnu(const tf2::Quaternion & quaternion_enu_flu);
double computeYawRate(double previous_yaw_rad, double current_yaw_rad, double dt_sec);

std::array<double, 3> nedToEnuPosition(double north_m, double east_m, double down_m);
std::array<double, 3> nedToEnuVector(double north_mps, double east_mps, double down_mps);
std::array<double, 3> nedToEnuVariance(const std::array<float, 3> & ned_variance);

tf2::Quaternion px4QuaternionToEnuFlu(const std::array<float, 4> & q_ned_frd);
tf2::Quaternion yawNedToEnuQuaternion(double yaw_ned_rad);

Px4OdomResolvedState resolvePx4OdomState(
  const std::optional<px4_msgs::msg::VehicleLocalPosition> & local_position,
  const std::optional<px4_msgs::msg::VehicleOdometry> & vehicle_odometry);

}  // namespace uav_bridge
