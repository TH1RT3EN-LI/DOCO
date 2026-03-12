#pragma once

#include <array>
#include <optional>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace uav_bridge
{

struct UavStateBridgeResolvedState
{
  bool position_valid{false};
  bool orientation_valid{false};
  bool linear_velocity_valid{false};

  std::array<double, 3> position_enu_m{};
  std::array<double, 3> linear_velocity_body_flu_mps{};
  tf2::Quaternion orientation_enu_flu{0.0, 0.0, 0.0, 1.0};
};

double normalizeAngleRad(double angle_rad);
double computeYawRateRad(double previous_yaw_rad, double current_yaw_rad, double dt_sec);
double yawNedToEnuRad(double yaw_ned_rad);
std::array<double, 3> nedToEnuPosition3D(double north_m, double east_m, double down_m);
std::array<double, 3> bodyFrdToFluVelocity(const std::array<float, 3> & velocity_body_frd);

tf2::Quaternion yawNedToEnuQuaternion(double yaw_ned_rad);
tf2::Quaternion px4QuaternionToEnuFluQuaternion(const std::array<float, 4> & q_ned_frd);

std::array<double, 3> velocityNedToBodyFlu(
  const std::array<float, 3> & velocity_ned,
  const std::array<float, 4> & q_ned_frd);

UavStateBridgeResolvedState resolveUavStateBridgeState(
  const std::optional<px4_msgs::msg::VehicleLocalPosition> & local_position,
  const std::optional<px4_msgs::msg::VehicleOdometry> & vehicle_odometry);

}  // namespace uav_bridge
