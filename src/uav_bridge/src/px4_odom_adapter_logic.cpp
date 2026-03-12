#include "uav_bridge/px4_odom_adapter_logic.hpp"

#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace uav_bridge
{

namespace
{

constexpr std::array<double, 3> kDefaultPositionVarianceEnuM2{0.02, 0.02, 0.02};
constexpr std::array<double, 3> kDefaultOrientationVarianceRad2{0.05, 0.05, 0.05};
constexpr std::array<double, 3> kDefaultVelocityVarianceEnuM2Ps2{0.10, 0.10, 0.10};

template<typename ArrayT>
bool isFiniteArray(const ArrayT & values)
{
  for (const auto & value : values) {
    if (!std::isfinite(static_cast<double>(value))) {
      return false;
    }
  }
  return true;
}

bool isFiniteQuaternion(const std::array<float, 4> & q)
{
  if (!isFiniteArray(q)) {
    return false;
  }

  const double norm_sq =
    static_cast<double>(q[0]) * static_cast<double>(q[0]) +
    static_cast<double>(q[1]) * static_cast<double>(q[1]) +
    static_cast<double>(q[2]) * static_cast<double>(q[2]) +
    static_cast<double>(q[3]) * static_cast<double>(q[3]);
  return norm_sq > 1.0e-12;
}

tf2::Quaternion quaternionFromPx4Array(const std::array<float, 4> & q)
{
  tf2::Quaternion quaternion(
    static_cast<double>(q[1]),
    static_cast<double>(q[2]),
    static_cast<double>(q[3]),
    static_cast<double>(q[0]));
  quaternion.normalize();
  return quaternion;
}

std::array<double, 3> bodyFrdVelocityToEnu(
  const std::array<float, 3> & velocity_body_frd,
  const std::array<float, 4> & q_ned_frd)
{
  const tf2::Matrix3x3 rotation_ned_frd(quaternionFromPx4Array(q_ned_frd));
  const tf2::Vector3 velocity_body(
    static_cast<double>(velocity_body_frd[0]),
    static_cast<double>(velocity_body_frd[1]),
    static_cast<double>(velocity_body_frd[2]));
  const tf2::Vector3 velocity_ned = rotation_ned_frd * velocity_body;
  return nedToEnuVector(velocity_ned.x(), velocity_ned.y(), velocity_ned.z());
}

bool hasValidLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg)
{
  return msg.xy_valid && msg.z_valid &&
         std::isfinite(static_cast<double>(msg.x)) &&
         std::isfinite(static_cast<double>(msg.y)) &&
         std::isfinite(static_cast<double>(msg.z));
}

bool hasValidLocalVelocity(const px4_msgs::msg::VehicleLocalPosition & msg)
{
  return msg.v_xy_valid && msg.v_z_valid &&
         std::isfinite(static_cast<double>(msg.vx)) &&
         std::isfinite(static_cast<double>(msg.vy)) &&
         std::isfinite(static_cast<double>(msg.vz));
}

bool hasValidOdometryPosition(const px4_msgs::msg::VehicleOdometry & msg)
{
  return msg.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
         isFiniteArray(msg.position);
}

bool hasValidOdometryOrientation(const px4_msgs::msg::VehicleOdometry & msg)
{
  return msg.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
         isFiniteQuaternion(msg.q);
}

bool hasValidOdometryVelocityNed(const px4_msgs::msg::VehicleOdometry & msg)
{
  return msg.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED &&
         isFiniteArray(msg.velocity);
}

bool hasValidOdometryVelocityBody(const px4_msgs::msg::VehicleOdometry & msg)
{
  return msg.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD &&
         isFiniteArray(msg.velocity) &&
         hasValidOdometryOrientation(msg);
}

template<typename ArrayT>
std::array<double, 3> copyFiniteWithDefaults(
  const ArrayT & values,
  const std::array<double, 3> & defaults)
{
  std::array<double, 3> out = defaults;
  for (std::size_t index = 0; index < out.size(); ++index) {
    const double value = static_cast<double>(values[index]);
    if (std::isfinite(value)) {
      out[index] = value;
    }
  }
  return out;
}

}  // namespace

double normalizeAngle(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double yawNedToEnu(double yaw_ned_rad)
{
  return normalizeAngle(M_PI_2 - yaw_ned_rad);
}

double yawFromQuaternionEnu(const tf2::Quaternion & quaternion_enu_flu)
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion_enu_flu).getRPY(roll, pitch, yaw);
  return yaw;
}

double computeYawRate(double previous_yaw_rad, double current_yaw_rad, double dt_sec)
{
  if (!std::isfinite(previous_yaw_rad) || !std::isfinite(current_yaw_rad) || dt_sec <= 1.0e-6) {
    return 0.0;
  }
  return normalizeAngle(current_yaw_rad - previous_yaw_rad) / dt_sec;
}

std::array<double, 3> nedToEnuPosition(double north_m, double east_m, double down_m)
{
  return {east_m, north_m, -down_m};
}

std::array<double, 3> nedToEnuVector(double north_mps, double east_mps, double down_mps)
{
  return {east_mps, north_mps, -down_mps};
}

std::array<double, 3> nedToEnuVariance(const std::array<float, 3> & ned_variance)
{
  return {
    std::isfinite(static_cast<double>(ned_variance[1])) ? static_cast<double>(ned_variance[1]) : std::numeric_limits<double>::quiet_NaN(),
    std::isfinite(static_cast<double>(ned_variance[0])) ? static_cast<double>(ned_variance[0]) : std::numeric_limits<double>::quiet_NaN(),
    std::isfinite(static_cast<double>(ned_variance[2])) ? static_cast<double>(ned_variance[2]) : std::numeric_limits<double>::quiet_NaN()};
}

tf2::Quaternion px4QuaternionToEnuFlu(const std::array<float, 4> & q_ned_frd)
{
  const tf2::Matrix3x3 rotation_ned_frd(quaternionFromPx4Array(q_ned_frd));
  const tf2::Matrix3x3 ned_to_enu(
    0.0, 1.0, 0.0,
    1.0, 0.0, 0.0,
    0.0, 0.0, -1.0);
  const tf2::Matrix3x3 flu_to_frd(
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0);

  const tf2::Matrix3x3 rotation_enu_flu = ned_to_enu * rotation_ned_frd * flu_to_frd;
  tf2::Quaternion quaternion_enu_flu;
  rotation_enu_flu.getRotation(quaternion_enu_flu);
  quaternion_enu_flu.normalize();
  return quaternion_enu_flu;
}

tf2::Quaternion yawNedToEnuQuaternion(double yaw_ned_rad)
{
  tf2::Quaternion quaternion_enu_flu;
  quaternion_enu_flu.setRPY(0.0, 0.0, yawNedToEnu(yaw_ned_rad));
  quaternion_enu_flu.normalize();
  return quaternion_enu_flu;
}

Px4OdomResolvedState resolvePx4OdomState(
  const std::optional<px4_msgs::msg::VehicleLocalPosition> & local_position,
  const std::optional<px4_msgs::msg::VehicleOdometry> & vehicle_odometry)
{
  Px4OdomResolvedState resolved;
  resolved.position_variance_enu_m2 = kDefaultPositionVarianceEnuM2;
  resolved.orientation_variance_rad2 = kDefaultOrientationVarianceRad2;
  resolved.velocity_variance_enu_m2ps2 = kDefaultVelocityVarianceEnuM2Ps2;

  if (vehicle_odometry.has_value()) {
    if (hasValidOdometryPosition(*vehicle_odometry)) {
      resolved.position_valid = true;
      resolved.used_odometry_position = true;
      resolved.position_enu_m = nedToEnuPosition(
        static_cast<double>(vehicle_odometry->position[0]),
        static_cast<double>(vehicle_odometry->position[1]),
        static_cast<double>(vehicle_odometry->position[2]));
    }

    if (hasValidOdometryOrientation(*vehicle_odometry)) {
      resolved.orientation_valid = true;
      resolved.used_odometry_orientation = true;
      resolved.orientation_enu_flu = px4QuaternionToEnuFlu(vehicle_odometry->q);
    }

    if (hasValidOdometryVelocityNed(*vehicle_odometry)) {
      resolved.linear_velocity_valid = true;
      resolved.used_odometry_velocity = true;
      resolved.linear_velocity_enu_mps = nedToEnuVector(
        static_cast<double>(vehicle_odometry->velocity[0]),
        static_cast<double>(vehicle_odometry->velocity[1]),
        static_cast<double>(vehicle_odometry->velocity[2]));
      resolved.velocity_variance_enu_m2ps2 = nedToEnuVariance(vehicle_odometry->velocity_variance);
    } else if (hasValidOdometryVelocityBody(*vehicle_odometry)) {
      resolved.linear_velocity_valid = true;
      resolved.used_odometry_velocity = true;
      resolved.linear_velocity_enu_mps = bodyFrdVelocityToEnu(vehicle_odometry->velocity, vehicle_odometry->q);
    }

    resolved.position_variance_enu_m2 = copyFiniteWithDefaults(
      nedToEnuVariance(vehicle_odometry->position_variance),
      kDefaultPositionVarianceEnuM2);
    resolved.orientation_variance_rad2 = copyFiniteWithDefaults(
      vehicle_odometry->orientation_variance,
      kDefaultOrientationVarianceRad2);
  }

  if (!resolved.position_valid && local_position.has_value() && hasValidLocalPosition(*local_position)) {
    resolved.position_valid = true;
    resolved.position_enu_m = nedToEnuPosition(
      static_cast<double>(local_position->x),
      static_cast<double>(local_position->y),
      static_cast<double>(local_position->z));
  }

  if (!resolved.orientation_valid && local_position.has_value() && std::isfinite(static_cast<double>(local_position->heading))) {
    resolved.orientation_valid = true;
    resolved.used_heading_fallback = true;
    resolved.orientation_enu_flu = yawNedToEnuQuaternion(static_cast<double>(local_position->heading));
  }

  if (!resolved.linear_velocity_valid && local_position.has_value() && hasValidLocalVelocity(*local_position)) {
    resolved.linear_velocity_valid = true;
    resolved.linear_velocity_enu_mps = nedToEnuVector(
      static_cast<double>(local_position->vx),
      static_cast<double>(local_position->vy),
      static_cast<double>(local_position->vz));
  }

  return resolved;
}

}  // namespace uav_bridge
