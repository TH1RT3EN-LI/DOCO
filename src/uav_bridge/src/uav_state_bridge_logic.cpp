#include "uav_bridge/uav_state_bridge_logic.hpp"

#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace uav_bridge
{
namespace
{

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
         isFiniteArray(msg.velocity);
}

std::array<float, 4> yawNedQuaternionArray(float yaw_ned_rad)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, static_cast<double>(yaw_ned_rad));
  quaternion.normalize();
  return {
    static_cast<float>(quaternion.w()),
    static_cast<float>(quaternion.x()),
    static_cast<float>(quaternion.y()),
    static_cast<float>(quaternion.z())};
}

}  // namespace

double normalizeAngleRad(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double computeYawRateRad(double previous_yaw_rad, double current_yaw_rad, double dt_sec)
{
  if (!std::isfinite(previous_yaw_rad) || !std::isfinite(current_yaw_rad) || dt_sec <= 1.0e-6) {
    return 0.0;
  }
  return normalizeAngleRad(current_yaw_rad - previous_yaw_rad) / dt_sec;
}

double yawNedToEnuRad(double yaw_ned_rad)
{
  return normalizeAngleRad(M_PI_2 - yaw_ned_rad);
}

std::array<double, 3> nedToEnuPosition3D(double north_m, double east_m, double down_m)
{
  return {east_m, north_m, -down_m};
}

std::array<double, 3> bodyFrdToFluVelocity(const std::array<float, 3> & velocity_body_frd)
{
  return {
    static_cast<double>(velocity_body_frd[0]),
    static_cast<double>(-velocity_body_frd[1]),
    static_cast<double>(-velocity_body_frd[2])};
}

tf2::Quaternion yawNedToEnuQuaternion(double yaw_ned_rad)
{
  tf2::Quaternion quaternion_enu_flu;
  quaternion_enu_flu.setRPY(0.0, 0.0, yawNedToEnuRad(yaw_ned_rad));
  quaternion_enu_flu.normalize();
  return quaternion_enu_flu;
}

tf2::Quaternion px4QuaternionToEnuFluQuaternion(const std::array<float, 4> & q_ned_frd)
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

std::array<double, 3> velocityNedToBodyFlu(
  const std::array<float, 3> & velocity_ned,
  const std::array<float, 4> & q_ned_frd)
{
  const tf2::Matrix3x3 rotation_ned_frd(quaternionFromPx4Array(q_ned_frd));
  const tf2::Vector3 velocity_world(
    static_cast<double>(velocity_ned[0]),
    static_cast<double>(velocity_ned[1]),
    static_cast<double>(velocity_ned[2]));
  const tf2::Vector3 velocity_body_frd = rotation_ned_frd.inverse() * velocity_world;
  return {
    velocity_body_frd.x(),
    -velocity_body_frd.y(),
    -velocity_body_frd.z()};
}

UavStateBridgeResolvedState resolveUavStateBridgeState(
  const std::optional<px4_msgs::msg::VehicleLocalPosition> & local_position,
  const std::optional<px4_msgs::msg::VehicleOdometry> & vehicle_odometry)
{
  UavStateBridgeResolvedState resolved;

  std::array<float, 4> orientation_q_ned_frd{};
  bool has_orientation_q = false;

  if (vehicle_odometry.has_value()) {
    if (hasValidOdometryPosition(*vehicle_odometry)) {
      resolved.position_valid = true;
      resolved.position_enu_m = nedToEnuPosition3D(
        static_cast<double>(vehicle_odometry->position[0]),
        static_cast<double>(vehicle_odometry->position[1]),
        static_cast<double>(vehicle_odometry->position[2]));
    }

    if (hasValidOdometryOrientation(*vehicle_odometry)) {
      resolved.orientation_valid = true;
      orientation_q_ned_frd = vehicle_odometry->q;
      has_orientation_q = true;
      resolved.orientation_enu_flu = px4QuaternionToEnuFluQuaternion(vehicle_odometry->q);
    }
  }

  if (!resolved.position_valid && local_position.has_value() && hasValidLocalPosition(*local_position)) {
    resolved.position_valid = true;
    resolved.position_enu_m = nedToEnuPosition3D(
      static_cast<double>(local_position->x),
      static_cast<double>(local_position->y),
      static_cast<double>(local_position->z));
  }

  if (!resolved.orientation_valid && local_position.has_value() &&
      std::isfinite(static_cast<double>(local_position->heading)))
  {
    resolved.orientation_valid = true;
    orientation_q_ned_frd = yawNedQuaternionArray(local_position->heading);
    has_orientation_q = true;
    resolved.orientation_enu_flu = yawNedToEnuQuaternion(local_position->heading);
  }

  if (vehicle_odometry.has_value()) {
    if (hasValidOdometryVelocityBody(*vehicle_odometry)) {
      resolved.linear_velocity_valid = true;
      resolved.linear_velocity_body_flu_mps = bodyFrdToFluVelocity(vehicle_odometry->velocity);
    } else if (hasValidOdometryVelocityNed(*vehicle_odometry) && has_orientation_q) {
      resolved.linear_velocity_valid = true;
      resolved.linear_velocity_body_flu_mps = velocityNedToBodyFlu(vehicle_odometry->velocity, orientation_q_ned_frd);
    }
  }

  if (!resolved.linear_velocity_valid && local_position.has_value() && hasValidLocalVelocity(*local_position) && has_orientation_q) {
    resolved.linear_velocity_valid = true;
    resolved.linear_velocity_body_flu_mps = velocityNedToBodyFlu(
      {local_position->vx, local_position->vy, local_position->vz},
      orientation_q_ned_frd);
  }

  return resolved;
}

}  // namespace uav_bridge
