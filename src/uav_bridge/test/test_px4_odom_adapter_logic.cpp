#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "uav_bridge/px4_odom_adapter_logic.hpp"

namespace
{

using uav_bridge::computeYawRate;
using uav_bridge::resolvePx4OdomState;
using uav_bridge::yawFromQuaternionEnu;
using uav_bridge::yawNedToEnu;

TEST(Px4OdomAdapterLogic, ResolvesOdometryPoseVelocityAndCovariance)
{
  px4_msgs::msg::VehicleOdometry odometry;
  odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  odometry.position = {1.0f, 2.0f, -3.0f};
  odometry.q = {1.0f, 0.0f, 0.0f, 0.0f};
  odometry.velocity = {4.0f, 5.0f, -6.0f};
  odometry.position_variance = {0.11f, 0.22f, 0.33f};
  odometry.orientation_variance = {0.44f, 0.55f, 0.66f};
  odometry.velocity_variance = {0.77f, 0.88f, 0.99f};

  const auto resolved = resolvePx4OdomState(std::nullopt, odometry);

  EXPECT_TRUE(resolved.position_valid);
  EXPECT_TRUE(resolved.orientation_valid);
  EXPECT_TRUE(resolved.linear_velocity_valid);
  EXPECT_TRUE(resolved.used_odometry_position);
  EXPECT_TRUE(resolved.used_odometry_orientation);
  EXPECT_TRUE(resolved.used_odometry_velocity);
  EXPECT_NEAR(resolved.position_enu_m[0], 2.0, 1.0e-6);
  EXPECT_NEAR(resolved.position_enu_m[1], 1.0, 1.0e-6);
  EXPECT_NEAR(resolved.position_enu_m[2], 3.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[0], 5.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[1], 4.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[2], 6.0, 1.0e-6);
  EXPECT_NEAR(resolved.position_variance_enu_m2[0], 0.22, 1.0e-6);
  EXPECT_NEAR(resolved.position_variance_enu_m2[1], 0.11, 1.0e-6);
  EXPECT_NEAR(resolved.position_variance_enu_m2[2], 0.33, 1.0e-6);
  EXPECT_NEAR(resolved.orientation_variance_rad2[0], 0.44, 1.0e-6);
  EXPECT_NEAR(resolved.orientation_variance_rad2[1], 0.55, 1.0e-6);
  EXPECT_NEAR(resolved.orientation_variance_rad2[2], 0.66, 1.0e-6);
  EXPECT_NEAR(resolved.velocity_variance_enu_m2ps2[0], 0.88, 1.0e-6);
  EXPECT_NEAR(resolved.velocity_variance_enu_m2ps2[1], 0.77, 1.0e-6);
  EXPECT_NEAR(resolved.velocity_variance_enu_m2ps2[2], 0.99, 1.0e-6);
  EXPECT_NEAR(yawFromQuaternionEnu(resolved.orientation_enu_flu), M_PI_2, 1.0e-6);
}

TEST(Px4OdomAdapterLogic, FallsBackToLocalPositionHeadingAndVelocity)
{
  px4_msgs::msg::VehicleLocalPosition local_position;
  local_position.xy_valid = true;
  local_position.z_valid = true;
  local_position.v_xy_valid = true;
  local_position.v_z_valid = true;
  local_position.x = 10.0f;
  local_position.y = -4.0f;
  local_position.z = -2.0f;
  local_position.vx = 1.0f;
  local_position.vy = -2.0f;
  local_position.vz = 0.5f;
  local_position.heading = 0.3f;

  px4_msgs::msg::VehicleOdometry odometry;
  odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN;
  odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
  odometry.q = {
    std::numeric_limits<float>::quiet_NaN(),
    0.0f,
    0.0f,
    0.0f};

  const auto resolved = resolvePx4OdomState(local_position, odometry);

  EXPECT_TRUE(resolved.position_valid);
  EXPECT_TRUE(resolved.orientation_valid);
  EXPECT_TRUE(resolved.linear_velocity_valid);
  EXPECT_FALSE(resolved.used_odometry_position);
  EXPECT_FALSE(resolved.used_odometry_orientation);
  EXPECT_FALSE(resolved.used_odometry_velocity);
  EXPECT_TRUE(resolved.used_heading_fallback);
  EXPECT_NEAR(resolved.position_enu_m[0], -4.0, 1.0e-6);
  EXPECT_NEAR(resolved.position_enu_m[1], 10.0, 1.0e-6);
  EXPECT_NEAR(resolved.position_enu_m[2], 2.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[0], -2.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[1], 1.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[2], -0.5, 1.0e-6);
  EXPECT_NEAR(yawFromQuaternionEnu(resolved.orientation_enu_flu), yawNedToEnu(0.3), 1.0e-6);
}

TEST(Px4OdomAdapterLogic, ConvertsBodyVelocityToWorldEnu)
{
  px4_msgs::msg::VehicleOdometry odometry;
  odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  odometry.position = {0.0f, 0.0f, 0.0f};
  odometry.q = {1.0f, 0.0f, 0.0f, 0.0f};
  odometry.velocity = {1.0f, 2.0f, 3.0f};

  const auto resolved = resolvePx4OdomState(std::nullopt, odometry);

  EXPECT_TRUE(resolved.linear_velocity_valid);
  EXPECT_TRUE(resolved.used_odometry_velocity);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[0], 2.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[1], 1.0, 1.0e-6);
  EXPECT_NEAR(resolved.linear_velocity_enu_mps[2], -3.0, 1.0e-6);
}

TEST(Px4OdomAdapterLogic, WrapsYawRateAcrossPiBoundary)
{
  const double yaw_rate = computeYawRate(3.13, -3.13, 0.1);
  EXPECT_NEAR(yaw_rate, 0.2318530718, 1.0e-6);
}

}  // namespace
