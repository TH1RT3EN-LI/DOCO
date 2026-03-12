#include <array>
#include <optional>

#include <gtest/gtest.h>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "uav_bridge/uav_state_bridge_logic.hpp"

namespace uav_bridge
{
namespace
{

TEST(UavStateBridgeLogic, ConvertsBodyFrdVelocityToBodyFlu)
{
  px4_msgs::msg::VehicleOdometry odom;
  odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  odom.q = {1.0f, 0.0f, 0.0f, 0.0f};
  odom.position = {0.0f, 0.0f, -1.0f};
  odom.velocity = {1.0f, 2.0f, -3.0f};

  const auto resolved = resolveUavStateBridgeState(std::nullopt, odom);
  ASSERT_TRUE(resolved.linear_velocity_valid);
  EXPECT_DOUBLE_EQ(resolved.linear_velocity_body_flu_mps[0], 1.0);
  EXPECT_DOUBLE_EQ(resolved.linear_velocity_body_flu_mps[1], -2.0);
  EXPECT_DOUBLE_EQ(resolved.linear_velocity_body_flu_mps[2], 3.0);
}

TEST(UavStateBridgeLogic, ConvertsWorldVelocityToBodyUsingHeadingFallback)
{
  px4_msgs::msg::VehicleLocalPosition local_position;
  local_position.xy_valid = true;
  local_position.z_valid = true;
  local_position.v_xy_valid = true;
  local_position.v_z_valid = true;
  local_position.x = 0.0f;
  local_position.y = 0.0f;
  local_position.z = -2.0f;
  local_position.vx = 2.0f;
  local_position.vy = 0.0f;
  local_position.vz = 0.0f;
  local_position.heading = 0.0f;

  const auto resolved = resolveUavStateBridgeState(local_position, std::nullopt);
  ASSERT_TRUE(resolved.linear_velocity_valid);
  EXPECT_NEAR(resolved.linear_velocity_body_flu_mps[0], 2.0, 1.0e-5);
  EXPECT_NEAR(resolved.linear_velocity_body_flu_mps[1], 0.0, 1.0e-5);
  EXPECT_NEAR(resolved.linear_velocity_body_flu_mps[2], 0.0, 1.0e-5);
}

TEST(UavStateBridgeLogic, UsesOdometryPositionWhenAvailable)
{
  px4_msgs::msg::VehicleLocalPosition local_position;
  local_position.xy_valid = true;
  local_position.z_valid = true;
  local_position.x = 1.0f;
  local_position.y = 2.0f;
  local_position.z = -3.0f;
  local_position.heading = 0.0f;

  px4_msgs::msg::VehicleOdometry odom;
  odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odom.q = {1.0f, 0.0f, 0.0f, 0.0f};
  odom.position = {4.0f, 5.0f, -6.0f};

  const auto resolved = resolveUavStateBridgeState(local_position, odom);
  ASSERT_TRUE(resolved.position_valid);
  EXPECT_DOUBLE_EQ(resolved.position_enu_m[0], 5.0);
  EXPECT_DOUBLE_EQ(resolved.position_enu_m[1], 4.0);
  EXPECT_DOUBLE_EQ(resolved.position_enu_m[2], 6.0);
}

}  // namespace
}  // namespace uav_bridge
