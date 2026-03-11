#include "relative_position_fusion/time_alignment_buffer.hpp"

#include <gtest/gtest.h>

namespace relative_position_fusion
{

TEST(TimeAlignmentBufferTest, ReturnsLatestFreshSamples)
{
  TimeAlignmentBuffer buffer(2.0);
  buffer.PushVelocity(VelocitySample2D{rclcpp::Time(1, 0, RCL_ROS_TIME), Eigen::Vector2d(0.1, 0.2), "vel"});
  buffer.PushVelocity(VelocitySample2D{rclcpp::Time(2, 0, RCL_ROS_TIME), Eigen::Vector2d(0.3, 0.4), "vel"});

  const auto sample = buffer.LatestVelocity(rclcpp::Time(2, 1000, RCL_ROS_TIME), 0.5);
  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(sample->velocity_global.x(), 0.3, 1e-9);
  EXPECT_NEAR(sample->velocity_global.y(), 0.4, 1e-9);
}

TEST(TimeAlignmentBufferTest, EstimatesVelocityFromPoseHistory)
{
  TimeAlignmentBuffer buffer(5.0);
  PoseSample2D pose0;
  pose0.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  pose0.position_global = Eigen::Vector2d(0.0, 0.0);
  buffer.PushPose(pose0);

  PoseSample2D pose1;
  pose1.stamp = rclcpp::Time(3, 0, RCL_ROS_TIME);
  pose1.position_global = Eigen::Vector2d(4.0, 2.0);
  buffer.PushPose(pose1);

  const auto velocity = buffer.EstimateVelocityFromPoseHistory(
    rclcpp::Time(3, 0, RCL_ROS_TIME), 0.1, 0.5, 5.0, "pose_diff");
  ASSERT_TRUE(velocity.has_value());
  EXPECT_NEAR(velocity->velocity_global.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity->velocity_global.y(), 1.0, 1e-9);
}

}  // namespace relative_position_fusion
