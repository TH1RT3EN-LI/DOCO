#include "relative_position_fusion/relative_tracking_controller.hpp"

#include <gtest/gtest.h>

namespace relative_position_fusion
{
namespace
{

RelativeTrackingController::Inputs MakeInputs(double now_sec)
{
  RelativeTrackingController::Inputs inputs;
  inputs.now_sec = now_sec;
  inputs.uav_state.valid = true;
  inputs.uav_state.stamp_sec = now_sec;
  inputs.uav_state.position_enu = Eigen::Vector3d(0.0, 0.0, 0.0);
  inputs.uav_state.yaw_enu = 0.0;
  inputs.uav_state.frame_id = "uav_odom";
  inputs.ugv_odom.valid = true;
  inputs.ugv_odom.stamp_sec = now_sec;
  inputs.ugv_odom.z_enu = 0.0;
  inputs.relative_pose.valid = true;
  inputs.relative_pose.stamp_sec = now_sec;
  inputs.relative_pose.position_body = Eigen::Vector2d::Zero();
  inputs.relative_pose.covariance = Eigen::Matrix2d::Identity() * 0.1;
  inputs.relocalize_requested = false;
  return inputs;
}

RelativeTrackingController::Config MakeConfig()
{
  RelativeTrackingController::Config config;
  config.return_height_m = 1.0;
  config.takeoff_height_tolerance_m = 0.1;
  config.takeoff_stable_s = 0.5;
  config.auto_track_after_takeoff = true;
  config.relative_pose_timeout_s = 0.25;
  config.uav_state_timeout_s = 0.25;
  config.ugv_odom_timeout_s = 0.25;
  config.xy_kp = 0.8;
  config.z_kp = 1.0;
  config.xy_deadband_m = 0.05;
  config.z_deadband_m = 0.05;
  config.max_xy_speed_mps = 0.8;
  config.max_z_speed_mps = 0.5;
  config.max_relative_covariance_m2 = 1.5;
  config.enable_heading_aligned_tracking = false;
  config.heading_control_min_xy_speed_mps = 0.15;
  config.yaw_kp = 1.8;
  config.max_yaw_rate_radps = 1.2;
  config.yaw_deadband_rad = 0.10;
  config.lateral_release_yaw_error_rad = 0.20;
  config.forward_only_yaw_error_rad = 0.55;
  config.stop_translate_yaw_error_rad = 1.05;
  config.disallow_reverse_motion = true;
  return config;
}

TEST(RelativeTrackingControllerTest, StartTransitionsToTrackingAfterStableTakeoff)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.5, inputs, &message));
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::TakeoffPose);

  const auto start_outputs = controller.Update(inputs);
  EXPECT_TRUE(start_outputs.publish_position_target);
  EXPECT_NEAR(start_outputs.position_target_enu.z(), 1.5, 1.0e-9);
  EXPECT_FALSE(start_outputs.publish_velocity_body);

  inputs = MakeInputs(1.0);
  inputs.uav_state.position_enu.z() = 1.46;
  controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::TakeoffPose);

  inputs.now_sec = 1.6;
  inputs.uav_state.stamp_sec = 1.6;
  inputs.ugv_odom.stamp_sec = 1.6;
  inputs.relative_pose.stamp_sec = 1.6;
  controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::Tracking);

  inputs.now_sec = 1.61;
  inputs.uav_state.stamp_sec = 1.61;
  inputs.ugv_odom.stamp_sec = 1.61;
  inputs.relative_pose.stamp_sec = 1.61;
  const auto tracking_outputs = controller.Update(inputs);
  EXPECT_TRUE(tracking_outputs.publish_velocity_body);
}

TEST(RelativeTrackingControllerTest, StartDoesNotRequireRelativePoseDuringTakeoff)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);
  inputs.relative_pose.valid = false;

  std::string message;
  ASSERT_TRUE(controller.Start(1.2, inputs, &message));
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::TakeoffPose);

  const auto outputs = controller.Update(inputs);
  EXPECT_TRUE(outputs.publish_position_target);
  EXPECT_FALSE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.position_target_enu.z(), 1.2, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, StartCanStayInPositionModeAfterTakeoff)
{
  auto config = MakeConfig();
  config.auto_track_after_takeoff = false;
  RelativeTrackingController controller(config);
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.5, inputs, &message));
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::TakeoffPose);

  inputs = MakeInputs(1.0);
  inputs.uav_state.position_enu.z() = 1.46;
  controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::TakeoffPose);

  inputs.now_sec = 1.6;
  inputs.uav_state.stamp_sec = 1.6;
  inputs.ugv_odom.stamp_sec = 1.6;
  inputs.relative_pose.stamp_sec = 1.6;
  controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::PositionManual);

  inputs.now_sec = 1.61;
  inputs.uav_state.stamp_sec = 1.61;
  inputs.ugv_odom.stamp_sec = 1.61;
  inputs.relative_pose.stamp_sec = 1.61;
  const auto outputs = controller.Update(inputs);
  EXPECT_TRUE(outputs.publish_position_target);
  EXPECT_FALSE(outputs.publish_velocity_body);
}

TEST(RelativeTrackingControllerTest, PositionModePublishesCurrentPoseAndStopsVelocityOutput)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  inputs.uav_state.position_enu = Eigen::Vector3d(1.0, -2.0, 1.2);
  inputs.uav_state.yaw_enu = 0.3;
  ASSERT_TRUE(controller.EnterPositionMode(inputs, &message));

  const auto outputs = controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::PositionManual);
  EXPECT_TRUE(outputs.publish_position_target);
  EXPECT_FALSE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.position_target_enu.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(outputs.position_target_enu.y(), -2.0, 1.0e-9);
  EXPECT_NEAR(outputs.position_target_enu.z(), 1.2, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_target_enu, 0.3, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, GoAboveUgvFromPositionManualResumesTrackingAtReturnHeight)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(2.0, inputs, &message));
  ASSERT_TRUE(controller.EnterPositionMode(inputs, &message));
  controller.Update(inputs);

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.uav_state.position_enu.z() = 2.0;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.5, -0.25);
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  const auto outputs = controller.Update(inputs);
  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::Tracking);
  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.4, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), -0.2, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.z(), -0.5, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.0, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, RelocalizationForcesDegradedHold)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relocalize_requested = true;
  const auto outputs = controller.Update(inputs);

  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::DegradedHold);
  EXPECT_TRUE(outputs.request_hold);
  EXPECT_FALSE(outputs.publish_velocity_body);
}

TEST(RelativeTrackingControllerTest, StaleRelativePoseForcesDegradedHold)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.5;
  inputs.uav_state.stamp_sec = 0.5;
  inputs.ugv_odom.stamp_sec = 0.5;
  inputs.relative_pose.stamp_sec = 0.0;
  const auto outputs = controller.Update(inputs);

  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::DegradedHold);
  EXPECT_TRUE(outputs.request_hold);
  EXPECT_FALSE(outputs.publish_velocity_body);
}

TEST(RelativeTrackingControllerTest, LargeRelativeCovarianceForcesDegradedHold)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.covariance = Eigen::Matrix2d::Identity() * 2.0;
  const auto outputs = controller.Update(inputs);

  EXPECT_EQ(controller.mode(), RelativeTrackingController::Mode::DegradedHold);
  EXPECT_TRUE(outputs.request_hold);
  EXPECT_FALSE(outputs.publish_velocity_body);
}

TEST(RelativeTrackingControllerTest, ControlLawAppliesDeadbandAndClamp)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.uav_state.position_enu.z() = -1.0;
  inputs.relative_pose.position_body = Eigen::Vector2d(2.0, 0.0);
  auto outputs = controller.Update(inputs);
  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.8, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.z(), 0.5, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.0, 1.0e-9);

  inputs.now_sec = 0.2;
  inputs.uav_state.stamp_sec = 0.2;
  inputs.ugv_odom.stamp_sec = 0.2;
  inputs.relative_pose.stamp_sec = 0.2;
  inputs.uav_state.position_enu.z() = 1.02;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.01, -0.02);
  outputs = controller.Update(inputs);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.z(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.0, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, HeadingAlignedTrackingKeepsLegacyBehaviorWhenDisabled)
{
  RelativeTrackingController controller(MakeConfig());
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.5, 0.25);
  const auto outputs = controller.Update(inputs);

  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.4, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.2, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.0, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, HeadingAlignedTrackingIgnoresSmallNominalXySpeed)
{
  auto config = MakeConfig();
  config.enable_heading_aligned_tracking = true;
  RelativeTrackingController controller(config);
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.10, 0.05);
  const auto outputs = controller.Update(inputs);

  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.08, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.04, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.0, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, HeadingAlignedTrackingScalesLateralVelocityFirst)
{
  auto config = MakeConfig();
  config.enable_heading_aligned_tracking = true;
  RelativeTrackingController controller(config);
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.5, 0.15466812480481165);
  const auto outputs = controller.Update(inputs);

  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.4, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.08838178560274951, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 0.54, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, HeadingAlignedTrackingSuppressesSidewaysFlightBeforeForwardMotion)
{
  auto config = MakeConfig();
  config.enable_heading_aligned_tracking = true;
  RelativeTrackingController controller(config);
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.position_body = Eigen::Vector2d(0.375, 0.3158581426736547);
  const auto outputs = controller.Update(inputs);

  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.21, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 1.2, 1.0e-9);
}

TEST(RelativeTrackingControllerTest, HeadingAlignedTrackingTurnsInPlaceWhenTargetFallsBehind)
{
  auto config = MakeConfig();
  config.enable_heading_aligned_tracking = true;
  RelativeTrackingController controller(config);
  auto inputs = MakeInputs(0.0);

  std::string message;
  ASSERT_TRUE(controller.Start(1.0, inputs, &message));
  ASSERT_TRUE(controller.GoAboveUgv(inputs, &message));

  inputs.now_sec = 0.1;
  inputs.uav_state.stamp_sec = 0.1;
  inputs.ugv_odom.stamp_sec = 0.1;
  inputs.relative_pose.stamp_sec = 0.1;
  inputs.relative_pose.position_body = Eigen::Vector2d(-0.5, 0.0);
  const auto outputs = controller.Update(inputs);

  EXPECT_TRUE(outputs.publish_velocity_body);
  EXPECT_NEAR(outputs.velocity_body_mps.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.velocity_body_mps.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(outputs.yaw_rate_radps, 1.2, 1.0e-9);
}

}  // namespace
}  // namespace relative_position_fusion
