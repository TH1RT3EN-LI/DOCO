#include <gtest/gtest.h>

#include "ugv_sim_tools/initial_pose_logic.hpp"

TEST(InitialPoseLogic, ClampPublishCount)
{
  EXPECT_EQ(ugv_sim_tools::clampPublishCount(0), 1);
  EXPECT_EQ(ugv_sim_tools::clampPublishCount(4), 4);
}

TEST(InitialPoseLogic, YawQuaternion)
{
  const auto quaternion = ugv_sim_tools::yawToQuaternion(0.0);
  EXPECT_DOUBLE_EQ(quaternion.x, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.y, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.z, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.w, 1.0);
}

TEST(InitialPoseLogic, BurstCompletesWithoutAmclWait)
{
  ugv_sim_tools::InitialPoseConfig config;
  config.publish_count = 2;
  config.require_subscriber = false;
  config.wait_for_amcl_pose = false;

  ugv_sim_tools::InitialPoseStateMachine state_machine(config);
  auto result = state_machine.tick(true, 0.1);
  EXPECT_TRUE(result.should_publish);
  EXPECT_FALSE(result.done_now);

  result = state_machine.tick(true, 0.2);
  EXPECT_TRUE(result.should_publish);
  EXPECT_TRUE(result.done_now);
  EXPECT_EQ(result.phase, ugv_sim_tools::InitialPosePhase::kCompleted);
}

TEST(InitialPoseLogic, WaitsForSubscriberAndAmcl)
{
  ugv_sim_tools::InitialPoseConfig config;
  config.publish_count = 1;
  config.require_subscriber = true;
  config.wait_for_amcl_pose = true;

  ugv_sim_tools::InitialPoseStateMachine state_machine(config);
  auto result = state_machine.tick(false, 0.1);
  EXPECT_TRUE(result.waiting_for_subscriber);

  result = state_machine.tick(true, 0.2);
  EXPECT_TRUE(result.should_publish);
  EXPECT_TRUE(result.initial_burst_completed_now);
  EXPECT_EQ(result.phase, ugv_sim_tools::InitialPosePhase::kWaitingForAmclPose);

  EXPECT_TRUE(state_machine.onAmclPoseReceived());
  EXPECT_TRUE(state_machine.isDone());
}
