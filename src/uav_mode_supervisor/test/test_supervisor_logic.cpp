#include <gtest/gtest.h>

#include <limits>

#include "uav_mode_supervisor/supervisor_logic.hpp"

namespace uav_mode_supervisor
{

namespace
{

SupervisorLogic::FusionStatus ReadyFusion()
{
  SupervisorLogic::FusionStatus status;
  status.diagnostics_seen = true;
  status.initialized = true;
  status.relocalize_requested = false;
  status.reason = "ok";
  return status;
}

SupervisorLogic::VisualLandingStatus VisualStatus(
  bool active,
  const std::string & phase,
  bool target_detected = true)
{
  SupervisorLogic::VisualLandingStatus status;
  status.state_seen = true;
  status.active = active;
  status.phase = phase;
  status.target_detected = target_detected;
  return status;
}

}  // namespace

TEST(SupervisorLogicTest, RejectsTrackingWhenFusionNotReady)
{
  SupervisorLogic logic;
  const auto plan = logic.BuildCommandPlan("start_tracking", 1.5f);
  EXPECT_FALSE(plan.accepted);
  EXPECT_EQ(plan.message, "fusion is not ready for relative tracking");
}

TEST(SupervisorLogicTest, VisualLandingPreemptsTracking)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_tracking = logic.BuildCommandPlan("start_tracking", 1.5f);
  ASSERT_TRUE(start_tracking.accepted);
  logic.ApplySuccessfulPlan(start_tracking);

  const auto start_visual = logic.BuildCommandPlan("start_visual_landing", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(start_visual.accepted);
  EXPECT_TRUE(start_visual.stop_tracking);
  EXPECT_TRUE(start_visual.start_visual_landing);
  EXPECT_EQ(start_visual.owner_after, SupervisorLogic::Owner::VisualLanding);
}

TEST(SupervisorLogicTest, TrackingPreemptsVisualLandingBeforeCommit)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_visual = logic.BuildCommandPlan("start_visual_landing", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(start_visual.accepted);
  logic.ApplySuccessfulPlan(start_visual);
  logic.UpdateVisualLandingStatus(VisualStatus(true, "TRACK_ALIGN"));

  const auto start_tracking = logic.BuildCommandPlan("start_tracking", 2.0f);
  ASSERT_TRUE(start_tracking.accepted);
  EXPECT_TRUE(start_tracking.stop_visual_landing);
  EXPECT_TRUE(start_tracking.start_tracking);
  EXPECT_EQ(start_tracking.owner_after, SupervisorLogic::Owner::RelativeTracking);
}

TEST(SupervisorLogicTest, PositionModePreemptsTracking)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_tracking = logic.BuildCommandPlan("start_tracking", 1.5f);
  ASSERT_TRUE(start_tracking.accepted);
  logic.ApplySuccessfulPlan(start_tracking);

  const auto position_mode = logic.BuildCommandPlan(
    "position", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(position_mode.accepted);
  EXPECT_TRUE(position_mode.stop_tracking);
  EXPECT_TRUE(position_mode.request_position_mode);
  EXPECT_EQ(position_mode.owner_after, SupervisorLogic::Owner::Idle);
}

TEST(SupervisorLogicTest, RejectsTrackingPreemptionWhenVisualLandingCommitted)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_visual = logic.BuildCommandPlan("start_visual_landing", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(start_visual.accepted);
  logic.ApplySuccessfulPlan(start_visual);
  logic.UpdateVisualLandingStatus(VisualStatus(true, "DESCEND_TRACK"));

  const auto start_tracking = logic.BuildCommandPlan("start_tracking", 2.0f);
  EXPECT_FALSE(start_tracking.accepted);
  EXPECT_EQ(start_tracking.message, "visual landing is in commit phase and cannot be preempted");
}

TEST(SupervisorLogicTest, RejectsPositionModePreemptionWhenVisualLandingCommitted)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_visual = logic.BuildCommandPlan(
    "start_visual_landing", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(start_visual.accepted);
  logic.ApplySuccessfulPlan(start_visual);
  logic.UpdateVisualLandingStatus(VisualStatus(true, "DESCEND_TRACK"));

  const auto position_mode = logic.BuildCommandPlan(
    "position_mode", std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(position_mode.accepted);
  EXPECT_EQ(
    position_mode.message,
    "visual landing is in commit phase and cannot be preempted");
}

TEST(SupervisorLogicTest, FusionDegradeTriggersAutomaticHold)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_tracking = logic.BuildCommandPlan("start_tracking", 1.5f);
  ASSERT_TRUE(start_tracking.accepted);
  logic.ApplySuccessfulPlan(start_tracking);

  auto degraded = ReadyFusion();
  degraded.relocalize_requested = true;
  degraded.reason = "rho_high";
  logic.UpdateFusionStatus(degraded);

  const auto plan = logic.BuildAutomaticPlan();
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->kind, SupervisorLogic::PlanKind::AutomaticSafety);
  EXPECT_TRUE(plan->stop_tracking);
  EXPECT_TRUE(plan->request_hold);
  EXPECT_EQ(plan->owner_after, SupervisorLogic::Owner::Hold);
}

TEST(SupervisorLogicTest, VisualLossRecoversToTrackingBeforeCommit)
{
  SupervisorLogic logic;
  logic.UpdateFusionStatus(ReadyFusion());

  auto start_tracking = logic.BuildCommandPlan("start_tracking", 2.0f);
  ASSERT_TRUE(start_tracking.accepted);
  logic.ApplySuccessfulPlan(start_tracking);

  auto start_visual = logic.BuildCommandPlan("start_visual_landing", std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(start_visual.accepted);
  logic.ApplySuccessfulPlan(start_visual);
  logic.UpdateVisualLandingStatus(VisualStatus(true, "TRACK_ALIGN"));
  logic.UpdateVisualLandingStatus(VisualStatus(true, "HOLD_WAIT", false));

  const auto plan = logic.BuildAutomaticPlan();
  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->kind, SupervisorLogic::PlanKind::AutomaticRecovery);
  EXPECT_TRUE(plan->stop_visual_landing);
  EXPECT_TRUE(plan->start_tracking);
  EXPECT_FLOAT_EQ(plan->tracking_height_m, 2.0f);
  EXPECT_EQ(plan->owner_after, SupervisorLogic::Owner::RelativeTracking);
}

}  // namespace uav_mode_supervisor
