#include <gtest/gtest.h>

#include "uav_visual_landing/visual_landing_logic.hpp"

namespace uav_visual_landing
{
namespace
{

TEST(VisualLandingLogic, RangeDecisionRejectsInconsistentRange)
{
  RangeConfig config;
  config.timeout_s = 0.15;
  config.min_m = 0.05f;
  config.max_m = 5.0f;
  config.max_diff_m = 0.35f;

  const auto decision = evaluateRangeDecision(1.5f, true, 0.8f, 0.01, config);
  EXPECT_FALSE(decision.valid);
  EXPECT_FLOAT_EQ(decision.effective_height_m, 1.5f);
}

TEST(VisualLandingLogic, AlignVerticalCommandIsAlwaysZero)
{
  EXPECT_FLOAT_EQ(computeClosedLoopVelocity(1.0f, 1.0f, 0.0f, 1.2f, 0.2f, 0.3f), 0.0f);
  EXPECT_GT(computeClosedLoopVelocity(1.5f, 1.0f, 0.0f, 1.2f, 0.2f, 0.3f), 0.0f);
  EXPECT_LT(computeClosedLoopVelocity(0.5f, 1.0f, 0.0f, 1.2f, 0.2f, 0.3f), 0.0f);
}

TEST(VisualLandingLogic, TargetLossFallsBackToHoldWait)
{
  EXPECT_EQ(nextPhaseOnTargetLoss(ControllerPhase::TrackAlign), ControllerPhase::HoldWait);
  EXPECT_EQ(nextPhaseOnTargetLoss(ControllerPhase::DescendTrack), ControllerPhase::HoldWait);
  EXPECT_EQ(nextPhaseOnTargetLoss(ControllerPhase::Terminal), ControllerPhase::HoldWait);
}

TEST(VisualLandingLogic, AlignmentWindowUsesEnterAndExitThresholds)
{
  AlignmentConfig config;
  config.enter_error = 0.03f;
  config.exit_error = 0.02f;
  config.enter_yaw = 0.12f;
  config.exit_yaw = 0.09f;

  EXPECT_TRUE(isAligned(0.01f, 0.01f, 0.05f, false, config));
  EXPECT_FALSE(isAligned(0.03f, 0.03f, 0.05f, true, config));
}

TEST(VisualLandingLogic, BodyRateLimitConstrainsStepChange)
{
  CommandRateLimitConfig config;
  config.max_acc_xy_mps2 = 1.0f;
  config.max_acc_z_mps2 = 0.5f;
  config.max_acc_yaw_radps2 = 2.0f;

  float vx = 1.0f;
  float vy = 0.0f;
  float vz = -1.0f;
  float yaw_rate = 1.0f;
  applyBodyRateLimit(vx, vy, vz, yaw_rate, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, config);

  EXPECT_NEAR(vx, 0.1f, 1.0e-5f);
  EXPECT_NEAR(vy, 0.0f, 1.0e-5f);
  EXPECT_NEAR(vz, -0.05f, 1.0e-5f);
  EXPECT_NEAR(yaw_rate, 0.2f, 1.0e-5f);
}

}  // namespace
}  // namespace uav_visual_landing
