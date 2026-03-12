#include <gtest/gtest.h>

#include "uav_visual_landing/visual_landing_logic.hpp"

namespace uav_visual_landing
{
namespace
{

TEST(VisualLandingLogic, HeightDecisionRejectsInconsistentMeasurement)
{
  HeightConfig config;
  config.timeout_s = 0.15;
  config.min_m = 0.05f;
  config.max_m = 5.0f;
  config.max_diff_m = 0.35f;

  const auto decision = evaluateHeightDecision(1.5f, true, 0.8f, 0.01, config);
  EXPECT_FALSE(decision.height_valid);
  EXPECT_FLOAT_EQ(decision.height_m, 1.5f);
  EXPECT_EQ(decision.height_source, HeightSource::Odom);
}

TEST(VisualLandingLogic, HeightDecisionPrefersFreshMeasurement)
{
  HeightConfig config;
  const auto decision = evaluateHeightDecision(1.5f, true, 1.45f, 0.01, config);
  EXPECT_TRUE(decision.height_valid);
  EXPECT_FLOAT_EQ(decision.height_m, 1.45f);
  EXPECT_EQ(decision.height_source, HeightSource::FlowRange);
}

TEST(VisualLandingLogic, HeightDecisionRejectsStaleMeasurement)
{
  HeightConfig config;
  const auto decision = evaluateHeightDecision(1.5f, true, 1.45f, 0.50, config);
  EXPECT_FALSE(decision.height_valid);
  EXPECT_FLOAT_EQ(decision.height_m, 1.5f);
  EXPECT_EQ(decision.height_source, HeightSource::Odom);
}

TEST(VisualLandingLogic, ClosedLoopVelocityHasExpectedSign)
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

TEST(VisualLandingLogic, MetricLateralErrorUsesActiveHeight)
{
  const auto error = computeMetricLateralError(0.10f, -0.20f, 0.30f, -0.40f, 2.0f);
  EXPECT_TRUE(error.valid);
  EXPECT_NEAR(error.x_m, 0.20f, 1.0e-5f);
  EXPECT_NEAR(error.y_m, -0.40f, 1.0e-5f);
  EXPECT_NEAR(error.norm_m, std::sqrt(0.20f * 0.20f + 0.40f * 0.40f), 1.0e-5f);
  EXPECT_NEAR(error.x_rate_mps, 0.60f, 1.0e-5f);
  EXPECT_NEAR(error.y_rate_mps, -0.80f, 1.0e-5f);
}

TEST(VisualLandingLogic, MetricAlignmentUsesHysteresisThresholds)
{
  AlignmentConfig config;
  MetricLateralError aligned_error;
  aligned_error.valid = true;
  aligned_error.norm_m = 0.07f;

  EXPECT_TRUE(isAligned(aligned_error, 0.05f, false, config));

  MetricLateralError hold_error;
  hold_error.valid = true;
  hold_error.norm_m = 0.06f;
  EXPECT_FALSE(isAligned(hold_error, 0.05f, true, config));
  EXPECT_FALSE(isAligned(aligned_error, 0.09f, false, config));
}

TEST(VisualLandingLogic, LimitedRateRejectsInvalidDt)
{
  float rate = 123.0f;
  EXPECT_FALSE(computeLimitedRate(1.0f, 0.0f, 0.001f, 0.005f, 0.20f, 2.0f, rate));
  EXPECT_FALSE(computeLimitedRate(1.0f, 0.0f, 0.30f, 0.005f, 0.20f, 2.0f, rate));
}

TEST(VisualLandingLogic, LimitedRateClampsMagnitude)
{
  float rate = 0.0f;
  EXPECT_TRUE(computeLimitedRate(1.0f, 0.0f, 0.05f, 0.005f, 0.20f, 2.0f, rate));
  EXPECT_NEAR(rate, 2.0f, 1.0e-5f);
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
