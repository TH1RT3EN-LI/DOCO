#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{
namespace
{

TEST(UavControlLogic, StartsInHold)
{
  UavControlModeTracker tracker;
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
}

TEST(UavControlLogic, VelocityBodyFallsBackToHoldOnTimeout)
{
  UavControlModeTracker tracker;
  tracker.requestVelocityBody();
  EXPECT_EQ(tracker.mode(), UavControlMode::VelocityBody);
  tracker.onVelocityCommandTimeout();
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
}

TEST(UavControlLogic, LandingCompletesToHold)
{
  UavControlModeTracker tracker;
  tracker.requestLanding();
  EXPECT_EQ(tracker.mode(), UavControlMode::Landing);
  tracker.onLandingComplete();
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
}

TEST(UavControlLogic, LandingRejectsVelocityOverride)
{
  UavControlModeTracker tracker;
  tracker.requestLanding();
  tracker.requestVelocityBody();
  EXPECT_EQ(tracker.mode(), UavControlMode::Landing);
}

TEST(UavControlLogic, PositionModeCanBeRequestedAndPreserved)
{
  UavControlModeTracker tracker;
  tracker.requestPosition();
  EXPECT_EQ(tracker.mode(), UavControlMode::Position);
  tracker.requestPosition();
  EXPECT_EQ(tracker.mode(), UavControlMode::Position);
}

TEST(UavControlLogic, LandingRejectsPositionOverride)
{
  UavControlModeTracker tracker;
  tracker.requestLanding();
  tracker.requestPosition();
  EXPECT_EQ(tracker.mode(), UavControlMode::Landing);
}

TEST(UavControlLogic, VelocityEstimateRequiresValidFlagsAndFiniteComponents)
{
  EXPECT_TRUE(hasValidVelocityEstimate(true, true, {0.1f, -0.2f, 0.3f}));
  EXPECT_FALSE(hasValidVelocityEstimate(false, true, {0.1f, -0.2f, 0.3f}));
  EXPECT_FALSE(hasValidVelocityEstimate(true, false, {0.1f, -0.2f, 0.3f}));
  EXPECT_FALSE(
    hasValidVelocityEstimate(
      true, true,
      {0.1f, std::numeric_limits<float>::quiet_NaN(), 0.3f}));
}

TEST(UavControlLogic, VelocityBodyCommandRequiresFiniteValues)
{
  EXPECT_TRUE(isFiniteVelocityBodyCommand(0.1f, -0.2f, 0.3f, 0.4f));
  EXPECT_FALSE(
    isFiniteVelocityBodyCommand(
      std::numeric_limits<float>::infinity(), -0.2f, 0.3f, 0.4f));
  EXPECT_FALSE(
    isFiniteVelocityBodyCommand(
      0.1f, -0.2f, 0.3f, std::numeric_limits<float>::quiet_NaN()));
}

TEST(UavControlLogic, VelocitySetpointRequiresFiniteValues)
{
  EXPECT_TRUE(isFiniteVelocitySetpoint({0.1f, -0.2f, 0.3f}, 0.4f));
  EXPECT_FALSE(
    isFiniteVelocitySetpoint(
      {0.1f, -0.2f, std::numeric_limits<float>::quiet_NaN()}, 0.4f));
  EXPECT_FALSE(
    isFiniteVelocitySetpoint(
      {0.1f, -0.2f, 0.3f}, std::numeric_limits<float>::infinity()));
}

}  // namespace
}  // namespace uav_bridge
