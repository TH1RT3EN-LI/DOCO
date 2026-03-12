#include <gtest/gtest.h>

#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{
namespace
{

TEST(UavControlLogic, VelocityBodyOverridesAutoAndFallsBackToHold)
{
  UavControlModeTracker tracker;
  EXPECT_EQ(tracker.mode(), UavControlMode::Auto);
  tracker.requestVelocityBody();
  EXPECT_EQ(tracker.mode(), UavControlMode::VelocityBody);
  tracker.onVelocityCommandTimeout();
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
}

TEST(UavControlLogic, AutoEnableRequiresTrajectoryAndNoLanding)
{
  UavControlModeTracker tracker;
  tracker.requestHold();
  EXPECT_FALSE(tracker.requestAutoEnable(false));
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
  EXPECT_TRUE(tracker.requestAutoEnable(true));
  EXPECT_EQ(tracker.mode(), UavControlMode::Auto);
  tracker.requestLanding();
  EXPECT_FALSE(tracker.requestAutoEnable(true));
  EXPECT_EQ(tracker.mode(), UavControlMode::Landing);
}

TEST(UavControlLogic, LandingCompletesToHold)
{
  UavControlModeTracker tracker;
  tracker.requestLanding();
  EXPECT_EQ(tracker.mode(), UavControlMode::Landing);
  tracker.onLandingComplete();
  EXPECT_EQ(tracker.mode(), UavControlMode::Hold);
}

}  // namespace
}  // namespace uav_bridge
