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

}  // namespace
}  // namespace uav_bridge
