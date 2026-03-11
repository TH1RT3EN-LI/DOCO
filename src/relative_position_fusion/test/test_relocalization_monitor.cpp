#include "relative_position_fusion/relocalization_monitor.hpp"

#include <gtest/gtest.h>

namespace relative_position_fusion
{

TEST(RelocalizationMonitorTest, RequestsWhileUninitialized)
{
  RelocalizationMonitor monitor;
  const RelocalizationDecision decision = monitor.Update(
    RelocalizationInput{false, false, 0.0, 0});
  EXPECT_TRUE(decision.requested);
  EXPECT_EQ(decision.reason, "uninitialized");
}

TEST(RelocalizationMonitorTest, UsesEnterHoldCycles)
{
  RelocalizationMonitor monitor(RelocalizationMonitorConfig{1.0, 0.7, 3, 2, 5});
  EXPECT_TRUE(monitor.Update(RelocalizationInput{true, true, 0.5, 0}).requested);
  EXPECT_FALSE(monitor.Update(RelocalizationInput{true, true, 0.5, 0}).requested);
  RelocalizationDecision decision;
  for (int index = 0; index < 2; ++index) {
    decision = monitor.Update(RelocalizationInput{true, true, 1.2, 0});
    EXPECT_FALSE(decision.requested);
  }
  decision = monitor.Update(RelocalizationInput{true, true, 1.2, 0});
  EXPECT_TRUE(decision.requested);
}

TEST(RelocalizationMonitorTest, UsesExitHoldCycles)
{
  RelocalizationMonitor monitor(RelocalizationMonitorConfig{1.0, 0.7, 1, 2, 5});
  EXPECT_TRUE(monitor.Update(RelocalizationInput{true, true, 1.2, 0}).requested);
  EXPECT_TRUE(monitor.Update(RelocalizationInput{true, true, 0.5, 0}).requested);
  EXPECT_FALSE(monitor.Update(RelocalizationInput{true, true, 0.5, 0}).requested);
}

TEST(RelocalizationMonitorTest, GateRejectsCanForceRequest)
{
  RelocalizationMonitor monitor(RelocalizationMonitorConfig{1.0, 0.7, 3, 2, 2});
  const RelocalizationDecision decision = monitor.Update(
    RelocalizationInput{true, true, 0.2, 2});
  EXPECT_TRUE(decision.requested);
  EXPECT_EQ(decision.reason, "gate_rejects");
}

}  // namespace relative_position_fusion
