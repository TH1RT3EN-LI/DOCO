#include <gtest/gtest.h>

#include "ugv_sim_tools/odom_to_tf_logic.hpp"

TEST(OdomToTfLogic, PublishesOnFirstAndNewerStamps)
{
  EXPECT_TRUE(ugv_sim_tools::shouldPublishOdomTf(std::nullopt, 10));
  EXPECT_TRUE(ugv_sim_tools::shouldPublishOdomTf(10, 11));
}

TEST(OdomToTfLogic, SkipsDuplicateAndOlderStamps)
{
  EXPECT_FALSE(ugv_sim_tools::shouldPublishOdomTf(10, 10));
  EXPECT_FALSE(ugv_sim_tools::shouldPublishOdomTf(10, 9));
}
