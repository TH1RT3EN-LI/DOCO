#include "relative_position_fusion/covariance_utils.hpp"
#include "relative_position_fusion/relative_position_filter.hpp"

#include <gtest/gtest.h>

namespace relative_position_fusion
{

TEST(RelativePositionFilterTest, PredictMatchesConstantVelocityModel)
{
  RelativePositionFilter filter;
  filter.Initialize(Eigen::Vector2d(1.0, 2.0), 0.1 * Eigen::Matrix2d::Identity());
  filter.Predict(0.5, Eigen::Vector2d(2.0, -1.0), 0.0);

  EXPECT_NEAR(filter.state().x(), 2.0, 1e-9);
  EXPECT_NEAR(filter.state().y(), 1.5, 1e-9);
}

TEST(RelativePositionFilterTest, CovarianceGrowsWithoutMeasurement)
{
  RelativePositionFilter filter;
  filter.Initialize(Eigen::Vector2d::Zero(), 0.1 * Eigen::Matrix2d::Identity());
  const double trace_before = filter.covariance().trace();
  filter.Predict(2.0, Eigen::Vector2d::Zero(), 0.5);
  EXPECT_GT(filter.covariance().trace(), trace_before);
}

TEST(RelativePositionFilterTest, GateRejectsLargeJump)
{
  RelativePositionFilter filter;
  filter.Initialize(Eigen::Vector2d::Zero(), 0.01 * Eigen::Matrix2d::Identity());

  const GatingResult result = filter.Gate(
    Eigen::Vector2d(5.0, 0.0),
    0.01 * Eigen::Matrix2d::Identity(),
    5.99);

  EXPECT_FALSE(result.accepted);
  EXPECT_GT(result.mahalanobis_distance_sq, 5.99);
}

TEST(RelativePositionFilterTest, UpdatePullsStateTowardMeasurement)
{
  RelativePositionFilter filter;
  filter.Initialize(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());
  filter.Update(Eigen::Vector2d(1.0, 0.0), 0.1 * Eigen::Matrix2d::Identity());

  EXPECT_GT(filter.state().x(), 0.0);
  EXPECT_LT(filter.state().x(), 1.0);
}

TEST(RelativePositionFilterTest, RhoUsesSquareRootOfMaxEigenvalue)
{
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  covariance(0, 0) = 0.25;
  covariance(1, 1) = 0.09;

  const double rho = ComputeRhoMeters(covariance, 2.0);
  EXPECT_NEAR(rho, 1.0, 1e-9);
}

}  // namespace relative_position_fusion
