#pragma once

#include <Eigen/Core>

#include <limits>

namespace relative_position_fusion
{

struct GatingResult
{
  bool accepted{false};
  double mahalanobis_distance_sq{std::numeric_limits<double>::infinity()};
  Eigen::Vector2d innovation{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d innovation_covariance{Eigen::Matrix2d::Zero()};
};

class RelativePositionFilter
{
public:
  RelativePositionFilter() = default;

  void Reset();

  bool initialized() const;

  void Initialize(const Eigen::Vector2d & measurement, const Eigen::Matrix2d & measurement_covariance);

  void Predict(double dt_sec, const Eigen::Vector2d & relative_velocity, double q_rate_m2_per_s);

  GatingResult Gate(
    const Eigen::Vector2d & measurement,
    const Eigen::Matrix2d & measurement_covariance,
    double chi2_threshold) const;

  void Update(const Eigen::Vector2d & measurement, const Eigen::Matrix2d & measurement_covariance);

  const Eigen::Vector2d & state() const;

  const Eigen::Matrix2d & covariance() const;

private:
  static Eigen::Matrix2d MakePositiveSemiDefinite(const Eigen::Matrix2d & matrix);

  bool initialized_{false};
  Eigen::Vector2d state_{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance_{Eigen::Matrix2d::Identity()};
};

}  // namespace relative_position_fusion
