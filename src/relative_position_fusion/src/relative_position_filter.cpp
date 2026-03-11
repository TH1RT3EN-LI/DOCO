#include "relative_position_fusion/relative_position_filter.hpp"

#include "relative_position_fusion/covariance_utils.hpp"

#include <Eigen/Cholesky>

#include <algorithm>

namespace relative_position_fusion
{

void RelativePositionFilter::Reset()
{
  initialized_ = false;
  state_.setZero();
  covariance_.setIdentity();
}

bool RelativePositionFilter::initialized() const
{
  return initialized_;
}

void RelativePositionFilter::Initialize(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & measurement_covariance)
{
  state_ = measurement;
  covariance_ = MakePositiveSemiDefinite(measurement_covariance);
  initialized_ = true;
}

void RelativePositionFilter::Predict(
  double dt_sec,
  const Eigen::Vector2d & relative_velocity,
  double q_rate_m2_per_s)
{
  if (!initialized_) {
    return;
  }

  const double safe_dt_sec = std::max(0.0, dt_sec);
  const double safe_q_rate = std::max(0.0, q_rate_m2_per_s);
  state_ += safe_dt_sec * relative_velocity;
  covariance_ += safe_q_rate * safe_dt_sec * Eigen::Matrix2d::Identity();
  covariance_ = MakePositiveSemiDefinite(covariance_);
}

GatingResult RelativePositionFilter::Gate(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & measurement_covariance,
  double chi2_threshold) const
{
  GatingResult result;
  if (!initialized_) {
    return result;
  }

  result.innovation = measurement - state_;
  result.innovation_covariance =
    MakePositiveSemiDefinite(covariance_ + measurement_covariance);

  Eigen::LDLT<Eigen::Matrix2d> ldlt(result.innovation_covariance);
  if (ldlt.info() != Eigen::Success) {
    result.accepted = false;
    return result;
  }

  const Eigen::Vector2d solved = ldlt.solve(result.innovation);
  result.mahalanobis_distance_sq = result.innovation.dot(solved);
  result.accepted =
    std::isfinite(result.mahalanobis_distance_sq) &&
    result.mahalanobis_distance_sq <= chi2_threshold;
  return result;
}

void RelativePositionFilter::Update(
  const Eigen::Vector2d & measurement,
  const Eigen::Matrix2d & measurement_covariance)
{
  if (!initialized_) {
    return;
  }

  const Eigen::Matrix2d innovation_covariance =
    MakePositiveSemiDefinite(covariance_ + measurement_covariance);
  Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_covariance);
  if (ldlt.info() != Eigen::Success) {
    return;
  }

  const Eigen::Matrix2d kalman_gain = covariance_ * ldlt.solve(Eigen::Matrix2d::Identity());
  const Eigen::Vector2d innovation = measurement - state_;
  state_ += kalman_gain * innovation;

  const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d joseph =
    (identity - kalman_gain) * covariance_ * (identity - kalman_gain).transpose() +
    kalman_gain * measurement_covariance * kalman_gain.transpose();
  covariance_ = MakePositiveSemiDefinite(joseph);
}

const Eigen::Vector2d & RelativePositionFilter::state() const
{
  return state_;
}

const Eigen::Matrix2d & RelativePositionFilter::covariance() const
{
  return covariance_;
}

Eigen::Matrix2d RelativePositionFilter::MakePositiveSemiDefinite(const Eigen::Matrix2d & matrix)
{
  Eigen::Matrix2d symmetric = Symmetrize(matrix);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix2d::Identity();
  }

  Eigen::Vector2d eigenvalues = solver.eigenvalues();
  eigenvalues = eigenvalues.cwiseMax(1e-9);
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace relative_position_fusion
