#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace relative_position_fusion
{

inline bool IsFinite(double value)
{
  return std::isfinite(value);
}

inline bool IsFiniteVector(const Eigen::Vector2d & value)
{
  return IsFinite(value.x()) && IsFinite(value.y());
}

inline bool IsFiniteMatrix(const Eigen::Matrix2d & value)
{
  return IsFinite(value(0, 0)) && IsFinite(value(0, 1)) &&
         IsFinite(value(1, 0)) && IsFinite(value(1, 1));
}

inline double NormalizeAngle(double angle_rad)
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

inline double QuaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
  const double cosy_cosp =
    1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z));
  return NormalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}

inline geometry_msgs::msg::Quaternion YawToQuaternion(double yaw_rad)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(0.5 * yaw_rad);
  quaternion.w = std::cos(0.5 * yaw_rad);
  return quaternion;
}

inline Eigen::Matrix2d RotationMatrix2d(double yaw_rad)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Eigen::Matrix2d rotation;
  rotation << c, -s,
              s, c;
  return rotation;
}

inline Eigen::Vector2d RotateVector2d(const Eigen::Vector2d & value, double yaw_rad)
{
  return RotationMatrix2d(yaw_rad) * value;
}

inline Eigen::Matrix2d RotatePlanarCovariance(
  const Eigen::Matrix2d & covariance,
  double yaw_rad)
{
  const Eigen::Matrix2d rotation = RotationMatrix2d(yaw_rad);
  return rotation * covariance * rotation.transpose();
}

inline Eigen::Matrix2d ExtractPlanarCovariance(const std::array<double, 36> & covariance)
{
  Eigen::Matrix2d planar = Eigen::Matrix2d::Zero();
  planar(0, 0) = covariance[0];
  planar(0, 1) = covariance[1];
  planar(1, 0) = covariance[6];
  planar(1, 1) = covariance[7];
  return planar;
}

inline Eigen::Matrix2d ExtractPlanarCovariance(const double * covariance)
{
  Eigen::Matrix2d planar = Eigen::Matrix2d::Zero();
  planar(0, 0) = covariance[0];
  planar(0, 1) = covariance[1];
  planar(1, 0) = covariance[6];
  planar(1, 1) = covariance[7];
  return planar;
}

inline Eigen::Matrix2d Symmetrize(const Eigen::Matrix2d & value)
{
  return 0.5 * (value + value.transpose());
}

inline Eigen::Matrix2d SanitizePlanarCovariance(
  const Eigen::Matrix2d & covariance,
  double floor_m2,
  double ceiling_m2,
  double fallback_m2,
  double age_sec,
  double age_inflation_m2_per_s)
{
  Eigen::Matrix2d result = covariance;
  if (!IsFiniteMatrix(result)) {
    result = fallback_m2 * Eigen::Matrix2d::Identity();
  }

  result = Symmetrize(result);
  result(0, 0) = std::clamp(result(0, 0), floor_m2, ceiling_m2);
  result(1, 1) = std::clamp(result(1, 1), floor_m2, ceiling_m2);

  const double max_cross = std::sqrt(result(0, 0) * result(1, 1));
  result(0, 1) = std::clamp(result(0, 1), -max_cross, max_cross);
  result(1, 0) = std::clamp(result(1, 0), -max_cross, max_cross);

  if (std::isfinite(age_sec) && age_sec > 0.0 && age_inflation_m2_per_s > 0.0) {
    const double inflation = age_sec * age_inflation_m2_per_s;
    result(0, 0) = std::clamp(result(0, 0) + inflation, floor_m2, ceiling_m2);
    result(1, 1) = std::clamp(result(1, 1) + inflation, floor_m2, ceiling_m2);
  }

  return Symmetrize(result);
}

inline double MaxEigenvalue(const Eigen::Matrix2d & covariance)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(Symmetrize(covariance));
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, solver.eigenvalues().maxCoeff());
}

inline double ComputeRhoMeters(const Eigen::Matrix2d & covariance, double gamma)
{
  const double lambda_max = MaxEigenvalue(covariance);
  if (!std::isfinite(lambda_max)) {
    return std::numeric_limits<double>::infinity();
  }
  return gamma * std::sqrt(std::max(0.0, lambda_max));
}

}  // namespace relative_position_fusion
