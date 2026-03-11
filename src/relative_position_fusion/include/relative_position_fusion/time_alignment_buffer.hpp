#pragma once

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include <deque>
#include <mutex>
#include <optional>
#include <string>

namespace relative_position_fusion
{

struct PoseSample2D
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Vector2d position_global{Eigen::Vector2d::Zero()};
  double yaw_global{0.0};
  bool yaw_valid{false};
  Eigen::Matrix2d covariance_global{Eigen::Matrix2d::Identity()};
  bool covariance_valid{false};
  std::string source;
};

struct VelocitySample2D
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Vector2d velocity_global{Eigen::Vector2d::Zero()};
  std::string source;
};

class TimeAlignmentBuffer
{
public:
  explicit TimeAlignmentBuffer(double buffer_length_sec = 2.0);

  void SetBufferLengthSec(double buffer_length_sec);

  void PushPose(const PoseSample2D & sample);

  void PushVelocity(const VelocitySample2D & sample);

  std::optional<PoseSample2D> LatestPose(const rclcpp::Time & now, double max_age_sec) const;

  std::optional<VelocitySample2D> LatestVelocity(
    const rclcpp::Time & now,
    double max_age_sec) const;

  std::optional<VelocitySample2D> EstimateVelocityFromPoseHistory(
    const rclcpp::Time & now,
    double max_pose_age_sec,
    double min_dt_sec,
    double max_dt_sec,
    const std::string & fallback_source) const;

private:
  template<typename SampleT>
  static void PruneDeque(std::deque<SampleT> * samples, const rclcpp::Time & newest_stamp, double buffer_length_sec);

  double buffer_length_sec_{2.0};
  mutable std::mutex mutex_;
  std::deque<PoseSample2D> pose_samples_;
  std::deque<VelocitySample2D> velocity_samples_;
};

}  // namespace relative_position_fusion
