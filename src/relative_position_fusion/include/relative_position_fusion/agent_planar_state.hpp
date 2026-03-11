#pragma once

#include <Eigen/Core>

#include <rclcpp/time.hpp>

#include <limits>
#include <string>

namespace relative_position_fusion
{

struct AgentPlanarState
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time velocity_stamp{0, 0, RCL_ROS_TIME};

  Eigen::Vector2d p_global{Eigen::Vector2d::Zero()};
  Eigen::Vector2d v_global{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d sigma_p{Eigen::Matrix2d::Identity()};

  double yaw_global{0.0};
  double pose_age_sec{std::numeric_limits<double>::infinity()};
  double velocity_age_sec{std::numeric_limits<double>::infinity()};

  bool pose_valid{false};
  bool velocity_valid{false};
  bool covariance_valid{false};
  bool yaw_valid{false};

  bool pose_from_fallback{false};
  bool velocity_from_fallback{false};

  std::string pose_source;
  std::string velocity_source;
};

}  // namespace relative_position_fusion
