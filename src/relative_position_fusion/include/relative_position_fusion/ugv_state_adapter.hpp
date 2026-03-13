#pragma once

#include "relative_position_fusion/agent_planar_state.hpp"
#include "relative_position_fusion/time_alignment_buffer.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/node.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>
#include <string>

namespace relative_position_fusion
{

class UgvStateAdapter
{
public:
  struct Config
  {
    std::string global_frame{"global"};
    std::string amcl_pose_topic{"/amcl_pose"};
    std::string filtered_odom_topic{"/ugv/odometry/filtered"};
    std::string odom_topic{"/ugv/odom"};
    double pose_timeout_sec{0.25};
    double velocity_timeout_sec{0.15};
    double buffer_length_sec{2.0};
    double min_pose_diff_velocity_dt_sec{0.03};
    double max_pose_diff_velocity_dt_sec{0.25};
    double covariance_floor_m2{1e-4};
    double covariance_ceiling_m2{25.0};
    double covariance_nan_fallback_m2{1.0};
    double covariance_age_inflation_m2_per_s{0.25};
    double tf_lookup_timeout_sec{0.03};
    bool filtered_twist_in_child_frame{true};
    bool odom_twist_in_child_frame{true};
    bool assume_odom_is_global{false};
  };

  UgvStateAdapter(
    rclcpp::Node * node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const Config & config);

  void HandleAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);

  void HandleFilteredOdometry(const nav_msgs::msg::Odometry & msg);

  void HandleBaseOdometry(const nav_msgs::msg::Odometry & msg);

  AgentPlanarState BuildState(const rclcpp::Time & now) const;

  std::optional<AgentPlanarState> BuildPoseStateNear(
    const rclcpp::Time & target_stamp,
    const rclcpp::Time & now,
    double max_abs_dt_sec,
    double * abs_dt_sec = nullptr) const;

private:
  rclcpp::Time ResolveStamp(const builtin_interfaces::msg::Time & stamp_msg) const;

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Config config_;
  mutable std::mutex mutex_;
  TimeAlignmentBuffer amcl_buffer_;
  TimeAlignmentBuffer filtered_buffer_;
  TimeAlignmentBuffer odom_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace relative_position_fusion
