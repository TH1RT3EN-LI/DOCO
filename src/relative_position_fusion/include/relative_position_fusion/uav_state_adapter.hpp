#pragma once

#include "relative_position_fusion/agent_planar_state.hpp"
#include "relative_position_fusion/time_alignment_buffer.hpp"

#include <nav_msgs/msg/odometry.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/node.hpp>

#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>
#include <string>

namespace relative_position_fusion
{

class UavStateAdapter
{
public:
  struct Config
  {
    std::string global_frame{"global"};
    std::string odom_topic{"/uav/state/odometry"};
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
    bool twist_in_child_frame{false};
  };

  UavStateAdapter(
    rclcpp::Node * node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const Config & config);

  void HandleOdometry(const nav_msgs::msg::Odometry & msg);

  AgentPlanarState BuildState(const rclcpp::Time & now) const;

private:
  rclcpp::Time ResolveStamp(const builtin_interfaces::msg::Time & stamp_msg) const;

  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  Config config_;
  mutable std::mutex mutex_;
  TimeAlignmentBuffer buffer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace relative_position_fusion
