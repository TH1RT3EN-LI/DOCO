#include "relative_position_fusion/uav_state_adapter.hpp"

#include "relative_position_fusion/covariance_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <utility>

namespace relative_position_fusion
{

namespace
{

geometry_msgs::msg::TransformStamped LookupTransform(
  tf2_ros::Buffer & tf_buffer,
  const std::string & target_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  double timeout_sec)
{
  if (source_frame.empty() || source_frame == target_frame) {
    geometry_msgs::msg::TransformStamped identity;
    identity.header.stamp = stamp;
    identity.header.frame_id = target_frame;
    identity.child_frame_id = source_frame.empty() ? target_frame : source_frame;
    identity.transform.rotation.w = 1.0;
    return identity;
  }

  const auto timeout = rclcpp::Duration::from_seconds(std::max(0.0, timeout_sec));
  try {
    return tf_buffer.lookupTransform(target_frame, source_frame, stamp, timeout);
  } catch (const tf2::TransformException &) {
    return tf_buffer.lookupTransform(
      target_frame, source_frame, rclcpp::Time(0, 0, stamp.get_clock_type()), timeout);
  }
}

AgentPlanarState BuildPoseStateFromSample(
  const PoseSample2D & pose_sample,
  const UavStateAdapter::Config & config,
  const rclcpp::Time & now)
{
  AgentPlanarState state;
  state.pose_valid = true;
  state.pose_stamp = pose_sample.stamp;
  state.p_global = pose_sample.position_global;
  state.yaw_global = pose_sample.yaw_global;
  state.yaw_valid = pose_sample.yaw_valid;
  state.pose_age_sec = (now - pose_sample.stamp).seconds();
  state.pose_source = pose_sample.source;
  state.sigma_p = SanitizePlanarCovariance(
    pose_sample.covariance_global,
    config.covariance_floor_m2,
    config.covariance_ceiling_m2,
    config.covariance_nan_fallback_m2,
    state.pose_age_sec,
    config.covariance_age_inflation_m2_per_s);
  state.covariance_valid = true;
  state.stamp = state.pose_stamp;
  return state;
}

}  // namespace

UavStateAdapter::UavStateAdapter(
  rclcpp::Node * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const Config & config)
: node_(node),
  tf_buffer_(std::move(tf_buffer)),
  config_(config),
  buffer_(config.buffer_length_sec)
{
  const auto sensor_qos = rclcpp::SensorDataQoS();
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic,
    sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      HandleOdometry(*msg);
    });
}

void UavStateAdapter::HandleOdometry(const nav_msgs::msg::Odometry & msg)
{
  const rclcpp::Time stamp = ResolveStamp(msg.header.stamp);
  const std::string pose_frame = msg.header.frame_id.empty() ? config_.global_frame : msg.header.frame_id;

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = msg.header;
  pose_in.header.stamp = stamp;
  pose_in.header.frame_id = pose_frame;
  pose_in.pose = msg.pose.pose;

  PoseSample2D pose_sample;
  pose_sample.stamp = stamp;
  pose_sample.source = config_.odom_topic;

  try {
    const auto transform = LookupTransform(
      *tf_buffer_, config_.global_frame, pose_frame, stamp, config_.tf_lookup_timeout_sec);
    geometry_msgs::msg::PoseStamped pose_global;
    tf2::doTransform(pose_in, pose_global, transform);
    pose_sample.position_global = Eigen::Vector2d(
      pose_global.pose.position.x,
      pose_global.pose.position.y);
    pose_sample.yaw_global = QuaternionToYaw(pose_global.pose.orientation);
    pose_sample.yaw_valid = std::isfinite(pose_sample.yaw_global);

    Eigen::Matrix2d covariance = ExtractPlanarCovariance(msg.pose.covariance);
    covariance = RotatePlanarCovariance(covariance, QuaternionToYaw(transform.transform.rotation));
    pose_sample.covariance_global = covariance;
    pose_sample.covariance_valid = IsFiniteMatrix(covariance);
    buffer_.PushPose(pose_sample);

    Eigen::Vector2d velocity(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
    if (IsFiniteVector(velocity)) {
      VelocitySample2D velocity_sample;
      velocity_sample.stamp = stamp;
      velocity_sample.source = config_.odom_topic;
      if (config_.twist_in_child_frame) {
        if (!pose_sample.yaw_valid) {
          return;
        }
        velocity_sample.velocity_global = RotateVector2d(velocity, pose_sample.yaw_global);
      } else {
        velocity_sample.velocity_global = RotateVector2d(
          velocity, QuaternionToYaw(transform.transform.rotation));
      }
      buffer_.PushVelocity(velocity_sample);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "uav state adapter failed to transform %s -> %s: %s",
      pose_frame.c_str(), config_.global_frame.c_str(), ex.what());
  }
}

AgentPlanarState UavStateAdapter::BuildState(const rclcpp::Time & now) const
{
  std::scoped_lock lock(mutex_);
  AgentPlanarState state;

  const auto pose_sample = buffer_.LatestPose(now, config_.pose_timeout_sec);
  if (pose_sample.has_value()) {
    state = BuildPoseStateFromSample(*pose_sample, config_, now);
  }

  auto velocity_sample = buffer_.LatestVelocity(now, config_.velocity_timeout_sec);
  if (!velocity_sample.has_value()) {
    velocity_sample = buffer_.EstimateVelocityFromPoseHistory(
      now,
      config_.pose_timeout_sec,
      config_.min_pose_diff_velocity_dt_sec,
      config_.max_pose_diff_velocity_dt_sec,
      config_.odom_topic + ":pose_diff");
    if (velocity_sample.has_value()) {
      state.velocity_from_fallback = true;
    }
  }

  if (velocity_sample.has_value()) {
    state.velocity_valid = true;
    state.velocity_stamp = velocity_sample->stamp;
    state.v_global = velocity_sample->velocity_global;
    state.velocity_age_sec = (now - velocity_sample->stamp).seconds();
    state.velocity_source = velocity_sample->source;
  }

  if (state.pose_valid || state.velocity_valid) {
    state.stamp = state.pose_valid ? state.pose_stamp : state.velocity_stamp;
    if (state.pose_valid && state.velocity_valid) {
      state.stamp = state.pose_stamp > state.velocity_stamp ? state.pose_stamp : state.velocity_stamp;
    }
  }

  return state;
}

std::optional<AgentPlanarState> UavStateAdapter::BuildPoseStateNear(
  const rclcpp::Time & target_stamp,
  const rclcpp::Time & now,
  double max_abs_dt_sec,
  double * abs_dt_sec) const
{
  std::scoped_lock lock(mutex_);

  const auto pose_match = buffer_.NearestPose(
    target_stamp,
    now,
    config_.pose_timeout_sec,
    max_abs_dt_sec);
  if (!pose_match.has_value()) {
    return std::nullopt;
  }

  if (abs_dt_sec != nullptr) {
    *abs_dt_sec = pose_match->abs_dt_sec;
  }

  return BuildPoseStateFromSample(pose_match->sample, config_, now);
}

rclcpp::Time UavStateAdapter::ResolveStamp(const builtin_interfaces::msg::Time & stamp_msg) const
{
  if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
    return node_->now();
  }
  return rclcpp::Time(stamp_msg, RCL_ROS_TIME);
}

}  // namespace relative_position_fusion
