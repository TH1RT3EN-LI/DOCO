#include "relative_position_fusion/ugv_state_adapter.hpp"

#include "relative_position_fusion/covariance_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>
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

PoseSample2D BuildPoseSample(
  tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame,
  const std::string & source_frame,
  const rclcpp::Time & stamp,
  const geometry_msgs::msg::Pose & pose,
  const Eigen::Matrix2d & covariance,
  bool covariance_valid,
  const std::string & source,
  double timeout_sec)
{
  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header.stamp = stamp;
  pose_in.header.frame_id = source_frame;
  pose_in.pose = pose;

  const auto transform = LookupTransform(tf_buffer, global_frame, source_frame, stamp, timeout_sec);
  geometry_msgs::msg::PoseStamped pose_global;
  tf2::doTransform(pose_in, pose_global, transform);

  PoseSample2D sample;
  sample.stamp = stamp;
  sample.position_global = Eigen::Vector2d(
    pose_global.pose.position.x,
    pose_global.pose.position.y);
  sample.yaw_global = QuaternionToYaw(pose_global.pose.orientation);
  sample.yaw_valid = std::isfinite(sample.yaw_global);
  sample.source = source;
  sample.covariance_global = RotatePlanarCovariance(
    covariance, QuaternionToYaw(transform.transform.rotation));
  sample.covariance_valid = covariance_valid && IsFiniteMatrix(sample.covariance_global);
  return sample;
}

std::optional<VelocitySample2D> BuildVelocitySample(
  const Eigen::Vector2d & raw_velocity,
  bool twist_in_child_frame,
  const PoseSample2D & pose_sample,
  double frame_rotation_yaw,
  const rclcpp::Time & stamp,
  const std::string & source)
{
  if (!IsFiniteVector(raw_velocity)) {
    return std::nullopt;
  }

  VelocitySample2D sample;
  sample.stamp = stamp;
  sample.source = source;
  if (twist_in_child_frame) {
    if (!pose_sample.yaw_valid) {
      return std::nullopt;
    }
    sample.velocity_global = RotateVector2d(raw_velocity, pose_sample.yaw_global);
  } else {
    sample.velocity_global = RotateVector2d(raw_velocity, frame_rotation_yaw);
  }
  return sample;
}

}  // namespace

UgvStateAdapter::UgvStateAdapter(
  rclcpp::Node * node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const Config & config)
: node_(node),
  tf_buffer_(std::move(tf_buffer)),
  config_(config),
  amcl_buffer_(config.buffer_length_sec),
  filtered_buffer_(config.buffer_length_sec),
  odom_buffer_(config.buffer_length_sec)
{
  const auto sensor_qos = rclcpp::SensorDataQoS();
  amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_.amcl_pose_topic,
    10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      HandleAmclPose(*msg);
    });
  filtered_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    config_.filtered_odom_topic,
    sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      HandleFilteredOdometry(*msg);
    });
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    config_.odom_topic,
    sensor_qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      HandleBaseOdometry(*msg);
    });
}

void UgvStateAdapter::HandleAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  const rclcpp::Time stamp = ResolveStamp(msg.header.stamp);
  const std::string source_frame = msg.header.frame_id.empty() ? config_.global_frame : msg.header.frame_id;

  try {
    const PoseSample2D sample = BuildPoseSample(
      *tf_buffer_,
      config_.global_frame,
      source_frame,
      stamp,
      msg.pose.pose,
      ExtractPlanarCovariance(msg.pose.covariance),
      true,
      config_.amcl_pose_topic,
      config_.tf_lookup_timeout_sec);
    amcl_buffer_.PushPose(sample);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ugv state adapter failed to transform AMCL pose %s -> %s: %s",
      source_frame.c_str(), config_.global_frame.c_str(), ex.what());
  }
}

void UgvStateAdapter::HandleFilteredOdometry(const nav_msgs::msg::Odometry & msg)
{
  const rclcpp::Time stamp = ResolveStamp(msg.header.stamp);
  const std::string pose_frame = msg.header.frame_id.empty() ? config_.global_frame : msg.header.frame_id;

  try {
    const PoseSample2D pose_sample = BuildPoseSample(
      *tf_buffer_,
      config_.global_frame,
      pose_frame,
      stamp,
      msg.pose.pose,
      ExtractPlanarCovariance(msg.pose.covariance),
      true,
      config_.filtered_odom_topic,
      config_.tf_lookup_timeout_sec);
    filtered_buffer_.PushPose(pose_sample);

    const auto transform = LookupTransform(
      *tf_buffer_, config_.global_frame, pose_frame, stamp, config_.tf_lookup_timeout_sec);
    const auto velocity_sample = BuildVelocitySample(
      Eigen::Vector2d(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      config_.filtered_twist_in_child_frame,
      pose_sample,
      QuaternionToYaw(transform.transform.rotation),
      stamp,
      config_.filtered_odom_topic);
    if (velocity_sample.has_value()) {
      filtered_buffer_.PushVelocity(*velocity_sample);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ugv state adapter failed to transform filtered odom %s -> %s: %s",
      pose_frame.c_str(), config_.global_frame.c_str(), ex.what());
  }
}

void UgvStateAdapter::HandleBaseOdometry(const nav_msgs::msg::Odometry & msg)
{
  const rclcpp::Time stamp = ResolveStamp(msg.header.stamp);
  const std::string pose_frame = msg.header.frame_id.empty() ? config_.global_frame : msg.header.frame_id;

  try {
    const PoseSample2D pose_sample = BuildPoseSample(
      *tf_buffer_,
      config_.global_frame,
      pose_frame,
      stamp,
      msg.pose.pose,
      ExtractPlanarCovariance(msg.pose.covariance),
      true,
      config_.odom_topic,
      config_.tf_lookup_timeout_sec);
    odom_buffer_.PushPose(pose_sample);

    const auto transform = LookupTransform(
      *tf_buffer_, config_.global_frame, pose_frame, stamp, config_.tf_lookup_timeout_sec);
    const auto velocity_sample = BuildVelocitySample(
      Eigen::Vector2d(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      config_.odom_twist_in_child_frame,
      pose_sample,
      QuaternionToYaw(transform.transform.rotation),
      stamp,
      config_.odom_topic);
    if (velocity_sample.has_value()) {
      odom_buffer_.PushVelocity(*velocity_sample);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "ugv state adapter failed to transform odom %s -> %s: %s",
      pose_frame.c_str(), config_.global_frame.c_str(), ex.what());
  }
}

AgentPlanarState UgvStateAdapter::BuildState(const rclcpp::Time & now) const
{
  std::scoped_lock lock(mutex_);
  AgentPlanarState state;

  const auto amcl_pose = amcl_buffer_.LatestPose(now, config_.pose_timeout_sec);
  const auto odom_pose = odom_buffer_.LatestPose(now, config_.pose_timeout_sec);

  std::optional<PoseSample2D> selected_pose;
  TimeAlignmentBuffer const * selected_pose_buffer = nullptr;
  if (amcl_pose.has_value()) {
    selected_pose = amcl_pose;
    selected_pose_buffer = &amcl_buffer_;
    state.pose_from_fallback = false;
  } else if (config_.assume_odom_is_global && odom_pose.has_value()) {
    selected_pose = odom_pose;
    selected_pose_buffer = &odom_buffer_;
    state.pose_from_fallback = true;
  }

  if (selected_pose.has_value()) {
    state.pose_valid = true;
    state.pose_stamp = selected_pose->stamp;
    state.p_global = selected_pose->position_global;
    state.yaw_global = selected_pose->yaw_global;
    state.yaw_valid = selected_pose->yaw_valid;
    state.pose_age_sec = (now - selected_pose->stamp).seconds();
    state.pose_source = selected_pose->source;
    state.sigma_p = SanitizePlanarCovariance(
      selected_pose->covariance_global,
      config_.covariance_floor_m2,
      config_.covariance_ceiling_m2,
      config_.covariance_nan_fallback_m2,
      state.pose_age_sec,
      config_.covariance_age_inflation_m2_per_s);
    state.covariance_valid = true;
  }

  auto velocity_sample = filtered_buffer_.LatestVelocity(now, config_.velocity_timeout_sec);
  if (!velocity_sample.has_value()) {
    velocity_sample = odom_buffer_.LatestVelocity(now, config_.velocity_timeout_sec);
    if (velocity_sample.has_value()) {
      state.velocity_from_fallback = true;
    }
  }
  if (!velocity_sample.has_value() && selected_pose_buffer != nullptr) {
    velocity_sample = selected_pose_buffer->EstimateVelocityFromPoseHistory(
      now,
      config_.pose_timeout_sec,
      config_.min_pose_diff_velocity_dt_sec,
      config_.max_pose_diff_velocity_dt_sec,
      state.pose_source + ":pose_diff");
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

rclcpp::Time UgvStateAdapter::ResolveStamp(const builtin_interfaces::msg::Time & stamp_msg) const
{
  if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
    return node_->now();
  }
  return rclcpp::Time(stamp_msg, RCL_ROS_TIME);
}

}  // namespace relative_position_fusion
