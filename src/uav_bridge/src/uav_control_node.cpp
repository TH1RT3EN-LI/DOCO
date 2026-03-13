#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "uav_bridge/math_utils.hpp"
#include "uav_bridge/uav_control_logic.hpp"

namespace uav_bridge
{

class UavControlNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

  enum class Px4TimestampSource : uint8_t
  {
    System = 0,
    GazeboSim = 1,
  };

public:
  UavControlNode()
  : Node("uav_control_node")
  {
    this->declare_parameter<std::string>("position_topic", "/uav/control/pose");
    this->declare_parameter<std::string>("velocity_body_topic", "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("offboard_mode_topic", "/uav/fmu/in/offboard_control_mode");
    this->declare_parameter<std::string>("trajectory_setpoint_topic", "/uav/fmu/in/trajectory_setpoint");
    this->declare_parameter<std::string>("vehicle_command_topic", "/uav/fmu/in/vehicle_command");
    this->declare_parameter<std::string>("vehicle_status_topic", "/uav/fmu/out/vehicle_status");
    this->declare_parameter<std::string>("px4_local_position_topic", "/uav/fmu/out/vehicle_local_position");
    this->declare_parameter<std::string>("takeoff_service", "/uav/control/command/takeoff");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>("position_mode_service", "/uav/control/command/position_mode");
    this->declare_parameter<std::string>("land_service", "/uav/control/command/land");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<int>("warmup_cycles", 20);
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<int>("target_component", 1);
    this->declare_parameter<int>("source_system", 1);
    this->declare_parameter<int>("source_component", 1);
    this->declare_parameter<double>("takeoff_height_m", 1.0);
    this->declare_parameter<double>("takeoff_height_tolerance_m", 0.10);
    this->declare_parameter<double>("takeoff_stable_s", 0.5);
    this->declare_parameter<double>("max_velocity_setpoint_mps", 1.5);
    this->declare_parameter<double>("max_acceleration_setpoint_mps2", 2.0);
    this->declare_parameter<double>("velocity_body_timeout_ms", 200.0);
    this->declare_parameter<double>("velocity_mode_max_acc_xy_mps2", 1.0);
    this->declare_parameter<double>("velocity_mode_max_acc_z_mps2", 0.6);
    this->declare_parameter<double>("velocity_mode_max_acc_yaw_radps2", 1.5);
    this->declare_parameter<double>("state_timeout_s", 0.20);
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<std::string>("position_command_frame_id", "");
    this->declare_parameter<std::string>("px4_timestamp_source", "system");
    this->declare_parameter<std::string>("gz_world_name", "test");
    this->declare_parameter<std::string>("gz_clock_topic", "");

    position_topic_ = this->get_parameter("position_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    offboard_mode_topic_ = this->get_parameter("offboard_mode_topic").as_string();
    trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
    vehicle_command_topic_ = this->get_parameter("vehicle_command_topic").as_string();
    vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
    px4_local_position_topic_ = this->get_parameter("px4_local_position_topic").as_string();
    takeoff_service_ = this->get_parameter("takeoff_service").as_string();
    hold_service_ = this->get_parameter("hold_service").as_string();
    position_mode_service_ = this->get_parameter("position_mode_service").as_string();
    land_service_ = this->get_parameter("land_service").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    warmup_cycles_ = this->get_parameter("warmup_cycles").as_int();
    target_system_ = static_cast<uint8_t>(this->get_parameter("target_system").as_int());
    target_component_ = static_cast<uint8_t>(this->get_parameter("target_component").as_int());
    source_system_ = static_cast<uint8_t>(this->get_parameter("source_system").as_int());
    source_component_ = static_cast<uint8_t>(this->get_parameter("source_component").as_int());
    takeoff_height_m_ = static_cast<float>(this->get_parameter("takeoff_height_m").as_double());
    takeoff_height_tolerance_m_ = static_cast<float>(this->get_parameter("takeoff_height_tolerance_m").as_double());
    takeoff_stable_s_ = this->get_parameter("takeoff_stable_s").as_double();
    max_velocity_setpoint_mps_ = static_cast<float>(this->get_parameter("max_velocity_setpoint_mps").as_double());
    max_acceleration_setpoint_mps2_ = static_cast<float>(this->get_parameter("max_acceleration_setpoint_mps2").as_double());
    velocity_body_timeout_ms_ = this->get_parameter("velocity_body_timeout_ms").as_double();
    velocity_mode_max_acc_xy_mps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_xy_mps2").as_double());
    velocity_mode_max_acc_z_mps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_z_mps2").as_double());
    velocity_mode_max_acc_yaw_radps2_ = static_cast<float>(this->get_parameter("velocity_mode_max_acc_yaw_radps2").as_double());
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    position_command_frame_id_ = this->get_parameter("position_command_frame_id").as_string();
    gz_world_name_ = this->get_parameter("gz_world_name").as_string();
    gz_clock_topic_ = this->get_parameter("gz_clock_topic").as_string();

    const std::string timestamp_source = this->get_parameter("px4_timestamp_source").as_string();
    px4_timestamp_source_ = timestamp_source == "gz_sim" ? Px4TimestampSource::GazeboSim : Px4TimestampSource::System;

    if (px4_timestamp_source_ == Px4TimestampSource::GazeboSim) {
      if (gz_clock_topic_.empty()) {
        gz_clock_topic_ = "/clock";
      }
      RCLCPP_INFO(
        this->get_logger(),
        "px4_timestamp_source=gz_sim now uses the ROS clock exposed on %s; gz_world_name is kept for compatibility",
        gz_clock_topic_.c_str());
    }

    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, 10);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic_, 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, 10);

    auto velocity_body_qos = rclcpp::SensorDataQoS();
    velocity_body_qos.keep_last(1);

    position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      position_topic_, 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        handlePositionTarget(*msg);
      });
    velocity_body_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_body_topic_, velocity_body_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
      {
        handleVelocityBodyTarget(*msg);
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        const bool has_stamp =
          (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        current_position_enu_ = {
          static_cast<float>(msg->pose.pose.position.x),
          static_cast<float>(msg->pose.pose.position.y),
          static_cast<float>(msg->pose.pose.position.z)};
        current_yaw_enu_ = quaternionToYaw(msg->pose.pose.orientation);
        current_orientation_enu_flu_ = tf2::Quaternion(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);
        current_orientation_enu_flu_.normalize();
        current_state_frame_id_ = msg->header.frame_id;
        last_state_time_ = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
        has_state_ = true;
      });

    px4_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_local_position_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
      {
        has_local_position_ = msg->xy_valid && msg->z_valid;
        has_local_velocity_ = hasValidVelocityEstimate(
          msg->v_xy_valid, msg->v_z_valid,
          {msg->vx, msg->vy, msg->vz});
      });

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {
        is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        current_nav_state_ = msg->nav_state;
        is_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        if (!is_armed_ && mode_tracker_.mode() == UavControlMode::Landing) {
          clearPostLandingCommands();
          mode_tracker_.onLandingComplete();
          resetWarmup();
          RCLCPP_INFO(this->get_logger(), "landing complete, control mode -> HOLD (targets cleared)");
        }
      });

    takeoff_srv_ = this->create_service<Trigger>(
      takeoff_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleTakeoffRequest(response->message);
      });

    hold_srv_ = this->create_service<Trigger>(
      hold_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleHoldRequest(response->message);
      });

    position_mode_srv_ = this->create_service<Trigger>(
      position_mode_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handlePositionModeRequest(response->message);
      });

    land_srv_ = this->create_service<Trigger>(
      land_service_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = handleLandRequest(response->message);
      });

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(timer_period, std::bind(&UavControlNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "uav_control_node: position=%s velocity_body=%s state=%s takeoff=%s hold=%s position_mode=%s land=%s",
      position_topic_.c_str(), velocity_body_topic_.c_str(), state_topic_.c_str(),
      takeoff_service_.c_str(), hold_service_.c_str(), position_mode_service_.c_str(), land_service_.c_str());
  }

private:
  static const char * modeName(UavControlMode mode)
  {
    switch (mode) {
      case UavControlMode::Hold:
        return "HOLD";
      case UavControlMode::Takeoff:
        return "TAKEOFF";
      case UavControlMode::Position:
        return "POSITION";
      case UavControlMode::Px4PositionHold:
        return "PX4_POSITION_HOLD";
      case UavControlMode::VelocityBody:
        return "VELOCITY_BODY";
      case UavControlMode::Landing:
        return "LANDING";
    }
    return "UNKNOWN";
  }

  void resetWarmup()
  {
    setpoint_counter_ = 0;
  }

  void clearPostLandingCommands()
  {
    has_velocity_body_cmd_ = false;
    velocity_mode_rate_limit_initialized_ = false;
    last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
    last_velocity_mode_yawspeed_ned_ = 0.0f;
    hold_target_valid_ = false;
    position_target_valid_ = false;
  }

  void logModeChange(UavControlMode previous, const char * reason)
  {
    if (previous != mode_tracker_.mode()) {
      if (mode_tracker_.mode() != UavControlMode::VelocityBody) {
        velocity_mode_rate_limit_initialized_ = false;
        last_velocity_mode_ned_ = {0.0f, 0.0f, 0.0f};
        last_velocity_mode_yawspeed_ned_ = 0.0f;
      }
      RCLCPP_INFO(this->get_logger(), "control mode -> %s (%s)", modeName(mode_tracker_.mode()), reason);
      resetWarmup();
    }
  }

  uint64_t nowMicros()
  {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
  }

  bool ensureTimestampReady(const char * context, uint64_t stamp)
  {
    if (stamp != 0) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "waiting for valid timestamp before %s", context);
    return false;
  }

  bool isPx4ExecutionReady() const
  {
    return has_local_position_ && has_state_ && stateFresh();
  }

  bool isPx4VelocityExecutionReady() const
  {
    return isPx4ExecutionReady() && has_local_velocity_;
  }

  bool stateFresh() const
  {
    if (!has_state_ || last_state_time_.nanoseconds() == 0) {
      return false;
    }

    const double age_s = (this->now() - last_state_time_).seconds();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= state_timeout_s_;
  }

  void publishVehicleCommand(uint32_t command, float param1, float param2)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("vehicle command", stamp)) {
      return;
    }
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = stamp;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void maybeEnableOffboardAndArm()
  {
    if (setpoint_counter_ < warmup_cycles_) {
      ++setpoint_counter_;
      return;
    }
    if (!isPx4ExecutionReady()) {
      ++setpoint_counter_;
      return;
    }
    const int cycles_after_warmup = setpoint_counter_ - warmup_cycles_;
    constexpr int kRetryIntervalCycles = 50;
    if ((!is_offboard_mode_ || !is_armed_) && (cycles_after_warmup % kRetryIntervalCycles == 0)) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
    }
    ++setpoint_counter_;
  }

  void publishPositionSetpoint(
    const std::array<float, 3> & position_enu,
    float yaw_enu,
    const std::array<float, 3> & velocity_enu,
    const std::array<float, 3> & acceleration_enu)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("position setpoint", stamp)) {
      return;
    }

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = stamp;
    offboard_mode.position = true;
    offboard_mode.velocity = true;
    offboard_mode.acceleration = true;
    offboard_mode_pub_->publish(offboard_mode);

    const auto pos_ned = enuPositionToNed(position_enu[0], position_enu[1], position_enu[2]);
    std::array<float, 3> velocity_ned = {velocity_enu[1], velocity_enu[0], -velocity_enu[2]};
    std::array<float, 3> acceleration_ned = {acceleration_enu[1], acceleration_enu[0], -acceleration_enu[2]};
    clampVectorNorm(velocity_ned, max_velocity_setpoint_mps_);
    clampVectorNorm(acceleration_ned, max_acceleration_setpoint_mps2_);

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    setpoint.position = {pos_ned[0], pos_ned[1], pos_ned[2]};
    setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
    setpoint.acceleration = {acceleration_ned[0], acceleration_ned[1], acceleration_ned[2]};
    setpoint.jerk = {0.0f, 0.0f, 0.0f};
    setpoint.yaw = enuYawToNed(yaw_enu);
    setpoint.yawspeed = 0.0f;
    trajectory_setpoint_pub_->publish(setpoint);
    maybeEnableOffboardAndArm();
  }

  void publishVelocityBodySetpoint(const geometry_msgs::msg::TwistStamped & cmd)
  {
    const uint64_t stamp = nowMicros();
    if (!ensureTimestampReady("velocity setpoint", stamp) || !has_state_) {
      return;
    }

    if (!isFiniteVelocityBodyCommand(
        static_cast<float>(cmd.twist.linear.x),
        static_cast<float>(cmd.twist.linear.y),
        static_cast<float>(cmd.twist.linear.z),
        static_cast<float>(cmd.twist.angular.z)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite velocity_body command");
      return;
    }

    if (!isPx4VelocityExecutionReady()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "waiting for PX4 local velocity estimate before velocity offboard setpoint");
      return;
    }

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = stamp;
    offboard_mode.position = false;
    offboard_mode.velocity = true;
    offboard_mode.acceleration = false;
    offboard_mode_pub_->publish(offboard_mode);

    const tf2::Matrix3x3 rotation_enu_flu(current_orientation_enu_flu_);
    const tf2::Vector3 velocity_body(
      static_cast<double>(cmd.twist.linear.x),
      static_cast<double>(cmd.twist.linear.y),
      static_cast<double>(cmd.twist.linear.z));
    const tf2::Vector3 velocity_enu = rotation_enu_flu * velocity_body;
    std::array<float, 3> velocity_ned = {
      static_cast<float>(velocity_enu.y()),
      static_cast<float>(velocity_enu.x()),
      static_cast<float>(-velocity_enu.z())};
    clampVectorNorm(velocity_ned, max_velocity_setpoint_mps_);
    applyVelocityModeRateLimit(velocity_ned, cmd.twist.angular.z);

    if (!isFiniteVelocitySetpoint(velocity_ned, last_velocity_mode_yawspeed_ned_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "rejecting non-finite NED velocity setpoint");
      return;
    }

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    setpoint.position = {nan, nan, nan};
    setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
    setpoint.acceleration = {nan, nan, nan};
    setpoint.jerk = {nan, nan, nan};
    setpoint.yaw = nan;
    setpoint.yawspeed = last_velocity_mode_yawspeed_ned_;
    trajectory_setpoint_pub_->publish(setpoint);
    maybeEnableOffboardAndArm();
  }

  void applyVelocityModeRateLimit(std::array<float, 3> & velocity_ned, double yaw_rate_enu)
  {
    const float dt = publish_rate_hz_ > 1.0e-6 ? static_cast<float>(1.0 / publish_rate_hz_) : 0.02f;
    if (!velocity_mode_rate_limit_initialized_) {
      last_velocity_mode_ned_ = velocity_ned;
      last_velocity_mode_yawspeed_ned_ = static_cast<float>(-yaw_rate_enu);
      velocity_mode_rate_limit_initialized_ = true;
      return;
    }

    const float max_dxy = std::max(0.0f, velocity_mode_max_acc_xy_mps2_) * dt;
    if (max_dxy > 0.0f) {
      const float dvx = velocity_ned[0] - last_velocity_mode_ned_[0];
      const float dvy = velocity_ned[1] - last_velocity_mode_ned_[1];
      const float dxy = std::sqrt((dvx * dvx) + (dvy * dvy));
      if (dxy > max_dxy && dxy > 1.0e-6f) {
        const float scale = max_dxy / dxy;
        velocity_ned[0] = last_velocity_mode_ned_[0] + (dvx * scale);
        velocity_ned[1] = last_velocity_mode_ned_[1] + (dvy * scale);
      }
    }

    const float max_dz = std::max(0.0f, velocity_mode_max_acc_z_mps2_) * dt;
    velocity_ned[2] = std::clamp(
      velocity_ned[2],
      last_velocity_mode_ned_[2] - max_dz,
      last_velocity_mode_ned_[2] + max_dz);

    const float target_yawspeed_ned = static_cast<float>(-yaw_rate_enu);
    const float max_dyaw = std::max(0.0f, velocity_mode_max_acc_yaw_radps2_) * dt;
    last_velocity_mode_yawspeed_ned_ = std::clamp(
      target_yawspeed_ned,
      last_velocity_mode_yawspeed_ned_ - max_dyaw,
      last_velocity_mode_yawspeed_ned_ + max_dyaw);

    last_velocity_mode_ned_ = velocity_ned;
  }

  bool captureCurrentPose(std::string & message)
  {
    if (!has_state_ || !isFiniteVector(current_position_enu_)) {
      message = "state odometry is not available yet";
      return false;
    }
    hold_target_position_enu_ = current_position_enu_;
    hold_target_yaw_enu_ = current_yaw_enu_;
    hold_target_valid_ = true;
    return true;
  }

  void handlePositionTarget(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!position_command_frame_id_.empty() &&
      !msg.header.frame_id.empty() &&
      msg.header.frame_id != position_command_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring position target frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), position_command_frame_id_.c_str());
      return;
    }

    position_target_position_enu_ = {
      static_cast<float>(msg.pose.position.x),
      static_cast<float>(msg.pose.position.y),
      static_cast<float>(msg.pose.position.z)};
    if (!isFiniteVector(position_target_position_enu_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring non-finite position target");
      return;
    }

    const double quat_norm =
      (msg.pose.orientation.x * msg.pose.orientation.x) +
      (msg.pose.orientation.y * msg.pose.orientation.y) +
      (msg.pose.orientation.z * msg.pose.orientation.z) +
      (msg.pose.orientation.w * msg.pose.orientation.w);
    if (quat_norm > 1.0e-6) {
      position_target_yaw_enu_ = quaternionToYaw(msg.pose.orientation);
    } else if (has_state_) {
      position_target_yaw_enu_ = current_yaw_enu_;
    }

    position_target_valid_ = true;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPosition();
    logModeChange(previous, "position target");
  }

  void handleVelocityBodyTarget(const geometry_msgs::msg::TwistStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body frame '%s', expected '%s'",
        msg.header.frame_id.c_str(), base_frame_id_.c_str());
      return;
    }

    if (!isFiniteVelocityBodyCommand(
        static_cast<float>(msg.twist.linear.x),
        static_cast<float>(msg.twist.linear.y),
        static_cast<float>(msg.twist.linear.z),
        static_cast<float>(msg.twist.angular.z)))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring non-finite velocity_body command");
      return;
    }

    if (!isPx4VelocityExecutionReady()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "ignoring velocity_body command until PX4 local velocity estimate is valid");
      return;
    }

    last_velocity_body_cmd_ = msg;
    last_velocity_body_time_us_ = nowMicros();
    has_velocity_body_cmd_ = true;
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestVelocityBody();
    logModeChange(previous, "velocity body");
  }

  bool handleTakeoffRequest(std::string & message)
  {
    if (!captureCurrentPose(message)) {
      return false;
    }
    hold_target_position_enu_[2] += takeoff_height_m_;
    takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestTakeoff();
    logModeChange(previous, "takeoff");
    message = "takeoff started";
    return true;
  }

  bool handleHoldRequest(std::string & message)
  {
    if (!captureCurrentPose(message)) {
      return false;
    }
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestHold();
    logModeChange(previous, "hold");
    message = "hold target captured";
    return true;
  }

  bool handleLandRequest(std::string & message)
  {
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestLanding();
    land_command_sent_ = false;
    landing_retry_counter_ = 0;
    logModeChange(previous, "land");
    message = "landing requested";
    return true;
  }

  bool handlePositionModeRequest(std::string & message)
  {
    const auto previous = mode_tracker_.mode();
    mode_tracker_.requestPx4PositionHold();
    position_mode_retry_counter_ = 0;
    has_velocity_body_cmd_ = false;
    position_target_valid_ = false;
    logModeChange(previous, "px4 position mode");
    message = "px4 position mode requested";
    return true;
  }

  void publishPositionTarget()
  {
    if (!position_target_valid_) {
      return;
    }
    publishPositionSetpoint(
      position_target_position_enu_,
      position_target_yaw_enu_,
      {0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f});
  }

  void publishHoldTarget()
  {
    if (!hold_target_valid_) {
      return;
    }
    publishPositionSetpoint(
      hold_target_position_enu_, hold_target_yaw_enu_,
      {0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f});
  }

  void handleTakeoffMode()
  {
    if (!hold_target_valid_ || !has_state_) {
      return;
    }
    publishHoldTarget();
    const float height_error = std::abs(hold_target_position_enu_[2] - current_position_enu_[2]);
    if (height_error <= takeoff_height_tolerance_m_) {
      if (takeoff_reached_since_.nanoseconds() == 0) {
        takeoff_reached_since_ = this->now();
      } else if ((this->now() - takeoff_reached_since_).seconds() >= takeoff_stable_s_) {
        const auto previous = mode_tracker_.mode();
        mode_tracker_.requestHold();
        logModeChange(previous, "takeoff reached");
      }
    } else {
      takeoff_reached_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }

  void handleVelocityBodyMode()
  {
    if (!isPx4VelocityExecutionReady()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "PX4 state or local velocity estimate lost during velocity mode, switching to hold");
      has_velocity_body_cmd_ = false;
      std::string ignored;
      handleHoldRequest(ignored);
      return;
    }

    if (!has_velocity_body_cmd_) {
      return;
    }
    const uint64_t now_us = nowMicros();
    if (!ensureTimestampReady("velocity command", now_us)) {
      return;
    }
    const double elapsed_ms = static_cast<double>(now_us - last_velocity_body_time_us_) / 1000.0;
    if (elapsed_ms > velocity_body_timeout_ms_) {
      has_velocity_body_cmd_ = false;
      mode_tracker_.onVelocityCommandTimeout();
      std::string ignored;
      handleHoldRequest(ignored);
      return;
    }
    publishVelocityBodySetpoint(last_velocity_body_cmd_);
  }

  void handleLandingMode()
  {
    constexpr int kRetryIntervalCycles = 50;
    if (!land_command_sent_ || landing_retry_counter_ % kRetryIntervalCycles == 0) {
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
      land_command_sent_ = true;
    }
    ++landing_retry_counter_;
  }

  void handlePx4PositionMode()
  {
    if (current_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL) {
      return;
    }

    constexpr int kRetryIntervalCycles = 50;
    constexpr float kMavStandardModePositionHold = 1.0f;
    if (position_mode_retry_counter_ % kRetryIntervalCycles == 0) {
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_STANDARD_MODE,
        kMavStandardModePositionHold,
        0.0f);
    }
    ++position_mode_retry_counter_;
  }

  void timerCallback()
  {
    switch (mode_tracker_.mode()) {
      case UavControlMode::Hold:
        publishHoldTarget();
        break;
      case UavControlMode::Takeoff:
        handleTakeoffMode();
        break;
      case UavControlMode::Position:
        publishPositionTarget();
        break;
      case UavControlMode::Px4PositionHold:
        handlePx4PositionMode();
        break;
      case UavControlMode::VelocityBody:
        handleVelocityBodyMode();
        break;
      case UavControlMode::Landing:
        handleLandingMode();
        break;
    }
  }

  std::string position_topic_;
  std::string velocity_body_topic_;
  std::string state_topic_;
  std::string offboard_mode_topic_;
  std::string trajectory_setpoint_topic_;
  std::string vehicle_command_topic_;
  std::string vehicle_status_topic_;
  std::string px4_local_position_topic_;
  std::string takeoff_service_;
  std::string hold_service_;
  std::string position_mode_service_;
  std::string land_service_;
  std::string base_frame_id_;
  std::string position_command_frame_id_;
  std::string gz_world_name_;
  std::string gz_clock_topic_;
  double publish_rate_hz_{50.0};
  int warmup_cycles_{20};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};
  float takeoff_height_m_{1.0f};
  float takeoff_height_tolerance_m_{0.10f};
  double takeoff_stable_s_{0.5};
  float max_velocity_setpoint_mps_{1.5f};
  float max_acceleration_setpoint_mps2_{2.0f};
  double velocity_body_timeout_ms_{200.0};
  float velocity_mode_max_acc_xy_mps2_{1.0f};
  float velocity_mode_max_acc_z_mps2_{0.6f};
  float velocity_mode_max_acc_yaw_radps2_{1.5f};
  double state_timeout_s_{0.20};
  Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  UavControlModeTracker mode_tracker_{};
  geometry_msgs::msg::TwistStamped last_velocity_body_cmd_{};
  bool has_velocity_body_cmd_{false};
  uint64_t last_velocity_body_time_us_{0};
  std::array<float, 3> last_velocity_mode_ned_{0.0f, 0.0f, 0.0f};
  float last_velocity_mode_yawspeed_ned_{0.0f};
  bool velocity_mode_rate_limit_initialized_{false};
  std::array<float, 3> current_position_enu_{0.0f, 0.0f, 0.0f};
  tf2::Quaternion current_orientation_enu_flu_{0.0, 0.0, 0.0, 1.0};
  float current_yaw_enu_{0.0f};
  std::string current_state_frame_id_;
  bool has_state_{false};
  bool has_local_position_{false};
  bool has_local_velocity_{false};
  bool is_armed_{false};
  bool is_offboard_mode_{false};
  uint8_t current_nav_state_{px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL};
  std::array<float, 3> hold_target_position_enu_{0.0f, 0.0f, 0.0f};
  float hold_target_yaw_enu_{0.0f};
  bool hold_target_valid_{false};
  std::array<float, 3> position_target_position_enu_{0.0f, 0.0f, 0.0f};
  float position_target_yaw_enu_{0.0f};
  bool position_target_valid_{false};
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time takeoff_reached_since_{0, 0, RCL_ROS_TIME};
  int setpoint_counter_{0};
  int position_mode_retry_counter_{0};
  bool land_command_sent_{false};
  int landing_retry_counter_{0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<Trigger>::SharedPtr hold_srv_;
  rclcpp::Service<Trigger>::SharedPtr position_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::UavControlNode>());
  rclcpp::shutdown();
  return 0;
}
