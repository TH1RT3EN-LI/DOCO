#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <uav_visual_landing/msg/landing_controller_state.hpp>
#include <uav_visual_landing/msg/target_observation.hpp>
#include <uav_visual_landing/visual_landing_logic.hpp>

namespace uav_visual_landing
{

class VisualLandingNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

public:
  VisualLandingNode()
  : Node("visual_landing_node")
  {
    this->declare_parameter<std::string>("target_observation_topic", "/uav/visual_landing/target_observation");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("velocity_body_topic", "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>("controller_state_topic", "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>("land_service", "/uav/control/command/land");
    this->declare_parameter<std::string>("range_topic", "/uav/fmu/in/distance_sensor");
    this->declare_parameter<std::string>("start_service", "/uav/visual_landing/command/start");
    this->declare_parameter<std::string>("stop_service", "/uav/visual_landing/command/stop");

    this->declare_parameter<double>("kp_xy", 0.50);
    this->declare_parameter<double>("kp_yaw", 1.20);
    this->declare_parameter<double>("max_vxy", 0.40);
    this->declare_parameter<double>("max_vyaw", 0.60);
    this->declare_parameter<double>("vel_damping_xy", 0.25);
    this->declare_parameter<double>("vel_damping_yaw", 0.18);
    this->declare_parameter<double>("yaw_deadband_rad", 0.03);
    this->declare_parameter<double>("min_height_for_xy", 0.35);
    this->declare_parameter<double>("align_enter_error", 0.03);
    this->declare_parameter<double>("align_exit_error", 0.02);
    this->declare_parameter<double>("align_enter_yaw", 0.12);
    this->declare_parameter<double>("align_exit_yaw", 0.09);
    this->declare_parameter<double>("hold_verify_s", 0.50);
    this->declare_parameter<double>("descend_speed", 0.12);
    this->declare_parameter<double>("terminal_descend_speed", 0.06);
    this->declare_parameter<double>("terminal_entry_height_m", 0.60);
    this->declare_parameter<double>("land_height_m", 0.12);
    this->declare_parameter<double>("z_hold_kp", 1.20);
    this->declare_parameter<double>("z_hold_max_vz", 0.18);
    this->declare_parameter<double>("z_descend_kp", 1.30);
    this->declare_parameter<double>("z_descend_max_vz", 0.25);
    this->declare_parameter<double>("z_terminal_kp", 1.10);
    this->declare_parameter<double>("z_terminal_max_vz", 0.12);
    this->declare_parameter<double>("vel_damping_z", 0.18);
    this->declare_parameter<double>("max_acc_xy", 0.80);
    this->declare_parameter<double>("max_acc_z", 0.50);
    this->declare_parameter<double>("max_acc_yaw", 1.20);
    this->declare_parameter<double>("observation_timeout_s", 0.30);
    this->declare_parameter<double>("range_timeout_s", 0.15);
    this->declare_parameter<double>("range_min_m", 0.05);
    this->declare_parameter<double>("range_max_m", 5.0);
    this->declare_parameter<double>("range_consistency_max_diff_m", 0.35);
    this->declare_parameter<double>("control_rate_hz", 30.0);
    this->declare_parameter<double>("status_rate_hz", 5.0);

    target_observation_topic_ = this->get_parameter("target_observation_topic").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    controller_state_topic_ = this->get_parameter("controller_state_topic").as_string();
    hold_service_ = this->get_parameter("hold_service").as_string();
    land_service_ = this->get_parameter("land_service").as_string();
    range_topic_ = this->get_parameter("range_topic").as_string();
    start_service_name_ = this->get_parameter("start_service").as_string();
    stop_service_name_ = this->get_parameter("stop_service").as_string();
    kp_xy_ = static_cast<float>(this->get_parameter("kp_xy").as_double());
    kp_yaw_ = static_cast<float>(this->get_parameter("kp_yaw").as_double());
    max_vxy_ = static_cast<float>(this->get_parameter("max_vxy").as_double());
    max_vyaw_ = static_cast<float>(this->get_parameter("max_vyaw").as_double());
    vel_damping_xy_ = static_cast<float>(this->get_parameter("vel_damping_xy").as_double());
    vel_damping_yaw_ = static_cast<float>(this->get_parameter("vel_damping_yaw").as_double());
    yaw_deadband_rad_ = static_cast<float>(this->get_parameter("yaw_deadband_rad").as_double());
    min_height_for_xy_ = static_cast<float>(this->get_parameter("min_height_for_xy").as_double());
    align_config_.enter_error = static_cast<float>(this->get_parameter("align_enter_error").as_double());
    align_config_.exit_error = static_cast<float>(this->get_parameter("align_exit_error").as_double());
    align_config_.enter_yaw = static_cast<float>(this->get_parameter("align_enter_yaw").as_double());
    align_config_.exit_yaw = static_cast<float>(this->get_parameter("align_exit_yaw").as_double());
    hold_verify_s_ = this->get_parameter("hold_verify_s").as_double();
    descend_speed_ = static_cast<float>(this->get_parameter("descend_speed").as_double());
    terminal_descend_speed_ = static_cast<float>(this->get_parameter("terminal_descend_speed").as_double());
    terminal_entry_height_m_ = static_cast<float>(this->get_parameter("terminal_entry_height_m").as_double());
    land_height_m_ = static_cast<float>(this->get_parameter("land_height_m").as_double());
    z_hold_kp_ = static_cast<float>(this->get_parameter("z_hold_kp").as_double());
    z_hold_max_vz_ = static_cast<float>(this->get_parameter("z_hold_max_vz").as_double());
    z_descend_kp_ = static_cast<float>(this->get_parameter("z_descend_kp").as_double());
    z_descend_max_vz_ = static_cast<float>(this->get_parameter("z_descend_max_vz").as_double());
    z_terminal_kp_ = static_cast<float>(this->get_parameter("z_terminal_kp").as_double());
    z_terminal_max_vz_ = static_cast<float>(this->get_parameter("z_terminal_max_vz").as_double());
    vel_damping_z_ = static_cast<float>(this->get_parameter("vel_damping_z").as_double());
    command_rate_limit_.max_acc_xy_mps2 = static_cast<float>(this->get_parameter("max_acc_xy").as_double());
    command_rate_limit_.max_acc_z_mps2 = static_cast<float>(this->get_parameter("max_acc_z").as_double());
    command_rate_limit_.max_acc_yaw_radps2 = static_cast<float>(this->get_parameter("max_acc_yaw").as_double());
    observation_timeout_s_ = this->get_parameter("observation_timeout_s").as_double();
    range_config_.timeout_s = this->get_parameter("range_timeout_s").as_double();
    range_config_.min_m = static_cast<float>(this->get_parameter("range_min_m").as_double());
    range_config_.max_m = static_cast<float>(this->get_parameter("range_max_m").as_double());
    range_config_.max_diff_m = static_cast<float>(this->get_parameter("range_consistency_max_diff_m").as_double());
    const double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    const double status_rate_hz = this->get_parameter("status_rate_hz").as_double();
    status_period_s_ = status_rate_hz > 1.0e-6 ? (1.0 / status_rate_hz) : 0.2;

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_body_topic_, 10);
    controller_state_pub_ = this->create_publisher<uav_visual_landing::msg::LandingControllerState>(
      controller_state_topic_, rclcpp::QoS(1).reliable().transient_local());
    hold_client_ = this->create_client<Trigger>(hold_service_);
    land_client_ = this->create_client<Trigger>(land_service_);

    observation_sub_ = this->create_subscription<uav_visual_landing::msg::TargetObservation>(
      target_observation_topic_, 10,
      [this](const uav_visual_landing::msg::TargetObservation::SharedPtr msg)
      {
        last_observation_ = *msg;
        const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
        last_observation_time_ = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
        has_observation_ = true;
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        current_height_m_ = static_cast<float>(msg->pose.pose.position.z);
        current_body_vx_ = static_cast<float>(msg->twist.twist.linear.x);
        current_body_vy_ = static_cast<float>(msg->twist.twist.linear.y);
        current_body_vz_ = static_cast<float>(msg->twist.twist.linear.z);
        current_yaw_rate_ = static_cast<float>(msg->twist.twist.angular.z);
        has_state_ = true;
      });

    range_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      range_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
      {
        if (!std::isfinite(msg->current_distance)) {
          return;
        }
        range_height_m_ = msg->current_distance;
        range_stamp_ = this->now();
        has_range_ = true;
      });

    start_srv_ = this->create_service<Trigger>(
      start_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = start(response->message);
      });

    stop_srv_ = this->create_service<Trigger>(
      stop_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = stop(response->message);
      });

    control_dt_s_ = control_rate_hz > 1.0e-6 ? static_cast<float>(1.0 / control_rate_hz) : (1.0f / 30.0f);
    const double period_ms = control_rate_hz > 1.0e-6 ? (1000.0 / control_rate_hz) : (1000.0 / 30.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&VisualLandingNode::timerCallback, this));

    publishControllerState(true, "ODOM", false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    RCLCPP_INFO(
      this->get_logger(),
      "visual_landing_node: observation=%s state=%s velocity_body=%s hold=%s land=%s start=%s stop=%s",
      target_observation_topic_.c_str(), state_topic_.c_str(), velocity_body_topic_.c_str(),
      hold_service_.c_str(), land_service_.c_str(), start_service_name_.c_str(), stop_service_name_.c_str());
  }

private:
  bool start(std::string & message)
  {
    active_ = true;
    land_requested_ = false;
    transitionTo(ControllerPhase::HoldWait, true);
    message = "visual landing started";
    return true;
  }

  bool stop(std::string & message)
  {
    active_ = false;
    land_requested_ = false;
    align_hold_started_ = false;
    z_target_initialized_ = false;
    transitionTo(ControllerPhase::Ready, true);
    message = "visual landing stopped";
    return true;
  }

  void requestHold()
  {
    if (!hold_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "hold service not ready");
      return;
    }
    auto req = std::make_shared<Trigger::Request>();
    hold_client_->async_send_request(req);
  }

  void requestLand()
  {
    if (land_requested_) {
      return;
    }
    if (!land_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "land service not ready");
      return;
    }
    land_requested_ = true;
    auto req = std::make_shared<Trigger::Request>();
    land_client_->async_send_request(req);
  }

  void transitionTo(ControllerPhase phase, bool invoke_hold)
  {
    if (phase_ == phase) {
      return;
    }
    phase_ = phase;
    phase_entered_time_ = this->now();
    align_hold_started_ = false;
    z_target_initialized_ = false;
    if (invoke_hold && (phase == ControllerPhase::Ready || phase == ControllerPhase::HoldWait || phase == ControllerPhase::HoldVerify)) {
      requestHold();
    }
    if (phase == ControllerPhase::Land) {
      requestLand();
    }
    RCLCPP_INFO(this->get_logger(), "[visual_landing] phase -> %s", phaseName(phase));
  }

  bool observationFresh() const
  {
    if (!has_observation_ || !last_observation_.detected) {
      return false;
    }
    return (this->now() - last_observation_time_).seconds() <= observation_timeout_s_;
  }

  RangeDecision rangeDecision() const
  {
    const double age_s = has_range_ ? (this->now() - range_stamp_).seconds() : -1.0;
    return evaluateRangeDecision(current_height_m_, has_range_, range_height_m_, age_s, range_config_);
  }

  bool aligned() const
  {
    return isAligned(
      last_observation_.err_u_norm,
      last_observation_.err_v_norm,
      last_observation_.yaw_err_rad,
      align_in_window_,
      align_config_);
  }

  void publishVelocityBody(float vx, float vy, float vz, float yaw_rate)
  {
    applyBodyRateLimit(
      vx, vy, vz, yaw_rate,
      last_cmd_vx_, last_cmd_vy_, last_cmd_vz_, last_cmd_yaw_rate_,
      control_dt_s_, command_rate_limit_);

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "uav_base_link";
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    msg.twist.angular.z = yaw_rate;
    velocity_pub_->publish(msg);
    last_cmd_vx_ = vx;
    last_cmd_vy_ = vy;
    last_cmd_vz_ = vz;
    last_cmd_yaw_rate_ = yaw_rate;
  }

  void publishControllerState(
    bool force,
    const std::string & height_source,
    bool range_valid,
    float range_height,
    float cmd_vx,
    float cmd_vy,
    float cmd_vz,
    float cmd_yaw_rate)
  {
    const rclcpp::Time now = this->now();
    if (!force && status_initialized_) {
      if ((now - last_status_time_).seconds() < status_period_s_) {
        return;
      }
    }
    uav_visual_landing::msg::LandingControllerState msg;
    msg.header.stamp = now;
    msg.active = active_;
    msg.phase = phaseName(phase_);
    msg.target_detected = has_observation_ && last_observation_.detected;
    msg.observation_age_s = has_observation_ ? static_cast<float>((now - last_observation_time_).seconds()) : -1.0f;
    msg.target_confidence = has_observation_ ? last_observation_.confidence : 0.0f;
    msg.height_source = height_source;
    msg.current_height_m = current_height_m_;
    msg.range_valid = range_valid;
    msg.range_height_m = range_height;
    msg.cmd_vx = cmd_vx;
    msg.cmd_vy = cmd_vy;
    msg.cmd_vz = cmd_vz;
    msg.cmd_yaw_rate = cmd_yaw_rate;
    controller_state_pub_->publish(msg);
    last_status_time_ = now;
    status_initialized_ = true;
  }

  void computeTrackingCommand(float xy_limit, float & vx, float & vy, float & yaw_rate) const
  {
    const float ctrl_height = std::max(current_height_m_, min_height_for_xy_);
    vx = (-(kp_xy_ * last_observation_.err_v_norm) * ctrl_height) - (vel_damping_xy_ * current_body_vx_);
    vy = (-(kp_xy_ * last_observation_.err_u_norm) * ctrl_height) - (vel_damping_xy_ * current_body_vy_);
    const float norm = std::sqrt((vx * vx) + (vy * vy));
    if (norm > xy_limit && norm > 1.0e-6f) {
      const float scale = xy_limit / norm;
      vx *= scale;
      vy *= scale;
    }
    if (std::abs(last_observation_.yaw_err_rad) < yaw_deadband_rad_) {
      yaw_rate = 0.0f;
    } else {
      yaw_rate = (-(kp_yaw_ * last_observation_.yaw_err_rad)) - (vel_damping_yaw_ * current_yaw_rate_);
      yaw_rate = clamp(yaw_rate, max_vyaw_);
    }
  }

  void initializeZTarget(float target_height_m)
  {
    z_target_height_m_ = target_height_m;
    z_target_initialized_ = true;
  }

  float computeZCommand(
    float current_height_m,
    float current_body_vz,
    float target_height_m,
    float kp,
    float max_vz) const
  {
    return computeClosedLoopVelocity(
      target_height_m,
      current_height_m,
      current_body_vz,
      kp,
      vel_damping_z_,
      max_vz);
  }

  void timerCallback()
  {
    std::string height_source = "ODOM";
    const auto range = rangeDecision();
    const bool fresh_observation = observationFresh();
    const bool currently_aligned = fresh_observation && aligned();
    align_in_window_ = currently_aligned;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_vz = 0.0f;
    float cmd_yaw_rate = 0.0f;

    if (!active_ || !has_state_) {
      publishControllerState(false, height_source, range.valid, range.effective_height_m, 0.0f, 0.0f, 0.0f, 0.0f);
      return;
    }

    switch (phase_) {
      case ControllerPhase::Ready:
        break;
      case ControllerPhase::HoldWait:
        if (!z_target_initialized_) {
          initializeZTarget(current_height_m_);
        }
        if (fresh_observation) {
          transitionTo(ControllerPhase::TrackAlign, false);
        }
        break;
      case ControllerPhase::TrackAlign:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!z_target_initialized_) {
          initializeZTarget(current_height_m_);
        }
        computeTrackingCommand(max_vxy_, cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          current_height_m_, current_body_vz_, z_target_height_m_,
          z_hold_kp_, z_hold_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (currently_aligned) {
          transitionTo(ControllerPhase::HoldVerify, true);
        }
        break;
      case ControllerPhase::HoldVerify:
        if (!z_target_initialized_) {
          initializeZTarget(current_height_m_);
        }
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!currently_aligned) {
          transitionTo(ControllerPhase::TrackAlign, false);
          break;
        }
        if (!align_hold_started_) {
          align_hold_started_ = true;
          align_hold_since_ = this->now();
        } else if ((this->now() - align_hold_since_).seconds() >= hold_verify_s_) {
          transitionTo(
            current_height_m_ <= terminal_entry_height_m_ ? ControllerPhase::Terminal : ControllerPhase::DescendTrack,
            false);
        }
        break;
      case ControllerPhase::DescendTrack:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!z_target_initialized_) {
          initializeZTarget(current_height_m_);
        }
        z_target_height_m_ = std::max(terminal_entry_height_m_, z_target_height_m_ - (descend_speed_ * control_dt_s_));
        computeTrackingCommand(max_vxy_, cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          current_height_m_, current_body_vz_, z_target_height_m_,
          z_descend_kp_, z_descend_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (current_height_m_ <= terminal_entry_height_m_) {
          transitionTo(ControllerPhase::Terminal, false);
        }
        break;
      case ControllerPhase::Terminal:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!range.valid) {
          transitionTo(ControllerPhase::HoldWait, true);
          break;
        }
        height_source = "RANGE";
        if (!z_target_initialized_) {
          initializeZTarget(range.effective_height_m);
        }
        z_target_height_m_ = std::max(land_height_m_, z_target_height_m_ - (terminal_descend_speed_ * control_dt_s_));
        computeTrackingCommand(std::min(max_vxy_, 0.15f), cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          range.effective_height_m, current_body_vz_, z_target_height_m_,
          z_terminal_kp_, z_terminal_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (range.effective_height_m <= land_height_m_) {
          transitionTo(ControllerPhase::Land, false);
        }
        break;
      case ControllerPhase::Land:
        height_source = range.valid ? "RANGE" : "ODOM";
        break;
    }

    publishControllerState(false, height_source, range.valid, range.effective_height_m, cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
  }

  std::string target_observation_topic_;
  std::string state_topic_;
  std::string velocity_body_topic_;
  std::string controller_state_topic_;
  std::string hold_service_;
  std::string land_service_;
  std::string range_topic_;
  std::string start_service_name_;
  std::string stop_service_name_;
  AlignmentConfig align_config_{};
  RangeConfig range_config_{};
  float kp_xy_{0.5f};
  float kp_yaw_{1.2f};
  float max_vxy_{0.4f};
  float max_vyaw_{0.6f};
  float vel_damping_xy_{0.25f};
  float vel_damping_yaw_{0.18f};
  float yaw_deadband_rad_{0.03f};
  float min_height_for_xy_{0.35f};
  double hold_verify_s_{0.5};
  float descend_speed_{0.12f};
  float terminal_descend_speed_{0.06f};
  float terminal_entry_height_m_{0.60f};
  float land_height_m_{0.12f};
  double observation_timeout_s_{0.30};
  bool active_{false};
  ControllerPhase phase_{ControllerPhase::Ready};
  bool align_in_window_{false};
  bool align_hold_started_{false};
  bool land_requested_{false};
  bool has_observation_{false};
  bool has_state_{false};
  bool has_range_{false};
  float current_height_m_{0.0f};
  float current_body_vx_{0.0f};
  float current_body_vy_{0.0f};
  float current_body_vz_{0.0f};
  float current_yaw_rate_{0.0f};
  float range_height_m_{0.0f};
  float last_cmd_vx_{0.0f};
  float last_cmd_vy_{0.0f};
  float last_cmd_vz_{0.0f};
  float last_cmd_yaw_rate_{0.0f};
  float z_target_height_m_{0.0f};
  bool z_target_initialized_{false};
  float z_hold_kp_{1.2f};
  float z_hold_max_vz_{0.18f};
  float z_descend_kp_{1.3f};
  float z_descend_max_vz_{0.25f};
  float z_terminal_kp_{1.1f};
  float z_terminal_max_vz_{0.12f};
  float vel_damping_z_{0.18f};
  float control_dt_s_{1.0f / 30.0f};
  CommandRateLimitConfig command_rate_limit_{};
  double status_period_s_{0.2};
  bool status_initialized_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time range_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_entered_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_hold_since_{0, 0, RCL_ROS_TIME};
  uav_visual_landing::msg::TargetObservation last_observation_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::LandingControllerState>::SharedPtr controller_state_pub_;
  rclcpp::Subscription<uav_visual_landing::msg::TargetObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr range_sub_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::Client<Trigger>::SharedPtr land_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_visual_landing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_visual_landing::VisualLandingNode>());
  rclcpp::shutdown();
  return 0;
}
