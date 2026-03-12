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
    this->declare_parameter<std::string>(
      "target_observation_topic",
      "/uav/visual_landing/target_observation");
    this->declare_parameter<std::string>("state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>(
      "velocity_body_topic",
      "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>(
      "controller_state_topic",
      "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>("land_service", "/uav/control/command/land");
    this->declare_parameter<std::string>("range_topic", "/uav/fmu/in/distance_sensor");
    this->declare_parameter<std::string>("start_service", "/uav/visual_landing/command/start");
    this->declare_parameter<std::string>("stop_service", "/uav/visual_landing/command/stop");

    this->declare_parameter<double>("kp_xy_m", 0.80);
    this->declare_parameter<double>("kd_xy_m", 0.15);
    this->declare_parameter<double>("xy_error_alpha", 0.35);
    this->declare_parameter<double>("xy_error_rate_alpha", 0.25);
    this->declare_parameter<double>("xy_error_rate_limit", 2.0);
    this->declare_parameter<double>("xy_error_rate_dt_min_s", 0.005);
    this->declare_parameter<double>("xy_error_rate_dt_max_s", 0.20);

    this->declare_parameter<double>("kp_yaw", 1.20);
    this->declare_parameter<double>("max_vyaw", 0.60);
    this->declare_parameter<double>("vel_damping_yaw", 0.18);
    this->declare_parameter<double>("yaw_deadband_rad", 0.03);

    this->declare_parameter<double>("align_enter_lateral_m", 0.08);
    this->declare_parameter<double>("align_exit_lateral_m", 0.05);
    this->declare_parameter<double>("align_enter_yaw_rad", 0.08);
    this->declare_parameter<double>("align_exit_yaw_rad", 0.06);
    this->declare_parameter<double>("max_vxy", 0.40);

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

    kp_xy_m_ = static_cast<float>(this->get_parameter("kp_xy_m").as_double());
    kd_xy_m_ = static_cast<float>(this->get_parameter("kd_xy_m").as_double());
    xy_error_alpha_ = static_cast<float>(this->get_parameter("xy_error_alpha").as_double());
    xy_error_rate_alpha_ =
      static_cast<float>(this->get_parameter("xy_error_rate_alpha").as_double());
    xy_error_rate_limit_ =
      static_cast<float>(this->get_parameter("xy_error_rate_limit").as_double());
    xy_error_rate_dt_min_s_ =
      static_cast<float>(this->get_parameter("xy_error_rate_dt_min_s").as_double());
    xy_error_rate_dt_max_s_ =
      static_cast<float>(this->get_parameter("xy_error_rate_dt_max_s").as_double());

    kp_yaw_ = static_cast<float>(this->get_parameter("kp_yaw").as_double());
    max_vyaw_ = static_cast<float>(this->get_parameter("max_vyaw").as_double());
    vel_damping_yaw_ = static_cast<float>(this->get_parameter("vel_damping_yaw").as_double());
    yaw_deadband_rad_ = static_cast<float>(this->get_parameter("yaw_deadband_rad").as_double());

    alignment_config_.align_enter_lateral_m =
      static_cast<float>(this->get_parameter("align_enter_lateral_m").as_double());
    alignment_config_.align_exit_lateral_m =
      static_cast<float>(this->get_parameter("align_exit_lateral_m").as_double());
    alignment_config_.align_enter_yaw_rad =
      static_cast<float>(this->get_parameter("align_enter_yaw_rad").as_double());
    alignment_config_.align_exit_yaw_rad =
      static_cast<float>(this->get_parameter("align_exit_yaw_rad").as_double());
    max_vxy_ = static_cast<float>(this->get_parameter("max_vxy").as_double());

    hold_verify_s_ = this->get_parameter("hold_verify_s").as_double();
    descend_speed_ = static_cast<float>(this->get_parameter("descend_speed").as_double());
    terminal_descend_speed_ =
      static_cast<float>(this->get_parameter("terminal_descend_speed").as_double());
    terminal_entry_height_m_ =
      static_cast<float>(this->get_parameter("terminal_entry_height_m").as_double());
    land_height_m_ = static_cast<float>(this->get_parameter("land_height_m").as_double());
    z_hold_kp_ = static_cast<float>(this->get_parameter("z_hold_kp").as_double());
    z_hold_max_vz_ = static_cast<float>(this->get_parameter("z_hold_max_vz").as_double());
    z_descend_kp_ = static_cast<float>(this->get_parameter("z_descend_kp").as_double());
    z_descend_max_vz_ = static_cast<float>(this->get_parameter("z_descend_max_vz").as_double());
    z_terminal_kp_ = static_cast<float>(this->get_parameter("z_terminal_kp").as_double());
    z_terminal_max_vz_ = static_cast<float>(this->get_parameter("z_terminal_max_vz").as_double());
    vel_damping_z_ = static_cast<float>(this->get_parameter("vel_damping_z").as_double());

    command_rate_limit_.max_acc_xy_mps2 =
      static_cast<float>(this->get_parameter("max_acc_xy").as_double());
    command_rate_limit_.max_acc_z_mps2 =
      static_cast<float>(this->get_parameter("max_acc_z").as_double());
    command_rate_limit_.max_acc_yaw_radps2 =
      static_cast<float>(this->get_parameter("max_acc_yaw").as_double());

    observation_timeout_s_ = this->get_parameter("observation_timeout_s").as_double();
    height_config_.timeout_s = this->get_parameter("range_timeout_s").as_double();
    height_config_.min_m = static_cast<float>(this->get_parameter("range_min_m").as_double());
    height_config_.max_m = static_cast<float>(this->get_parameter("range_max_m").as_double());
    height_config_.max_diff_m =
      static_cast<float>(this->get_parameter("range_consistency_max_diff_m").as_double());

    const double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    const double status_rate_hz = this->get_parameter("status_rate_hz").as_double();
    control_dt_s_ = control_rate_hz >
      1.0e-6 ? static_cast<float>(1.0 / control_rate_hz) : (1.0f / 30.0f);
    status_period_s_ = status_rate_hz > 1.0e-6 ? (1.0 / status_rate_hz) : 0.2;

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      velocity_body_topic_,
      10);
    controller_state_pub_ = this->create_publisher<uav_visual_landing::msg::LandingControllerState>(
      controller_state_topic_, rclcpp::QoS(1).reliable().transient_local());
    hold_client_ = this->create_client<Trigger>(hold_service_);
    land_client_ = this->create_client<Trigger>(land_service_);

    observation_sub_ = this->create_subscription<uav_visual_landing::msg::TargetObservation>(
      target_observation_topic_, 10,
      [this](const uav_visual_landing::msg::TargetObservation::SharedPtr msg)
      {
        onObservation(*msg);
      });

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        odom_height_m_ = static_cast<float>(msg->pose.pose.position.z);
        current_body_vz_ = static_cast<float>(msg->twist.twist.linear.z);
        current_yaw_rate_ = static_cast<float>(msg->twist.twist.angular.z);
        has_state_ = true;
      });

    height_measurement_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      range_topic_, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
      {
        if (!std::isfinite(msg->current_distance)) {
          return;
        }
        height_measurement_m_ = msg->current_distance;
        height_measurement_stamp_ = this->now();
        has_height_measurement_ = true;
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

    const double period_ms = control_rate_hz >
      1.0e-6 ? (1000.0 / control_rate_hz) : (1000.0 / 30.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&VisualLandingNode::timerCallback, this));

    publishControllerState(
      true, HeightDecision{}, 0.0f, MetricLateralError{}, 0.0f, 0.0f, 0.0f,
      0.0f);
    RCLCPP_INFO(
      this->get_logger(),
      "visual_landing_node: observation=%s state=%s velocity_body=%s hold=%s land=%s start=%s stop=%s",
      target_observation_topic_.c_str(), state_topic_.c_str(), velocity_body_topic_.c_str(),
      hold_service_.c_str(), land_service_.c_str(),
      start_service_name_.c_str(), stop_service_name_.c_str());
  }

private:
  bool start(std::string & message)
  {
    active_ = true;
    land_requested_ = false;
    clearVisionTrackingState();
    transitionTo(ControllerPhase::HoldWait, true);
    message = "visual landing started";
    return true;
  }

  bool stop(std::string & message)
  {
    active_ = false;
    land_requested_ = false;
    clearVisionTrackingState();
    transitionTo(ControllerPhase::Ready, true);
    message = "visual landing stopped";
    return true;
  }

  void clearVisionTrackingState()
  {
    filter_initialized_ = false;
    err_u_f_ = 0.0f;
    err_v_f_ = 0.0f;
    derr_u_f_ = 0.0f;
    derr_v_f_ = 0.0f;
    prev_err_u_raw_ = 0.0f;
    prev_err_v_raw_ = 0.0f;
    align_in_window_ = false;
    align_hold_started_ = false;
    z_target_initialized_ = false;
  }

  void onObservation(const uav_visual_landing::msg::TargetObservation & msg)
  {
    last_observation_ = msg;
    const bool has_stamp = (msg.header.stamp.sec != 0) || (msg.header.stamp.nanosec != 0);
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg.header.stamp) : this->now();
    last_observation_time_ = stamp;
    has_observation_ = true;

    if (!msg.detected || !std::isfinite(msg.err_u_norm) || !std::isfinite(msg.err_v_norm) ||
      !std::isfinite(msg.yaw_err_rad))
    {
      return;
    }

    if (!filter_initialized_) {
      err_u_f_ = msg.err_u_norm;
      err_v_f_ = msg.err_v_norm;
      derr_u_f_ = 0.0f;
      derr_v_f_ = 0.0f;
      prev_err_u_raw_ = msg.err_u_norm;
      prev_err_v_raw_ = msg.err_v_norm;
      prev_observation_stamp_ = stamp;
      filter_initialized_ = true;
      return;
    }

    err_u_f_ = lerp(err_u_f_, msg.err_u_norm, xy_error_alpha_);
    err_v_f_ = lerp(err_v_f_, msg.err_v_norm, xy_error_alpha_);

    const float dt_s = static_cast<float>((stamp - prev_observation_stamp_).seconds());
    float raw_rate_u = 0.0f;
    float raw_rate_v = 0.0f;
    if (computeLimitedRate(
        msg.err_u_norm, prev_err_u_raw_, dt_s,
        xy_error_rate_dt_min_s_, xy_error_rate_dt_max_s_, xy_error_rate_limit_, raw_rate_u))
    {
      derr_u_f_ = lerp(derr_u_f_, raw_rate_u, xy_error_rate_alpha_);
    }
    if (computeLimitedRate(
        msg.err_v_norm, prev_err_v_raw_, dt_s,
        xy_error_rate_dt_min_s_, xy_error_rate_dt_max_s_, xy_error_rate_limit_, raw_rate_v))
    {
      derr_v_f_ = lerp(derr_v_f_, raw_rate_v, xy_error_rate_alpha_);
    }

    prev_err_u_raw_ = msg.err_u_norm;
    prev_err_v_raw_ = msg.err_v_norm;
    prev_observation_stamp_ = stamp;
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
    if (phase == ControllerPhase::Ready || phase == ControllerPhase::HoldWait) {
      clearVisionTrackingState();
    }
    if (invoke_hold &&
      (phase == ControllerPhase::Ready || phase == ControllerPhase::HoldWait ||
      phase == ControllerPhase::HoldVerify))
    {
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

  HeightDecision heightDecision() const
  {
    const double age_s =
      has_height_measurement_ ? (this->now() - height_measurement_stamp_).seconds() : -1.0;
    return evaluateHeightDecision(
      odom_height_m_, has_height_measurement_, height_measurement_m_,
      age_s, height_config_);
  }

  bool aligned(const MetricLateralError & lateral_error) const
  {
    if (!filter_initialized_) {
      return false;
    }
    return isAligned(
      lateral_error, last_observation_.yaw_err_rad, align_in_window_,
      alignment_config_);
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
    const HeightDecision & height_decision,
    float control_height_m,
    const MetricLateralError & lateral_error,
    float cmd_vx,
    float cmd_vy,
    float cmd_vz,
    float cmd_yaw_rate)
  {
    const rclcpp::Time now = this->now();
    if (!force && status_initialized_ && ((now - last_status_time_).seconds() < status_period_s_)) {
      return;
    }

    const float z_error = z_target_initialized_ ? (z_target_height_m_ - control_height_m) : 0.0f;

    uav_visual_landing::msg::LandingControllerState msg;
    msg.header.stamp = now;
    msg.active = active_;
    msg.phase = phaseName(phase_);
    msg.target_detected = has_observation_ && last_observation_.detected;
    msg.observation_age_s =
      has_observation_ ? static_cast<float>((now - last_observation_time_).seconds()) : -1.0f;
    msg.target_confidence = has_observation_ ? last_observation_.confidence : 0.0f;
    msg.height_source = heightSourceName(height_decision.height_source);
    msg.odom_height_m = odom_height_m_;
    msg.height_valid = height_decision.height_valid;
    msg.height_measurement_m = has_height_measurement_ ? height_measurement_m_ : 0.0f;
    msg.control_height_m = control_height_m;
    msg.tag_depth_valid = has_observation_ && last_observation_.tag_depth_valid;
    msg.tag_depth_m =
      (has_observation_ &&
      last_observation_.tag_depth_valid) ? last_observation_.tag_depth_m : 0.0f;
    msg.align_enter_lateral_m = alignment_config_.align_enter_lateral_m;
    msg.align_exit_lateral_m = alignment_config_.align_exit_lateral_m;
    msg.active_max_vxy = max_vxy_;
    msg.err_u_norm_filtered = err_u_f_;
    msg.err_v_norm_filtered = err_v_f_;
    msg.err_u_rate_norm_s = derr_u_f_;
    msg.err_v_rate_norm_s = derr_v_f_;
    msg.lateral_error_valid = lateral_error.valid;
    msg.lateral_error_x_m = lateral_error.x_m;
    msg.lateral_error_y_m = lateral_error.y_m;
    msg.lateral_error_m = lateral_error.norm_m;
    msg.lateral_error_rate_x_mps = lateral_error.x_rate_mps;
    msg.lateral_error_rate_y_mps = lateral_error.y_rate_mps;
    msg.z_target_height_m = z_target_initialized_ ? z_target_height_m_ : control_height_m;
    msg.z_error_m = z_error;
    msg.xy_control_mode = "vision_pd_metric";
    msg.cmd_vx = cmd_vx;
    msg.cmd_vy = cmd_vy;
    msg.cmd_vz = cmd_vz;
    msg.cmd_yaw_rate = cmd_yaw_rate;
    controller_state_pub_->publish(msg);
    last_status_time_ = now;
    status_initialized_ = true;
  }

  void computeTrackingCommand(
    const MetricLateralError & lateral_error,
    float & vx,
    float & vy,
    float & yaw_rate) const
  {
    vx = 0.0f;
    vy = 0.0f;
    if (lateral_error.valid) {
      vx = -((kp_xy_m_ * lateral_error.y_m) + (kd_xy_m_ * lateral_error.y_rate_mps));
      vy = -((kp_xy_m_ * lateral_error.x_m) + (kd_xy_m_ * lateral_error.x_rate_mps));
      const float vxy_norm = std::sqrt((vx * vx) + (vy * vy));
      if (vxy_norm > max_vxy_ && vxy_norm > 1.0e-6f) {
        const float scale = max_vxy_ / vxy_norm;
        vx *= scale;
        vy *= scale;
      }
    }

    if (std::abs(last_observation_.yaw_err_rad) < yaw_deadband_rad_) {
      yaw_rate = 0.0f;
    } else {
      yaw_rate = clamp(
        (-(kp_yaw_ * last_observation_.yaw_err_rad)) - (vel_damping_yaw_ * current_yaw_rate_),
        max_vyaw_);
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
    const HeightDecision height_decision = heightDecision();
    const float control_height_m = height_decision.height_m;
    const bool fresh_observation = observationFresh();
    const MetricLateralError lateral_error = (fresh_observation && filter_initialized_) ?
      computeMetricLateralError(err_u_f_, err_v_f_, derr_u_f_, derr_v_f_, control_height_m) :
      MetricLateralError{};
    const bool currently_aligned = fresh_observation && aligned(lateral_error);
    align_in_window_ = currently_aligned;

    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_vz = 0.0f;
    float cmd_yaw_rate = 0.0f;

    if (!active_ || !has_state_) {
      publishControllerState(
        false, height_decision, control_height_m, lateral_error, 0.0f, 0.0f,
        0.0f, 0.0f);
      return;
    }

    switch (phase_) {
      case ControllerPhase::Ready:
        break;
      case ControllerPhase::HoldWait:
        if (!z_target_initialized_) {
          initializeZTarget(control_height_m);
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
          initializeZTarget(control_height_m);
        }
        computeTrackingCommand(lateral_error, cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_, z_hold_kp_,
          z_hold_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (currently_aligned) {
          transitionTo(ControllerPhase::HoldVerify, true);
        }
        break;
      case ControllerPhase::HoldVerify:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!currently_aligned) {
          transitionTo(ControllerPhase::TrackAlign, false);
          break;
        }
        if (!z_target_initialized_) {
          initializeZTarget(control_height_m);
        }
        if (!align_hold_started_) {
          align_hold_started_ = true;
          align_hold_since_ = this->now();
        } else if ((this->now() - align_hold_since_).seconds() >= hold_verify_s_) {
          if (control_height_m <= terminal_entry_height_m_) {
            if (!height_decision.height_valid) {
              transitionTo(ControllerPhase::HoldWait, true);
            } else {
              transitionTo(ControllerPhase::Terminal, false);
            }
          } else {
            transitionTo(ControllerPhase::DescendTrack, false);
          }
        }
        break;
      case ControllerPhase::DescendTrack:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (control_height_m <= terminal_entry_height_m_ && !height_decision.height_valid) {
          transitionTo(ControllerPhase::HoldWait, true);
          break;
        }
        if (!z_target_initialized_) {
          initializeZTarget(control_height_m);
        }
        z_target_height_m_ =
          std::max(terminal_entry_height_m_, z_target_height_m_ - (descend_speed_ * control_dt_s_));
        computeTrackingCommand(lateral_error, cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_,
          z_descend_kp_, z_descend_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (control_height_m <= terminal_entry_height_m_) {
          if (height_decision.height_valid) {
            transitionTo(ControllerPhase::Terminal, false);
          } else {
            transitionTo(ControllerPhase::HoldWait, true);
          }
        }
        break;
      case ControllerPhase::Terminal:
        if (!fresh_observation) {
          transitionTo(nextPhaseOnTargetLoss(phase_), true);
          break;
        }
        if (!height_decision.height_valid) {
          transitionTo(ControllerPhase::HoldWait, true);
          break;
        }
        if (!z_target_initialized_) {
          initializeZTarget(control_height_m);
        }
        z_target_height_m_ =
          std::max(land_height_m_, z_target_height_m_ - (terminal_descend_speed_ * control_dt_s_));
        computeTrackingCommand(lateral_error, cmd_vx, cmd_vy, cmd_yaw_rate);
        cmd_vz = computeZCommand(
          control_height_m, current_body_vz_, z_target_height_m_,
          z_terminal_kp_, z_terminal_max_vz_);
        publishVelocityBody(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate);
        if (control_height_m <= land_height_m_) {
          transitionTo(ControllerPhase::Land, false);
        }
        break;
      case ControllerPhase::Land:
        break;
    }

    publishControllerState(
      false, height_decision, control_height_m, lateral_error, cmd_vx, cmd_vy,
      cmd_vz, cmd_yaw_rate);
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
  HeightConfig height_config_{};
  AlignmentConfig alignment_config_{};
  float kp_xy_m_{0.80f};
  float kd_xy_m_{0.15f};
  float max_vxy_{0.40f};
  float xy_error_alpha_{0.35f};
  float xy_error_rate_alpha_{0.25f};
  float xy_error_rate_limit_{2.0f};
  float xy_error_rate_dt_min_s_{0.005f};
  float xy_error_rate_dt_max_s_{0.20f};
  float kp_yaw_{1.2f};
  float max_vyaw_{0.6f};
  float vel_damping_yaw_{0.18f};
  float yaw_deadband_rad_{0.03f};
  double hold_verify_s_{0.5};
  float descend_speed_{0.12f};
  float terminal_descend_speed_{0.06f};
  float terminal_entry_height_m_{0.60f};
  float land_height_m_{0.12f};
  float z_hold_kp_{1.2f};
  float z_hold_max_vz_{0.18f};
  float z_descend_kp_{1.3f};
  float z_descend_max_vz_{0.25f};
  float z_terminal_kp_{1.1f};
  float z_terminal_max_vz_{0.12f};
  float vel_damping_z_{0.18f};
  CommandRateLimitConfig command_rate_limit_{};
  double observation_timeout_s_{0.30};
  bool active_{false};
  ControllerPhase phase_{ControllerPhase::Ready};
  bool align_in_window_{false};
  bool align_hold_started_{false};
  bool land_requested_{false};
  bool has_observation_{false};
  bool has_state_{false};
  bool has_height_measurement_{false};
  bool filter_initialized_{false};
  float odom_height_m_{0.0f};
  float current_body_vz_{0.0f};
  float current_yaw_rate_{0.0f};
  float height_measurement_m_{0.0f};
  float err_u_f_{0.0f};
  float err_v_f_{0.0f};
  float derr_u_f_{0.0f};
  float derr_v_f_{0.0f};
  float prev_err_u_raw_{0.0f};
  float prev_err_v_raw_{0.0f};
  float last_cmd_vx_{0.0f};
  float last_cmd_vy_{0.0f};
  float last_cmd_vz_{0.0f};
  float last_cmd_yaw_rate_{0.0f};
  float z_target_height_m_{0.0f};
  bool z_target_initialized_{false};
  float control_dt_s_{1.0f / 30.0f};
  double status_period_s_{0.2};
  bool status_initialized_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_observation_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time height_measurement_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_entered_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_hold_since_{0, 0, RCL_ROS_TIME};
  uav_visual_landing::msg::TargetObservation last_observation_{};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::LandingControllerState>::SharedPtr
    controller_state_pub_;
  rclcpp::Subscription<uav_visual_landing::msg::TargetObservation>::SharedPtr observation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr height_measurement_sub_;
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
