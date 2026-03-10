#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <uav_visual_landing/msg/landing_error.hpp>
#include <uav_visual_landing/msg/landing_status.hpp>

namespace uav_visual_landing
{

static float quaternionToYaw(double qx, double qy, double qz, double qw)
{
  const double siny = 2.0 * (qw * qz + qx * qy);
  const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  return static_cast<float>(std::atan2(siny, cosy));
}

class VisualLandingNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

  enum class State : uint8_t
  {
    READY,
    IDLE,
    ALIGN,
    SEARCH_RISE,
    DESCEND,
    LAND,
    DONE
  };

public:
  VisualLandingNode() : Node("visual_landing_node")
  {
    this->declare_parameter<std::string>("landing_error_topic", "/uav/visual_landing/landing_error");
    this->declare_parameter<std::string>("odom_topic", "/uav/odom");
    this->declare_parameter<std::string>("velocity_topic", "/uav/control/velocity");
    this->declare_parameter<std::string>("land_service", "/uav/control/land");
    this->declare_parameter<std::string>("active_topic", "/uav/visual_landing/active");
    this->declare_parameter<std::string>("status_topic", "/uav/visual_landing/status");
    this->declare_parameter<double>("status_rate_hz", 5.0);
    this->declare_parameter<bool>("auto_start", false);

    this->declare_parameter<double>("kp_xy", 0.22);
    this->declare_parameter<double>("kp_yaw", 0.45);
    this->declare_parameter<double>("max_vxy", 0.18);
    this->declare_parameter<double>("max_vyaw", 0.18);
    this->declare_parameter<double>("max_error_norm", 0.50);
    this->declare_parameter<double>("speed_scale_xy", 1.0);
    this->declare_parameter<double>("speed_scale_yaw", 1.0);
    this->declare_parameter<double>("speed_scale_z", 1.0);

    this->declare_parameter<double>("align_enter_error", 0.026);
    this->declare_parameter<double>("align_exit_error", 0.018);
    this->declare_parameter<double>("align_enter_yaw", 0.10);
    this->declare_parameter<double>("align_exit_yaw", 0.07);
    this->declare_parameter<double>("align_hold_s", 0.8);
    this->declare_parameter<double>("align_z_kp", 0.80);
    this->declare_parameter<double>("align_z_max_vz", 0.08);
    this->declare_parameter<double>("align_z_deadband_m", 0.02);

    this->declare_parameter<double>("alpha_xy", 0.20);
    this->declare_parameter<double>("max_acc_xy", 0.30);
    this->declare_parameter<double>("max_acc_yaw", 0.80);
    this->declare_parameter<double>("vel_damping_xy", 0.30);
    this->declare_parameter<double>("vel_damping_yaw", 0.22);
    this->declare_parameter<double>("min_height_for_xy", 0.25);
    this->declare_parameter<double>("yaw_freeze_threshold", 0.45);
    this->declare_parameter<double>("yaw_slow_threshold", 0.25);
    this->declare_parameter<double>("yaw_slow_scale", 0.30);
    this->declare_parameter<double>("yaw_cmd_deadband", 0.03);

    this->declare_parameter<double>("yaw_filter_alpha", 0.35);
    this->declare_parameter<double>("yaw_filter_beta", 0.08);
    this->declare_parameter<double>("yaw_innovation_base_rad", 0.20);
    this->declare_parameter<double>("yaw_innovation_conf_scale", 0.15);
    this->declare_parameter<double>("yaw_rate_state_limit", 1.2);

    this->declare_parameter<double>("descend_speed", 0.12);
    this->declare_parameter<double>("land_height_m", 0.12);
    this->declare_parameter<double>("lost_enter_s", 0.45);
    this->declare_parameter<double>("reacquire_confirm_s", 0.20);
    this->declare_parameter<double>("search_rise_step_m", 0.20);
    this->declare_parameter<double>("search_rise_speed_mps", 0.16);
    this->declare_parameter<double>("search_max_rise_m", 1.0);
    this->declare_parameter<double>("search_timeout_s", 6.0);
    this->declare_parameter<std::string>("height_distance_topic", "/uav/fmu/in/distance_sensor");
    this->declare_parameter<std::string>("optical_flow_topic", "/uav/fmu/in/sensor_optical_flow");
    this->declare_parameter<std::string>("vehicle_status_topic", "/uav/fmu/out/vehicle_status");
    this->declare_parameter<double>("height_source_timeout_s", 0.15);
    this->declare_parameter<double>("height_range_min_m", 0.05);
    this->declare_parameter<double>("height_range_max_m", 5.0);
    this->declare_parameter<double>("height_consistency_max_diff_m", 0.35);
    this->declare_parameter<double>("flow_quality_timeout_s", 0.20);
    this->declare_parameter<int>("flow_quality_min_for_land", 30);
    this->declare_parameter<double>("terminal_entry_height_m", 0.30);
    this->declare_parameter<double>("direct_land_max_vxy_mps", 0.10);
    this->declare_parameter<double>("direct_land_recent_seen_s", 0.30);
    this->declare_parameter<double>("descend_xy_slow_height_m", 0.25);
    this->declare_parameter<double>("descend_xy_min_scale", 0.20);
    this->declare_parameter<double>("control_rate_hz", 30.0);

    kp_xy_ = static_cast<float>(this->get_parameter("kp_xy").as_double());
    kp_yaw_ = static_cast<float>(this->get_parameter("kp_yaw").as_double());
    max_vxy_ = static_cast<float>(this->get_parameter("max_vxy").as_double());
    max_vyaw_ = static_cast<float>(this->get_parameter("max_vyaw").as_double());
    max_error_norm_ = static_cast<float>(this->get_parameter("max_error_norm").as_double());
    speed_scale_xy_ = static_cast<float>(this->get_parameter("speed_scale_xy").as_double());
    speed_scale_yaw_ = static_cast<float>(this->get_parameter("speed_scale_yaw").as_double());
    speed_scale_z_ = static_cast<float>(this->get_parameter("speed_scale_z").as_double());

    align_enter_error_ = static_cast<float>(this->get_parameter("align_enter_error").as_double());
    align_exit_error_ = static_cast<float>(this->get_parameter("align_exit_error").as_double());
    align_enter_yaw_ = static_cast<float>(this->get_parameter("align_enter_yaw").as_double());
    align_exit_yaw_ = static_cast<float>(this->get_parameter("align_exit_yaw").as_double());
    align_hold_s_ = this->get_parameter("align_hold_s").as_double();
    align_z_kp_ = static_cast<float>(this->get_parameter("align_z_kp").as_double());
    align_z_max_vz_ = static_cast<float>(this->get_parameter("align_z_max_vz").as_double());
    align_z_deadband_m_ = static_cast<float>(this->get_parameter("align_z_deadband_m").as_double());

    alpha_xy_ = clamp01(static_cast<float>(this->get_parameter("alpha_xy").as_double()));
    max_acc_xy_ = static_cast<float>(this->get_parameter("max_acc_xy").as_double());
    max_acc_yaw_ = static_cast<float>(this->get_parameter("max_acc_yaw").as_double());
    vel_damping_xy_ = static_cast<float>(this->get_parameter("vel_damping_xy").as_double());
    vel_damping_yaw_ = static_cast<float>(this->get_parameter("vel_damping_yaw").as_double());
    min_height_for_xy_ = static_cast<float>(this->get_parameter("min_height_for_xy").as_double());

    yaw_freeze_threshold_ = static_cast<float>(this->get_parameter("yaw_freeze_threshold").as_double());
    yaw_slow_threshold_ = static_cast<float>(this->get_parameter("yaw_slow_threshold").as_double());
    yaw_slow_scale_ = clamp01(static_cast<float>(this->get_parameter("yaw_slow_scale").as_double()));
    yaw_cmd_deadband_ = static_cast<float>(this->get_parameter("yaw_cmd_deadband").as_double());

    yaw_filter_alpha_ = clamp01(static_cast<float>(this->get_parameter("yaw_filter_alpha").as_double()));
    yaw_filter_beta_ = std::max(0.0f, static_cast<float>(this->get_parameter("yaw_filter_beta").as_double()));
    yaw_innovation_base_rad_ = std::max(1.0e-3f, static_cast<float>(this->get_parameter("yaw_innovation_base_rad").as_double()));
    yaw_innovation_conf_scale_ = std::max(0.0f, static_cast<float>(this->get_parameter("yaw_innovation_conf_scale").as_double()));
    yaw_rate_state_limit_ = static_cast<float>(this->get_parameter("yaw_rate_state_limit").as_double());

    descend_speed_ = static_cast<float>(this->get_parameter("descend_speed").as_double());
    land_height_m_ = static_cast<float>(this->get_parameter("land_height_m").as_double());
    lost_enter_s_ = this->get_parameter("lost_enter_s").as_double();
    reacquire_confirm_s_ = this->get_parameter("reacquire_confirm_s").as_double();
    search_rise_step_m_ = static_cast<float>(this->get_parameter("search_rise_step_m").as_double());
    search_rise_speed_mps_ = static_cast<float>(this->get_parameter("search_rise_speed_mps").as_double());
    search_max_rise_m_ = static_cast<float>(this->get_parameter("search_max_rise_m").as_double());
    search_timeout_s_ = this->get_parameter("search_timeout_s").as_double();
    height_source_timeout_s_ = this->get_parameter("height_source_timeout_s").as_double();
    height_range_min_m_ = static_cast<float>(this->get_parameter("height_range_min_m").as_double());
    height_range_max_m_ = static_cast<float>(this->get_parameter("height_range_max_m").as_double());
    height_consistency_max_diff_m_ = static_cast<float>(
      this->get_parameter("height_consistency_max_diff_m").as_double());
    flow_quality_timeout_s_ = this->get_parameter("flow_quality_timeout_s").as_double();
    flow_quality_min_for_land_ = this->get_parameter("flow_quality_min_for_land").as_int();
    terminal_entry_height_m_ = static_cast<float>(this->get_parameter("terminal_entry_height_m").as_double());
    direct_land_max_vxy_mps_ = static_cast<float>(this->get_parameter("direct_land_max_vxy_mps").as_double());
    direct_land_recent_seen_s_ = this->get_parameter("direct_land_recent_seen_s").as_double();
    descend_xy_slow_height_m_ = static_cast<float>(this->get_parameter("descend_xy_slow_height_m").as_double());
    descend_xy_min_scale_ = static_cast<float>(this->get_parameter("descend_xy_min_scale").as_double());

    max_error_norm_ = std::max(0.0f, max_error_norm_);
    align_enter_error_ = std::max(0.0f, align_enter_error_);
    align_exit_error_ = std::max(0.0f, std::min(align_exit_error_, align_enter_error_));
    align_enter_yaw_ = std::max(0.0f, align_enter_yaw_);
    align_exit_yaw_ = std::max(0.0f, std::min(align_exit_yaw_, align_enter_yaw_));
    align_z_kp_ = std::max(0.0f, align_z_kp_);
    align_z_max_vz_ = std::max(0.0f, align_z_max_vz_);
    align_z_deadband_m_ = std::max(0.0f, align_z_deadband_m_);

    max_vxy_ = std::max(0.0f, max_vxy_);
    max_vyaw_ = std::max(0.0f, max_vyaw_);
    max_acc_xy_ = std::max(0.0f, max_acc_xy_);
    max_acc_yaw_ = std::max(0.0f, max_acc_yaw_);
    speed_scale_xy_ = std::max(0.0f, speed_scale_xy_);
    speed_scale_yaw_ = std::max(0.0f, speed_scale_yaw_);
    speed_scale_z_ = std::max(0.0f, speed_scale_z_);
    vel_damping_xy_ = std::max(0.0f, vel_damping_xy_);
    vel_damping_yaw_ = std::max(0.0f, vel_damping_yaw_);
    min_height_for_xy_ = std::max(0.0f, min_height_for_xy_);

    yaw_slow_threshold_ = std::max(0.0f, std::min(yaw_slow_threshold_, yaw_freeze_threshold_));
    yaw_cmd_deadband_ = std::max(0.0f, yaw_cmd_deadband_);
    yaw_rate_state_limit_ = std::max(max_vyaw_, std::max(0.0f, yaw_rate_state_limit_));

    lost_enter_s_ = std::max(0.01, lost_enter_s_);
    reacquire_confirm_s_ = std::max(0.0, reacquire_confirm_s_);
    search_rise_step_m_ = std::max(0.0f, search_rise_step_m_);
    search_rise_speed_mps_ = std::max(0.0f, search_rise_speed_mps_);
    search_max_rise_m_ = std::max(0.0f, search_max_rise_m_);
    search_timeout_s_ = std::max(0.0, search_timeout_s_);
    height_source_timeout_s_ = std::max(0.01, height_source_timeout_s_);
    height_range_min_m_ = std::max(0.0f, height_range_min_m_);
    height_range_max_m_ = std::max(height_range_min_m_ + 1.0e-3f, height_range_max_m_);
    height_consistency_max_diff_m_ = std::max(0.0f, height_consistency_max_diff_m_);
    flow_quality_timeout_s_ = std::max(0.01, flow_quality_timeout_s_);
    flow_quality_min_for_land_ = std::max(0, std::min(flow_quality_min_for_land_, 255));
    terminal_entry_height_m_ = std::max(0.0f, terminal_entry_height_m_);
    direct_land_max_vxy_mps_ = std::max(0.0f, direct_land_max_vxy_mps_);
    direct_land_recent_seen_s_ = std::max(0.0, direct_land_recent_seen_s_);
    descend_xy_slow_height_m_ = std::max(0.0f, descend_xy_slow_height_m_);
    descend_xy_min_scale_ = clamp01(descend_xy_min_scale_);

    const bool auto_start = this->get_parameter("auto_start").as_bool();
    const std::string vel_topic = this->get_parameter("velocity_topic").as_string();
    const std::string land_srv = this->get_parameter("land_service").as_string();
    const std::string height_distance_topic = this->get_parameter("height_distance_topic").as_string();
    const std::string optical_flow_topic = this->get_parameter("optical_flow_topic").as_string();
    const std::string vehicle_status_topic = this->get_parameter("vehicle_status_topic").as_string();
    const double status_rate_hz = this->get_parameter("status_rate_hz").as_double();
    status_period_s_ = status_rate_hz > 1.0e-6 ? (1.0 / status_rate_hz) : 0.2;
    constexpr const char * kStartService = "/uav/visual_landing/start";
    constexpr const char * kStopService = "/uav/visual_landing/stop";

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10);
    active_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("active_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = this->create_publisher<uav_visual_landing::msg::LandingStatus>(
      this->get_parameter("status_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    last_seen_ = this->now();

    landing_error_sub_ = this->create_subscription<uav_visual_landing::msg::LandingError>(
      this->get_parameter("landing_error_topic").as_string(), 10,
      [this](const uav_visual_landing::msg::LandingError::SharedPtr msg)
      {
        tag_detected_ = msg->detected;
        if (!msg->detected) {
          return;
        }

        if (!std::isfinite(msg->err_u_norm) ||
            !std::isfinite(msg->err_v_norm) ||
            !std::isfinite(msg->yaw_err_rad))
        {
          tag_detected_ = false;
          return;
        }

        last_err_u_ = msg->err_u_norm;
        last_err_v_ = msg->err_v_norm;
        const float yaw_err = normalizeAngle(msg->yaw_err_rad);
        last_detection_confidence_ = clamp01(msg->confidence);
        updateFilteredErrors(last_err_u_, last_err_v_, yaw_err, last_detection_confidence_);
        last_seen_ = this->now();
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        current_height_ = static_cast<float>(msg->pose.pose.position.z);
        current_yaw_ = quaternionToYaw(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        const float vbx = static_cast<float>(msg->twist.twist.linear.x);
        const float vby = static_cast<float>(msg->twist.twist.linear.y);
        const float cy = std::cos(current_yaw_);
        const float sy = std::sin(current_yaw_);
        current_vx_world_ = cy * vbx - sy * vby;
        current_vy_world_ = sy * vbx + cy * vby;

        current_yaw_rate_ = static_cast<float>(msg->twist.twist.angular.z);
        has_odom_ = true;
      });
    distance_sub_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(
      height_distance_topic,
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::DistanceSensor::SharedPtr msg)
      {
        if (!std::isfinite(msg->current_distance)) {
          return;
        }
        range_height_m_ = msg->current_distance;
        range_signal_quality_ = msg->signal_quality;
        range_height_stamp_ = this->now();
        has_range_height_ = true;
      });
    optical_flow_sub_ = this->create_subscription<px4_msgs::msg::SensorOpticalFlow>(
      optical_flow_topic,
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::SensorOpticalFlow::SharedPtr msg)
      {
        flow_quality_ = msg->quality;
        flow_quality_stamp_ = this->now();
        has_flow_quality_ = true;
      });
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic,
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
      {
        vehicle_armed_ =
          msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        has_vehicle_status_ = true;

        if (active_ && state_ == State::DONE && !vehicle_armed_) {
          RCLCPP_INFO(
            this->get_logger(),
            "[visual_landing] landing completed and vehicle disarmed, returning to READY");
          deactivateToReady();
        }
      });

    land_client_ = this->create_client<Trigger>(land_srv);

    start_srv_ = this->create_service<Trigger>(
      kStartService,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = start(response->message);
      });

    stop_srv_ = this->create_service<Trigger>(
      kStopService,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = stop(response->message);
      });

    const double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    control_dt_s_ = control_rate_hz > 1.0e-6 ? 1.0 / control_rate_hz : (1.0 / 30.0);
    const double period_ms = 1000.0 * control_dt_s_;
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(period_ms)),
      std::bind(&VisualLandingNode::timerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "[visual_landing] Ready. kp_xy=%.2f kp_yaw=%.2f vxy<=%.2f vyaw<=%.2f "
      "speed_scale=(xy=%.2f yaw=%.2f z=%.2f) "
      "align_enter=(%.3f,%.3frad) align_exit=(%.3f,%.3frad) hold=%.2fs z_hold=(kp=%.2f vmax=%.2f db=%.2f) "
      "filter=(alpha_xy=%.2f yaw_alpha=%.2f yaw_beta=%.2f) "
      "yaw_gate=(base=%.2f conf_scale=%.2f state_limit=%.2f) "
      "acc=(xy=%.2f yaw=%.2f) damp=(xy=%.2f yaw=%.2f) "
      "yaw_xy=(slow=%.2f freeze=%.2f scale=%.2f deadband=%.2f) "
      "lost=(enter=%.2fs reacquire=%.2fs) descend=(v=%.2fmps land_h=%.2fm) "
      "search=(step=%.2fm speed=%.2fmps max=%.2fm timeout=%.1fs) "
      "height=(timeout=%.2fs range=[%.2f,%.2f] diff<=%.2f) "
      "flow=(q>=%d timeout=%.2fs) "
      "terminal=(entry_h=%.2f max_vxy=%.2f seen<=%.2fs) "
      "descend_xy=(slow_h=%.2f min_scale=%.2f)",
      kp_xy_, kp_yaw_, max_vxy_, max_vyaw_,
      speed_scale_xy_, speed_scale_yaw_, speed_scale_z_,
      align_enter_error_, align_enter_yaw_, align_exit_error_, align_exit_yaw_, align_hold_s_,
      align_z_kp_, align_z_max_vz_, align_z_deadband_m_,
      alpha_xy_, yaw_filter_alpha_, yaw_filter_beta_,
      yaw_innovation_base_rad_, yaw_innovation_conf_scale_, yaw_rate_state_limit_,
      max_acc_xy_, max_acc_yaw_, vel_damping_xy_, vel_damping_yaw_,
      yaw_slow_threshold_, yaw_freeze_threshold_, yaw_slow_scale_, yaw_cmd_deadband_,
      lost_enter_s_, reacquire_confirm_s_, descend_speed_, land_height_m_,
      search_rise_step_m_, search_rise_speed_mps_, search_max_rise_m_, search_timeout_s_,
      height_source_timeout_s_, height_range_min_m_, height_range_max_m_, height_consistency_max_diff_m_,
      flow_quality_min_for_land_, flow_quality_timeout_s_,
      terminal_entry_height_m_, direct_land_max_vxy_mps_,
      direct_land_recent_seen_s_, descend_xy_slow_height_m_, descend_xy_min_scale_);

    if (auto_start) {
      std::string start_msg;
      start(start_msg);
      RCLCPP_INFO(
        this->get_logger(),
        "[visual_landing] auto_start enabled, controller entered landing chain via %s",
        kStartService);
    } else {
      publishActiveStatus();
      publishLandingStatus(true);
      RCLCPP_INFO(
        this->get_logger(),
        "[visual_landing] standby on startup, call %s to enter landing chain",
        kStartService);
    }
  }

  bool start(std::string & message)
  {
    if (active_) {
      message = "visual landing controller is already active";
      publishActiveStatus();
      publishLandingStatus(true);
      return true;
    }

    active_ = true;
    land_service_called_ = false;
    last_seen_ = this->now();
    landing_start_height_initialized_ = false;
    clearAlignmentTracking();
    resetReacquireCandidate();
    transitionTo(State::IDLE);
    message = "visual landing controller started and waiting for target";
    publishActiveStatus();
    publishLandingStatus(true);
    return true;
  }

  bool stop(std::string & message)
  {
    if (!active_) {
      message = "visual landing controller is already inactive";
      publishActiveStatus();
      publishLandingStatus(true);
      return true;
    }

    deactivateToReady();
    message = "visual landing controller stopped and returned to standby";
    return true;
  }

private:
  struct HeightEstimate
  {
    float effective_m{0.0f};
    bool source_valid{false};
    bool using_range{false};
    bool range_available{false};
    bool range_fresh{false};
    bool range_in_bounds{false};
    bool range_consistent{false};
    float range_m{0.0f};
    float range_age_s{-1.0f};
    int8_t range_signal_quality{-1};
  };

  static const char * stateName(State s)
  {
    const char * names[] = {
      "READY", "IDLE", "ALIGN", "SEARCH_RISE",
      "DESCEND", "LAND", "DONE"};
    return names[static_cast<int>(s)];
  }

  void publishActiveStatus()
  {
    std_msgs::msg::Bool msg;
    msg.data = active_;
    active_pub_->publish(msg);
  }

  void publishLandingStatus(bool force)
  {
    const rclcpp::Time now = this->now();
    if (!force && status_publish_initialized_) {
      if (status_period_s_ > 0.0 && (now - last_status_publish_time_).seconds() < status_period_s_) {
        return;
      }
    }

    uav_visual_landing::msg::LandingStatus msg;
    const HeightEstimate height_est = getHeightEstimate();
    msg.header.stamp = now;
    msg.active = active_;
    msg.tag_detected = tag_detected_;
    msg.state = stateName(state_);
    msg.current_height_m = height_est.effective_m;
    msg.lost_duration_s = static_cast<float>(std::max(0.0, (now - last_seen_).seconds()));
    msg.err_u_norm_raw = last_err_u_;
    msg.err_v_norm_raw = last_err_v_;
    msg.err_u_norm_filtered = err_u_f_;
    msg.err_v_norm_filtered = err_v_f_;
    msg.yaw_err_rad_filtered = err_yaw_f_;
    msg.detection_confidence = last_detection_confidence_;
    msg.range_available = height_est.range_available;
    msg.range_fresh = height_est.range_fresh;
    msg.range_in_bounds = height_est.range_in_bounds;
    msg.range_consistent = height_est.range_consistent;
    msg.height_using_range = height_est.using_range;
    msg.range_height_m = height_est.range_m;
    msg.range_age_s = height_est.range_age_s;
    msg.range_signal_quality = height_est.range_signal_quality;
    status_pub_->publish(msg);

    last_status_publish_time_ = now;
    status_publish_initialized_ = true;
  }

  static float clamp(float v, float limit)
  {
    return v > limit ? limit : (v < -limit ? -limit : v);
  }

  static float clamp01(float v)
  {
    if (v < 0.0f) {
      return 0.0f;
    }
    if (v > 1.0f) {
      return 1.0f;
    }
    return v;
  }

  static float clampDelta(float target, float previous, float max_delta)
  {
    if (max_delta <= 0.0f) {
      return target;
    }
    const float delta = target - previous;
    if (delta > max_delta) {
      return previous + max_delta;
    }
    if (delta < -max_delta) {
      return previous - max_delta;
    }
    return target;
  }

  static float normalizeAngle(float a)
  {
    while (a > static_cast<float>(M_PI)) {
      a -= 2.0f * static_cast<float>(M_PI);
    }
    while (a < -static_cast<float>(M_PI)) {
      a += 2.0f * static_cast<float>(M_PI);
    }
    return a;
  }

  void transitionTo(State s)
  {
    state_ = s;
    align_height_ref_initialized_ = false;
    if (s != State::ALIGN) {
      align_hold_timer_started_ = false;
      align_in_window_ = false;
      terminal_align_pending_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "[visual_landing] state -> %s", stateName(s));
    publishLandingStatus(true);
  }

  void resetReacquireCandidate()
  {
    reacquire_candidate_started_ = false;
  }

  void deactivateToReady()
  {
    active_ = false;
    land_service_called_ = false;
    clearAlignmentTracking();
    resetReacquireCandidate();
    transitionTo(State::READY);
    publishActiveStatus();
    publishLandingStatus(true);
  }

  bool targetLost() const
  {
    const double dt = (this->now() - last_seen_).seconds();
    return dt > lost_enter_s_;
  }

  HeightEstimate getHeightEstimate()
  {
    HeightEstimate out;
    out.effective_m = current_height_;
    out.source_valid = std::isfinite(current_height_);
    out.range_available = has_range_height_;
    out.range_signal_quality = range_signal_quality_;
    out.range_m = range_height_m_;

    if (!has_range_height_) {
      return out;
    }

    const double range_age_s = (this->now() - range_height_stamp_).seconds();
    out.range_age_s = static_cast<float>(std::max(0.0, range_age_s));
    const bool range_fresh = range_age_s >= 0.0 && range_age_s <= height_source_timeout_s_;
    const bool range_in_bounds = std::isfinite(range_height_m_) &&
      range_height_m_ >= height_range_min_m_ &&
      range_height_m_ <= height_range_max_m_;
    out.range_fresh = range_fresh;
    out.range_in_bounds = range_in_bounds;

    bool range_consistent = true;
    if (std::isfinite(current_height_)) {
      range_consistent = std::abs(range_height_m_ - current_height_) <= height_consistency_max_diff_m_;
      if (!range_consistent && range_fresh && range_in_bounds) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[visual_landing] ignore range height mismatch range=%.2f odom=%.2f diff=%.2f > %.2f",
          range_height_m_, current_height_, std::abs(range_height_m_ - current_height_),
          height_consistency_max_diff_m_);
      }
    }
    out.range_consistent = range_consistent;

    if (!range_fresh || !range_in_bounds || !range_consistent) {
      return out;
    }

    out.effective_m = range_height_m_;
    out.source_valid = true;
    out.using_range = true;
    return out;
  }

  float computeDescendXYScale(float effective_height) const
  {
    if (descend_xy_slow_height_m_ <= 1.0e-3f) {
      return 1.0f;
    }
    if (effective_height >= descend_xy_slow_height_m_) {
      return 1.0f;
    }

    const float ratio = std::max(0.0f, effective_height) / descend_xy_slow_height_m_;
    return std::max(descend_xy_min_scale_, ratio);
  }

  bool reacquireConfirmed()
  {
    if (!tag_detected_ || targetLost()) {
      resetReacquireCandidate();
      return false;
    }

    if (!reacquire_candidate_started_) {
      reacquire_candidate_since_ = this->now();
      reacquire_candidate_started_ = true;
      return reacquire_confirm_s_ <= 1.0e-6;
    }

    const double held = (this->now() - reacquire_candidate_since_).seconds();
    return held >= reacquire_confirm_s_;
  }

  void beginSearchRise()
  {
    clearAlignmentTracking();
    resetReacquireCandidate();
    const HeightEstimate height_est = getHeightEstimate();
    const float search_ref_height = height_est.source_valid ? height_est.effective_m : current_height_;

    if (!landing_start_height_initialized_) {
      landing_start_height_ = search_ref_height;
      landing_start_height_initialized_ = true;
    }

    const float search_ceiling = landing_start_height_ + search_max_rise_m_;
    search_target_height_ = std::min(search_ref_height + search_rise_step_m_, search_ceiling);
    search_started_time_ = this->now();

    RCLCPP_WARN(
      this->get_logger(),
      "[visual_landing] tag lost in ALIGN, search rise h=%.2f -> %.2f",
      search_ref_height, search_target_height_);
    transitionTo(State::SEARCH_RISE);
  }

  float effectiveKpXY() const
  {
    return kp_xy_ * speed_scale_xy_;
  }

  float effectiveMaxVXY() const
  {
    return max_vxy_ * speed_scale_xy_;
  }

  float effectiveMaxAccXY() const
  {
    return max_acc_xy_ * speed_scale_xy_;
  }

  float effectiveKpYaw() const
  {
    return kp_yaw_ * speed_scale_yaw_;
  }

  float effectiveMaxVYaw() const
  {
    return max_vyaw_ * speed_scale_yaw_;
  }

  float effectiveMaxAccYaw() const
  {
    return max_acc_yaw_ * speed_scale_yaw_;
  }

  float effectiveYawRateStateLimit() const
  {
    return std::max(effectiveMaxVYaw(), yaw_rate_state_limit_ * speed_scale_yaw_);
  }

  float effectiveDescendSpeed() const
  {
    return descend_speed_ * speed_scale_z_;
  }

  float effectiveAlignZKp() const
  {
    return align_z_kp_ * speed_scale_z_;
  }

  float effectiveAlignMaxVz() const
  {
    return align_z_max_vz_ * speed_scale_z_;
  }

  float effectiveSearchRiseSpeed() const
  {
    return search_rise_speed_mps_ * speed_scale_z_;
  }

  float computeAlignVzCommand(float effective_height)
  {
    if (!std::isfinite(effective_height)) {
      return 0.0f;
    }

    if (!align_height_ref_initialized_) {
      align_height_ref_m_ = effective_height;
      align_height_ref_initialized_ = true;
      return 0.0f;
    }

    const float err_h = align_height_ref_m_ - effective_height;
    if (std::abs(err_h) < align_z_deadband_m_) {
      return 0.0f;
    }

    return clamp(effectiveAlignZKp() * err_h, effectiveAlignMaxVz());
  }

  float computeYawRateCommand() const
  {
    const float effective_limit = effectiveMaxVYaw();
    if (effective_limit <= 0.0f) {
      return 0.0f;
    }

    const float yaw_abs = std::abs(err_yaw_f_);
    if (yaw_abs < yaw_cmd_deadband_) {
      return 0.0f;
    }

    const float yaw_cmd = (-effectiveKpYaw() * err_yaw_f_) - (vel_damping_yaw_ * current_yaw_rate_);
    return clamp(yaw_cmd, effective_limit);
  }

  void publishVelocityRaw(float vx, float vy, float vz, float yaw_rate)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = static_cast<double>(vx);
    t.linear.y = static_cast<double>(vy);
    t.linear.z = static_cast<double>(vz);
    t.angular.z = static_cast<double>(yaw_rate);
    vel_pub_->publish(t);
  }

  void applyCommandRateLimit(float &vx, float &vy, float &yaw_rate)
  {
    if (!cmd_rate_limit_initialized_) {
      last_cmd_vx_ = vx;
      last_cmd_vy_ = vy;
      last_cmd_yaw_rate_ = yaw_rate;
      cmd_rate_limit_initialized_ = true;
      return;
    }

    const float max_acc_xy = effectiveMaxAccXY();
    const float max_dxy = max_acc_xy > 0.0f ? (max_acc_xy * control_dt_s_) : 0.0f;
    if (max_dxy > 0.0f) {
      const float dvx = vx - last_cmd_vx_;
      const float dvy = vy - last_cmd_vy_;
      const float dxy = std::sqrt((dvx * dvx) + (dvy * dvy));
      if (dxy > max_dxy && dxy > 1.0e-6f) {
        const float scale = max_dxy / dxy;
        vx = last_cmd_vx_ + (dvx * scale);
        vy = last_cmd_vy_ + (dvy * scale);
      }
    }

    const float max_acc_yaw = effectiveMaxAccYaw();
    const float max_dyaw = max_acc_yaw > 0.0f ? (max_acc_yaw * control_dt_s_) : 0.0f;
    yaw_rate = clampDelta(yaw_rate, last_cmd_yaw_rate_, max_dyaw);
  }

  void publishVelocityLimited(float vx, float vy, float vz, float yaw_rate)
  {
    applyCommandRateLimit(vx, vy, yaw_rate);
    publishVelocityRaw(vx, vy, vz, yaw_rate);

    last_cmd_vx_ = vx;
    last_cmd_vy_ = vy;
    last_cmd_yaw_rate_ = yaw_rate;
    cmd_rate_limit_initialized_ = true;
  }

  void publishHover()
  {
    publishVelocityRaw(0.0f, 0.0f, 0.0f, 0.0f);
    last_cmd_vx_ = 0.0f;
    last_cmd_vy_ = 0.0f;
    last_cmd_yaw_rate_ = 0.0f;
    cmd_rate_limit_initialized_ = true;
  }

  void clearAlignmentTracking()
  {
    align_hold_timer_started_ = false;
    align_in_window_ = false;
    has_filtered_error_ = false;
    yaw_error_rate_f_ = 0.0f;
    last_detection_confidence_ = 0.0f;
  }

  void callLandService()
  {
    if (!land_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "[visual_landing] land service not ready, retrying");
      return;
    }

    auto req = std::make_shared<Trigger::Request>();
    land_client_->async_send_request(
      req,
      [this](rclcpp::Client<Trigger>::SharedFuture fut)
      {
        auto res = fut.get();
        if (res->success) {
          RCLCPP_INFO(this->get_logger(), "[visual_landing] land service called: %s", res->message.c_str());
          transitionTo(State::DONE);
        } else {
          RCLCPP_WARN(this->get_logger(), "[visual_landing] land service failed: %s", res->message.c_str());
        }
      });
  }

  void updateFilteredErrors(float err_u, float err_v, float err_yaw, float confidence)
  {
    const float conf = clamp01(confidence);

    if (!has_filtered_error_) {
      err_u_f_ = err_u;
      err_v_f_ = err_v;
      err_yaw_f_ = err_yaw;
      yaw_error_rate_f_ = 0.0f;
      has_filtered_error_ = true;
      return;
    }

    err_u_f_ = alpha_xy_ * err_u + (1.0f - alpha_xy_) * err_u_f_;
    err_v_f_ = alpha_xy_ * err_v + (1.0f - alpha_xy_) * err_v_f_;

    const float e_pred = normalizeAngle(err_yaw_f_ + (yaw_error_rate_f_ * control_dt_s_));
    const float nu = normalizeAngle(err_yaw - e_pred);
    const float nu_max = yaw_innovation_base_rad_ + (yaw_innovation_conf_scale_ * (1.0f - conf));

    const float yaw_rate_state_limit = effectiveYawRateStateLimit();

    if (std::abs(nu) > nu_max) {
      err_yaw_f_ = e_pred;
      yaw_error_rate_f_ = clamp(yaw_error_rate_f_ * 0.95f, yaw_rate_state_limit);
      return;
    }

    err_yaw_f_ = normalizeAngle(e_pred + (yaw_filter_alpha_ * nu));
    const float dt_safe = std::max(control_dt_s_, 1.0e-3f);
    yaw_error_rate_f_ = clamp(yaw_error_rate_f_ + (yaw_filter_beta_ * (nu / dt_safe)), yaw_rate_state_limit);
  }

  void computeIBVS(float &vx_out, float &vy_out, float &vyaw_out)
  {
    const float yaw_abs = std::abs(err_yaw_f_);
    float yaw_xy_scale = 1.0f;
    if (yaw_abs > yaw_freeze_threshold_) {
      yaw_xy_scale = 0.0f;
    } else if (yaw_abs > yaw_slow_threshold_) {
      yaw_xy_scale = yaw_slow_scale_;
    }

    const float err_u = clamp(err_u_f_, max_error_norm_);
    const float err_v = clamp(err_v_f_, max_error_norm_);
    const HeightEstimate height_est = getHeightEstimate();
    const float ctrl_height = height_est.source_valid ? height_est.effective_m : current_height_;
    const float height_for_xy = std::max(ctrl_height, min_height_for_xy_);
    const float kp_xy = effectiveKpXY();
    const float max_vxy = effectiveMaxVXY();

    const float vx_body = clamp(yaw_xy_scale * (-kp_xy * err_v * height_for_xy), max_vxy);
    const float vy_body = clamp(yaw_xy_scale * (-kp_xy * err_u * height_for_xy), max_vxy);

    const float cos_yaw = std::cos(current_yaw_);
    const float sin_yaw = std::sin(current_yaw_);
    vx_out = (cos_yaw * vx_body) - (sin_yaw * vy_body);
    vy_out = (sin_yaw * vx_body) + (cos_yaw * vy_body);

    vx_out -= vel_damping_xy_ * current_vx_world_;
    vy_out -= vel_damping_xy_ * current_vy_world_;

    const float vxy_norm = std::sqrt((vx_out * vx_out) + (vy_out * vy_out));
    if (vxy_norm > max_vxy && vxy_norm > 1.0e-6f) {
      const float scale = max_vxy / vxy_norm;
      vx_out *= scale;
      vy_out *= scale;
    }

    vyaw_out = computeYawRateCommand();
  }

  bool isAligned()
  {
    if (!has_filtered_error_) {
      align_in_window_ = false;
      return false;
    }

    const float e = std::sqrt((err_u_f_ * err_u_f_) + (err_v_f_ * err_v_f_));
    const float yaw_abs = std::abs(err_yaw_f_);

    if (align_in_window_) {
      if (e > align_exit_error_ || yaw_abs > align_exit_yaw_) {
        align_in_window_ = false;
      }
    } else {
      if (e < align_enter_error_ && yaw_abs < align_enter_yaw_) {
        align_in_window_ = true;
      }
    }

    return align_in_window_;
  }

  void timerCallback()
  {
    if (!has_odom_ || !active_) {
      publishLandingStatus(false);
      return;
    }

    const HeightEstimate height_est = getHeightEstimate();
    const float effective_height = height_est.source_valid ? height_est.effective_m : current_height_;

    if (!landing_start_height_initialized_) {
      landing_start_height_ = effective_height;
      landing_start_height_initialized_ = true;
    }

    switch (state_) {
      case State::READY:
        publishHover();
        break;

      case State::IDLE:
        publishHover();
        if (reacquireConfirmed()) {
          transitionTo(State::ALIGN);
        }
        break;

      case State::ALIGN:
      {
        if (targetLost()) {
          beginSearchRise();
          break;
        }

        resetReacquireCandidate();
        const float vz_align = computeAlignVzCommand(effective_height);
        if (!has_filtered_error_) {
          publishVelocityLimited(0.0f, 0.0f, vz_align, 0.0f);
          break;
        }

        float vx = 0.0f;
        float vy = 0.0f;
        float vyaw = 0.0f;
        computeIBVS(vx, vy, vyaw);
        publishVelocityLimited(vx, vy, vz_align, vyaw);

        if (isAligned()) {
          if (align_hold_timer_started_) {
            const double held = (this->now() - align_stable_since_).seconds();
            if (held >= align_hold_s_) {
              if (terminal_align_pending_) {
                RCLCPP_INFO(
                  this->get_logger(),
                  "[visual_landing] terminal ALIGN satisfied, switching to LAND");
                transitionTo(State::LAND);
              } else {
                transitionTo(State::DESCEND);
              }
            }
          } else {
            align_stable_since_ = this->now();
            align_hold_timer_started_ = true;
          }
        } else {
          align_hold_timer_started_ = false;
        }
        break;
      }

      case State::SEARCH_RISE:
      {
        if (reacquireConfirmed()) {
          transitionTo(State::ALIGN);
          publishHover();
          break;
        }

        if (tag_detected_ && !targetLost()) {
          publishHover();
          break;
        }

        const double search_elapsed = (this->now() - search_started_time_).seconds();
        if (search_elapsed >= search_timeout_s_) {
          RCLCPP_WARN(this->get_logger(), "[visual_landing] search timeout %.1fs, entering IDLE hover", search_timeout_s_);
          transitionTo(State::IDLE);
          publishHover();
          break;
        }

        const float height_margin = search_target_height_ - effective_height;
        float vz = 0.0f;
        if (height_margin > 0.03f) {
          vz = effectiveSearchRiseSpeed();
        }
        publishVelocityLimited(0.0f, 0.0f, vz, 0.0f);
        break;
      }

      case State::DESCEND:
      {
        if (height_est.source_valid && effective_height <= terminal_entry_height_m_) {
          terminal_align_pending_ = true;
          RCLCPP_INFO(
            this->get_logger(),
            "[visual_landing] terminal height reached, switching to final ALIGN h=%.2f<=%.2f",
            effective_height, terminal_entry_height_m_);
          transitionTo(State::ALIGN);
          break;
        }

        if (targetLost()) {
          transitionTo(State::ALIGN);
          publishHover();
          RCLCPP_WARN(
            this->get_logger(),
            "[visual_landing] tag lost during DESCEND, back to ALIGN h=%.2f",
            effective_height);
          break;
        }

        resetReacquireCandidate();
        if (!has_filtered_error_) {
          publishHover();
          break;
        }

        float vx = 0.0f;
        float vy = 0.0f;
        float vyaw = 0.0f;
        computeIBVS(vx, vy, vyaw);
        const float descend_xy_scale = computeDescendXYScale(effective_height);
        vx *= descend_xy_scale;
        vy *= descend_xy_scale;
        publishVelocityLimited(vx, vy, -effectiveDescendSpeed(), vyaw);

        RCLCPP_INFO(
          this->get_logger(),
          "[visual_landing] descending h=%.2fm(range=%d) xy_scale=%.2f err_n_raw=(%.4f,%.4f) err_n_f=(%.4f,%.4f) yaw_f=%.3f conf=%.2f",
          effective_height, height_est.using_range ? 1 : 0,
          descend_xy_scale,
          last_err_u_, last_err_v_, err_u_f_, err_v_f_, err_yaw_f_, last_detection_confidence_);

        if (effective_height <= land_height_m_) {
          transitionTo(State::LAND);
        }
        break;
      }

      case State::LAND:
        publishHover();
        if (!land_service_called_) {
          land_service_called_ = true;
          callLandService();
        }
        break;

      case State::DONE:
        if (has_vehicle_status_ && !vehicle_armed_) {
          RCLCPP_INFO(
            this->get_logger(),
            "[visual_landing] DONE confirmed with vehicle disarmed, returning to READY");
          deactivateToReady();
        }
        break;
    }

    publishLandingStatus(false);
  }

  State state_{State::READY};
  rclcpp::Time last_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time align_stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time search_started_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reacquire_candidate_since_{0, 0, RCL_ROS_TIME};

  bool align_hold_timer_started_{false};
  bool align_in_window_{false};
  bool reacquire_candidate_started_{false};
  bool landing_start_height_initialized_{false};
  bool terminal_align_pending_{false};

  float last_err_u_{0.0f};
  float last_err_v_{0.0f};
  float last_detection_confidence_{0.0f};
  float err_u_f_{0.0f};
  float err_v_f_{0.0f};
  float err_yaw_f_{0.0f};
  float yaw_error_rate_f_{0.0f};

  bool has_filtered_error_{false};
  bool tag_detected_{false};
  bool land_service_called_{false};
  bool has_odom_{false};
  bool active_{false};
  bool has_vehicle_status_{false};
  bool vehicle_armed_{false};

  float current_height_{0.0f};
  float current_yaw_{0.0f};
  float current_vx_world_{0.0f};
  float current_vy_world_{0.0f};
  float current_yaw_rate_{0.0f};
  bool has_range_height_{false};
  bool has_flow_quality_{false};
  float range_height_m_{0.0f};
  int8_t range_signal_quality_{-1};
  uint8_t flow_quality_{0};
  rclcpp::Time range_height_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time flow_quality_stamp_{0, 0, RCL_ROS_TIME};

  float search_target_height_{0.0f};
  float landing_start_height_{0.0f};

  float kp_xy_{0.0f};
  float kp_yaw_{0.0f};
  float max_vxy_{0.0f};
  float max_vyaw_{0.0f};
  float max_error_norm_{0.0f};
  float speed_scale_xy_{1.0f};
  float speed_scale_yaw_{1.0f};
  float speed_scale_z_{1.0f};

  float align_enter_error_{0.0f};
  float align_exit_error_{0.0f};
  float align_enter_yaw_{0.0f};
  float align_exit_yaw_{0.0f};
  double align_hold_s_{0.0};
  float align_z_kp_{0.0f};
  float align_z_max_vz_{0.0f};
  float align_z_deadband_m_{0.0f};
  float align_height_ref_m_{0.0f};
  bool align_height_ref_initialized_{false};

  float alpha_xy_{0.0f};
  float max_acc_xy_{0.0f};
  float max_acc_yaw_{0.0f};
  float vel_damping_xy_{0.0f};
  float vel_damping_yaw_{0.0f};
  float min_height_for_xy_{0.0f};

  float yaw_freeze_threshold_{0.0f};
  float yaw_slow_threshold_{0.0f};
  float yaw_slow_scale_{0.0f};
  float yaw_cmd_deadband_{0.0f};

  float yaw_filter_alpha_{0.35f};
  float yaw_filter_beta_{0.08f};
  float yaw_innovation_base_rad_{0.20f};
  float yaw_innovation_conf_scale_{0.15f};
  float yaw_rate_state_limit_{1.2f};

  float descend_speed_{0.0f};
  float land_height_m_{0.0f};
  double height_source_timeout_s_{0.15};
  float height_range_min_m_{0.05f};
  float height_range_max_m_{5.0f};
  float height_consistency_max_diff_m_{0.35f};
  double flow_quality_timeout_s_{0.20};
  int flow_quality_min_for_land_{30};
  float terminal_entry_height_m_{0.60f};
  float direct_land_max_vxy_mps_{0.10f};
  double direct_land_recent_seen_s_{0.30};
  float descend_xy_slow_height_m_{0.25f};
  float descend_xy_min_scale_{0.20f};

  double lost_enter_s_{0.45};
  double reacquire_confirm_s_{0.20};
  float search_rise_step_m_{0.20f};
  float search_rise_speed_mps_{0.16f};
  float search_max_rise_m_{1.0f};
  double search_timeout_s_{6.0};

  float control_dt_s_{1.0f / 30.0f};
  float last_cmd_vx_{0.0f};
  float last_cmd_vy_{0.0f};
  float last_cmd_yaw_rate_{0.0f};
  bool cmd_rate_limit_initialized_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<uav_visual_landing::msg::LandingStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<uav_visual_landing::msg::LandingError>::SharedPtr landing_error_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distance_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorOpticalFlow>::SharedPtr optical_flow_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Client<Trigger>::SharedPtr land_client_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  double status_period_s_{0.2};
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  bool status_publish_initialized_{false};
};

}  // namespace uav_visual_landing

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_visual_landing::VisualLandingNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
