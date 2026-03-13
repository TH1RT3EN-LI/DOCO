#include "relative_position_fusion/relative_tracking_controller.hpp"
#include "relative_position_fusion/srv/start_relative_tracking.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

namespace relative_position_fusion
{

namespace
{

Eigen::Matrix2d ExtractPlanarCovariance(const std::array<double, 36> & covariance)
{
  Eigen::Matrix2d result = Eigen::Matrix2d::Zero();
  result(0, 0) = covariance[0];
  result(0, 1) = covariance[1];
  result(1, 0) = covariance[6];
  result(1, 1) = covariance[7];
  return result;
}

geometry_msgs::msg::Quaternion YawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
  const double cosy_cosp =
    1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)));
  return std::atan2(siny_cosp, cosy_cosp);
}

double ResolveStampSeconds(
  const builtin_interfaces::msg::Time & stamp_msg,
  const rclcpp::Time & fallback_stamp)
{
  if (stamp_msg.sec == 0 && stamp_msg.nanosec == 0) {
    return fallback_stamp.seconds();
  }
  return rclcpp::Time(stamp_msg, RCL_ROS_TIME).seconds();
}

}  // namespace

class RelativeTrackingNode : public rclcpp::Node
{
  using StartRelativeTracking = relative_position_fusion::srv::StartRelativeTracking;
  using Trigger = std_srvs::srv::Trigger;

public:
  RelativeTrackingNode()
  : Node("relative_tracking_node"),
    controller_(LoadConfig())
  {
    relative_pose_body_topic_ =
      this->get_parameter("relative_pose_body_topic").as_string();
    relocalize_requested_topic_ =
      this->get_parameter("relocalize_requested_topic").as_string();
    uav_state_topic_ = this->get_parameter("uav_state_topic").as_string();
    ugv_odom_topic_ = this->get_parameter("ugv_odom_topic").as_string();
    velocity_body_topic_ = this->get_parameter("velocity_body_topic").as_string();
    position_topic_ = this->get_parameter("position_topic").as_string();
    hold_service_name_ = this->get_parameter("hold_service").as_string();
    uav_position_mode_service_name_ = this->get_parameter("uav_position_mode_service").as_string();
    start_service_name_ = this->get_parameter("start_service").as_string();
    position_mode_service_name_ =
      this->get_parameter("position_mode_service").as_string();
    go_above_ugv_service_name_ =
      this->get_parameter("go_above_ugv_service").as_string();
    stop_service_name_ = this->get_parameter("stop_service").as_string();
    control_rate_hz_ = std::max(1.0, this->get_parameter("control_rate_hz").as_double());

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(1);

    relative_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      relative_pose_body_topic_, sensor_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
      {
        last_relative_pose_ = *msg;
        last_relative_pose_received_at_ = this->now();
        has_relative_pose_ = true;
      });
    relocalize_requested_sub_ =
      this->create_subscription<std_msgs::msg::Bool>(
      relocalize_requested_topic_, sensor_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        relocalize_requested_ = msg->data;
      });
    uav_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      uav_state_topic_, sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        last_uav_state_ = *msg;
        last_uav_state_received_at_ = this->now();
        has_uav_state_ = true;
      });
    ugv_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ugv_odom_topic_, sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        last_ugv_odom_ = *msg;
        last_ugv_odom_received_at_ = this->now();
        has_ugv_odom_ = true;
      });

    position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(position_topic_, 10);
    velocity_body_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_body_topic_, sensor_qos);
    hold_client_ = this->create_client<Trigger>(hold_service_name_);
    uav_position_mode_client_ = this->create_client<Trigger>(uav_position_mode_service_name_);

    start_srv_ = this->create_service<StartRelativeTracking>(
      start_service_name_,
      [this](
        const std::shared_ptr<StartRelativeTracking::Request> request,
        std::shared_ptr<StartRelativeTracking::Response> response)
      {
        const auto inputs = BuildInputs();
        std::string message;
        response->success = controller_.Start(request->target_height_m, inputs, &message);
        response->message = message;
      });
    position_mode_srv_ = this->create_service<Trigger>(
      position_mode_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        const auto inputs = BuildInputs();
        response->success = controller_.EnterPositionMode(inputs, &response->message);
        if (response->success) {
          response->success = RequestUavPositionMode(&response->message);
        }
      });
    go_above_ugv_srv_ = this->create_service<Trigger>(
      go_above_ugv_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        const auto inputs = BuildInputs();
        response->success = controller_.GoAboveUgv(inputs, &response->message);
      });
    stop_srv_ = this->create_service<Trigger>(
      stop_service_name_,
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
      {
        response->success = controller_.Stop(&response->message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&RelativeTrackingNode::OnTimer, this));

    last_logged_mode_ = controller_.mode();
    RCLCPP_INFO(
      this->get_logger(),
      "relative_tracking_node: rel_pose=%s uav_state=%s ugv_odom=%s velocity_body=%s position=%s",
      relative_pose_body_topic_.c_str(),
      uav_state_topic_.c_str(),
      ugv_odom_topic_.c_str(),
      velocity_body_topic_.c_str(),
      position_topic_.c_str());
  }

private:
  RelativeTrackingController::Config LoadConfig()
  {
    this->declare_parameter<std::string>(
      "relative_pose_body_topic", "/relative_position/estimate/uav_body");
    this->declare_parameter<std::string>(
      "relocalize_requested_topic", "/relative_position/relocalize_requested");
    this->declare_parameter<std::string>("uav_state_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("ugv_odom_topic", "/ugv/odom");
    this->declare_parameter<std::string>(
      "velocity_body_topic", "/uav/control/setpoint/velocity_body");
    this->declare_parameter<std::string>("position_topic", "/uav/control/pose");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<std::string>(
      "uav_position_mode_service", "/uav/control/command/position_mode");
    this->declare_parameter<std::string>(
      "start_service", "/uav/relative_tracking/command/start");
    this->declare_parameter<std::string>(
      "position_mode_service", "/uav/relative_tracking/command/position_mode");
    this->declare_parameter<std::string>(
      "go_above_ugv_service", "/uav/relative_tracking/command/go_above_ugv");
    this->declare_parameter<std::string>(
      "stop_service", "/uav/relative_tracking/command/stop");
    this->declare_parameter<double>("control_rate_hz", 30.0);
    this->declare_parameter<double>("return_height_m", 1.0);
    this->declare_parameter<double>("takeoff_height_tolerance_m", 0.10);
    this->declare_parameter<double>("takeoff_stable_s", 0.5);
    this->declare_parameter<bool>("auto_track_after_takeoff", true);
    this->declare_parameter<double>("relative_pose_timeout_s", 0.25);
    this->declare_parameter<double>("uav_state_timeout_s", 0.25);
    this->declare_parameter<double>("ugv_odom_timeout_s", 0.25);
    this->declare_parameter<double>("xy_kp", 0.8);
    this->declare_parameter<double>("z_kp", 1.0);
    this->declare_parameter<double>("xy_deadband_m", 0.05);
    this->declare_parameter<double>("z_deadband_m", 0.05);
    this->declare_parameter<double>("max_xy_speed_mps", 0.8);
    this->declare_parameter<double>("max_z_speed_mps", 0.5);
    this->declare_parameter<double>("max_relative_covariance_m2", 1.5);

    RelativeTrackingController::Config config;
    config.return_height_m = this->get_parameter("return_height_m").as_double();
    config.takeoff_height_tolerance_m =
      this->get_parameter("takeoff_height_tolerance_m").as_double();
    config.takeoff_stable_s = this->get_parameter("takeoff_stable_s").as_double();
    config.auto_track_after_takeoff =
      this->get_parameter("auto_track_after_takeoff").as_bool();
    config.relative_pose_timeout_s = this->get_parameter("relative_pose_timeout_s").as_double();
    config.uav_state_timeout_s = this->get_parameter("uav_state_timeout_s").as_double();
    config.ugv_odom_timeout_s = this->get_parameter("ugv_odom_timeout_s").as_double();
    config.xy_kp = this->get_parameter("xy_kp").as_double();
    config.z_kp = this->get_parameter("z_kp").as_double();
    config.xy_deadband_m = this->get_parameter("xy_deadband_m").as_double();
    config.z_deadband_m = this->get_parameter("z_deadband_m").as_double();
    config.max_xy_speed_mps = this->get_parameter("max_xy_speed_mps").as_double();
    config.max_z_speed_mps = this->get_parameter("max_z_speed_mps").as_double();
    config.max_relative_covariance_m2 =
      this->get_parameter("max_relative_covariance_m2").as_double();
    return config;
  }

  RelativeTrackingController::Inputs BuildInputs() const
  {
    RelativeTrackingController::Inputs inputs;
    const rclcpp::Time now = this->now();
    inputs.now_sec = now.seconds();
    inputs.relocalize_requested = relocalize_requested_;

    if (has_uav_state_) {
      inputs.uav_state.valid = true;
      inputs.uav_state.stamp_sec =
        ResolveStampSeconds(last_uav_state_.header.stamp, last_uav_state_received_at_);
      inputs.uav_state.position_enu = Eigen::Vector3d(
        last_uav_state_.pose.pose.position.x,
        last_uav_state_.pose.pose.position.y,
        last_uav_state_.pose.pose.position.z);
      inputs.uav_state.yaw_enu = YawFromQuaternion(last_uav_state_.pose.pose.orientation);
      inputs.uav_state.frame_id = last_uav_state_.header.frame_id;
    }

    if (has_ugv_odom_) {
      inputs.ugv_odom.valid = true;
      inputs.ugv_odom.stamp_sec =
        ResolveStampSeconds(last_ugv_odom_.header.stamp, last_ugv_odom_received_at_);
      inputs.ugv_odom.z_enu = last_ugv_odom_.pose.pose.position.z;
    }

    if (has_relative_pose_) {
      inputs.relative_pose.valid = true;
      inputs.relative_pose.stamp_sec =
        ResolveStampSeconds(last_relative_pose_.header.stamp, last_relative_pose_received_at_);
      inputs.relative_pose.position_body = Eigen::Vector2d(
        last_relative_pose_.pose.pose.position.x,
        last_relative_pose_.pose.pose.position.y);
      inputs.relative_pose.covariance =
        ExtractPlanarCovariance(last_relative_pose_.pose.covariance);
    }

    return inputs;
  }

  void OnTimer()
  {
    const auto outputs = controller_.Update(BuildInputs());
    if (controller_.mode() != last_logged_mode_) {
      RCLCPP_INFO(
        this->get_logger(),
        "relative tracking mode -> %s (%s)",
        RelativeTrackingController::ModeName(controller_.mode()),
        controller_.last_reason().c_str());
      last_logged_mode_ = controller_.mode();
    }

    if (outputs.request_hold) {
      RequestHold();
    }
    if (outputs.publish_position_target) {
      PublishPositionTarget(outputs);
    }
    if (outputs.publish_velocity_body) {
      PublishVelocityTarget(outputs);
    }
  }

  void PublishPositionTarget(const RelativeTrackingController::Outputs & outputs)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = outputs.position_frame_id;
    msg.pose.position.x = outputs.position_target_enu.x();
    msg.pose.position.y = outputs.position_target_enu.y();
    msg.pose.position.z = outputs.position_target_enu.z();
    msg.pose.orientation = YawToQuaternion(outputs.yaw_target_enu);
    position_pub_->publish(msg);
  }

  void PublishVelocityTarget(const RelativeTrackingController::Outputs & outputs)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "uav_base_link";
    msg.twist.linear.x = outputs.velocity_body_mps.x();
    msg.twist.linear.y = outputs.velocity_body_mps.y();
    msg.twist.linear.z = outputs.velocity_body_mps.z();
    msg.twist.angular.z = outputs.yaw_rate_radps;
    velocity_body_pub_->publish(msg);
  }

  void RequestHold()
  {
    if (!hold_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "hold service %s is not ready", hold_service_name_.c_str());
      return;
    }

    auto request = std::make_shared<Trigger::Request>();
    hold_client_->async_send_request(request);
  }

  bool RequestUavPositionMode(std::string * message)
  {
    if (!uav_position_mode_client_->service_is_ready()) {
      if (message != nullptr) {
        if (!message->empty()) {
          *message += "; ";
        }
        *message += "uav control position mode service is not ready";
      }
      return false;
    }

    auto request = std::make_shared<Trigger::Request>();
    uav_position_mode_client_->async_send_request(request);
    if (message != nullptr) {
      if (!message->empty()) {
        *message += "; ";
      }
      *message += "requested uav control position mode";
    }
    return true;
  }

  std::string relative_pose_body_topic_;
  std::string relocalize_requested_topic_;
  std::string uav_state_topic_;
  std::string ugv_odom_topic_;
  std::string velocity_body_topic_;
  std::string position_topic_;
  std::string hold_service_name_;
  std::string uav_position_mode_service_name_;
  std::string start_service_name_;
  std::string position_mode_service_name_;
  std::string go_above_ugv_service_name_;
  std::string stop_service_name_;
  double control_rate_hz_{30.0};

  RelativeTrackingController controller_;
  RelativeTrackingController::Mode last_logged_mode_{RelativeTrackingController::Mode::Idle};

  bool has_relative_pose_{false};
  bool has_uav_state_{false};
  bool has_ugv_odom_{false};
  bool relocalize_requested_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped last_relative_pose_{};
  nav_msgs::msg::Odometry last_uav_state_{};
  nav_msgs::msg::Odometry last_ugv_odom_{};
  rclcpp::Time last_relative_pose_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_uav_state_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ugv_odom_received_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    relative_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relocalize_requested_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uav_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ugv_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_body_pub_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::Client<Trigger>::SharedPtr uav_position_mode_client_;
  rclcpp::Service<StartRelativeTracking>::SharedPtr start_srv_;
  rclcpp::Service<Trigger>::SharedPtr position_mode_srv_;
  rclcpp::Service<Trigger>::SharedPtr go_above_ugv_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace relative_position_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relative_position_fusion::RelativeTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
