#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

#include "uav_bridge/uav_state_bridge_logic.hpp"

namespace uav_bridge
{

class UavStateBridgeNode : public rclcpp::Node
{
public:
  UavStateBridgeNode()
  : Node("uav_state_bridge_node")
  {
    this->declare_parameter<std::string>("vehicle_local_position_topic", "/uav/fmu/out/vehicle_local_position");
    this->declare_parameter<std::string>("vehicle_odometry_topic", "/uav/fmu/out/vehicle_odometry");
    this->declare_parameter<std::string>("output_odometry_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("map_frame_id", "uav_map");
    this->declare_parameter<std::string>("odom_frame_id", "uav_odom");
    this->declare_parameter<std::string>("base_frame_id", "uav_base_link");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<bool>("publish_odometry", true);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("publish_map_to_odom_tf", true);
    this->declare_parameter<bool>("log_state", false);

    vehicle_local_position_topic_ = this->get_parameter("vehicle_local_position_topic").as_string();
    vehicle_odometry_topic_ = this->get_parameter("vehicle_odometry_topic").as_string();
    output_odometry_topic_ = this->get_parameter("output_odometry_topic").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    publish_rate_hz_ = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    publish_odometry_ = this->get_parameter("publish_odometry").as_bool();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_map_to_odom_tf_ = this->get_parameter("publish_map_to_odom_tf").as_bool();
    log_state_ = this->get_parameter("log_state").as_bool();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      vehicle_local_position_topic_, sensor_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
      {
        std::scoped_lock lock(state_mutex_);
        latest_local_position_ = *msg;
        has_new_state_ = true;
      });

    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_topic_, sensor_qos,
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
      {
        std::scoped_lock lock(state_mutex_);
        latest_vehicle_odometry_ = *msg;
        has_new_state_ = true;
      });

    if (publish_odometry_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odometry_topic_, 10);
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      if (publish_map_to_odom_tf_) {
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishStaticMapToOdomTf();
      }
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&UavStateBridgeNode::publishSnapshot, this));

    RCLCPP_INFO(
      this->get_logger(),
      "uav_state_bridge_node: local_position=%s vehicle_odometry=%s output=%s odom_frame=%s base_frame=%s",
      vehicle_local_position_topic_.c_str(),
      vehicle_odometry_topic_.c_str(),
      output_odometry_topic_.c_str(),
      odom_frame_id_.c_str(),
      base_frame_id_.c_str());
  }

private:
  void publishStaticMapToOdomTf()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = map_frame_id_;
    tf_msg.child_frame_id = odom_frame_id_;
    tf_msg.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishSnapshot()
  {
    std::optional<px4_msgs::msg::VehicleLocalPosition> local_position;
    std::optional<px4_msgs::msg::VehicleOdometry> vehicle_odometry;
    {
      std::scoped_lock lock(state_mutex_);
      if (!has_new_state_) {
        return;
      }
      local_position = latest_local_position_;
      vehicle_odometry = latest_vehicle_odometry_;
      has_new_state_ = false;
    }

    const auto resolved = resolveUavStateBridgeState(local_position, vehicle_odometry);
    if (!resolved.position_valid || !resolved.orientation_valid) {
      return;
    }

    const auto stamp = this->now();
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose.position.x = resolved.position_enu_m[0];
    odom_msg.pose.pose.position.y = resolved.position_enu_m[1];
    odom_msg.pose.pose.position.z = resolved.position_enu_m[2];
    odom_msg.pose.pose.orientation.x = resolved.orientation_enu_flu.x();
    odom_msg.pose.pose.orientation.y = resolved.orientation_enu_flu.y();
    odom_msg.pose.pose.orientation.z = resolved.orientation_enu_flu.z();
    odom_msg.pose.pose.orientation.w = resolved.orientation_enu_flu.w();
    if (resolved.linear_velocity_valid) {
      odom_msg.twist.twist.linear.x = resolved.linear_velocity_body_flu_mps[0];
      odom_msg.twist.twist.linear.y = resolved.linear_velocity_body_flu_mps[1];
      odom_msg.twist.twist.linear.z = resolved.linear_velocity_body_flu_mps[2];
    }

    const double yaw_enu = tf2::getYaw(resolved.orientation_enu_flu);
    if (last_yaw_enu_rad_.has_value() && last_publish_stamp_.has_value()) {
      const double dt_sec = (stamp - *last_publish_stamp_).seconds();
      odom_msg.twist.twist.angular.z = computeYawRateRad(*last_yaw_enu_rad_, yaw_enu, dt_sec);
    }
    last_yaw_enu_rad_ = yaw_enu;
    last_publish_stamp_ = stamp;

    if (publish_odometry_) {
      odom_pub_->publish(odom_msg);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_id_;
      tf_msg.child_frame_id = base_frame_id_;
      tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
      tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
      tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
      tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    if (log_state_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "state pose=(%.2f, %.2f, %.2f) vel_body=(%.2f, %.2f, %.2f)",
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z);
    }
  }

  std::string vehicle_local_position_topic_;
  std::string vehicle_odometry_topic_;
  std::string output_odometry_topic_;
  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double publish_rate_hz_{50.0};
  bool publish_odometry_{true};
  bool publish_tf_{true};
  bool publish_map_to_odom_tf_{true};
  bool log_state_{false};
  bool has_new_state_{false};
  std::mutex state_mutex_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> latest_local_position_;
  std::optional<px4_msgs::msg::VehicleOdometry> latest_vehicle_odometry_;
  std::optional<double> last_yaw_enu_rad_;
  std::optional<rclcpp::Time> last_publish_stamp_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::UavStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
