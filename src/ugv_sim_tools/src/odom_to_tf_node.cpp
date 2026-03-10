#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv_sim_tools/odom_to_tf_logic.hpp"

namespace
{

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000LL * 1000LL * 1000LL +
         static_cast<int64_t>(stamp.nanosec);
}

}

class OdomToTfNode : public rclcpp::Node
{
public:
  explicit OdomToTfNode(const rclcpp::NodeOptions & options)
  : Node("odom_to_tf", options),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    const auto input_topic = declare_parameter<std::string>("input_odom_topic", "/ugv/odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "ugv_odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "ugv_base_footprint");
    const auto publish_hz = declare_parameter<double>("publish_hz", 50.0);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    qos.durability_volatile();

    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_odom_ = msg;
      });

    const auto publish_period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_hz));
    timer_ = rclcpp::create_timer(
      get_node_base_interface(),
      get_node_timers_interface(),
      get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "odom_to_tf started: odom='%s' -> TF %s->%s @%.1fHz (BEST_EFFORT input)",
      input_topic.c_str(),
      odom_frame_id_.c_str(),
      child_frame_id_.c_str(),
      publish_hz);
  }

private:
  void onTimer()
  {
    if (!latest_odom_) {
      return;
    }

    const auto stamp_ns = stampToNanoseconds(latest_odom_->header.stamp);
    if (!ugv_sim_tools::shouldPublishOdomTf(last_pub_stamp_ns_, stamp_ns)) {
      return;
    }
    last_pub_stamp_ns_ = stamp_ns;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = latest_odom_->header.stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = latest_odom_->pose.pose.position.x;
    transform.transform.translation.y = latest_odom_->pose.pose.position.y;
    transform.transform.translation.z = latest_odom_->pose.pose.position.z;
    transform.transform.rotation = latest_odom_->pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_id_;
  std::string child_frame_id_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  std::optional<int64_t> last_pub_stamp_ns_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<OdomToTfNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
