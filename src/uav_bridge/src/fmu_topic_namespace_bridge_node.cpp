#include <string>
#include <utility>
#include <vector>

#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/sensor_optical_flow.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_bridge
{

class FmuTopicNamespaceBridgeNode : public rclcpp::Node
{
public:
  FmuTopicNamespaceBridgeNode() : Node("fmu_topic_namespace_bridge")
  {
    this->declare_parameter<std::string>("namespaced_fmu_prefix", "/uav/fmu");
    this->declare_parameter<std::string>("global_fmu_prefix", "/fmu");

    namespaced_fmu_prefix_ = normalizePrefix(
      this->get_parameter("namespaced_fmu_prefix").as_string());
    global_fmu_prefix_ = normalizePrefix(
      this->get_parameter("global_fmu_prefix").as_string());

    if (namespaced_fmu_prefix_ == global_fmu_prefix_) {
      RCLCPP_WARN(
        this->get_logger(),
        "namespaced_fmu_prefix matches global_fmu_prefix (%s); relay disabled",
        namespaced_fmu_prefix_.c_str());
      return;
    }

    const auto command_qos = rclcpp::QoS(10);
    const auto sensor_qos = rclcpp::SensorDataQoS();

    bridgeNamespacedToGlobal<px4_msgs::msg::OffboardControlMode>(
      "/in/offboard_control_mode", command_qos);
    bridgeNamespacedToGlobal<px4_msgs::msg::TrajectorySetpoint>(
      "/in/trajectory_setpoint", command_qos);
    bridgeNamespacedToGlobal<px4_msgs::msg::VehicleCommand>(
      "/in/vehicle_command", command_qos);
    bridgeNamespacedToGlobal<px4_msgs::msg::DistanceSensor>(
      "/in/distance_sensor", sensor_qos);
    bridgeNamespacedToGlobal<px4_msgs::msg::SensorOpticalFlow>(
      "/in/sensor_optical_flow", sensor_qos);

    bridgeGlobalToNamespaced<px4_msgs::msg::VehicleLocalPosition>(
      "/out/vehicle_local_position", sensor_qos);
    bridgeGlobalToNamespaced<px4_msgs::msg::VehicleStatus>(
      "/out/vehicle_status", sensor_qos);
    bridgeGlobalToNamespaced<px4_msgs::msg::VehicleOdometry>(
      "/out/vehicle_odometry", sensor_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "fmu topic namespace bridge active: namespaced=%s <-> global=%s",
      namespaced_fmu_prefix_.c_str(),
      global_fmu_prefix_.c_str());
  }

private:
  template<typename MsgT>
  void bridgeNamespacedToGlobal(const std::string & suffix, const rclcpp::QoS & qos)
  {
    const auto src = namespaced_fmu_prefix_ + suffix;
    const auto dst = global_fmu_prefix_ + suffix;
    auto pub = this->create_publisher<MsgT>(dst, qos);
    auto sub = this->create_subscription<MsgT>(
      src,
      qos,
      [pub](const typename MsgT::SharedPtr msg) {
        pub->publish(*msg);
      });
    publishers_.push_back(pub);
    subscriptions_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "relay %s -> %s", src.c_str(), dst.c_str());
  }

  template<typename MsgT>
  void bridgeGlobalToNamespaced(const std::string & suffix, const rclcpp::QoS & qos)
  {
    const auto src = global_fmu_prefix_ + suffix;
    const auto dst = namespaced_fmu_prefix_ + suffix;
    auto pub = this->create_publisher<MsgT>(dst, qos);
    auto sub = this->create_subscription<MsgT>(
      src,
      qos,
      [pub](const typename MsgT::SharedPtr msg) {
        pub->publish(*msg);
      });
    publishers_.push_back(pub);
    subscriptions_.push_back(sub);
    RCLCPP_INFO(this->get_logger(), "relay %s -> %s", src.c_str(), dst.c_str());
  }

  static std::string normalizePrefix(std::string prefix)
  {
    if (prefix.empty()) {
      return "/";
    }
    if (prefix.front() != '/') {
      prefix.insert(prefix.begin(), '/');
    }
    while (prefix.size() > 1 && prefix.back() == '/') {
      prefix.pop_back();
    }
    return prefix;
  }

  std::string namespaced_fmu_prefix_;
  std::string global_fmu_prefix_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::FmuTopicNamespaceBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
