/**
 * @file rangefinder_bridge_node.cpp
 * @brief Bridge GZ gpu_lidar (single-point downward) -> PX4 DistanceSensor via uXRCE-DDS
 *
 * Subscribes to the Gazebo gpu_lidar LaserScan topic from the distance_sensor on
 * the UAV model, converts the first range reading to a px4_msgs::msg::DistanceSensor
 * message and publishes it on /uav/fmu/in/distance_sensor.
 */
#include <cmath>
#include <optional>
#include <string>

#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

#include <uav_bridge/gz_topic_utils.hpp>

namespace uav_bridge {

class RangefinderBridgeNode : public rclcpp::Node {
public:
  RangefinderBridgeNode() : Node("rangefinder_bridge_node") {
    // ---------- parameters ----------
    declare_parameter<std::string>("gz_world_name", "test");
    declare_parameter<std::string>("model_name", "uav");
    declare_parameter<std::string>("link_name", "base_link");
    declare_parameter<std::string>("sensor_name", "distance_sensor");
    declare_parameter<std::string>("gz_topic_override", "");
    declare_parameter<std::string>("ros_topic", "/uav/fmu/in/distance_sensor");

    const auto gz_world_name = get_parameter("gz_world_name").as_string();
    const auto model  = get_parameter("model_name").as_string();
    const auto link   = get_parameter("link_name").as_string();
    const auto sensor = get_parameter("sensor_name").as_string();
    auto gz_topic     = get_parameter("gz_topic_override").as_string();
    const auto ros_topic = get_parameter("ros_topic").as_string();

    if (gz_topic.empty()) {
      gz_topic = gz_topics::Scan(gz_world_name, model, link, sensor);
    }

    // ROS publisher (best-effort sensor QoS to match uXRCE-DDS)
    pub_ = create_publisher<px4_msgs::msg::DistanceSensor>(
        ros_topic, rclcpp::SensorDataQoS());

    // GZ subscriber
    bool ok = gz_node_.Subscribe(gz_topic, &RangefinderBridgeNode::OnLaserScan, this);
    if (!ok) {
      RCLCPP_FATAL(get_logger(), "Failed to subscribe to GZ topic: %s", gz_topic.c_str());
      throw std::runtime_error("GZ lidar subscribe failed");
    }

    RCLCPP_INFO(get_logger(), "Rangefinder bridge: GZ [%s] -> ROS [%s]",
                gz_topic.c_str(), ros_topic.c_str());
  }

private:
  gz::transport::Node gz_node_;
  rclcpp::Publisher<px4_msgs::msg::DistanceSensor>::SharedPtr pub_;

  void OnLaserScan(const gz::msgs::LaserScan &msg) {
    if (msg.ranges_size() == 0) return;

    px4_msgs::msg::DistanceSensor out;

    // PX4 uses microseconds since boot; use GZ sim time
    if (msg.has_header() && msg.header().has_stamp()) {
      out.timestamp = static_cast<uint64_t>(msg.header().stamp().sec()) * 1000000ULL +
                      static_cast<uint64_t>(msg.header().stamp().nsec()) / 1000ULL;
    } else {
      out.timestamp = 0;  // PX4 will use its own timestamp
    }

    out.min_distance = static_cast<float>(msg.range_min());
    out.max_distance = static_cast<float>(msg.range_max());
    out.current_distance = static_cast<float>(msg.ranges(0));
    out.variance = 0.0f;
    out.signal_quality = -1;  // unknown
    out.type = px4_msgs::msg::DistanceSensor::MAV_DISTANCE_SENSOR_LASER;

    // The sensor in SDF is mounted pointing downward (pose 0 0 -0.05 0 1.5708 0)
    // We use the world_pose quaternion from the message to detect orientation,
    // but the simplest approach for a known downward sensor is to hardcode it.
    out.orientation = px4_msgs::msg::DistanceSensor::ROTATION_DOWNWARD_FACING;

    // Narrow FOV for single-point rangefinder
    out.h_fov = 0.0f;
    out.v_fov = 0.0f;

    // Device ID: simulation bus
    // bus_type=SIMULATION(4), devtype=0x50, bus=1, address=1
    // Encoding: bus_type(4bits) | bus(8bits) | address(8bits) | devtype(12bits)
    out.device_id = (4U << 28) | (1U << 16) | (1U << 8) | 0x50U;

    pub_->publish(out);
  }
};

}  // namespace uav_bridge

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::RangefinderBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
