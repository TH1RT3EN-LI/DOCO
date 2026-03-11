#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <uav_bridge/gz_topic_utils.hpp>

namespace uav_bridge {
class MonoCameraBridgeNode : public rclcpp::Node {
public:
  MonoCameraBridgeNode() : Node("mono_camera_bridge_node") {
    this->declare_parameter<std::string>("gz_image_topic", "");
    this->declare_parameter<std::string>("gz_world_name", "test");
    this->declare_parameter<std::string>("model_name", "uav");
    this->declare_parameter<std::string>("link_name", "base_link");
    this->declare_parameter<std::string>("sensor_name", "mono_camera");

    this->declare_parameter<std::string>("ros_image_topic",
                                         "/uav/camera/image_raw");
    this->declare_parameter<std::string>("frame_id",
                                         "mono_camera_optical_frame");

    std::string gz_image_topic =
        this->get_parameter("gz_image_topic").as_string();
    std::string gz_world_name =
        this->get_parameter("gz_world_name").as_string();
    std::string model_name = this->get_parameter("model_name").as_string();
    std::string link_name = this->get_parameter("link_name").as_string();
    std::string sensor_name = this->get_parameter("sensor_name").as_string();
    std::string ros_image_topic =
        this->get_parameter("ros_image_topic").as_string();

    _frame_id = this->get_parameter("frame_id").as_string();

    if (gz_image_topic.empty()) {
      gz_image_topic = gz_topics::Image(gz_world_name, model_name, link_name,
                                        sensor_name);
    }

    this->_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
        ros_image_topic, rclcpp::SensorDataQoS());

    bool ok = this->_gz_node.Subscribe(gz_image_topic,
                                       &MonoCameraBridgeNode::OnGzImage, this);

    if (!ok) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to subscribe Gazebo mono image topic: %s",
                   gz_image_topic.c_str());
      throw std::runtime_error("Gazebo mono image topic subscribe failed");
    }

    RCLCPP_INFO(this->get_logger(),
                "[mono image topic] Gazebo topic: %s -> Ros topic: %s",
                gz_image_topic.c_str(), ros_image_topic.c_str());
  }

private:
  std::string _frame_id;
  gz::transport::Node _gz_node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
  std::optional<int64_t> last_image_stamp_ns_;

  rclcpp::Time ResolveStamp(const gz::msgs::Image &gz_msg) const {
    if (gz_msg.has_header() && gz_msg.header().has_stamp()) {
      return rclcpp::Time(gz_msg.header().stamp().sec(),
                          gz_msg.header().stamp().nsec(), RCL_ROS_TIME);
    }
    return this->now();
  }

  bool AcceptStamp(const rclcpp::Time &stamp) {
    const int64_t stamp_ns = stamp.nanoseconds();
    if (!last_image_stamp_ns_.has_value() || stamp_ns > *last_image_stamp_ns_) {
      last_image_stamp_ns_ = stamp_ns;
      return true;
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping out-of-order mono image: current=%ld last=%ld",
        static_cast<long>(stamp_ns), static_cast<long>(*last_image_stamp_ns_));
    return false;
  }

  static std::string ToRosEncoding(gz::msgs::PixelFormatType fmt,
                                   rclcpp::Logger logger) {
    if (fmt == gz::msgs::RGB_INT8) {
      return sensor_msgs::image_encodings::RGB8;
    }

    RCLCPP_FATAL(logger,
                 "Gazebo mono image PixelFormatType: %d not mapper RGB_INT8",
                 static_cast<int>(fmt));

    throw std::runtime_error(
        "Gazebo mono image PixelFormatType not mapper RGB_INT8");
  }

  void OnGzImage(const gz::msgs::Image &gz_msg) {
    sensor_msgs::msg::Image ros_msg;

    ros_msg.header.stamp = ResolveStamp(gz_msg);
    if (!AcceptStamp(ros_msg.header.stamp)) {
      return;
    }
    ros_msg.header.frame_id = this->_frame_id;

    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    ros_msg.encoding =
        ToRosEncoding(gz_msg.pixel_format_type(), this->get_logger());

    ros_msg.is_bigendian = false;
    ros_msg.step = gz_msg.step();

    const std::string &data = gz_msg.data();
    ros_msg.data.assign(data.begin(), data.end());

    this->_image_pub->publish(std::move(ros_msg));
  }
};
} // namespace uav_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::MonoCameraBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
