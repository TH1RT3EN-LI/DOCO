#include <cstdint>
#include <cstring>
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
    this->declare_parameter<std::string>("link_name", "uav_base_link");
    this->declare_parameter<std::string>("sensor_name", "uav_mono_camera");

    this->declare_parameter<std::string>("ros_image_topic",
                                         "/uav/camera/image_raw");
    this->declare_parameter<std::string>("frame_id",
                                         "uav_camera_optical_frame");

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

  static bool FillRosMonoImage(const gz::msgs::Image &gz_msg,
                               sensor_msgs::msg::Image &ros_msg,
                               rclcpp::Logger logger) {
    ros_msg.encoding = sensor_msgs::image_encodings::MONO8;
    ros_msg.is_bigendian = false;
    ros_msg.step = ros_msg.width;

    const size_t width = static_cast<size_t>(ros_msg.width);
    const size_t height = static_cast<size_t>(ros_msg.height);
    const size_t src_step = static_cast<size_t>(gz_msg.step());
    const std::string &data = gz_msg.data();
    const size_t required_bytes = src_step * height;

    if (data.size() < required_bytes) {
      RCLCPP_WARN(logger,
                  "Gazebo mono image payload too small: got=%zu need=%zu",
                  data.size(), required_bytes);
      return false;
    }

    if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
      ros_msg.data.resize(width * height);
      const uint8_t *src =
          reinterpret_cast<const uint8_t *>(data.data());
      uint8_t *dst = ros_msg.data.data();

      for (size_t row = 0; row < height; ++row) {
        std::memcpy(dst + (row * width), src + (row * src_step), width);
      }
      return true;
    }

    if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
      ros_msg.data.resize(width * height);
      const uint8_t *src =
          reinterpret_cast<const uint8_t *>(data.data());
      uint8_t *dst = ros_msg.data.data();

      for (size_t row = 0; row < height; ++row) {
        const uint8_t *src_row = src + (row * src_step);
        uint8_t *dst_row = dst + (row * width);
        for (size_t col = 0; col < width; ++col) {
          const size_t src_idx = col * 3;
          const uint8_t r = src_row[src_idx + 0];
          const uint8_t g = src_row[src_idx + 1];
          const uint8_t b = src_row[src_idx + 2];
          dst_row[col] = static_cast<uint8_t>(
              (77u * static_cast<uint32_t>(r) +
               150u * static_cast<uint32_t>(g) +
               29u * static_cast<uint32_t>(b) + 128u) >>
              8u);
        }
      }
      return true;
    }

    RCLCPP_WARN(logger,
                "Unsupported Gazebo mono image pixel format: %d",
                static_cast<int>(gz_msg.pixel_format_type()));
    return false;
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
    if (!FillRosMonoImage(gz_msg, ros_msg, this->get_logger())) {
      return;
    }

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
