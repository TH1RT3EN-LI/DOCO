#include <cmath>
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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <uav_bridge/gz_topic_utils.hpp>

namespace uav_bridge
{
class MonoCameraBridgeNode : public rclcpp::Node
{
public:
  MonoCameraBridgeNode()
  : Node("mono_camera_bridge_node")
  {
    this->declare_parameter<std::string>("gz_image_topic", "");
    this->declare_parameter<std::string>("gz_world_name", "test");
    this->declare_parameter<std::string>("model_name", "uav");
    this->declare_parameter<std::string>("link_name", "uav_base_link");
    this->declare_parameter<std::string>("sensor_name", "uav_mono_camera");
    this->declare_parameter<std::string>("ros_image_topic", "/uav/camera/image_raw");
    this->declare_parameter<std::string>("ros_camera_info_topic", "/uav/camera/camera_info");
    this->declare_parameter<std::string>("frame_id", "uav_camera_optical_frame");
    this->declare_parameter<double>("camera_hfov_rad", 1.3962634);

    std::string gz_image_topic = this->get_parameter("gz_image_topic").as_string();
    const std::string gz_world_name = this->get_parameter("gz_world_name").as_string();
    const std::string model_name = this->get_parameter("model_name").as_string();
    const std::string link_name = this->get_parameter("link_name").as_string();
    const std::string sensor_name = this->get_parameter("sensor_name").as_string();
    const std::string ros_image_topic = this->get_parameter("ros_image_topic").as_string();
    const std::string ros_camera_info_topic = this->get_parameter("ros_camera_info_topic").as_string();

    frame_id_ = this->get_parameter("frame_id").as_string();
    camera_hfov_rad_ = this->get_parameter("camera_hfov_rad").as_double();

    if (gz_image_topic.empty()) {
      gz_image_topic = gz_topics::Image(gz_world_name, model_name, link_name, sensor_name);
    }

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(ros_image_topic, rclcpp::SensorDataQoS());
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      ros_camera_info_topic, rclcpp::SensorDataQoS());

    const bool ok = gz_node_.Subscribe(gz_image_topic, &MonoCameraBridgeNode::onGzImage, this);
    if (!ok) {
      throw std::runtime_error("Gazebo mono image topic subscribe failed");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "mono_camera_bridge_node: image %s -> %s, camera_info -> %s",
      gz_image_topic.c_str(), ros_image_topic.c_str(), ros_camera_info_topic.c_str());
  }

private:
  static bool fillRosMonoImage(
    const gz::msgs::Image & gz_msg,
    sensor_msgs::msg::Image & ros_msg,
    rclcpp::Logger logger)
  {
    ros_msg.encoding = sensor_msgs::image_encodings::MONO8;
    ros_msg.is_bigendian = false;
    ros_msg.step = ros_msg.width;

    const size_t width = static_cast<size_t>(ros_msg.width);
    const size_t height = static_cast<size_t>(ros_msg.height);
    const size_t src_step = static_cast<size_t>(gz_msg.step());
    const std::string & data = gz_msg.data();
    const size_t required_bytes = src_step * height;
    if (data.size() < required_bytes) {
      RCLCPP_WARN(logger, "Gazebo mono image payload too small: got=%zu need=%zu", data.size(), required_bytes);
      return false;
    }

    if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
      ros_msg.data.resize(width * height);
      const uint8_t * src = reinterpret_cast<const uint8_t *>(data.data());
      uint8_t * dst = ros_msg.data.data();
      for (size_t row = 0; row < height; ++row) {
        std::memcpy(dst + (row * width), src + (row * src_step), width);
      }
      return true;
    }

    if (gz_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
      ros_msg.data.resize(width * height);
      const uint8_t * src = reinterpret_cast<const uint8_t *>(data.data());
      uint8_t * dst = ros_msg.data.data();
      for (size_t row = 0; row < height; ++row) {
        const uint8_t * row_src = src + (row * src_step);
        for (size_t col = 0; col < width; ++col) {
          const size_t base = 3 * col;
          const uint32_t gray =
            (static_cast<uint32_t>(row_src[base + 0]) * 299U) +
            (static_cast<uint32_t>(row_src[base + 1]) * 587U) +
            (static_cast<uint32_t>(row_src[base + 2]) * 114U);
          dst[(row * width) + col] = static_cast<uint8_t>(gray / 1000U);
        }
      }
      return true;
    }

    RCLCPP_WARN(logger, "Unsupported Gazebo mono image pixel format: %d", static_cast<int>(gz_msg.pixel_format_type()));
    return false;
  }

  rclcpp::Time resolveStamp(const gz::msgs::Image & gz_msg) const
  {
    if (gz_msg.has_header() && gz_msg.header().has_stamp()) {
      return rclcpp::Time(gz_msg.header().stamp().sec(), gz_msg.header().stamp().nsec(), RCL_ROS_TIME);
    }
    return this->now();
  }

  bool acceptStamp(const rclcpp::Time & stamp)
  {
    const int64_t stamp_ns = stamp.nanoseconds();
    if (!last_image_stamp_ns_.has_value() || stamp_ns > *last_image_stamp_ns_) {
      last_image_stamp_ns_ = stamp_ns;
      return true;
    }
    return false;
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(const sensor_msgs::msg::Image & image_msg) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header = image_msg.header;
    info.width = image_msg.width;
    info.height = image_msg.height;
    const double width = std::max(1.0, static_cast<double>(image_msg.width));
    const double height = std::max(1.0, static_cast<double>(image_msg.height));
    const double fx = camera_hfov_rad_ > 1.0e-6 ? (width / (2.0 * std::tan(camera_hfov_rad_ / 2.0))) : (width / 2.0);
    const double fy = fx;
    const double cx = width / 2.0;
    const double cy = height / 2.0;
    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    return info;
  }

  void onGzImage(const gz::msgs::Image & gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_msg.header.stamp = resolveStamp(gz_msg);
    if (!acceptStamp(ros_msg.header.stamp)) {
      return;
    }
    ros_msg.header.frame_id = frame_id_;
    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    if (!fillRosMonoImage(gz_msg, ros_msg, this->get_logger())) {
      return;
    }
    image_pub_->publish(ros_msg);
    camera_info_pub_->publish(makeCameraInfo(ros_msg));
  }

  std::string frame_id_;
  double camera_hfov_rad_{1.3962634};
  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::optional<int64_t> last_image_stamp_ns_;
};
}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::MonoCameraBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
