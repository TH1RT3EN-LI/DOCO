#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace uav_bridge {
class StereoCameraBridgeNode : public rclcpp::Node {
public:
  StereoCameraBridgeNode() : Node("stereo_camera_bridge_node") {
    this->declare_parameter<std::string>("gz_rgb_image_topic", "");
    this->declare_parameter<std::string>("gz_depth_image_topic", "");
    this->declare_parameter<std::string>("gz_points_topic", "");
    this->declare_parameter<std::string>("world_name", "baylands_world");
    this->declare_parameter<std::string>("model_name", "uav");
    this->declare_parameter<std::string>("link_name", "base_link");
    this->declare_parameter<std::string>("rgb_sensor_name",
                                         "stereo_depth_camera_rgb");
    this->declare_parameter<std::string>("depth_sensor_name",
                                         "stereo_depth_camera_depth");
    this->declare_parameter<std::string>("ros_rgb_topic",
                                         "/uav/stereo/rgb/image_raw");
    this->declare_parameter<std::string>("ros_depth_topic",
                                         "/uav/stereo/depth/image_raw");
    this->declare_parameter<std::string>("ros_points_topic",
                                         "/uav/stereo/depth/points");
    this->declare_parameter<std::string>("frame_id",
                                         "stereo_camera_optical_frame");
    this->declare_parameter<int>("points_downsample_step", 4);

    std::string gz_rgb_image_topic =
        this->get_parameter("gz_rgb_image_topic").as_string();
    std::string gz_depth_image_topic =
        this->get_parameter("gz_depth_image_topic").as_string();
    std::string gz_points_topic =
        this->get_parameter("gz_points_topic").as_string();
    const std::string world_name =
        this->get_parameter("world_name").as_string();
    const std::string model_name =
        this->get_parameter("model_name").as_string();
    const std::string link_name = this->get_parameter("link_name").as_string();
    const std::string rgb_sensor_name =
        this->get_parameter("rgb_sensor_name").as_string();
    const std::string depth_sensor_name =
        this->get_parameter("depth_sensor_name").as_string();
    const std::string ros_rgb_topic =
        this->get_parameter("ros_rgb_topic").as_string();
    const std::string ros_depth_topic =
        this->get_parameter("ros_depth_topic").as_string();
    const std::string ros_points_topic =
        this->get_parameter("ros_points_topic").as_string();

    _rgb_frame_id = this->get_parameter("frame_id").as_string();
    _depth_frame_id = _rgb_frame_id;
    _points_frame_id = _rgb_frame_id;
    points_downsample_step_ = std::max(
        1, static_cast<int>(
               this->get_parameter("points_downsample_step").as_int()));

    if (gz_rgb_image_topic.empty()) {
      gz_rgb_image_topic = "/world/" + world_name + "/model/" + model_name +
                           "/link/" + link_name + "/sensor/" + rgb_sensor_name +
                           "/image";
    }
    if (gz_depth_image_topic.empty()) {
      gz_depth_image_topic = "/world/" + world_name + "/model/" + model_name +
                             "/link/" + link_name + "/sensor/" +
                             depth_sensor_name + "/depth_image";
    }
    if (gz_points_topic.empty()) {
      gz_points_topic = "/world/" + world_name + "/model/" + model_name +
                        "/link/" + link_name + "/sensor/" + depth_sensor_name +
                        "/depth_image/points";
    }

    rclcpp::QoS image_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS depth_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    rclcpp::QoS points_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    _rgb_pub = this->create_publisher<sensor_msgs::msg::Image>(ros_rgb_topic,
                                                               image_qos);
    _depth_pub = this->create_publisher<sensor_msgs::msg::Image>(
        ros_depth_topic, depth_qos);
    _points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ros_points_topic, points_qos);

    _gz_node.Subscribe(gz_rgb_image_topic,
                       &StereoCameraBridgeNode::OnGzRgbImage, this);
    _gz_node.Subscribe(gz_depth_image_topic,
                       &StereoCameraBridgeNode::OnGzDepthImage, this);
    _gz_node.Subscribe(gz_points_topic, &StereoCameraBridgeNode::OnGzPoints,
                       this);

    RCLCPP_INFO(this->get_logger(),
                "[stereo rgb topic] Gazebo topic: %s -> Ros topic: %s",
                gz_rgb_image_topic.c_str(), ros_rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "[stereo depth topic] Gazebo topic: %s -> Ros topic: %s",
                gz_depth_image_topic.c_str(), ros_depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "[stereo points topic] Gazebo topic: %s -> Ros topic: %s",
                gz_points_topic.c_str(), ros_points_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "[stereo points] downsample step: %d",
                points_downsample_step_);
  }

private:
  std::string _rgb_frame_id;
  std::string _depth_frame_id;
  std::string _points_frame_id;
  int points_downsample_step_{4};
  std::optional<int64_t> last_rgb_stamp_ns_;
  std::optional<int64_t> last_depth_stamp_ns_;
  std::optional<int64_t> last_points_stamp_ns_;

  gz::transport::Node _gz_node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _rgb_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _points_pub;

  rclcpp::Time ResolveStamp(const gz::msgs::Image &gz_msg) const {
    return rclcpp::Time(gz_msg.header().stamp().sec(),
                        gz_msg.header().stamp().nsec(), RCL_ROS_TIME);
  }

  rclcpp::Time ResolveStamp(const gz::msgs::PointCloudPacked &gz_msg) const {
    return rclcpp::Time(gz_msg.header().stamp().sec(),
                        gz_msg.header().stamp().nsec(), RCL_ROS_TIME);
  }

  bool AcceptStamp(const rclcpp::Time &stamp,
                   std::optional<int64_t> &last_stamp_ns,
                   const char *stream_name) {
    const int64_t stamp_ns = stamp.nanoseconds();
    if (!last_stamp_ns.has_value() || stamp_ns > *last_stamp_ns) {
      last_stamp_ns = stamp_ns;
      return true;
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dropping out-of-order %s: current=%ld last=%ld",
                         stream_name, static_cast<long>(stamp_ns),
                         static_cast<long>(*last_stamp_ns));
    return false;
  }

  static uint8_t
  ToRosPointFieldDatatype(gz::msgs::PointCloudPacked_Field::DataType type) {
    switch (type) {
    case gz::msgs::PointCloudPacked_Field::INT8:
      return sensor_msgs::msg::PointField::INT8;
    case gz::msgs::PointCloudPacked_Field::UINT8:
      return sensor_msgs::msg::PointField::UINT8;
    case gz::msgs::PointCloudPacked_Field::INT16:
      return sensor_msgs::msg::PointField::INT16;
    case gz::msgs::PointCloudPacked_Field::UINT16:
      return sensor_msgs::msg::PointField::UINT16;
    case gz::msgs::PointCloudPacked_Field::INT32:
      return sensor_msgs::msg::PointField::INT32;
    case gz::msgs::PointCloudPacked_Field::UINT32:
      return sensor_msgs::msg::PointField::UINT32;
    case gz::msgs::PointCloudPacked_Field::FLOAT32:
      return sensor_msgs::msg::PointField::FLOAT32;
    case gz::msgs::PointCloudPacked_Field::FLOAT64:
      return sensor_msgs::msg::PointField::FLOAT64;
    default:
      break;
    }
    return sensor_msgs::msg::PointField::FLOAT32;
  }

  static void
  RotatePointCloudToOptical(sensor_msgs::msg::PointCloud2 &ros_msg) {
    const size_t point_count =
        static_cast<size_t>(ros_msg.width) * ros_msg.height;
    uint8_t *data = ros_msg.data.data();
    for (size_t i = 0; i < point_count; ++i) {
      uint8_t *p = data + i * ros_msg.point_step;
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::memcpy(&x, p + 0, sizeof(float));
      std::memcpy(&y, p + 4, sizeof(float));
      std::memcpy(&z, p + 8, sizeof(float));

      const float x_opt = -y;
      const float y_opt = -z;
      const float z_opt = x;

      std::memcpy(p + 0, &x_opt, sizeof(float));
      std::memcpy(p + 4, &y_opt, sizeof(float));
      std::memcpy(p + 8, &z_opt, sizeof(float));
    }
  }

  void OnGzRgbImage(const gz::msgs::Image &gz_msg) {
    sensor_msgs::msg::Image ros_msg;

    ros_msg.header.stamp = ResolveStamp(gz_msg);
    if (!AcceptStamp(ros_msg.header.stamp, last_rgb_stamp_ns_, "stereo rgb")) {
      return;
    }
    ros_msg.header.frame_id = _rgb_frame_id;

    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    ros_msg.encoding = sensor_msgs::image_encodings::RGB8;

    ros_msg.is_bigendian = false;
    ros_msg.step = gz_msg.step();

    const std::string &data = gz_msg.data();
    ros_msg.data.assign(data.begin(), data.end());

    _rgb_pub->publish(std::move(ros_msg));
  }

  void OnGzDepthImage(const gz::msgs::Image &gz_msg) {
    sensor_msgs::msg::Image ros_msg;

    ros_msg.header.stamp = ResolveStamp(gz_msg);
    if (!AcceptStamp(ros_msg.header.stamp, last_depth_stamp_ns_,
                     "stereo depth")) {
      return;
    }
    ros_msg.header.frame_id = _depth_frame_id;

    ros_msg.height = gz_msg.height();
    ros_msg.width = gz_msg.width();
    ros_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    ros_msg.is_bigendian = false;
    ros_msg.step = gz_msg.step();

    const std::string &data = gz_msg.data();
    ros_msg.data.assign(data.begin(), data.end());

    _depth_pub->publish(std::move(ros_msg));
  }

  void OnGzPoints(const gz::msgs::PointCloudPacked &gz_msg) {
    sensor_msgs::msg::PointCloud2 ros_msg;

    ros_msg.header.stamp = ResolveStamp(gz_msg);
    if (!AcceptStamp(ros_msg.header.stamp, last_points_stamp_ns_,
                     "stereo points")) {
      return;
    }
    ros_msg.header.frame_id = _points_frame_id;

    const uint32_t src_height = gz_msg.height();
    const uint32_t src_width = gz_msg.width();
    const uint32_t point_step = gz_msg.point_step();
    const uint32_t src_row_step = gz_msg.row_step();
    const uint32_t step = static_cast<uint32_t>(points_downsample_step_);
    const uint32_t dst_height = (src_height + step - 1u) / step;
    const uint32_t dst_width = (src_width + step - 1u) / step;

    ros_msg.height = dst_height;
    ros_msg.width = dst_width;
    ros_msg.point_step = point_step;
    ros_msg.row_step = point_step * dst_width;
    ros_msg.data.resize(static_cast<size_t>(ros_msg.row_step) * dst_height);

    const std::string &src_data = gz_msg.data();
    uint8_t *dst = ros_msg.data.data();

    for (uint32_t v = 0; v < dst_height; ++v) {
      const uint32_t src_v = v * step;
      const size_t src_row_offset = static_cast<size_t>(src_v) * src_row_step;
      const size_t dst_row_offset = static_cast<size_t>(v) * ros_msg.row_step;
      for (uint32_t u = 0; u < dst_width; ++u) {
        const uint32_t src_u = u * step;
        const size_t src_offset =
            src_row_offset + static_cast<size_t>(src_u) * point_step;
        const size_t dst_offset =
            dst_row_offset + static_cast<size_t>(u) * point_step;
        std::memcpy(dst + dst_offset, src_data.data() + src_offset, point_step);
      }
    }

    ros_msg.is_bigendian = gz_msg.is_bigendian();
    ros_msg.is_dense = gz_msg.is_dense();

    ros_msg.fields.resize(gz_msg.field_size());
    for (int i = 0; i < gz_msg.field_size(); ++i) {
      const auto &gz_field = gz_msg.field(i);
      auto &ros_field = ros_msg.fields[static_cast<size_t>(i)];
      ros_field.name = gz_field.name();
      ros_field.offset = gz_field.offset();
      ros_field.datatype = ToRosPointFieldDatatype(gz_field.datatype());
      ros_field.count = gz_field.count();
    }

    RotatePointCloudToOptical(ros_msg);

    _points_pub->publish(std::move(ros_msg));
  }
};
} // namespace uav_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::StereoCameraBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
