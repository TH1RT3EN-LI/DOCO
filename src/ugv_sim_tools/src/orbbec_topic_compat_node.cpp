#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ugv_sim_tools/point_cloud_utils.hpp"

class OrbbecTopicCompatNode : public rclcpp::Node
{
public:
  explicit OrbbecTopicCompatNode(const rclcpp::NodeOptions & options)
  : Node("orbbec_topic_compat_node", options)
  {
    input_color_image_topic_ =
      declare_parameter<std::string>("input_color_image_topic", "/ugv/camera_sim/color/image_raw");
    input_color_info_topic_ =
      declare_parameter<std::string>("input_color_info_topic", "/ugv/camera_sim/color/camera_info");
    input_depth_image_topic_ =
      declare_parameter<std::string>("input_depth_image_topic", "/ugv/camera_sim/depth/image_raw");
    input_depth_info_topic_ =
      declare_parameter<std::string>("input_depth_info_topic", "/ugv/camera_sim/depth/camera_info");
    input_depth_points_topic_ =
      declare_parameter<std::string>("input_depth_points_topic", "/ugv/camera_sim/depth/points");

    output_color_image_topic_ =
      declare_parameter<std::string>("output_color_image_topic", "/ugv/camera/color/image_raw");
    output_color_info_topic_ =
      declare_parameter<std::string>("output_color_info_topic", "/ugv/camera/color/camera_info");
    output_depth_image_topic_ =
      declare_parameter<std::string>("output_depth_image_topic", "/ugv/camera/depth/image_raw");
    output_depth_info_topic_ =
      declare_parameter<std::string>("output_depth_info_topic", "/ugv/camera/depth/camera_info");
    output_depth_points_topic_ =
      declare_parameter<std::string>("output_depth_points_topic", "/ugv/camera/depth/points");
    output_ir_image_topic_ =
      declare_parameter<std::string>("output_ir_image_topic", "/ugv/camera/ir/image_raw");
    output_ir_info_topic_ =
      declare_parameter<std::string>("output_ir_info_topic", "/ugv/camera/ir/camera_info");

    color_frame_id_ =
      declare_parameter<std::string>("color_frame_id", "ugv_camera_color_optical_frame");
    depth_frame_id_ =
      declare_parameter<std::string>("depth_frame_id", "ugv_camera_depth_optical_frame");
    ir_frame_id_ = declare_parameter<std::string>("ir_frame_id", "ugv_camera_ir_optical_frame");
    cloud_frame_id_ =
      declare_parameter<std::string>("cloud_frame_id", "ugv_camera_depth_optical_frame");
    publish_fake_ir_ = declare_parameter<bool>("publish_fake_ir", true);
    transform_cloud_to_optical_ = declare_parameter<bool>("transform_cloud_to_optical", true);

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_qos.reliable();
    pub_qos.durability_volatile();

    color_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(output_color_image_topic_, pub_qos);
    color_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(output_color_info_topic_, pub_qos);
    depth_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(output_depth_image_topic_, pub_qos);
    depth_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(output_depth_info_topic_, pub_qos);
    depth_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(output_depth_points_topic_, pub_qos);
    ir_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>(output_ir_image_topic_, pub_qos);
    ir_info_publisher_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(output_ir_info_topic_, pub_qos);

    color_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_color_image_topic_,
      sub_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.frame_id = color_frame_id_;
        color_image_publisher_->publish(output);
      });
    color_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_color_info_topic_,
      sub_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.frame_id = color_frame_id_;
        color_info_publisher_->publish(output);
      });
    depth_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_depth_image_topic_,
      sub_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.frame_id = depth_frame_id_;
        depth_image_publisher_->publish(output);
        if (publish_fake_ir_) {
          auto ir_output = output;
          ir_output.header.frame_id = ir_frame_id_;
          ir_image_publisher_->publish(ir_output);
        }
      });
    depth_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      input_depth_info_topic_,
      sub_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.frame_id = depth_frame_id_;
        depth_info_publisher_->publish(output);
        if (publish_fake_ir_) {
          auto ir_output = output;
          ir_output.header.frame_id = ir_frame_id_;
          ir_info_publisher_->publish(ir_output);
        }
      });
    depth_points_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_depth_points_topic_,
      sub_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        onDepthPoints(msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "orbbec_topic_compat_node started: %s -> %s, %s -> %s, %s -> %s, fake_ir=%s",
      input_color_image_topic_.c_str(),
      output_color_image_topic_.c_str(),
      input_depth_image_topic_.c_str(),
      output_depth_image_topic_.c_str(),
      input_depth_points_topic_.c_str(),
      output_depth_points_topic_.c_str(),
      publish_fake_ir_ ? "on" : "off");
  }

private:
  void onDepthPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    sensor_msgs::msg::PointCloud2 output = *msg;

    if (transform_cloud_to_optical_) {
      const auto result = ugv_sim_tools::transformCloudCameraToOptical(*msg);
      if (result.status == ugv_sim_tools::PointCloudTransformStatus::kMissingFields) {
        warnCloudFormatOnce(result.warning_message);
        return;
      }
      if (result.status == ugv_sim_tools::PointCloudTransformStatus::kCopiedWithoutTransform &&
        !result.warning_message.empty())
      {
        warnCloudFormatOnce(result.warning_message);
      }
      output = result.cloud;
    }

    output.header.frame_id = cloud_frame_id_;
    depth_points_publisher_->publish(output);
  }

  void warnCloudFormatOnce(const std::string & message)
  {
    if (warned_cloud_format_) {
      return;
    }
    warned_cloud_format_ = true;
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  std::string input_color_image_topic_;
  std::string input_color_info_topic_;
  std::string input_depth_image_topic_;
  std::string input_depth_info_topic_;
  std::string input_depth_points_topic_;
  std::string output_color_image_topic_;
  std::string output_color_info_topic_;
  std::string output_depth_image_topic_;
  std::string output_depth_info_topic_;
  std::string output_depth_points_topic_;
  std::string output_ir_image_topic_;
  std::string output_ir_info_topic_;
  std::string color_frame_id_;
  std::string depth_frame_id_;
  std::string ir_frame_id_;
  std::string cloud_frame_id_;
  bool publish_fake_ir_{true};
  bool transform_cloud_to_optical_{true};
  bool warned_cloud_format_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_info_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_points_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<OrbbecTopicCompatNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
