#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanFrameRewriterNode : public rclcpp::Node
{
public:
  explicit ScanFrameRewriterNode(const rclcpp::NodeOptions & options)
  : Node("scan_frame_rewriter", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/ugv/scan_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/ugv/scan");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "ugv_laser");
    restamp_with_now_ = declare_parameter<bool>("restamp_with_now", true);

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    pub_qos.reliable();
    pub_qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, pub_qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic,
      sub_qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.frame_id = output_frame_id_;
        if (restamp_with_now_) {
          output.header.stamp = get_clock()->now();
        }
        publisher_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(),
      "scan_frame_rewriter started: %s (BEST_EFFORT) -> %s (RELIABLE), frame_id='%s'",
      input_topic.c_str(),
      output_topic.c_str(),
      output_frame_id_.c_str());
  }

private:
  std::string output_frame_id_;
  bool restamp_with_now_{true};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<ScanFrameRewriterNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
