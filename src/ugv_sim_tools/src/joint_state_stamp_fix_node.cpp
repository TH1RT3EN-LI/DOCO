#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateStampFixNode : public rclcpp::Node
{
public:
  explicit JointStateStampFixNode(const rclcpp::NodeOptions & options)
  : Node("joint_state_stamp_fix", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/ugv/joint_states_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/ugv/joint_states");

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(50));
    input_qos.best_effort();
    input_qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic, rclcpp::QoS(20));
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic,
      input_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        auto output = *msg;
        output.header.stamp = get_clock()->now();
        publisher_->publish(output);
      });

    RCLCPP_INFO(
      get_logger(),
      "joint_state_stamp_fix started: %s -> %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<JointStateStampFixNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
