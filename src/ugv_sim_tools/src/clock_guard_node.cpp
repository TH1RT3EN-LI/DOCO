#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "ugv_sim_tools/clock_guard_logic.hpp"

namespace
{

int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000LL * 1000LL * 1000LL +
         static_cast<int64_t>(stamp.nanosec);
}

}

class ClockGuardNode : public rclcpp::Node
{
public:
  explicit ClockGuardNode(const rclcpp::NodeOptions & options)
  : Node("ugv_clock_guard", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/sim/clock_raw");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/clock");
    config_.allow_clock_reset = declare_parameter<bool>("allow_clock_reset", true);
    config_.reset_newer_than_ns = static_cast<int64_t>(
      declare_parameter<double>("reset_newer_than_sec", 30.0) * 1000.0 * 1000.0 * 1000.0);
    config_.reset_older_than_ns = static_cast<int64_t>(
      declare_parameter<double>("reset_older_than_sec", 5.0) * 1000.0 * 1000.0 * 1000.0);

    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    input_qos.best_effort();
    input_qos.durability_volatile();

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.reliable();
    output_qos.durability_volatile();

    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(output_topic, output_qos);
    subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      input_topic,
      input_qos,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        onClock(msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "clock_guard started: %s -> %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void onClock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & msg)
  {
    const auto now_ns = stampToNanoseconds(msg->clock);
    const auto previous_last_ns = last_ns_;
    const auto decision = ugv_sim_tools::evaluateClockGuard(previous_last_ns, now_ns, config_);

    if (decision.action == ugv_sim_tools::ClockGuardAction::kDrop) {
      return;
    }

    last_ns_ = decision.next_last_ns;
    if (
      decision.action == ugv_sim_tools::ClockGuardAction::kForwardReset &&
      previous_last_ns.has_value())
    {
      RCLCPP_WARN(
        get_logger(),
        "clock reset detected: %.3fs -> %.3fs, accepting new epoch",
        static_cast<double>(*previous_last_ns) / 1e9,
        static_cast<double>(now_ns) / 1e9);
    }

    publisher_->publish(*msg);
  }

  ugv_sim_tools::ClockGuardConfig config_;
  std::optional<int64_t> last_ns_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<ClockGuardNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
