#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ugv_sim_tools/initial_pose_logic.hpp"

using namespace std::chrono_literals;

class InitialPosePublisherNode : public rclcpp::Node
{
public:
  explicit InitialPosePublisherNode(const rclcpp::NodeOptions & options)
  : Node("initial_pose_publisher", options),
    start_monotonic_(std::chrono::steady_clock::now()),
    last_status_log_monotonic_(start_monotonic_)
  {
    topic_ = declare_parameter<std::string>("topic", "/ugv/initialpose");
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    x_ = declare_parameter<double>("x", 0.0);
    y_ = declare_parameter<double>("y", 0.0);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    const auto delay_sec = declare_parameter<double>("delay_sec", 1.0);
    const auto publish_count = declare_parameter<int>("publish_count", 5);
    const auto publish_period_sec = declare_parameter<double>("publish_period_sec", 0.2);
    const auto require_subscriber = declare_parameter<bool>("require_subscriber", true);
    const auto wait_for_amcl_pose = declare_parameter<bool>("wait_for_amcl_pose", true);
    max_wait_sec_ = declare_parameter<double>("max_wait_sec", 120.0);
    status_log_period_sec_ =
      std::max(0.1, declare_parameter<double>("status_log_period_sec", 2.0));

    state_machine_ = std::make_unique<ugv_sim_tools::InitialPoseStateMachine>(
      ugv_sim_tools::InitialPoseConfig{
      static_cast<int>(publish_count),
      require_subscriber,
      wait_for_amcl_pose,
      max_wait_sec_});

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_, 10);
    amcl_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_,
      10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr) {
        if (state_machine_->onAmclPoseReceived()) {
          markDone("AMCL pose received; initial pose accepted, shutting down.");
        }
      });

    start_timer_ = rclcpp::create_timer(
      get_node_base_interface(),
      get_node_timers_interface(),
      get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          delay_sec)),
      [this, publish_period_sec]() {
        start_timer_->cancel();
        publish_timer_ = rclcpp::create_timer(
          get_node_base_interface(),
          get_node_timers_interface(),
          get_clock(),
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(publish_period_sec)),
          [this]() {onPublishTimer();});
        RCLCPP_INFO(
          get_logger(),
          "initial_pose_publisher active; waiting for Nav2/AMCL to consume the pose.");
      });

    RCLCPP_INFO(
      get_logger(),
      "initial_pose_publisher armed: will publish to %s in frame '%s' after %.2fs "
      "(x=%.3f, y=%.3f, yaw=%.3f, initial_burst=%d, wait_for_amcl_pose=%s)",
      topic_.c_str(),
      frame_id_.c_str(),
      delay_sec,
      x_,
      y_,
      yaw_,
      ugv_sim_tools::clampPublishCount(publish_count),
      wait_for_amcl_pose ? "true" : "false");
  }

  bool isDone() const
  {
    return done_;
  }

private:
  void markDone(const std::string & message)
  {
    if (done_) {
      return;
    }
    done_ = true;
    if (start_timer_) {
      start_timer_->cancel();
    }
    if (publish_timer_) {
      publish_timer_->cancel();
    }
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  }

  void logStatus(const std::string & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_status_log_monotonic_ >= std::chrono::duration<double>(status_log_period_sec_)) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
      last_status_log_monotonic_ = now;
    }
  }

  void onPublishTimer()
  {
    if (done_) {
      return;
    }

    const double elapsed_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_monotonic_).count();
    const auto result = state_machine_->tick(count_subscribers(topic_) > 0, elapsed_sec);

    if (result.done_now && result.timed_out) {
      markDone(
        "Timed out after " + std::to_string(max_wait_sec_) +
        "s waiting for AMCL to accept the initial pose.");
      return;
    }

    if (result.waiting_for_subscriber) {
      logStatus(
        "Waiting for a subscriber on " + topic_ + " before publishing the initial pose...");
      return;
    }

    if (!result.should_publish) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.orientation = ugv_sim_tools::yawToQuaternion(yaw_);

    const auto covariance = ugv_sim_tools::buildConservativeInitialPoseCovariance();
    std::copy(covariance.begin(), covariance.end(), msg.pose.covariance.begin());

    publisher_->publish(msg);

    if (result.initial_burst_completed_now &&
      state_machine_->phase() == ugv_sim_tools::InitialPosePhase::kWaitingForAmclPose)
    {
      RCLCPP_INFO(
        get_logger(),
        "Published initial burst (%dx); continuing retries until %s arrives.",
        result.published_count,
        amcl_pose_topic_.c_str());
    }

    if (result.done_now) {
      markDone("Initial pose burst published; shutting down.");
      return;
    }

    if (state_machine_->phase() == ugv_sim_tools::InitialPosePhase::kWaitingForAmclPose) {
      logStatus(
        "Published " + std::to_string(result.published_count) +
        " initial pose message(s); still waiting for " + amcl_pose_topic_ + "...");
    }
  }

  std::string topic_;
  std::string amcl_pose_topic_;
  std::string frame_id_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double max_wait_sec_{120.0};
  double status_log_period_sec_{2.0};
  bool done_{false};
  std::chrono::steady_clock::time_point start_monotonic_;
  std::chrono::steady_clock::time_point last_status_log_monotonic_;
  std::unique_ptr<ugv_sim_tools::InitialPoseStateMachine> state_machine_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    amcl_pose_subscription_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<InitialPosePublisherNode>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !node->isDone()) {
    executor.spin_some(100ms);
    std::this_thread::sleep_for(10ms);
  }

  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
