#include "relative_position_fusion/uav_state_adapter.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <memory>
#include <limits>

namespace relative_position_fusion
{

namespace
{

builtin_interfaces::msg::Time MakeStamp(int32_t sec, uint32_t nanosec = 0U)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

}  // namespace

class UavStateAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void AddIdentityTransform(const std::string & parent, const std::string & child)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = node_->now();
    tf_msg.header.frame_id = parent;
    tf_msg.child_frame_id = child;
    tf_msg.transform.rotation.w = 1.0;
    buffer_->setTransform(tf_msg, "test", true);
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("uav_state_adapter_test_node");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    buffer_->setUsingDedicatedThread(true);
    AddIdentityTransform("global", "uav_map");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};

TEST_F(UavStateAdapterTest, BuildsStateFromOdometry)
{
  UavStateAdapter::Config config;
  config.global_frame = "global";
  config.odom_topic = "/uav/odom";
  config.tf_lookup_timeout_sec = 0.0;
  UavStateAdapter adapter(node_.get(), buffer_, config);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = MakeStamp(10);
  msg.header.frame_id = "uav_map";
  msg.child_frame_id = "uav_base_link";
  msg.pose.pose.position.x = 1.0;
  msg.pose.pose.position.y = 2.0;
  msg.pose.pose.orientation.w = 1.0;
  msg.twist.twist.linear.x = 0.3;
  msg.twist.twist.linear.y = -0.2;
  msg.pose.covariance[0] = 0.4;
  msg.pose.covariance[7] = 0.6;
  adapter.HandleOdometry(msg);

  const AgentPlanarState state = adapter.BuildState(rclcpp::Time(10, 0, RCL_ROS_TIME));
  EXPECT_TRUE(state.pose_valid);
  EXPECT_TRUE(state.velocity_valid);
  EXPECT_NEAR(state.p_global.x(), 1.0, 1e-9);
  EXPECT_NEAR(state.p_global.y(), 2.0, 1e-9);
  EXPECT_NEAR(state.v_global.x(), 0.3, 1e-9);
  EXPECT_NEAR(state.v_global.y(), -0.2, 1e-9);
  EXPECT_NEAR(state.sigma_p(0, 0), 0.4, 1e-9);
  EXPECT_NEAR(state.sigma_p(1, 1), 0.6, 1e-9);
}

TEST_F(UavStateAdapterTest, FallsBackToPoseDifferentiationForVelocity)
{
  UavStateAdapter::Config config;
  config.global_frame = "global";
  config.velocity_timeout_sec = 1.0;
  config.max_pose_diff_velocity_dt_sec = 2.0;
  config.tf_lookup_timeout_sec = 0.0;
  UavStateAdapter adapter(node_.get(), buffer_, config);

  nav_msgs::msg::Odometry msg0;
  msg0.header.stamp = MakeStamp(1);
  msg0.header.frame_id = "uav_map";
  msg0.pose.pose.orientation.w = 1.0;
  msg0.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg0.pose.pose.position.x = 0.0;
  msg0.pose.pose.position.y = 0.0;
  adapter.HandleOdometry(msg0);

  nav_msgs::msg::Odometry msg1 = msg0;
  msg1.header.stamp = MakeStamp(2);
  msg1.pose.pose.position.x = 1.0;
  msg1.pose.pose.position.y = 0.5;
  adapter.HandleOdometry(msg1);

  const AgentPlanarState state = adapter.BuildState(rclcpp::Time(2, 0, RCL_ROS_TIME));
  EXPECT_TRUE(state.velocity_valid);
  EXPECT_TRUE(state.velocity_from_fallback);
  EXPECT_NEAR(state.v_global.x(), 1.0, 1e-9);
  EXPECT_NEAR(state.v_global.y(), 0.5, 1e-9);
}

}  // namespace relative_position_fusion
