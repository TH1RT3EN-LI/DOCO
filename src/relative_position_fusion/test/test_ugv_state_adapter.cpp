#include "relative_position_fusion/covariance_utils.hpp"
#include "relative_position_fusion/ugv_state_adapter.hpp"

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <builtin_interfaces/msg/time.hpp>

#include <memory>

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

class UgvStateAdapterTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("ugv_state_adapter_test_node");
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    buffer_->setUsingDedicatedThread(true);
    AddIdentityTransform("global", "ugv_map");
    AddIdentityTransform("global", "ugv_odom");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
};

TEST_F(UgvStateAdapterTest, PrefersAmclPoseAndFilteredVelocity)
{
  UgvStateAdapter::Config config;
  config.global_frame = "global";
  config.filtered_twist_in_child_frame = true;
  UgvStateAdapter adapter(node_.get(), buffer_, config);

  geometry_msgs::msg::PoseWithCovarianceStamped amcl;
  amcl.header.stamp = MakeStamp(5);
  amcl.header.frame_id = "global";
  amcl.pose.pose.position.x = 5.0;
  amcl.pose.pose.position.y = 6.0;
  amcl.pose.pose.orientation = YawToQuaternion(1.5707963267948966);
  amcl.pose.covariance[0] = 0.2;
  amcl.pose.covariance[7] = 0.3;
  adapter.HandleAmclPose(amcl);

  nav_msgs::msg::Odometry filtered;
  filtered.header.stamp = MakeStamp(5);
  filtered.header.frame_id = "global";
  filtered.pose.pose.orientation = YawToQuaternion(1.5707963267948966);
  filtered.twist.twist.linear.x = 1.0;
  filtered.twist.twist.linear.y = 0.0;
  adapter.HandleFilteredOdometry(filtered);

  const AgentPlanarState state = adapter.BuildState(rclcpp::Time(5, 1000, RCL_ROS_TIME));
  EXPECT_TRUE(state.pose_valid);
  EXPECT_TRUE(state.velocity_valid);
  EXPECT_FALSE(state.pose_from_fallback);
  EXPECT_FALSE(state.velocity_from_fallback);
  EXPECT_EQ(state.pose_source, "/amcl_pose");
  EXPECT_EQ(state.velocity_source, "/ugv/odometry/filtered");
  EXPECT_NEAR(state.v_global.x(), 0.0, 1e-6);
  EXPECT_NEAR(state.v_global.y(), 1.0, 1e-6);
}

TEST_F(UgvStateAdapterTest, FallsBackToOdomPoseWhenEnabled)
{
  UgvStateAdapter::Config config;
  config.global_frame = "global";
  config.assume_odom_is_global = true;
  config.odom_twist_in_child_frame = false;
  UgvStateAdapter adapter(node_.get(), buffer_, config);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = MakeStamp(7);
  odom.header.frame_id = "global";
  odom.pose.pose.position.x = 2.0;
  odom.pose.pose.position.y = 3.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 0.5;
  odom.twist.twist.linear.y = 0.1;
  odom.pose.covariance[0] = 0.8;
  odom.pose.covariance[7] = 0.9;
  adapter.HandleBaseOdometry(odom);

  const AgentPlanarState state = adapter.BuildState(rclcpp::Time(7, 1000, RCL_ROS_TIME));
  EXPECT_TRUE(state.pose_valid);
  EXPECT_TRUE(state.velocity_valid);
  EXPECT_TRUE(state.pose_from_fallback);
  EXPECT_TRUE(state.velocity_from_fallback);
  EXPECT_EQ(state.pose_source, "/ugv/odom");
  EXPECT_EQ(state.velocity_source, "/ugv/odom");
  EXPECT_NEAR(state.p_global.x(), 2.0, 1e-9);
  EXPECT_NEAR(state.p_global.y(), 3.0, 1e-9);
}

}  // namespace relative_position_fusion
