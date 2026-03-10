#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace uav_bridge {
constexpr double kPi = 3.14159265358979323846;

class TfBridgeNode : public rclcpp::Node {
public:
  TfBridgeNode() : Node("tf_bridge_node") {
    this->declare_parameter<std::string>("gz_pose_topic", "");
    this->declare_parameter<std::string>("world_name", "baylands_world");
    this->declare_parameter<std::string>("model_name", "uav");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("odom_topic", "/uav/odom");
    this->declare_parameter<bool>("publish_odometry", true);
    this->declare_parameter<bool>("use_initial_pose_as_map_origin", false);

    this->gz_pose_topic_ = this->get_parameter("gz_pose_topic").as_string();
    this->world_name_ = this->get_parameter("world_name").as_string();
    this->model_name_ = this->get_parameter("model_name").as_string();
    this->map_frame_ = this->get_parameter("map_frame").as_string();
    this->base_frame_ = this->get_parameter("base_frame").as_string();
    this->odom_topic_ = this->get_parameter("odom_topic").as_string();
    this->publish_odometry_ = this->get_parameter("publish_odometry").as_bool();
    this->use_initial_pose_as_map_origin_ =
        this->get_parameter("use_initial_pose_as_map_origin").as_bool();

    if (this->gz_pose_topic_.empty()) {
      gz_pose_topic_ = "/world/" + this->world_name_ + "/pose/info";
    }

    this->tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    if (this->publish_odometry_) {
      this->odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
          this->odom_topic_, 10);
    }

    bool ok = this->gz_node_.Subscribe(this->gz_pose_topic_,
                                       &TfBridgeNode::OnPoseInfo, this);

    if (!ok) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to subscribe Gazebo pose topic (for tf publish): %s",
                   this->gz_pose_topic_.c_str());
      throw std::runtime_error(
          "Gazebo pose/info subscribe failed(for tf publish)");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[pose->tf] Gazebo topic: %s, model_name: %s, tf: %s -> %s, odom: %s, "
        "use_initial_pose_as_map_origin: %s",
        this->gz_pose_topic_.c_str(), this->model_name_.c_str(),
        this->map_frame_.c_str(), this->base_frame_.c_str(),
        this->publish_odometry_ ? this->odom_topic_.c_str() : "<disabled>",
        this->use_initial_pose_as_map_origin_ ? "true" : "false");
  }

private:
  gz::transport::Node gz_node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string gz_pose_topic_;
  std::string world_name_;
  std::string model_name_;
  std::string resolved_entity_name_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  bool publish_odometry_{true};
  bool use_initial_pose_as_map_origin_{false};

  struct PoseState {
    rclcpp::Time stamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
  };

  std::optional<PoseState> prev_state_;
  std::optional<PoseState> origin_state_;
  std::optional<PoseState> origin_candidate_state_;
  std::optional<int64_t> last_pose_stamp_ns_;
  int origin_candidate_stable_samples_{0};

  static constexpr int kOriginLockRequiredSamples = 12;
  static constexpr double kOriginLockPositionToleranceM = 0.01;

  std::optional<gz::msgs::Pose> FindPoseByName(const gz::msgs::Pose_V &msg,
                                               const std::string &name) const {
    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &pose = msg.pose(i);
      if (pose.name() == name) {
        return pose;
      }
    }
    return std::nullopt;
  }

  std::optional<gz::msgs::Pose> FindModelPose(const gz::msgs::Pose_V &msg) const {
    const std::string model_instance_prefix = this->model_name_ + "_";

    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &pose = msg.pose(i);
      const std::string pose_name = pose.name();

      if (pose_name == this->model_name_ ||
          (pose_name.rfind(model_instance_prefix, 0) == 0 &&
           pose_name.find("::") == std::string::npos)) {
        return pose;
      }
    }
    return std::nullopt;
  }

  std::optional<gz::msgs::Pose> FindRelativeBasePose(const gz::msgs::Pose_V &msg) const {
    if (const auto exact = this->FindPoseByName(msg, this->base_frame_)) {
      return exact;
    }
    return this->FindPoseByName(msg, this->model_name_ + "::" + this->base_frame_);
  }

  PoseState PoseMsgToState(const gz::msgs::Pose &pose,
                           const rclcpp::Time &stamp) const {
    PoseState state;
    state.stamp = stamp;
    state.x = pose.position().x();
    state.y = pose.position().y();
    state.z = pose.position().z();
    state.qx = pose.orientation().x();
    state.qy = pose.orientation().y();
    state.qz = pose.orientation().z();
    state.qw = pose.orientation().w();
    return state;
  }

  PoseState ComposePoseStates(const PoseState &parent,
                              const PoseState &child_relative) const {
    tf2::Quaternion q_parent(parent.qx, parent.qy, parent.qz, parent.qw);
    tf2::Quaternion q_child(child_relative.qx, child_relative.qy,
                            child_relative.qz, child_relative.qw);
    q_parent.normalize();
    q_child.normalize();

    tf2::Vector3 child_translation(child_relative.x, child_relative.y,
                                   child_relative.z);
    const tf2::Vector3 world_translation =
        tf2::quatRotate(q_parent, child_translation);
    const tf2::Quaternion world_rotation = q_parent * q_child;

    PoseState world_state = parent;
    world_state.x += world_translation.x();
    world_state.y += world_translation.y();
    world_state.z += world_translation.z();
    world_state.qx = world_rotation.x();
    world_state.qy = world_rotation.y();
    world_state.qz = world_rotation.z();
    world_state.qw = world_rotation.w();
    return world_state;
  }

  void LogResolvedEntity(const std::string &entity_name) {
    if (this->resolved_entity_name_ != entity_name) {
      this->resolved_entity_name_ = entity_name;
      RCLCPP_INFO(this->get_logger(), "[pose->tf] resolved entity: %s",
                  entity_name.c_str());
    }
  }

  bool EnsureOriginLocked(const PoseState &world_state) {
    if (!this->use_initial_pose_as_map_origin_) {
      return true;
    }

    if (this->origin_state_.has_value()) {
      return true;
    }

    if (!this->origin_candidate_state_.has_value()) {
      this->origin_candidate_state_ = world_state;
      this->origin_candidate_stable_samples_ = 1;
      return false;
    }

    const PoseState &candidate = this->origin_candidate_state_.value();
    const double dx = world_state.x - candidate.x;
    const double dy = world_state.y - candidate.y;
    const double dz = world_state.z - candidate.z;
    const double position_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (position_delta <= kOriginLockPositionToleranceM) {
      this->origin_candidate_state_ = world_state;
      this->origin_candidate_stable_samples_ += 1;
    } else {
      this->origin_candidate_state_ = world_state;
      this->origin_candidate_stable_samples_ = 1;
    }

    if (this->origin_candidate_stable_samples_ < kOriginLockRequiredSamples) {
      return false;
    }

    this->origin_state_ = world_state;
    RCLCPP_INFO(this->get_logger(),
                "[pose->tf] map origin locked to initial UAV pose: "
                "(x=%.3f, y=%.3f, z=%.3f)",
                world_state.x, world_state.y, world_state.z);
    return true;
  }

  void PublishDynamicTf(const PoseState &state) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = state.stamp;
    tf_msg.header.frame_id = this->map_frame_;
    tf_msg.child_frame_id = this->base_frame_;
    tf_msg.transform.translation.x = state.x;
    tf_msg.transform.translation.y = state.y;
    tf_msg.transform.translation.z = state.z;
    tf_msg.transform.rotation.x = state.qx;
    tf_msg.transform.rotation.y = state.qy;
    tf_msg.transform.rotation.z = state.qz;
    tf_msg.transform.rotation.w = state.qw;

    this->tf_broadcaster_->sendTransform(tf_msg);
  }
  void PublishOdometry(const PoseState &state) {
    if (!this->publish_odometry_ || !this->odom_pub_) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = state.stamp;
    odom.header.frame_id = this->map_frame_;
    odom.child_frame_id = this->base_frame_;

    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;
    odom.pose.pose.position.z = state.z;
    odom.pose.pose.orientation.x = state.qx;
    odom.pose.pose.orientation.y = state.qy;
    odom.pose.pose.orientation.z = state.qz;
    odom.pose.pose.orientation.w = state.qw;

    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[14] = 0.02;
    odom.pose.covariance[21] = 0.05;
    odom.pose.covariance[28] = 0.05;
    odom.pose.covariance[35] = 0.05;

    odom.twist.covariance[0] = 0.10;
    odom.twist.covariance[7] = 0.10;
    odom.twist.covariance[14] = 0.10;
    odom.twist.covariance[21] = 0.20;
    odom.twist.covariance[28] = 0.20;
    odom.twist.covariance[35] = 0.20;

    if (this->prev_state_) {
      const double dt = (state.stamp - this->prev_state_->stamp).seconds();
      if (dt > 1e-6) {
        odom.twist.twist.linear.x = (state.x - this->prev_state_->x) / dt;
        odom.twist.twist.linear.y = (state.y - this->prev_state_->y) / dt;
        odom.twist.twist.linear.z = (state.z - this->prev_state_->z) / dt;

        tf2::Quaternion q_prev(this->prev_state_->qx, this->prev_state_->qy,
                               this->prev_state_->qz, this->prev_state_->qw);
        tf2::Quaternion q_curr(state.qx, state.qy, state.qz, state.qw);
        q_prev.normalize();
        q_curr.normalize();

        tf2::Quaternion q_delta = q_prev.inverse() * q_curr;
        q_delta.normalize();

        double angle = q_delta.getAngle();
        tf2::Vector3 axis = q_delta.getAxis();
        if (!std::isfinite(axis.x()) || !std::isfinite(axis.y()) ||
            !std::isfinite(axis.z())) {
          axis = tf2::Vector3(0.0, 0.0, 0.0);
          angle = 0.0;
        }
        if (angle > kPi) {
          angle -= 2.0 * kPi;
        }

        odom.twist.twist.angular.x = axis.x() * angle / dt;
        odom.twist.twist.angular.y = axis.y() * angle / dt;
        odom.twist.twist.angular.z = axis.z() * angle / dt;
      }
    }

    this->odom_pub_->publish(odom);
  }

  rclcpp::Time ResolveStamp(const gz::msgs::Pose_V &msg) const {
    if (msg.has_header() && msg.header().has_stamp()) {
      return rclcpp::Time(msg.header().stamp().sec(),
                          msg.header().stamp().nsec(), RCL_ROS_TIME);
    }
    return this->now();
  }

  bool AcceptPoseStamp(const rclcpp::Time &stamp) {
    const int64_t stamp_ns = stamp.nanoseconds();
    if (!this->last_pose_stamp_ns_.has_value() ||
        stamp_ns > *this->last_pose_stamp_ns_) {
      this->last_pose_stamp_ns_ = stamp_ns;
      return true;
    }

    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping out-of-order pose/info frame: current=%ld last=%ld",
        static_cast<long>(stamp_ns),
        static_cast<long>(*this->last_pose_stamp_ns_));
    return false;
  }

  std::optional<PoseState> ToMapFrameState(const PoseState &world_state) {
    PoseState map_state = world_state;
    if (!this->use_initial_pose_as_map_origin_) {
      return map_state;
    }

    if (!this->EnsureOriginLocked(world_state)) {
      return std::nullopt;
    }

    const PoseState &origin = this->origin_state_.value();
    map_state.x = world_state.x - origin.x;
    map_state.y = world_state.y - origin.y;
    map_state.z = world_state.z - origin.z;
    return map_state;
  }

  void OnPoseInfo(const gz::msgs::Pose_V &msg) {
    const auto model_pose = this->FindModelPose(msg);
    if (!model_pose.has_value()) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Target entity not found in /pose/info (model_name=%s)",
          this->model_name_.c_str());
      return;
    }

    const rclcpp::Time stamp = this->ResolveStamp(msg);
    if (!this->AcceptPoseStamp(stamp)) {
      return;
    }
    PoseState world_state = this->PoseMsgToState(model_pose.value(), stamp);

    if (const auto relative_base_pose = this->FindRelativeBasePose(msg)) {
      world_state = this->ComposePoseStates(
          world_state, this->PoseMsgToState(relative_base_pose.value(), stamp));
      this->LogResolvedEntity(this->model_name_ + " + " + this->base_frame_);
    } else {
      this->LogResolvedEntity(model_pose->name());
    }

    const auto map_state = this->ToMapFrameState(world_state);
    if (!map_state.has_value()) {
      return;
    }
    this->PublishDynamicTf(map_state.value());
    this->PublishOdometry(map_state.value());
    this->prev_state_ = map_state.value();
  }
};
} // namespace uav_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::TfBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
