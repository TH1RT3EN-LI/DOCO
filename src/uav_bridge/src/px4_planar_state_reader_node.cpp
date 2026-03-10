#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_bridge
{

class Px4PlanarStateReaderNode : public rclcpp::Node
{
public:
  Px4PlanarStateReaderNode()
  : Node("px4_planar_state_reader_node")
  {
    this->declare_parameter<std::string>(
      "vehicle_local_position_topic", "/uav/fmu/out/vehicle_local_position");
    this->declare_parameter<std::string>("vehicle_odometry_topic", "/uav/fmu/out/vehicle_odometry");
    this->declare_parameter<std::string>("output_odom_topic", "/uav/px4/planar_odom");
    this->declare_parameter<std::string>("world_frame_id", "map");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<bool>("publish_odometry", true);
    this->declare_parameter<bool>("log_state", true);

    vehicle_local_position_topic_ =
      this->get_parameter("vehicle_local_position_topic").as_string();
    vehicle_odometry_topic_ = this->get_parameter("vehicle_odometry_topic").as_string();
    output_odom_topic_ = this->get_parameter("output_odom_topic").as_string();
    world_frame_id_ = this->get_parameter("world_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    publish_odometry_ = this->get_parameter("publish_odometry").as_bool();
    log_state_ = this->get_parameter("log_state").as_bool();

    const auto sensor_qos = rclcpp::SensorDataQoS();

    local_position_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      vehicle_local_position_topic_, sensor_qos,
      std::bind(&Px4PlanarStateReaderNode::localPositionCallback, this, std::placeholders::_1));

    vehicle_odometry_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_topic_, sensor_qos,
      std::bind(&Px4PlanarStateReaderNode::vehicleOdometryCallback, this, std::placeholders::_1));

    if (publish_odometry_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);
    }

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = this->create_wall_timer(
      timer_period, std::bind(&Px4PlanarStateReaderNode::publishSnapshot, this));

    RCLCPP_INFO(
      this->get_logger(),
      "px4 planar state reader: local_pos=%s vehicle_odom=%s output_odom=%s rate=%.1f",
      vehicle_local_position_topic_.c_str(),
      vehicle_odometry_topic_.c_str(),
      output_odom_topic_.c_str(),
      publish_rate_hz_);
  }

private:
  struct Covariance2D
  {
    double xx{std::numeric_limits<double>::quiet_NaN()};
    double xy{0.0};
    double yx{0.0};
    double yy{std::numeric_limits<double>::quiet_NaN()};
  };

  struct PlanarState
  {
    bool position_valid{false};
    bool velocity_valid{false};
    bool covariance_valid{false};
    bool globally_referenced{false};
    bool covariance_has_cross_term{false};

    std::array<double, 2> p_ned_m{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> v_ned_mps{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
    Covariance2D sigma_xy_ned_m2{};

    std::array<double, 2> p_enu_m{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
    std::array<double, 2> v_enu_mps{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
    Covariance2D sigma_xy_enu_m2{};

    double ref_lat_deg{std::numeric_limits<double>::quiet_NaN()};
    double ref_lon_deg{std::numeric_limits<double>::quiet_NaN()};
    double ref_alt_m{std::numeric_limits<double>::quiet_NaN()};
    double eph_m{std::numeric_limits<double>::quiet_NaN()};

    uint64_t px4_timestamp_us{0};
  };

  static double quietNaN()
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static bool isFinite(double value)
  {
    return std::isfinite(value);
  }

  static std::array<double, 2> nedToEnu(double north, double east)
  {
    return {east, north};
  }

  static Covariance2D nedToEnu(const Covariance2D & ned_cov)
  {
    Covariance2D enu_cov;
    enu_cov.xx = ned_cov.yy;
    enu_cov.xy = ned_cov.yx;
    enu_cov.yx = ned_cov.xy;
    enu_cov.yy = ned_cov.xx;
    return enu_cov;
  }

  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    std::scoped_lock lock(state_mutex_);

    state_.px4_timestamp_us = msg->timestamp;
    state_.globally_referenced = msg->xy_global;
    state_.eph_m = msg->eph;

    if (msg->xy_valid) {
      state_.position_valid = true;
      state_.p_ned_m = {static_cast<double>(msg->x), static_cast<double>(msg->y)};
      state_.p_enu_m = nedToEnu(state_.p_ned_m[0], state_.p_ned_m[1]);
    } else {
      state_.position_valid = false;
      state_.p_ned_m = {quietNaN(), quietNaN()};
      state_.p_enu_m = {quietNaN(), quietNaN()};
    }

    if (msg->v_xy_valid) {
      state_.velocity_valid = true;
      state_.v_ned_mps = {static_cast<double>(msg->vx), static_cast<double>(msg->vy)};
      state_.v_enu_mps = nedToEnu(state_.v_ned_mps[0], state_.v_ned_mps[1]);
    } else {
      state_.velocity_valid = false;
      state_.v_ned_mps = {quietNaN(), quietNaN()};
      state_.v_enu_mps = {quietNaN(), quietNaN()};
    }

    if (msg->xy_global) {
      state_.ref_lat_deg = msg->ref_lat;
      state_.ref_lon_deg = msg->ref_lon;
      state_.ref_alt_m = msg->ref_alt;
    } else {
      state_.ref_lat_deg = quietNaN();
      state_.ref_lon_deg = quietNaN();
      state_.ref_alt_m = quietNaN();
    }

    has_new_state_ = true;
  }

  void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    std::scoped_lock lock(state_mutex_);

    state_.px4_timestamp_us = std::max(state_.px4_timestamp_us, msg->timestamp);

    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "vehicle_odometry pose_frame=%u, template expects NED=1", msg->pose_frame);
    } else if (!state_.position_valid &&
      isFinite(msg->position[0]) &&
      isFinite(msg->position[1]))
    {
      state_.position_valid = true;
      state_.p_ned_m = {static_cast<double>(msg->position[0]), static_cast<double>(msg->position[1])};
      state_.p_enu_m = nedToEnu(state_.p_ned_m[0], state_.p_ned_m[1]);
    }

    if (msg->velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "vehicle_odometry velocity_frame=%u, template expects NED=1", msg->velocity_frame);
    } else if (!state_.velocity_valid &&
      isFinite(msg->velocity[0]) &&
      isFinite(msg->velocity[1]))
    {
      state_.velocity_valid = true;
      state_.v_ned_mps = {static_cast<double>(msg->velocity[0]), static_cast<double>(msg->velocity[1])};
      state_.v_enu_mps = nedToEnu(state_.v_ned_mps[0], state_.v_ned_mps[1]);
    }

    const double var_x = msg->position_variance[0];
    const double var_y = msg->position_variance[1];

    if (isFinite(var_x) && isFinite(var_y)) {
      state_.covariance_valid = true;
      state_.covariance_has_cross_term = false;
      state_.sigma_xy_ned_m2.xx = var_x;
      state_.sigma_xy_ned_m2.xy = 0.0;
      state_.sigma_xy_ned_m2.yx = 0.0;
      state_.sigma_xy_ned_m2.yy = var_y;
      state_.sigma_xy_enu_m2 = nedToEnu(state_.sigma_xy_ned_m2);
    } else {
      state_.covariance_valid = false;
      state_.covariance_has_cross_term = false;
      state_.sigma_xy_ned_m2 = Covariance2D{};
      state_.sigma_xy_enu_m2 = Covariance2D{};
    }

    has_new_state_ = true;
  }

  void publishSnapshot()
  {
    PlanarState snapshot;
    {
      std::scoped_lock lock(state_mutex_);
      if (!has_new_state_) {
        return;
      }
      snapshot = state_;
      has_new_state_ = false;
    }

    if (publish_odometry_ && odom_pub_ && snapshot.position_valid) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = this->now();
      odom_msg.header.frame_id = world_frame_id_;
      odom_msg.child_frame_id = base_frame_id_;
      odom_msg.pose.pose.position.x = snapshot.p_enu_m[0];
      odom_msg.pose.pose.position.y = snapshot.p_enu_m[1];
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.twist.twist.linear.x = snapshot.v_enu_mps[0];
      odom_msg.twist.twist.linear.y = snapshot.v_enu_mps[1];
      odom_msg.twist.twist.linear.z = 0.0;

      std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
      std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);

      if (snapshot.covariance_valid) {
        odom_msg.pose.covariance[0] = snapshot.sigma_xy_enu_m2.xx;
        odom_msg.pose.covariance[1] = snapshot.sigma_xy_enu_m2.xy;
        odom_msg.pose.covariance[6] = snapshot.sigma_xy_enu_m2.yx;
        odom_msg.pose.covariance[7] = snapshot.sigma_xy_enu_m2.yy;
      }

      odom_pub_->publish(odom_msg);
    }

    if (log_state_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "p_ka NED=[%.3f, %.3f] ENU=[%.3f, %.3f] | v_ka NED=[%.3f, %.3f] ENU=[%.3f, %.3f] | "
        "Sigma_xy(NED)=[[%.4f, %.4f],[%.4f, %.4f]] | global_ref=%s%s",
        snapshot.p_ned_m[0], snapshot.p_ned_m[1],
        snapshot.p_enu_m[0], snapshot.p_enu_m[1],
        snapshot.v_ned_mps[0], snapshot.v_ned_mps[1],
        snapshot.v_enu_mps[0], snapshot.v_enu_mps[1],
        snapshot.sigma_xy_ned_m2.xx, snapshot.sigma_xy_ned_m2.xy,
        snapshot.sigma_xy_ned_m2.yx, snapshot.sigma_xy_ned_m2.yy,
        snapshot.globally_referenced ? "true" : "false",
        snapshot.covariance_has_cross_term ? "" : " (diag only in current PX4 bridge)");
    }
  }

  std::mutex state_mutex_;
  PlanarState state_;
  bool has_new_state_{false};

  std::string vehicle_local_position_topic_;
  std::string vehicle_odometry_topic_;
  std::string output_odom_topic_;
  std::string world_frame_id_;
  std::string base_frame_id_;

  double publish_rate_hz_{20.0};
  bool publish_odometry_{true};
  bool log_state_{true};

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace uav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_bridge::Px4PlanarStateReaderNode>());
  rclcpp::shutdown();
  return 0;
}
