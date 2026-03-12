#include "relative_position_fusion/covariance_utils.hpp"
#include "relative_position_fusion/diagnostics_publisher.hpp"
#include "relative_position_fusion/relocalization_monitor.hpp"
#include "relative_position_fusion/relative_position_filter.hpp"
#include "relative_position_fusion/uav_state_adapter.hpp"
#include "relative_position_fusion/ugv_state_adapter.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

namespace relative_position_fusion
{

class RelativePositionFusionNode : public rclcpp::Node
{
public:
  RelativePositionFusionNode()
  : Node("relative_position_fuser")
  {
    DeclareParameters();
    LoadParameters();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    uav_adapter_ = std::make_unique<UavStateAdapter>(this, tf_buffer_, uav_config_);
    ugv_adapter_ = std::make_unique<UgvStateAdapter>(this, tf_buffer_, ugv_config_);
    monitor_ = RelocalizationMonitor(monitor_config_);
    diagnostics_builder_ = std::make_unique<DiagnosticsPublisher>(this->get_logger());

    relative_pose_global_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      relative_pose_global_topic_, 10);
    relative_pose_body_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      relative_pose_body_topic_, 10);
    relative_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>(relative_velocity_topic_, 10);
    relocalize_requested_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(relocalize_requested_topic_, 10);
    diagnostics_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)));
    timer_ = this->create_wall_timer(timer_period, std::bind(&RelativePositionFusionNode::OnTimer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "relative position fusion active: global_frame=%s uav_odom=%s ugv_amcl=%s ugv_filtered=%s ugv_odom=%s rate=%.1f",
      global_frame_.c_str(),
      uav_config_.odom_topic.c_str(),
      ugv_config_.amcl_pose_topic.c_str(),
      ugv_config_.filtered_odom_topic.c_str(),
      ugv_config_.odom_topic.c_str(),
      publish_rate_hz_);
  }

private:
  struct Measurement
  {
    bool valid{false};
    Eigen::Vector2d z{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d r{Eigen::Matrix2d::Identity()};
  };

  void DeclareParameters()
  {
    this->declare_parameter<double>("publish_rate_hz", 20.0);
    this->declare_parameter<std::string>("global_frame", "global");
    this->declare_parameter<std::string>("uav_body_frame", "uav_base_link");
    this->declare_parameter<double>("tf_lookup_timeout_sec", 0.03);

    this->declare_parameter<std::string>("uav_odom_topic", "/uav/state/odometry");
    this->declare_parameter<std::string>("ugv_amcl_pose_topic", "/amcl_pose");
    this->declare_parameter<std::string>("ugv_filtered_odom_topic", "/ugv/odometry/filtered");
    this->declare_parameter<std::string>("ugv_odom_topic", "/ugv/odom");

    this->declare_parameter<std::string>(
      "relative_pose_global_topic", "/relative_position/estimate/global");
    this->declare_parameter<std::string>(
      "relative_pose_body_topic", "/relative_position/estimate/uav_body");
    this->declare_parameter<std::string>(
      "relative_velocity_topic", "/relative_position/debug/relative_velocity");
    this->declare_parameter<std::string>(
      "relocalize_requested_topic", "/relative_position/relocalize_requested");
    this->declare_parameter<std::string>("diagnostics_topic", "/relative_position/diagnostics");

    this->declare_parameter<bool>("uav_twist_in_child_frame", false);
    this->declare_parameter<bool>("ugv_filtered_twist_in_child_frame", true);
    this->declare_parameter<bool>("ugv_odom_twist_in_child_frame", true);
    this->declare_parameter<bool>("assume_ugv_odom_is_global", false);

    this->declare_parameter<double>("pose_timeout_sec", 0.25);
    this->declare_parameter<double>("velocity_timeout_sec", 0.15);
    this->declare_parameter<double>("measurement_sync_tolerance_sec", 0.05);
    this->declare_parameter<double>("min_pose_diff_velocity_dt_sec", 0.03);
    this->declare_parameter<double>("max_pose_diff_velocity_dt_sec", 0.25);
    this->declare_parameter<double>("buffer_length_sec", 2.0);

    this->declare_parameter<double>("covariance_floor_m2", 1e-4);
    this->declare_parameter<double>("covariance_ceiling_m2", 25.0);
    this->declare_parameter<double>("covariance_nan_fallback_m2", 1.0);
    this->declare_parameter<double>("covariance_age_inflation_m2_per_s", 0.25);

    this->declare_parameter<double>("q_rate_nominal_m2_per_s", 0.004);
    this->declare_parameter<double>("q_rate_measurement_missing_m2_per_s", 0.012);
    this->declare_parameter<double>("q_rate_gate_rejected_m2_per_s", 0.020);
    this->declare_parameter<double>("q_rate_source_degraded_m2_per_s", 0.050);

    this->declare_parameter<bool>("use_measurement_gating", true);
    this->declare_parameter<double>("gate_chi2_threshold", 5.99);

    this->declare_parameter<double>("rho_gamma", 2.0);
    this->declare_parameter<double>("relocalize_enter_threshold_m", 1.0);
    this->declare_parameter<double>("relocalize_exit_threshold_m", 0.7);
    this->declare_parameter<int>("relocalize_enter_hold_cycles", 5);
    this->declare_parameter<int>("relocalize_exit_hold_cycles", 10);
    this->declare_parameter<int>("max_consecutive_gate_rejects_before_force_request", 10);
  }

  void LoadParameters()
  {
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    global_frame_ = this->get_parameter("global_frame").as_string();
    uav_body_frame_ = this->get_parameter("uav_body_frame").as_string();
    const double tf_lookup_timeout_sec = this->get_parameter("tf_lookup_timeout_sec").as_double();
    const double pose_timeout_sec = this->get_parameter("pose_timeout_sec").as_double();
    const double velocity_timeout_sec = this->get_parameter("velocity_timeout_sec").as_double();
    measurement_sync_tolerance_sec_ = this->get_parameter("measurement_sync_tolerance_sec").as_double();
    const double min_pose_diff_velocity_dt_sec =
      this->get_parameter("min_pose_diff_velocity_dt_sec").as_double();
    const double max_pose_diff_velocity_dt_sec =
      this->get_parameter("max_pose_diff_velocity_dt_sec").as_double();
    const double buffer_length_sec = this->get_parameter("buffer_length_sec").as_double();
    const double covariance_floor_m2 = this->get_parameter("covariance_floor_m2").as_double();
    const double covariance_ceiling_m2 = this->get_parameter("covariance_ceiling_m2").as_double();
    const double covariance_nan_fallback_m2 =
      this->get_parameter("covariance_nan_fallback_m2").as_double();
    const double covariance_age_inflation_m2_per_s =
      this->get_parameter("covariance_age_inflation_m2_per_s").as_double();

    relative_pose_global_topic_ = this->get_parameter("relative_pose_global_topic").as_string();
    relative_pose_body_topic_ = this->get_parameter("relative_pose_body_topic").as_string();
    relative_velocity_topic_ = this->get_parameter("relative_velocity_topic").as_string();
    relocalize_requested_topic_ = this->get_parameter("relocalize_requested_topic").as_string();
    diagnostics_topic_ = this->get_parameter("diagnostics_topic").as_string();

    uav_config_.global_frame = global_frame_;
    uav_config_.odom_topic = this->get_parameter("uav_odom_topic").as_string();
    uav_config_.pose_timeout_sec = pose_timeout_sec;
    uav_config_.velocity_timeout_sec = velocity_timeout_sec;
    uav_config_.buffer_length_sec = buffer_length_sec;
    uav_config_.min_pose_diff_velocity_dt_sec = min_pose_diff_velocity_dt_sec;
    uav_config_.max_pose_diff_velocity_dt_sec = max_pose_diff_velocity_dt_sec;
    uav_config_.covariance_floor_m2 = covariance_floor_m2;
    uav_config_.covariance_ceiling_m2 = covariance_ceiling_m2;
    uav_config_.covariance_nan_fallback_m2 = covariance_nan_fallback_m2;
    uav_config_.covariance_age_inflation_m2_per_s = covariance_age_inflation_m2_per_s;
    uav_config_.tf_lookup_timeout_sec = tf_lookup_timeout_sec;
    uav_config_.twist_in_child_frame = this->get_parameter("uav_twist_in_child_frame").as_bool();

    ugv_config_.global_frame = global_frame_;
    ugv_config_.amcl_pose_topic = this->get_parameter("ugv_amcl_pose_topic").as_string();
    ugv_config_.filtered_odom_topic = this->get_parameter("ugv_filtered_odom_topic").as_string();
    ugv_config_.odom_topic = this->get_parameter("ugv_odom_topic").as_string();
    ugv_config_.pose_timeout_sec = pose_timeout_sec;
    ugv_config_.velocity_timeout_sec = velocity_timeout_sec;
    ugv_config_.buffer_length_sec = buffer_length_sec;
    ugv_config_.min_pose_diff_velocity_dt_sec = min_pose_diff_velocity_dt_sec;
    ugv_config_.max_pose_diff_velocity_dt_sec = max_pose_diff_velocity_dt_sec;
    ugv_config_.covariance_floor_m2 = covariance_floor_m2;
    ugv_config_.covariance_ceiling_m2 = covariance_ceiling_m2;
    ugv_config_.covariance_nan_fallback_m2 = covariance_nan_fallback_m2;
    ugv_config_.covariance_age_inflation_m2_per_s = covariance_age_inflation_m2_per_s;
    ugv_config_.tf_lookup_timeout_sec = tf_lookup_timeout_sec;
    ugv_config_.filtered_twist_in_child_frame =
      this->get_parameter("ugv_filtered_twist_in_child_frame").as_bool();
    ugv_config_.odom_twist_in_child_frame =
      this->get_parameter("ugv_odom_twist_in_child_frame").as_bool();
    ugv_config_.assume_odom_is_global =
      this->get_parameter("assume_ugv_odom_is_global").as_bool();

    q_rate_nominal_m2_per_s_ = this->get_parameter("q_rate_nominal_m2_per_s").as_double();
    q_rate_measurement_missing_m2_per_s_ =
      this->get_parameter("q_rate_measurement_missing_m2_per_s").as_double();
    q_rate_gate_rejected_m2_per_s_ =
      this->get_parameter("q_rate_gate_rejected_m2_per_s").as_double();
    q_rate_source_degraded_m2_per_s_ =
      this->get_parameter("q_rate_source_degraded_m2_per_s").as_double();

    use_measurement_gating_ = this->get_parameter("use_measurement_gating").as_bool();
    gate_chi2_threshold_ = this->get_parameter("gate_chi2_threshold").as_double();
    rho_gamma_ = this->get_parameter("rho_gamma").as_double();

    monitor_config_.enter_threshold_m =
      this->get_parameter("relocalize_enter_threshold_m").as_double();
    monitor_config_.exit_threshold_m =
      this->get_parameter("relocalize_exit_threshold_m").as_double();
    monitor_config_.enter_hold_cycles =
      this->get_parameter("relocalize_enter_hold_cycles").as_int();
    monitor_config_.exit_hold_cycles =
      this->get_parameter("relocalize_exit_hold_cycles").as_int();
    monitor_config_.max_consecutive_gate_rejects_before_force_request =
      this->get_parameter("max_consecutive_gate_rejects_before_force_request").as_int();
  }

  void OnTimer()
  {
    const rclcpp::Time now = this->now();
    const AgentPlanarState uav_state = uav_adapter_->BuildState(now);
    const AgentPlanarState ugv_state = ugv_adapter_->BuildState(now);

    FusionDiagnostics diagnostics;
    diagnostics.uav_state = uav_state;
    diagnostics.ugv_state = ugv_state;

    const Measurement measurement = BuildMeasurement(uav_state, ugv_state);
    diagnostics.measurement_available = measurement.valid;

    const Eigen::Vector2d relative_velocity = BuildRelativeVelocity(uav_state, ugv_state);
    const bool control_valid = uav_state.velocity_valid && ugv_state.velocity_valid;

    PublishRelativeVelocity(now, relative_velocity);

    if (!filter_.initialized()) {
      if (measurement.valid) {
        filter_.Initialize(measurement.z, measurement.r);
        consecutive_gate_rejects_ = 0;
        diagnostics.initialized = true;
        diagnostics.measurement_used = true;
        diagnostics.filter_mode = "initialized";
      } else {
        diagnostics.initialized = false;
        diagnostics.filter_mode = "uninitialized";
        const RelocalizationDecision decision = monitor_.Update(
          RelocalizationInput{false, false, std::numeric_limits<double>::infinity(), 0});
        diagnostics.relocalize_requested = decision.requested;
        diagnostics.relocalization_reason = decision.reason;
        PublishRelocalizeRequested(decision.requested);
        PublishDiagnostics(now, diagnostics);
        last_tick_time_ = now;
        return;
      }
    } else {
      const double dt_sec = ResolveDt(now);
      std::string filter_mode = "nominal";
      if (!control_valid) {
        filter_mode = "source_degraded";
      } else if (!measurement.valid) {
        filter_mode = "measurement_missing";
      }

      filter_.Predict(dt_sec, control_valid ? relative_velocity : Eigen::Vector2d::Zero(), SelectQRate(filter_mode));
      diagnostics.initialized = true;
      diagnostics.filter_mode = filter_mode;

      if (measurement.valid) {
        const GatingResult gate_result = filter_.Gate(measurement.z, measurement.r, gate_chi2_threshold_);
        diagnostics.mahalanobis_d2 = gate_result.mahalanobis_distance_sq;
        if (!use_measurement_gating_ || gate_result.accepted) {
          filter_.Update(measurement.z, measurement.r);
          diagnostics.measurement_used = true;
          consecutive_gate_rejects_ = 0;
        } else {
          diagnostics.gate_rejected = true;
          diagnostics.filter_mode = "gate_rejected";
          consecutive_gate_rejects_ += 1;
        }
      }
    }

    diagnostics.lambda_max_m2 = MaxEigenvalue(filter_.covariance());
    diagnostics.rho_m = ComputeRhoMeters(filter_.covariance(), rho_gamma_);
    diagnostics.consecutive_gate_rejects = consecutive_gate_rejects_;

    const RelocalizationDecision decision = monitor_.Update(
      RelocalizationInput{
        filter_.initialized(),
        uav_state.pose_valid && ugv_state.pose_valid,
        diagnostics.rho_m,
        consecutive_gate_rejects_});
    diagnostics.relocalize_requested = decision.requested;
    diagnostics.relocalization_reason = decision.reason;

    PublishGlobalEstimate(now, filter_.state(), filter_.covariance());
    PublishBodyEstimate(now, uav_state, filter_.state(), filter_.covariance());
    PublishRelocalizeRequested(decision.requested);
    PublishDiagnostics(now, diagnostics);

    last_tick_time_ = now;
  }

  Measurement BuildMeasurement(
    const AgentPlanarState & uav_state,
    const AgentPlanarState & ugv_state) const
  {
    Measurement measurement;
    if (!uav_state.pose_valid || !ugv_state.pose_valid ||
      !uav_state.covariance_valid || !ugv_state.covariance_valid)
    {
      return measurement;
    }

    const double dt_sec = std::abs((ugv_state.pose_stamp - uav_state.pose_stamp).seconds());
    if (dt_sec > measurement_sync_tolerance_sec_) {
      return measurement;
    }

    measurement.valid = true;
    measurement.z = ugv_state.p_global - uav_state.p_global;
    measurement.r = Symmetrize(ugv_state.sigma_p + uav_state.sigma_p);
    return measurement;
  }

  Eigen::Vector2d BuildRelativeVelocity(
    const AgentPlanarState & uav_state,
    const AgentPlanarState & ugv_state) const
  {
    if (!uav_state.velocity_valid || !ugv_state.velocity_valid) {
      return Eigen::Vector2d::Zero();
    }
    return ugv_state.v_global - uav_state.v_global;
  }

  double ResolveDt(const rclcpp::Time & now) const
  {
    if (last_tick_time_.nanoseconds() == 0) {
      return 1.0 / std::max(1.0, publish_rate_hz_);
    }

    double dt_sec = (now - last_tick_time_).seconds();
    if (dt_sec <= 0.0 || dt_sec > 1.0) {
      dt_sec = 1.0 / std::max(1.0, publish_rate_hz_);
    }
    return dt_sec;
  }

  double SelectQRate(const std::string & mode) const
  {
    if (mode == "source_degraded") {
      return q_rate_source_degraded_m2_per_s_;
    }
    if (mode == "measurement_missing") {
      return q_rate_measurement_missing_m2_per_s_;
    }
    if (mode == "gate_rejected") {
      return q_rate_gate_rejected_m2_per_s_;
    }
    return q_rate_nominal_m2_per_s_;
  }

  void PublishGlobalEstimate(
    const rclcpp::Time & now,
    const Eigen::Vector2d & state,
    const Eigen::Matrix2d & covariance)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = global_frame_;
    msg.pose.pose.position.x = state.x();
    msg.pose.pose.position.y = state.y();
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    FillPlanarCovariance(covariance, &msg.pose.covariance);
    relative_pose_global_pub_->publish(msg);
  }

  void PublishBodyEstimate(
    const rclcpp::Time & now,
    const AgentPlanarState & uav_state,
    const Eigen::Vector2d & state,
    const Eigen::Matrix2d & covariance)
  {
    if (!uav_state.yaw_valid) {
      return;
    }

    const Eigen::Matrix2d rotation = RotationMatrix2d(uav_state.yaw_global).transpose();
    const Eigen::Vector2d state_body = rotation * state;
    const Eigen::Matrix2d covariance_body = rotation * covariance * rotation.transpose();

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = uav_body_frame_;
    msg.pose.pose.position.x = state_body.x();
    msg.pose.pose.position.y = state_body.y();
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;
    FillPlanarCovariance(covariance_body, &msg.pose.covariance);
    relative_pose_body_pub_->publish(msg);
  }

  void PublishRelativeVelocity(const rclcpp::Time & now, const Eigen::Vector2d & relative_velocity)
  {
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = global_frame_;
    msg.vector.x = relative_velocity.x();
    msg.vector.y = relative_velocity.y();
    msg.vector.z = 0.0;
    relative_velocity_pub_->publish(msg);
  }

  void PublishRelocalizeRequested(bool requested)
  {
    std_msgs::msg::Bool msg;
    msg.data = requested;
    relocalize_requested_pub_->publish(msg);
  }

  void PublishDiagnostics(const rclcpp::Time & now, const FusionDiagnostics & diagnostics)
  {
    diagnostics_pub_->publish(diagnostics_builder_->Build(now, diagnostics));
  }

  static void FillPlanarCovariance(
    const Eigen::Matrix2d & covariance,
    std::array<double, 36> * output)
  {
    output->fill(0.0);
    (*output)[0] = covariance(0, 0);
    (*output)[1] = covariance(0, 1);
    (*output)[6] = covariance(1, 0);
    (*output)[7] = covariance(1, 1);
    (*output)[14] = 1e6;
    (*output)[21] = 1e6;
    (*output)[28] = 1e6;
    (*output)[35] = 1e6;
  }

  double publish_rate_hz_{20.0};
  double measurement_sync_tolerance_sec_{0.05};
  double q_rate_nominal_m2_per_s_{0.004};
  double q_rate_measurement_missing_m2_per_s_{0.012};
  double q_rate_gate_rejected_m2_per_s_{0.020};
  double q_rate_source_degraded_m2_per_s_{0.050};
  double gate_chi2_threshold_{5.99};
  double rho_gamma_{2.0};
  bool use_measurement_gating_{true};

  std::string global_frame_;
  std::string uav_body_frame_;
  std::string relative_pose_global_topic_;
  std::string relative_pose_body_topic_;
  std::string relative_velocity_topic_;
  std::string relocalize_requested_topic_;
  std::string diagnostics_topic_;

  UavStateAdapter::Config uav_config_;
  UgvStateAdapter::Config ugv_config_;
  RelocalizationMonitorConfig monitor_config_{};
  RelativePositionFilter filter_;
  RelocalizationMonitor monitor_;
  std::unique_ptr<DiagnosticsPublisher> diagnostics_builder_;

  int consecutive_gate_rejects_{0};

  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<UavStateAdapter> uav_adapter_;
  std::unique_ptr<UgvStateAdapter> ugv_adapter_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    relative_pose_global_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    relative_pose_body_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr relative_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relocalize_requested_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace relative_position_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relative_position_fusion::RelativePositionFusionNode>());
  rclcpp::shutdown();
  return 0;
}
