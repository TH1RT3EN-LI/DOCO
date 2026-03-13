#pragma once

#include <Eigen/Core>

#include <string>

namespace relative_position_fusion
{

class RelativeTrackingController
{
public:
  enum class Mode
  {
    Idle,
    TakeoffPose,
    Tracking,
    PositionManual,
    DegradedHold,
  };

  struct Config
  {
    double return_height_m{1.0};
    double takeoff_height_tolerance_m{0.10};
    double takeoff_stable_s{0.5};
    bool auto_track_after_takeoff{true};
    double relative_pose_timeout_s{0.25};
    double uav_state_timeout_s{0.25};
    double ugv_odom_timeout_s{0.25};
    double xy_kp{0.8};
    double z_kp{1.0};
    double xy_deadband_m{0.05};
    double z_deadband_m{0.05};
    double max_xy_speed_mps{0.8};
    double max_z_speed_mps{0.5};
    double max_relative_covariance_m2{1.5};
  };

  struct UavState
  {
    bool valid{false};
    double stamp_sec{0.0};
    Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
    double yaw_enu{0.0};
    std::string frame_id;
  };

  struct UgvOdom
  {
    bool valid{false};
    double stamp_sec{0.0};
    double z_enu{0.0};
  };

  struct RelativePose
  {
    bool valid{false};
    double stamp_sec{0.0};
    Eigen::Vector2d position_body{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
  };

  struct Inputs
  {
    double now_sec{0.0};
    UavState uav_state;
    UgvOdom ugv_odom;
    RelativePose relative_pose;
    bool relocalize_requested{false};
  };

  struct Outputs
  {
    bool publish_position_target{false};
    Eigen::Vector3d position_target_enu{Eigen::Vector3d::Zero()};
    double yaw_target_enu{0.0};
    std::string position_frame_id;

    bool publish_velocity_body{false};
    Eigen::Vector3d velocity_body_mps{Eigen::Vector3d::Zero()};
    double yaw_rate_radps{0.0};

    bool request_hold{false};
    std::string message;
  };

  explicit RelativeTrackingController(const Config & config);

  Mode mode() const { return mode_; }
  bool session_started() const { return session_started_; }
  const std::string & last_reason() const { return last_reason_; }

  bool Start(double target_height_m, const Inputs & inputs, std::string * message);
  bool EnterPositionMode(const Inputs & inputs, std::string * message);
  bool GoAboveUgv(const Inputs & inputs, std::string * message);
  bool Stop(std::string * message);

  Outputs Update(const Inputs & inputs);

  static const char * ModeName(Mode mode);

private:
  bool ValidateFreshUavState(const Inputs & inputs, std::string * reason) const;
  bool ValidateFreshUgvOdom(const Inputs & inputs, std::string * reason) const;
  bool ValidateTrackingInputs(const Inputs & inputs, std::string * reason) const;
  void QueuePositionTarget(const Inputs & inputs, double target_z_m);
  void EnterDegradedHold(const std::string & reason);
  static double AgeSeconds(double now_sec, double stamp_sec);
  static double ApplyDeadband(double value, double deadband);
  static double ClampAbs(double value, double limit);

  Config config_;
  Mode mode_{Mode::Idle};
  bool session_started_{false};
  double active_target_height_m_{0.0};
  double takeoff_target_z_m_{0.0};
  double takeoff_reached_since_sec_{-1.0};

  bool pending_position_target_{false};
  Eigen::Vector3d pending_position_target_enu_{Eigen::Vector3d::Zero()};
  double pending_yaw_target_enu_{0.0};
  std::string pending_position_frame_id_;

  bool pending_hold_request_{false};
  std::string last_reason_{"idle"};
};

}  // namespace relative_position_fusion
