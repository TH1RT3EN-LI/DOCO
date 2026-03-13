#include "relative_position_fusion/relative_tracking_controller.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace relative_position_fusion
{

namespace
{

bool IsFinite(const Eigen::Vector2d & value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y());
}

bool IsFinite(const Eigen::Vector3d & value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

bool IsFinite(const Eigen::Matrix2d & value)
{
  return value.array().isFinite().all();
}

double MaxCovarianceEigenvalue(const Eigen::Matrix2d & covariance)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return std::numeric_limits<double>::infinity();
  }
  return solver.eigenvalues().maxCoeff();
}

double NormalizeAngle(double angle_rad)
{
  if (!std::isfinite(angle_rad)) {
    return 0.0;
  }
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double RampDown(double value, double start, double stop)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }

  const double safe_start = std::max(0.0, start);
  const double safe_stop = std::max(safe_start, stop);
  if (value <= safe_start) {
    return 1.0;
  }
  if (value >= safe_stop) {
    return 0.0;
  }
  if (safe_stop <= safe_start + 1.0e-9) {
    return 0.0;
  }
  return (safe_stop - value) / (safe_stop - safe_start);
}

}  // namespace

RelativeTrackingController::RelativeTrackingController(const Config & config)
: config_(config)
{
}

bool RelativeTrackingController::Start(
  double target_height_m,
  const Inputs & inputs,
  std::string * message)
{
  if (mode_ != Mode::Idle && mode_ != Mode::DegradedHold) {
    if (message != nullptr) {
      *message = "relative tracking already active";
    }
    return false;
  }
  if (!std::isfinite(target_height_m) || target_height_m <= 0.0) {
    if (message != nullptr) {
      *message = "target height must be finite and positive";
    }
    return false;
  }

  if (inputs.relocalize_requested) {
    if (message != nullptr) {
      *message = "fusion requested relocalization";
    }
    return false;
  }

  std::string reason;
  if (!ValidateFreshUavState(inputs, &reason) || !ValidateFreshUgvOdom(inputs, &reason)) {
    if (message != nullptr) {
      *message = reason;
    }
    return false;
  }

  session_started_ = true;
  active_target_height_m_ = target_height_m;
  takeoff_target_z_m_ = inputs.ugv_odom.z_enu + target_height_m;
  takeoff_reached_since_sec_ = -1.0;
  mode_ = Mode::TakeoffPose;
  last_reason_ = "takeoff pose active";
  QueuePositionTarget(inputs, takeoff_target_z_m_);

  if (message != nullptr) {
    *message = "relative tracking takeoff armed";
  }
  return true;
}

bool RelativeTrackingController::EnterPositionMode(
  const Inputs & inputs,
  std::string * message)
{
  if (!session_started_) {
    if (message != nullptr) {
      *message = "relative tracking has not been started";
    }
    return false;
  }

  std::string reason;
  if (!ValidateFreshUavState(inputs, &reason)) {
    if (message != nullptr) {
      *message = reason;
    }
    return false;
  }

  mode_ = Mode::PositionManual;
  last_reason_ = "manual position mode active";
  QueuePositionTarget(inputs, inputs.uav_state.position_enu.z());

  if (message != nullptr) {
    *message = "position mode active";
  }
  return true;
}

bool RelativeTrackingController::GoAboveUgv(
  const Inputs & inputs,
  std::string * message)
{
  if (!session_started_) {
    if (message != nullptr) {
      *message = "relative tracking has not been started";
    }
    return false;
  }

  std::string reason;
  if (!ValidateTrackingInputs(inputs, &reason)) {
    if (message != nullptr) {
      *message = reason;
    }
    return false;
  }

  active_target_height_m_ = config_.return_height_m;
  mode_ = Mode::Tracking;
  takeoff_reached_since_sec_ = -1.0;
  last_reason_ = "tracking above ugv";

  if (message != nullptr) {
    *message = "tracking resumed above ugv";
  }
  return true;
}

bool RelativeTrackingController::Stop(std::string * message)
{
  mode_ = Mode::Idle;
  session_started_ = false;
  takeoff_reached_since_sec_ = -1.0;
  pending_position_target_ = false;
  pending_hold_request_ = true;
  last_reason_ = "stopped";

  if (message != nullptr) {
    *message = "relative tracking stopped";
  }
  return true;
}

RelativeTrackingController::Outputs RelativeTrackingController::Update(const Inputs & inputs)
{
  Outputs outputs;

  if (mode_ == Mode::Tracking) {
    std::string degrade_reason;
    if (!ValidateTrackingInputs(inputs, &degrade_reason)) {
      EnterDegradedHold(degrade_reason);
    }
  } else if (mode_ == Mode::TakeoffPose) {
    if (inputs.relocalize_requested) {
      EnterDegradedHold("fusion requested relocalization");
    } else {
      std::string degrade_reason;
      if (!ValidateFreshUavState(inputs, &degrade_reason)) {
        EnterDegradedHold(degrade_reason);
      }
    }
  }

  if (pending_hold_request_) {
    outputs.request_hold = true;
    pending_hold_request_ = false;
  }

  if (pending_position_target_) {
    outputs.publish_position_target = true;
    outputs.position_target_enu = pending_position_target_enu_;
    outputs.yaw_target_enu = pending_yaw_target_enu_;
    outputs.position_frame_id = pending_position_frame_id_;
    pending_position_target_ = false;
  }

  switch (mode_) {
    case Mode::Idle:
      outputs.message = last_reason_;
      return outputs;
    case Mode::TakeoffPose:
      if (std::abs(takeoff_target_z_m_ - inputs.uav_state.position_enu.z()) <=
        config_.takeoff_height_tolerance_m)
      {
        if (takeoff_reached_since_sec_ < 0.0) {
          takeoff_reached_since_sec_ = inputs.now_sec;
        } else if ((inputs.now_sec - takeoff_reached_since_sec_) >= config_.takeoff_stable_s) {
          if (config_.auto_track_after_takeoff) {
            mode_ = Mode::Tracking;
            last_reason_ = "tracking active";
          } else {
            mode_ = Mode::PositionManual;
            last_reason_ = "position mode active after takeoff";
            QueuePositionTarget(inputs, inputs.uav_state.position_enu.z());
          }
          outputs.message = last_reason_;
          return outputs;
        }
      } else {
        takeoff_reached_since_sec_ = -1.0;
      }
      outputs.message = last_reason_;
      return outputs;
    case Mode::Tracking:
    {
      outputs.publish_velocity_body = true;
      const Eigen::Vector2d xy_error_body = inputs.relative_pose.position_body;
      Eigen::Vector2d xy_command = xy_error_body * config_.xy_kp;
      xy_command.x() = ApplyDeadband(xy_command.x(), config_.xy_deadband_m * config_.xy_kp);
      xy_command.y() = ApplyDeadband(xy_command.y(), config_.xy_deadband_m * config_.xy_kp);

      const double xy_norm = xy_command.norm();
      if (xy_norm > config_.max_xy_speed_mps && xy_norm > 1.0e-9) {
        xy_command *= config_.max_xy_speed_mps / xy_norm;
      }

      const Eigen::Vector2d nominal_xy_command = xy_command;
      if (config_.enable_heading_aligned_tracking) {
        const double heading_control_min_xy_speed =
          std::max(0.0, config_.heading_control_min_xy_speed_mps);
        const double nominal_xy_speed = nominal_xy_command.norm();
        if (nominal_xy_speed >= heading_control_min_xy_speed && nominal_xy_speed > 1.0e-9) {
          const double yaw_error =
            NormalizeAngle(std::atan2(nominal_xy_command.y(), nominal_xy_command.x()));
          const double yaw_deadband = std::max(0.0, config_.yaw_deadband_rad);
          const double yaw_error_for_rate = std::abs(yaw_error) < yaw_deadband ? 0.0 : yaw_error;
          outputs.yaw_rate_radps = ClampAbs(
            yaw_error_for_rate * config_.yaw_kp,
            config_.max_yaw_rate_radps);

          const double abs_yaw_error = std::abs(yaw_error);
          const double lateral_scale = RampDown(
            abs_yaw_error,
            config_.lateral_release_yaw_error_rad,
            config_.forward_only_yaw_error_rad);
          const double forward_scale = RampDown(
            abs_yaw_error,
            config_.forward_only_yaw_error_rad,
            config_.stop_translate_yaw_error_rad);

          xy_command.x() *= forward_scale;
          xy_command.y() *= lateral_scale;

          if (config_.disallow_reverse_motion && xy_command.x() < 0.0) {
            xy_command.x() = 0.0;
          }
        }
      }

      const double target_z_m = inputs.ugv_odom.z_enu + active_target_height_m_;
      double z_command = (target_z_m - inputs.uav_state.position_enu.z()) * config_.z_kp;
      z_command = ApplyDeadband(z_command, config_.z_deadband_m * config_.z_kp);
      z_command = ClampAbs(z_command, config_.max_z_speed_mps);

      outputs.velocity_body_mps.x() = xy_command.x();
      outputs.velocity_body_mps.y() = xy_command.y();
      outputs.velocity_body_mps.z() = z_command;
      outputs.message = last_reason_;
      return outputs;
    }
    case Mode::PositionManual:
      outputs.message = last_reason_;
      return outputs;
    case Mode::DegradedHold:
      outputs.message = last_reason_;
      return outputs;
  }

  outputs.message = last_reason_;
  return outputs;
}

const char * RelativeTrackingController::ModeName(Mode mode)
{
  switch (mode) {
    case Mode::Idle:
      return "idle";
    case Mode::TakeoffPose:
      return "takeoff_pose";
    case Mode::Tracking:
      return "tracking";
    case Mode::PositionManual:
      return "position_manual";
    case Mode::DegradedHold:
      return "degraded_hold";
  }
  return "unknown";
}

bool RelativeTrackingController::ValidateFreshUavState(
  const Inputs & inputs,
  std::string * reason) const
{
  if (!inputs.uav_state.valid || !IsFinite(inputs.uav_state.position_enu) ||
    !std::isfinite(inputs.uav_state.yaw_enu))
  {
    if (reason != nullptr) {
      *reason = "uav state unavailable";
    }
    return false;
  }

  if (AgeSeconds(inputs.now_sec, inputs.uav_state.stamp_sec) > config_.uav_state_timeout_s) {
    if (reason != nullptr) {
      *reason = "uav state timed out";
    }
    return false;
  }
  return true;
}

bool RelativeTrackingController::ValidateFreshUgvOdom(
  const Inputs & inputs,
  std::string * reason) const
{
  if (!inputs.ugv_odom.valid || !std::isfinite(inputs.ugv_odom.z_enu)) {
    if (reason != nullptr) {
      *reason = "ugv odom unavailable";
    }
    return false;
  }

  if (AgeSeconds(inputs.now_sec, inputs.ugv_odom.stamp_sec) > config_.ugv_odom_timeout_s) {
    if (reason != nullptr) {
      *reason = "ugv odom timed out";
    }
    return false;
  }
  return true;
}

bool RelativeTrackingController::ValidateTrackingInputs(
  const Inputs & inputs,
  std::string * reason) const
{
  if (inputs.relocalize_requested) {
    if (reason != nullptr) {
      *reason = "fusion requested relocalization";
    }
    return false;
  }
  if (!ValidateFreshUavState(inputs, reason)) {
    return false;
  }
  if (!ValidateFreshUgvOdom(inputs, reason)) {
    return false;
  }
  if (!inputs.relative_pose.valid || !IsFinite(inputs.relative_pose.position_body) ||
    !IsFinite(inputs.relative_pose.covariance))
  {
    if (reason != nullptr) {
      *reason = "relative pose unavailable";
    }
    return false;
  }
  if (AgeSeconds(inputs.now_sec, inputs.relative_pose.stamp_sec) > config_.relative_pose_timeout_s) {
    if (reason != nullptr) {
      *reason = "relative pose timed out";
    }
    return false;
  }
  if (MaxCovarianceEigenvalue(inputs.relative_pose.covariance) > config_.max_relative_covariance_m2) {
    if (reason != nullptr) {
      *reason = "relative pose covariance exceeded threshold";
    }
    return false;
  }
  return true;
}

void RelativeTrackingController::QueuePositionTarget(const Inputs & inputs, double target_z_m)
{
  pending_position_target_ = true;
  pending_position_target_enu_ = inputs.uav_state.position_enu;
  pending_position_target_enu_.z() = target_z_m;
  pending_yaw_target_enu_ = inputs.uav_state.yaw_enu;
  pending_position_frame_id_ = inputs.uav_state.frame_id;
}

void RelativeTrackingController::EnterDegradedHold(const std::string & reason)
{
  mode_ = Mode::DegradedHold;
  pending_position_target_ = false;
  pending_hold_request_ = true;
  takeoff_reached_since_sec_ = -1.0;
  last_reason_ = reason;
}

double RelativeTrackingController::AgeSeconds(double now_sec, double stamp_sec)
{
  if (!std::isfinite(now_sec) || !std::isfinite(stamp_sec)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, now_sec - stamp_sec);
}

double RelativeTrackingController::ApplyDeadband(double value, double deadband)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::abs(value) < std::max(0.0, deadband) ? 0.0 : value;
}

double RelativeTrackingController::ClampAbs(double value, double limit)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  const double safe_limit = std::max(0.0, limit);
  return std::clamp(value, -safe_limit, safe_limit);
}

}  // namespace relative_position_fusion
