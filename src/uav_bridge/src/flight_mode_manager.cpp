#include "uav_bridge/flight_mode_manager.hpp"

#include <cmath>

#include "uav_bridge/math_utils.hpp"

namespace uav_bridge
{

FlightModeManager::FlightModeManager(const Config & config, rclcpp::Logger logger)
: config_(config), logger_(logger)
{
}

void FlightModeManager::onPlannerOdom(const std::array<float, 3> & enu_pos, float yaw_enu)
{
  planner_odom_enu_ = enu_pos;
  planner_yaw_enu_ = yaw_enu;
  has_planner_yaw_ = true;
  has_planner_odom_ = true;
}

void FlightModeManager::onPlannerCommand(const quadrotor_msgs::msg::PositionCommand & cmd)
{
  last_auto_cmd_ = cmd;
  has_auto_cmd_ = true;
  if (!is_armed_ || !is_offboard_mode_) {
    planner_restart_pending_ = true;
    if (config_.require_planner_takeoff_clearance &&
      !planner_startup_sequence_pending_ &&
      !planner_takeoff_initialized_)
    {
      planner_startup_sequence_pending_ = true;
    }
  }
}

void FlightModeManager::onManualPose(const geometry_msgs::msg::PoseStamped & msg)
{
  manual_target_position_enu_ = {
    static_cast<float>(msg.pose.position.x),
    static_cast<float>(msg.pose.position.y),
    static_cast<float>(msg.pose.position.z)};

  if (!isFiniteVector(manual_target_position_enu_)) {
    RCLCPP_WARN(logger_, "ignored manual pose command with non-finite target");
    return;
  }

  const double quat_norm =
    (msg.pose.orientation.x * msg.pose.orientation.x) +
    (msg.pose.orientation.y * msg.pose.orientation.y) +
    (msg.pose.orientation.z * msg.pose.orientation.z) +
    (msg.pose.orientation.w * msg.pose.orientation.w);

  if (quat_norm > 1.0e-6) {
    manual_target_yaw_enu_ = quaternionToYaw(msg.pose.orientation);
  } else if (has_planner_yaw_) {
    manual_target_yaw_enu_ = planner_yaw_enu_;
  } else if (!manual_target_valid_ && has_auto_cmd_) {
    manual_target_yaw_enu_ = static_cast<float>(last_auto_cmd_.yaw);
  }

  manual_target_valid_ = true;
  setMode(Mode::Manual, "manual pose target", mode_ != Mode::Manual);
}

void FlightModeManager::onNudge(const geometry_msgs::msg::Twist & msg)
{
  if (!manual_target_valid_) {
    std::string err;
    if (!captureCurrentPose(err)) {
      RCLCPP_WARN(logger_, "cannot apply nudge: %s", err.c_str());
      return;
    }
  }

  const float ref_yaw = has_planner_yaw_ ? planner_yaw_enu_ : manual_target_yaw_enu_;
  float dx = static_cast<float>(msg.linear.x);
  float dy = static_cast<float>(msg.linear.y);

  if (config_.nudge_body_frame) {
    const float cy = std::cos(ref_yaw);
    const float sy = std::sin(ref_yaw);
    const float bx = dx, by = dy;
    dx = cy * bx - sy * by;
    dy = sy * bx + cy * by;
  }

  manual_target_position_enu_[0] += dx;
  manual_target_position_enu_[1] += dy;
  manual_target_position_enu_[2] += static_cast<float>(msg.linear.z);
  manual_target_yaw_enu_ = normalizeAngle(
    manual_target_yaw_enu_ + static_cast<float>(msg.angular.z));
  manual_target_valid_ = true;

  setMode(Mode::Manual, "manual nudge", mode_ != Mode::Manual);
}

void FlightModeManager::onVelocityCommand(const geometry_msgs::msg::Twist & msg, uint64_t now_us)
{
  last_velocity_cmd_ = msg;
  last_velocity_time_us_ = now_us;
  has_velocity_cmd_ = true;
  setMode(Mode::Velocity, "velocity cmd", mode_ != Mode::Velocity);
}

void FlightModeManager::onVehicleStatus(bool armed, bool offboard_mode, bool auto_land_mode)
{
  is_armed_ = armed;
  is_offboard_mode_ = offboard_mode;
  is_auto_land_mode_ = auto_land_mode;
}

bool FlightModeManager::requestTakeoff(std::string & message)
{
  std::string err;
  if (!captureCurrentPose(err)) {
    message = err;
    return false;
  }
  manual_target_position_enu_[2] += config_.takeoff_height_m;
  manual_target_valid_ = true;
  setMode(Mode::Manual, "takeoff", true);
  message = "takeoff target armed";
  return true;
}

bool FlightModeManager::requestHover(std::string & message)
{
  std::string err;
  if (!captureCurrentPose(err)) {
    message = err;
    return false;
  }
  setMode(Mode::Manual, "hover", true);
  message = "hover target locked to current odometry";
  return true;
}

bool FlightModeManager::requestLand(std::string & message)
{
  if (!manual_target_valid_) {
    std::string err;
    if (!captureCurrentPose(err)) {
      message = err;
      return false;
    }
  }
  setMode(Mode::Landing, "land", false);
  landing_command_sent_ = false;
  landing_retry_counter_ = 0;
  message = "landing requested";
  return true;
}

bool FlightModeManager::requestResumeAuto(std::string & message)
{
  if (!has_auto_cmd_) {
    message = "planner command has not been received yet";
    return false;
  }
  setMode(Mode::Planner, "resume auto", true);
  message = "planner control resumed";
  return true;
}

FlightModeManager::TickOutput FlightModeManager::tick(
  uint64_t now_us, const rclcpp::Time & ros_now, bool px4_execution_ready)
{
  TickOutput out{};

  if (mode_ == Mode::Velocity && has_velocity_cmd_ && now_us != 0) {
    if (last_velocity_time_us_ == 0) {
      last_velocity_time_us_ = now_us;
    } else {
      const double elapsed_ms =
        static_cast<double>(now_us - last_velocity_time_us_) / 1000.0;
      if (elapsed_ms > config_.velocity_timeout_ms) {
        RCLCPP_WARN(
          logger_,
          "velocity command timeout (%.0f ms > %.0f ms), falling back to hover",
          elapsed_ms, config_.velocity_timeout_ms);
        has_velocity_cmd_ = false;
        std::string dummy;
        requestHover(dummy);
      }
    }
  }

  if (planner_restart_pending_ && !planner_startup_sequence_pending_ &&
    mode_ == Mode::Planner && has_auto_cmd_ &&
    is_armed_ && is_offboard_mode_ && px4_execution_ready)
  {
    out.request_planner_restart = true;
    planner_restart_pending_ = false;
    RCLCPP_INFO(logger_, "requested planner trajectory restart after offboard became active");
  }

  switch (mode_) {
    case Mode::Planner:
      if (handlePlannerStartup(ros_now, out)) {
        break;
      }
      {
        quadrotor_msgs::msg::PositionCommand cmd{};
        if (getActivePositionCommand(cmd, ros_now)) {
          out.type = TickOutput::Type::Position;
          out.position_cmd = cmd;
          out.allow_mode_reassert = true;
        }
      }
      break;

    case Mode::Manual:
      {
        quadrotor_msgs::msg::PositionCommand cmd{};
        if (getActivePositionCommand(cmd, ros_now)) {
          out.type = TickOutput::Type::Position;
          out.position_cmd = cmd;
          out.allow_mode_reassert = true;
        }
      }
      break;

    case Mode::Landing:
      handleLanding(ros_now, out);
      break;

    case Mode::Velocity:
      if (has_velocity_cmd_) {
        out.type = TickOutput::Type::Velocity;
        out.velocity_cmd = last_velocity_cmd_;
        out.allow_mode_reassert = true;
      }
      break;
  }

  out.reset_warmup_counter = pending_warmup_reset_;
  pending_warmup_reset_ = false;
  return out;
}

void FlightModeManager::setMode(Mode new_mode, const char * reason, bool reset_warmup)
{
  if (mode_ != new_mode) {
    const char * names[] = {"planner", "manual", "landing", "velocity"};
    RCLCPP_INFO(
      logger_, "flight mode -> %s (%s)",
      names[static_cast<int>(new_mode)], reason);
  }
  mode_ = new_mode;
  if (new_mode != Mode::Planner) {
    planner_restart_pending_ = false;
    planner_startup_sequence_pending_ = false;
    planner_takeoff_initialized_ = false;
  }
  if (new_mode != Mode::Landing) {
    landing_command_sent_ = false;
    landing_retry_counter_ = 0;
  }
  if (reset_warmup) {
    pending_warmup_reset_ = true;
  }
}

bool FlightModeManager::captureCurrentPose(std::string & error_message)
{
  if (!has_planner_odom_ || !isFiniteVector(planner_odom_enu_)) {
    error_message = "planner odometry is not available yet";
    return false;
  }
  manual_target_position_enu_ = planner_odom_enu_;
  if (has_planner_yaw_) {
    manual_target_yaw_enu_ = planner_yaw_enu_;
  } else if (!manual_target_valid_) {
    if (has_auto_cmd_) {
      manual_target_yaw_enu_ = static_cast<float>(last_auto_cmd_.yaw);
    } else {
      manual_target_yaw_enu_ = 0.0f;
    }
  }
  manual_target_valid_ = true;
  return true;
}

quadrotor_msgs::msg::PositionCommand FlightModeManager::makeManualTargetCommand(
  const rclcpp::Time & stamp) const
{
  quadrotor_msgs::msg::PositionCommand cmd{};
  cmd.header.stamp = stamp;
  cmd.header.frame_id = config_.command_frame_id;
  cmd.position.x = manual_target_position_enu_[0];
  cmd.position.y = manual_target_position_enu_[1];
  cmd.position.z = manual_target_position_enu_[2];
  cmd.velocity.x = 0.0;
  cmd.velocity.y = 0.0;
  cmd.velocity.z = 0.0;
  cmd.acceleration.x = 0.0;
  cmd.acceleration.y = 0.0;
  cmd.acceleration.z = 0.0;
  cmd.yaw = manual_target_yaw_enu_;
  cmd.yaw_dot = 0.0;
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  return cmd;
}

bool FlightModeManager::getActivePositionCommand(
  quadrotor_msgs::msg::PositionCommand & cmd, const rclcpp::Time & stamp)
{
  switch (mode_) {
    case Mode::Planner:
      if (!has_auto_cmd_) return false;
      cmd = last_auto_cmd_;
      return true;
    case Mode::Manual:
    case Mode::Landing:
      if (!manual_target_valid_) return false;
      cmd = makeManualTargetCommand(stamp);
      return true;
    case Mode::Velocity:
      return false;
  }
  return false;
}

bool FlightModeManager::handlePlannerStartup(const rclcpp::Time & ros_now, TickOutput & out)
{
  if (!planner_startup_sequence_pending_) {
    return false;
  }

  if (!planner_takeoff_initialized_) {
    std::string err;
    if (!captureCurrentPose(err)) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 2000,
        "planner startup waiting for current pose: %s", err.c_str());
      return true;
    }
    manual_target_position_enu_[2] += config_.takeoff_height_m;
    manual_target_valid_ = true;
    planner_takeoff_target_z_ = manual_target_position_enu_[2];
    planner_takeoff_initialized_ = true;
    RCLCPP_INFO(
      logger_,
      "planner startup takeoff clearance armed to z=%.2f m before handing off to planner",
      planner_takeoff_target_z_);
  }

  constexpr float kPlannerTakeoffToleranceM = 0.15f;
  if (has_planner_odom_ &&
    planner_odom_enu_[2] >= planner_takeoff_target_z_ - kPlannerTakeoffToleranceM)
  {
    planner_startup_sequence_pending_ = false;
    planner_restart_pending_ = true;
    RCLCPP_INFO(logger_, "planner startup takeoff clearance reached, handing off to planner");
    return false;
  }

  out.type = TickOutput::Type::Position;
  out.position_cmd = makeManualTargetCommand(ros_now);
  out.allow_mode_reassert = true;
  return true;
}

void FlightModeManager::handleLanding(const rclcpp::Time & ros_now, TickOutput & out)
{
  constexpr int kLandRetryIntervalCycles = 50;

  if (!landing_command_sent_ ||
    (!is_auto_land_mode_ && (landing_retry_counter_ % kLandRetryIntervalCycles == 0)))
  {
    out.send_land_vehicle_cmd = true;
    landing_command_sent_ = true;
    RCLCPP_INFO(
      logger_,
      "sent land command (retry=%d, armed=%s, auto_land=%s)",
      landing_retry_counter_ / kLandRetryIntervalCycles,
      is_armed_ ? "true" : "false",
      is_auto_land_mode_ ? "true" : "false");
  }

  if (!is_auto_land_mode_ && is_armed_) {
    quadrotor_msgs::msg::PositionCommand hold_cmd{};
    if (getActivePositionCommand(hold_cmd, ros_now)) {
      out.type = TickOutput::Type::Position;
      out.position_cmd = hold_cmd;
      out.allow_mode_reassert = false;
    }
  }

  landing_retry_counter_ += 1;
}

}  // namespace uav_bridge
