#include "uav_bridge/px4_commander.hpp"

#include <cmath>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "uav_bridge/math_utils.hpp"

namespace uav_bridge
{

Px4Commander::Px4Commander(
  const Config & config,
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub,
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub,
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub,
  rclcpp::Logger logger)
: config_(config),
  offboard_pub_(offboard_pub),
  setpoint_pub_(setpoint_pub),
  cmd_pub_(cmd_pub),
  logger_(logger)
{
}

void Px4Commander::sendPositionSetpoint(
  const quadrotor_msgs::msg::PositionCommand & cmd,
  const std::array<float, 3> & frame_offset_ned,
  float frame_yaw_offset_ned,
  bool frame_aligned,
  uint64_t stamp_us,
  bool allow_mode_reassert,
  bool is_armed,
  bool is_offboard,
  bool px4_execution_ready)
{
  if (!ensureTimestampReady("position setpoint", stamp_us)) {
    return;
  }

  px4_msgs::msg::OffboardControlMode offboard_mode{};
  offboard_mode.timestamp = stamp_us;
  offboard_mode.position = true;
  offboard_mode.velocity = true;
  offboard_mode.acceleration = true;
  offboard_mode.attitude = false;
  offboard_mode.body_rate = false;
  offboard_mode.thrust_and_torque = false;
  offboard_mode.direct_actuator = false;
  offboard_pub_->publish(offboard_mode);

  const auto pos_ned_raw = enuPositionToNed(
    static_cast<float>(cmd.position.x),
    static_cast<float>(cmd.position.y),
    static_cast<float>(cmd.position.z));

  std::array<float, 3> pos_ned = {pos_ned_raw[0], pos_ned_raw[1], pos_ned_raw[2]};
  std::array<float, 3> velocity_ned = {
    static_cast<float>(cmd.velocity.y),
    static_cast<float>(cmd.velocity.x),
    static_cast<float>(-cmd.velocity.z)};
  std::array<float, 3> acceleration_ned = {
    static_cast<float>(cmd.acceleration.y),
    static_cast<float>(cmd.acceleration.x),
    static_cast<float>(-cmd.acceleration.z)};
  float yaw_ned = enuYawToNed(static_cast<float>(cmd.yaw));

  if (frame_aligned) {
    const float c = std::cos(frame_yaw_offset_ned);
    const float s = std::sin(frame_yaw_offset_ned);

    const float pos_x = (c * pos_ned[0]) - (s * pos_ned[1]);
    const float pos_y = (s * pos_ned[0]) + (c * pos_ned[1]);
    pos_ned[0] = pos_x + frame_offset_ned[0];
    pos_ned[1] = pos_y + frame_offset_ned[1];
    pos_ned[2] += frame_offset_ned[2];

    const float vel_x = (c * velocity_ned[0]) - (s * velocity_ned[1]);
    const float vel_y = (s * velocity_ned[0]) + (c * velocity_ned[1]);
    velocity_ned[0] = vel_x;
    velocity_ned[1] = vel_y;

    const float acc_x = (c * acceleration_ned[0]) - (s * acceleration_ned[1]);
    const float acc_y = (s * acceleration_ned[0]) + (c * acceleration_ned[1]);
    acceleration_ned[0] = acc_x;
    acceleration_ned[1] = acc_y;

    // Apply the same frame yaw offset to yaw setpoint so heading targets are
    // expressed in PX4 local frame consistently with rotated position vectors.
    yaw_ned = normalizeAngle(yaw_ned + frame_yaw_offset_ned);

  }

  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.timestamp = stamp_us;
  setpoint.position = {pos_ned[0], pos_ned[1], pos_ned[2]};

  const float vel_norm_before = vectorNorm(velocity_ned);
  const float acc_norm_before = vectorNorm(acceleration_ned);
  const bool vel_clamped = clampVectorNorm(velocity_ned, config_.max_velocity_mps);
  const bool acc_clamped = clampVectorNorm(acceleration_ned, config_.max_acceleration_mps2);

  if (vel_clamped || acc_clamped) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "clamped planner feedforward (vel %.2f->%.2f m/s, acc %.2f->%.2f m/s^2)",
      vel_norm_before, vectorNorm(velocity_ned),
      acc_norm_before, vectorNorm(acceleration_ned));
  }

  setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
  setpoint.acceleration = {acceleration_ned[0], acceleration_ned[1], acceleration_ned[2]};
  setpoint.jerk = {0.0f, 0.0f, 0.0f};
  setpoint.yaw = yaw_ned;
  float yawspeed_ned = static_cast<float>(-cmd.yaw_dot);
  // Suppress tiny yaw-rate feedforward noise that can cause slow self-spin in hover.
  if (std::abs(yawspeed_ned) < 0.02f) {
    yawspeed_ned = 0.0f;
  }
  setpoint.yawspeed = yawspeed_ned;
  setpoint_pub_->publish(setpoint);

  maybeReassertOffboard(allow_mode_reassert, is_armed, is_offboard, px4_execution_ready);
  setpoint_counter_ += 1;
  last_stamp_us_ = stamp_us;
}

void Px4Commander::sendVelocitySetpoint(
  const geometry_msgs::msg::Twist & cmd,
  uint64_t stamp_us,
  bool allow_mode_reassert,
  bool is_armed,
  bool is_offboard,
  bool px4_execution_ready)
{
  if (!ensureTimestampReady("velocity setpoint", stamp_us)) {
    return;
  }

  px4_msgs::msg::OffboardControlMode offboard_mode{};
  offboard_mode.timestamp = stamp_us;
  offboard_mode.position = false;
  offboard_mode.velocity = true;
  offboard_mode.acceleration = false;
  offboard_mode.attitude = false;
  offboard_mode.body_rate = false;
  offboard_mode.thrust_and_torque = false;
  offboard_mode.direct_actuator = false;
  offboard_pub_->publish(offboard_mode);

  std::array<float, 3> velocity_ned = {
    static_cast<float>(cmd.linear.y),
    static_cast<float>(cmd.linear.x),
    static_cast<float>(-cmd.linear.z)};

  const float vel_norm_before = vectorNorm(velocity_ned);
  if (clampVectorNorm(velocity_ned, config_.max_velocity_mps)) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "clamped velocity command feedforward %.2f->%.2f m/s",
      vel_norm_before, vectorNorm(velocity_ned));
  }

  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.timestamp = stamp_us;
  setpoint.position = {kNaN, kNaN, kNaN};
  setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
  setpoint.acceleration = {kNaN, kNaN, kNaN};
  setpoint.jerk = {kNaN, kNaN, kNaN};
  setpoint.yaw = kNaN;
  setpoint.yawspeed = static_cast<float>(-cmd.angular.z);
  setpoint_pub_->publish(setpoint);

  maybeReassertOffboard(allow_mode_reassert, is_armed, is_offboard, px4_execution_ready);
  setpoint_counter_ += 1;
  last_stamp_us_ = stamp_us;
}

void Px4Commander::sendVehicleCommand(
  uint32_t command, float param1, float param2, uint64_t stamp_us)
{
  if (!ensureTimestampReady("vehicle command", stamp_us)) {
    return;
  }

  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = stamp_us;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = config_.target_system;
  msg.target_component = config_.target_component;
  msg.source_system = config_.source_system;
  msg.source_component = config_.source_component;
  msg.from_external = true;
  cmd_pub_->publish(msg);
}

void Px4Commander::resetWarmup()
{
  setpoint_counter_ = 0;
}

bool Px4Commander::ensureTimestampReady(const char * context, uint64_t stamp)
{
  if (stamp != 0) {
    return true;
  }
  RCLCPP_WARN_THROTTLE(
    logger_, steady_clock_, 2000,
    "waiting for valid timestamp before sending %s to PX4", context);
  return false;
}

void Px4Commander::maybeReassertOffboard(
  bool allow_mode_reassert, bool is_armed, bool is_offboard, bool px4_execution_ready)
{
  if (!allow_mode_reassert || setpoint_counter_ < config_.warmup_cycles) {
    return;
  }

  const int cycles_after_warmup = setpoint_counter_ - config_.warmup_cycles;
  constexpr int kRetryIntervalCycles = 50;

  if (!px4_execution_ready) {
    if (cycles_after_warmup % kRetryIntervalCycles == 0) {
      RCLCPP_INFO(
        logger_,
        "waiting for PX4 execution ready before offboard enable");
    }
    return;
  }

  if ((!is_offboard || !is_armed) && (cycles_after_warmup % kRetryIntervalCycles == 0)) {
    sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f, last_stamp_us_);
    sendVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f, last_stamp_us_);
    RCLCPP_INFO(
      logger_,
      "sent offboard+arm command (retry=%d, armed=%s, offboard=%s)",
      cycles_after_warmup / kRetryIntervalCycles,
      is_armed ? "true" : "false",
      is_offboard ? "true" : "false");
  }
}

}  // namespace uav_bridge
