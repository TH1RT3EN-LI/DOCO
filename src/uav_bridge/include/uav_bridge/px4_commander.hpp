#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>

namespace uav_bridge
{

class Px4Commander
{
public:
  struct Config
  {
    uint8_t target_system{1};
    uint8_t target_component{1};
    uint8_t source_system{1};
    uint16_t source_component{1};
    int warmup_cycles{20};
    float max_velocity_mps{2.0f};
    float max_acceleration_mps2{4.0f};
  };

  Px4Commander(
    const Config & config,
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub,
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub,
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub,
    rclcpp::Logger logger);

  void sendPositionSetpoint(
    const quadrotor_msgs::msg::PositionCommand & cmd,
    const std::array<float, 3> & frame_offset_ned,
    float frame_yaw_offset_ned,
    bool frame_aligned,
    uint64_t stamp_us,
    bool allow_mode_reassert,
    bool is_armed,
    bool is_offboard,
    bool px4_execution_ready);

  void sendVelocitySetpoint(
    const geometry_msgs::msg::Twist & cmd,
    uint64_t stamp_us,
    bool allow_mode_reassert,
    bool is_armed,
    bool is_offboard,
    bool px4_execution_ready);

  void sendVehicleCommand(uint32_t command, float param1, float param2, uint64_t stamp_us);

  void resetWarmup();

private:
  bool ensureTimestampReady(const char * context, uint64_t stamp);
  void maybeReassertOffboard(
    bool allow_mode_reassert, bool is_armed, bool is_offboard, bool px4_execution_ready);

  Config config_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  int setpoint_counter_{0};
  uint64_t last_stamp_us_{0};
};

}  // namespace uav_bridge
