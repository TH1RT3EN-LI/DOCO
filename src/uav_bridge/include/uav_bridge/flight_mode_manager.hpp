#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace uav_bridge
{

class FlightModeManager
{
public:
  enum class Mode : uint8_t { Planner = 0, Manual = 1, Landing = 2, Velocity = 3 };

  struct Config
  {
    float takeoff_height_m{1.0f};
    bool nudge_body_frame{true};
    bool require_planner_takeoff_clearance{true};
    double velocity_timeout_ms{200.0};
  };

  struct TickOutput
  {
    enum class Type { None, Position, Velocity };
    Type type{Type::None};
    quadrotor_msgs::msg::PositionCommand position_cmd{};
    geometry_msgs::msg::Twist velocity_cmd{};
    bool allow_mode_reassert{false};
    bool send_land_vehicle_cmd{false};
    bool request_planner_restart{false};
    bool reset_warmup_counter{false};
  };

  explicit FlightModeManager(const Config & config, rclcpp::Logger logger);

  void onPlannerOdom(const std::array<float, 3> & enu_pos, float yaw_enu);
  void onPlannerCommand(const quadrotor_msgs::msg::PositionCommand & cmd);
  void onManualPose(const geometry_msgs::msg::PoseStamped & msg);
  void onNudge(const geometry_msgs::msg::Twist & msg);
  void onVelocityCommand(const geometry_msgs::msg::Twist & msg, uint64_t now_us);
  void onVehicleStatus(bool armed, bool offboard_mode, bool auto_land_mode);

  bool requestTakeoff(std::string & message);
  bool requestHover(std::string & message);
  bool requestLand(std::string & message);
  bool requestResumeAuto(std::string & message);

  TickOutput tick(uint64_t now_us, const rclcpp::Time & ros_now, bool px4_execution_ready);

  Mode getMode() const { return mode_; }
  bool isArmed() const { return is_armed_; }
  bool isOffboardMode() const { return is_offboard_mode_; }

private:
  void setMode(Mode new_mode, const char * reason, bool reset_warmup);
  bool captureCurrentPose(std::string & error_message);
  quadrotor_msgs::msg::PositionCommand makeManualTargetCommand(const rclcpp::Time & stamp) const;
  bool getActivePositionCommand(quadrotor_msgs::msg::PositionCommand & cmd, const rclcpp::Time & stamp);
  bool handlePlannerStartup(const rclcpp::Time & ros_now, TickOutput & out);
  void handleLanding(const rclcpp::Time & ros_now, TickOutput & out);

  Config config_;
  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  Mode mode_{Mode::Planner};

  bool has_planner_odom_{false};
  bool has_planner_yaw_{false};
  std::array<float, 3> planner_odom_enu_{0.0f, 0.0f, 0.0f};
  float planner_yaw_enu_{0.0f};

  bool has_auto_cmd_{false};
  quadrotor_msgs::msg::PositionCommand last_auto_cmd_{};

  bool manual_target_valid_{false};
  std::array<float, 3> manual_target_position_enu_{0.0f, 0.0f, 0.0f};
  float manual_target_yaw_enu_{0.0f};

  bool planner_restart_pending_{false};
  bool planner_startup_sequence_pending_{false};
  bool planner_takeoff_initialized_{false};
  float planner_takeoff_target_z_{0.0f};

  bool landing_command_sent_{false};
  int landing_retry_counter_{0};

  bool has_velocity_cmd_{false};
  geometry_msgs::msg::Twist last_velocity_cmd_{};
  uint64_t last_velocity_time_us_{0};

  bool is_armed_{false};
  bool is_offboard_mode_{false};
  bool is_auto_land_mode_{false};

  bool pending_warmup_reset_{false};
};

}  // namespace uav_bridge
