#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <uav_bridge/frame_aligner.hpp>
#include <uav_bridge/math_utils.hpp>

namespace uav_bridge
{
  class OffboardBridgeNode : public rclcpp::Node
  {
    using Empty = std_msgs::msg::Empty;
    using Trigger = std_srvs::srv::Trigger;

    enum class ControlSource : uint8_t
    {
      Planner = 0,
      Manual = 1,
      Landing = 2,
      Velocity = 3,
    };

    enum class Px4TimestampSource : uint8_t
    {
      System = 0,
      GazeboSim = 1,
    };

  public:
    OffboardBridgeNode() : Node("offboard_bridge_node")
    {
      this->declare_parameter<std::string>("command_topic", "/uav/planning/position_cmd");
      this->declare_parameter<std::string>("manual_pose_topic", "/uav/control/pose");
      this->declare_parameter<std::string>("nudge_topic", "/uav/control/nudge");
      this->declare_parameter<std::string>("velocity_topic", "/uav/control/velocity");
      this->declare_parameter<std::string>("command_frame_id", "uav_map");
      this->declare_parameter<double>("velocity_timeout_ms", 200.0);
      this->declare_parameter<std::string>("offboard_mode_topic", "/uav/fmu/in/offboard_control_mode");
      this->declare_parameter<std::string>("trajectory_setpoint_topic", "/uav/fmu/in/trajectory_setpoint");
      this->declare_parameter<std::string>("vehicle_command_topic", "/uav/fmu/in/vehicle_command");
      this->declare_parameter<std::string>("planner_odom_topic", "/uav/odom");
      this->declare_parameter<std::string>("px4_local_position_topic", "/uav/fmu/out/vehicle_local_position");
      this->declare_parameter<std::string>("vehicle_status_topic", "/uav/fmu/out/vehicle_status");
      this->declare_parameter<std::string>("restart_trajectory_topic", "/uav/planning/restart_trajectory");
      this->declare_parameter<std::string>("takeoff_service", "/uav/control/takeoff");
      this->declare_parameter<std::string>("hover_service", "/uav/control/hover");
      this->declare_parameter<std::string>("land_service", "/uav/control/land");
      this->declare_parameter<std::string>("resume_auto_service", "/uav/control/resume_auto");
      this->declare_parameter<double>("publish_rate_hz", 50.0);
      this->declare_parameter<int>("warmup_cycles", 20);
      this->declare_parameter<int>("target_system", 1);
      this->declare_parameter<int>("target_component", 1);
      this->declare_parameter<int>("source_system", 1);
      this->declare_parameter<int>("source_component", 1);
      this->declare_parameter<double>("takeoff_height_m", 1.0);
      this->declare_parameter<bool>("nudge_body_frame", true);
      this->declare_parameter<bool>("enable_frame_alignment", true);
      this->declare_parameter<bool>("lock_alignment_on_first_valid", true);
      this->declare_parameter<bool>("relock_on_local_position_reset", true);
      this->declare_parameter<bool>("require_planner_takeoff_clearance", true);
      this->declare_parameter<double>("max_velocity_setpoint_mps", 1.5);
      this->declare_parameter<double>("max_acceleration_setpoint_mps2", 2.0);
      this->declare_parameter<std::string>("px4_timestamp_source", "system");
      this->declare_parameter<std::string>("world_name", "test");
      this->declare_parameter<std::string>("gz_clock_topic", "");

      command_topic_ = this->get_parameter("command_topic").as_string();
      manual_pose_topic_ = this->get_parameter("manual_pose_topic").as_string();
      nudge_topic_ = this->get_parameter("nudge_topic").as_string();
      velocity_topic_ = this->get_parameter("velocity_topic").as_string();
      command_frame_id_ = this->get_parameter("command_frame_id").as_string();
      velocity_timeout_ms_ = this->get_parameter("velocity_timeout_ms").as_double();
      offboard_mode_topic_ = this->get_parameter("offboard_mode_topic").as_string();
      trajectory_setpoint_topic_ = this->get_parameter("trajectory_setpoint_topic").as_string();
      vehicle_command_topic_ = this->get_parameter("vehicle_command_topic").as_string();
      planner_odom_topic_ = this->get_parameter("planner_odom_topic").as_string();
      px4_local_position_topic_ = this->get_parameter("px4_local_position_topic").as_string();
      vehicle_status_topic_ = this->get_parameter("vehicle_status_topic").as_string();
      restart_trajectory_topic_ = this->get_parameter("restart_trajectory_topic").as_string();
      takeoff_service_ = this->get_parameter("takeoff_service").as_string();
      hover_service_ = this->get_parameter("hover_service").as_string();
      land_service_ = this->get_parameter("land_service").as_string();
      resume_auto_service_ = this->get_parameter("resume_auto_service").as_string();
      publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
      warmup_cycles_ = this->get_parameter("warmup_cycles").as_int();
      target_system_ = static_cast<uint8_t>(this->get_parameter("target_system").as_int());
      target_component_ = static_cast<uint8_t>(this->get_parameter("target_component").as_int());
      source_system_ = static_cast<uint8_t>(this->get_parameter("source_system").as_int());
      source_component_ = static_cast<uint16_t>(this->get_parameter("source_component").as_int());
      takeoff_height_m_ = static_cast<float>(this->get_parameter("takeoff_height_m").as_double());
      nudge_body_frame_ = this->get_parameter("nudge_body_frame").as_bool();
      enable_frame_alignment_ = this->get_parameter("enable_frame_alignment").as_bool();
      lock_alignment_on_first_valid_ = this->get_parameter("lock_alignment_on_first_valid").as_bool();
      relock_on_local_position_reset_ = this->get_parameter("relock_on_local_position_reset").as_bool();
      require_planner_takeoff_clearance_ = this->get_parameter("require_planner_takeoff_clearance").as_bool();
      max_velocity_setpoint_mps_ = static_cast<float>(this->get_parameter("max_velocity_setpoint_mps").as_double());
      max_acceleration_setpoint_mps2_ = static_cast<float>(this->get_parameter("max_acceleration_setpoint_mps2").as_double());
      world_name_ = this->get_parameter("world_name").as_string();
      gz_clock_topic_ = this->get_parameter("gz_clock_topic").as_string();

      const std::string px4_timestamp_source = this->get_parameter("px4_timestamp_source").as_string();
      if (px4_timestamp_source == "system")
      {
        px4_timestamp_source_ = Px4TimestampSource::System;
      }
      else if (px4_timestamp_source == "gz_sim")
      {
        px4_timestamp_source_ = Px4TimestampSource::GazeboSim;
      }
      else
      {
        RCLCPP_WARN(
            this->get_logger(),
            "unknown px4_timestamp_source='%s', falling back to system",
            px4_timestamp_source.c_str());
        px4_timestamp_source_ = Px4TimestampSource::System;
      }

      if (px4_timestamp_source_ == Px4TimestampSource::GazeboSim)
      {
        if (gz_clock_topic_.empty())
        {
          gz_clock_topic_ = "/world/" + world_name_ + "/clock";
        }

        const bool ok = gz_node_.Subscribe(gz_clock_topic_, &OffboardBridgeNode::onGzClock, this);
        if (!ok)
        {
          RCLCPP_FATAL(
              this->get_logger(),
              "failed to subscribe Gazebo clock topic: %s",
              gz_clock_topic_.c_str());
          throw std::runtime_error("Gazebo clock subscribe failed");
        }
      }

      frame_aligner_ = FrameAligner(
        FrameAligner::Config{lock_alignment_on_first_valid_, 0.02f, relock_on_local_position_reset_},
        this->get_logger());

      offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          offboard_mode_topic_, 10);
      trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          trajectory_setpoint_topic_, 10);
      vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
          vehicle_command_topic_, 10);
      restart_trajectory_pub_ = this->create_publisher<Empty>(restart_trajectory_topic_, 10);

      auto_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
          command_topic_, 10,
          [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
          {
            last_auto_cmd_ = *msg;
            has_auto_cmd_ = true;
            if (!is_armed_ || !is_offboard_mode_)
            {
              planner_restart_pending_ = true;
              if (require_planner_takeoff_clearance_ && !planner_startup_sequence_pending_ && !planner_takeoff_initialized_)
              {
                planner_startup_sequence_pending_ = true;
              }
            }
          });

      manual_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          manual_pose_topic_, 10,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            handleManualPoseCommand(*msg);
          });

      nudge_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          nudge_topic_, 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg)
          {
            handleNudgeCommand(*msg);
          });

      velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
          velocity_topic_, 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg)
          {
            handleVelocityCommand(*msg);
          });

      const auto sensor_qos = rclcpp::SensorDataQoS();

      planner_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          planner_odom_topic_, sensor_qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            planner_odom_enu_ = {
                static_cast<float>(msg->pose.pose.position.x),
                static_cast<float>(msg->pose.pose.position.y),
                static_cast<float>(msg->pose.pose.position.z)};
            planner_yaw_enu_ = quaternionToYaw(msg->pose.pose.orientation);
            has_planner_yaw_ = true;
            has_planner_odom_ = true;
            if (enable_frame_alignment_) {
              frame_aligner_.updatePlannerPose(planner_odom_enu_, planner_yaw_enu_);
            }
          });

      px4_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          px4_local_position_topic_, sensor_qos,
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
          {
            if (!msg->xy_valid || !msg->z_valid)
            {
              return;
            }
            frame_aligner_.updatePx4Pose(
              {msg->x, msg->y, msg->z},
              msg->xy_reset_counter,
              msg->z_reset_counter,
              msg->delta_xy[0],
              msg->delta_xy[1],
              msg->delta_z,
              msg->heading);
          });

      vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
          vehicle_status_topic_, sensor_qos,
          [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
          {
            is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
            is_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
            is_auto_land_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
          });

      takeoff_srv_ = this->create_service<Trigger>(
          takeoff_service_,
          [this](const std::shared_ptr<Trigger::Request> request,
                 std::shared_ptr<Trigger::Response> response)
          {
            (void)request;
            response->success = handleTakeoffRequest(response->message);
          });

      hover_srv_ = this->create_service<Trigger>(
          hover_service_,
          [this](const std::shared_ptr<Trigger::Request> request,
                 std::shared_ptr<Trigger::Response> response)
          {
            (void)request;
            response->success = handleHoverRequest(response->message);
          });

      land_srv_ = this->create_service<Trigger>(
          land_service_,
          [this](const std::shared_ptr<Trigger::Request> request,
                 std::shared_ptr<Trigger::Response> response)
          {
            (void)request;
            response->success = handleLandRequest(response->message);
          });

      resume_auto_srv_ = this->create_service<Trigger>(
          resume_auto_service_,
          [this](const std::shared_ptr<Trigger::Request> request,
                 std::shared_ptr<Trigger::Response> response)
          {
            (void)request;
            response->success = handleResumeAutoRequest(response->message);
          });

      const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_));

      timer_ = this->create_wall_timer(
          timer_period,
          std::bind(&OffboardBridgeNode::timerCallback, this));

      RCLCPP_INFO(
          this->get_logger(),
          "offboard bridge: auto_cmd=%s manual_pose=%s nudge=%s offboard=%s setpoint=%s vehicle_cmd=%s command_frame=%s rate=%.1f warmup=%d timestamp_source=%s%s%s",
          command_topic_.c_str(),
          manual_pose_topic_.c_str(),
          nudge_topic_.c_str(),
          offboard_mode_topic_.c_str(),
          trajectory_setpoint_topic_.c_str(),
          vehicle_command_topic_.c_str(),
          command_frame_id_.c_str(),
          publish_rate_hz_,
          warmup_cycles_,
          px4TimestampSourceName(px4_timestamp_source_),
          px4_timestamp_source_ == Px4TimestampSource::GazeboSim ? " gz_clock=" : "",
          px4_timestamp_source_ == Px4TimestampSource::GazeboSim ? gz_clock_topic_.c_str() : "");
    }

  private:
    static const char *px4TimestampSourceName(Px4TimestampSource source)
    {
      switch (source)
      {
      case Px4TimestampSource::System:
        return "system";
      case Px4TimestampSource::GazeboSim:
        return "gz_sim";
      }
      return "unknown";
    }

    void onGzClock(const gz::msgs::Clock &clock)
    {
      const uint64_t now_us =
          (static_cast<uint64_t>(clock.sim().sec()) * 1000000ULL) +
          (static_cast<uint64_t>(clock.sim().nsec()) / 1000ULL);

      const uint64_t previous_us = latest_gz_clock_us_.load();
      if (previous_us != 0 && now_us < previous_us)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            steady_clock_,
            2000,
            "ignored backward Gazebo clock jump: %.6f s -> %.6f s",
            static_cast<double>(previous_us) / 1e6,
            static_cast<double>(now_us) / 1e6);
        return;
      }

      latest_gz_clock_us_.store(now_us);
    }

    uint64_t nowMicros()
    {
      if (px4_timestamp_source_ == Px4TimestampSource::GazeboSim)
      {
        return latest_gz_clock_us_.load();
      }

      return static_cast<uint64_t>(px4_timestamp_system_clock_.now().nanoseconds() / 1000);
    }

    bool ensureTimestampReady(const char *context, uint64_t stamp)
    {
      if (stamp != 0)
      {
        return true;
      }

      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          steady_clock_,
          2000,
          "waiting for valid %s timestamp before sending %s to PX4",
          px4TimestampSourceName(px4_timestamp_source_),
          context);
      return false;
    }

    bool captureCurrentPose(std::string &error_message)
    {
      if (!has_planner_odom_ || !isFiniteVector(planner_odom_enu_))
      {
        error_message = "planner odometry is not available yet";
        return false;
      }

      manual_target_position_enu_ = planner_odom_enu_;
      if (has_planner_yaw_)
      {
        manual_target_yaw_enu_ = planner_yaw_enu_;
      }
      else if (manual_target_valid_)
      {
        manual_target_yaw_enu_ = manual_target_yaw_enu_;
      }
      else if (has_auto_cmd_)
      {
        manual_target_yaw_enu_ = static_cast<float>(last_auto_cmd_.yaw);
      }
      else
      {
        manual_target_yaw_enu_ = 0.0f;
      }

      manual_target_valid_ = true;
      return true;
    }

    void resetWarmup()
    {
      setpoint_counter_ = 0;
    }

    void setControlSource(ControlSource new_source, const char *reason, bool reset_warmup)
    {
      if (control_source_ != new_source)
      {
        RCLCPP_INFO(this->get_logger(), "control source -> %s (%s)", controlSourceName(new_source), reason);
      }
      control_source_ = new_source;
      if (new_source != ControlSource::Planner)
      {
        planner_restart_pending_ = false;
        planner_startup_sequence_pending_ = false;
        planner_takeoff_initialized_ = false;
      }
      if (new_source != ControlSource::Landing)
      {
        landing_command_sent_ = false;
        landing_retry_counter_ = 0;
      }
      if (reset_warmup)
      {
        resetWarmup();
      }
    }

    const char *controlSourceName(ControlSource source) const
    {
      switch (source)
      {
      case ControlSource::Planner:
        return "planner";
      case ControlSource::Manual:
        return "manual";
      case ControlSource::Landing:
        return "landing";
      case ControlSource::Velocity:
        return "velocity";
      }
      return "unknown";
    }

    quadrotor_msgs::msg::PositionCommand makeManualTargetCommand() const
    {
      quadrotor_msgs::msg::PositionCommand cmd{};
      cmd.header.stamp = this->now();
      cmd.header.frame_id = command_frame_id_;
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

    bool getActiveCommand(quadrotor_msgs::msg::PositionCommand &cmd)
    {
      switch (control_source_)
      {
      case ControlSource::Planner:
        if (!has_auto_cmd_)
        {
          return false;
        }
        cmd = last_auto_cmd_;
        return true;
      case ControlSource::Manual:
        if (!manual_target_valid_)
        {
          return false;
        }
        cmd = makeManualTargetCommand();
        return true;
      case ControlSource::Landing:
        if (!manual_target_valid_)
        {
          return false;
        }
        cmd = makeManualTargetCommand();
        return true;
      case ControlSource::Velocity:
        return false;
      }
      return false;
    }

    void publishVehicleCommand(uint32_t command, float param1, float param2)
    {
      const uint64_t stamp = nowMicros();
      if (!ensureTimestampReady("vehicle command", stamp))
      {
        return;
      }

      px4_msgs::msg::VehicleCommand msg{};
      msg.timestamp = stamp;
      msg.param1 = param1;
      msg.param2 = param2;
      msg.command = command;
      msg.target_system = target_system_;
      msg.target_component = target_component_;
      msg.source_system = source_system_;
      msg.source_component = source_component_;
      msg.from_external = true;
      vehicle_command_pub_->publish(msg);
    }

    bool isPx4ExecutionReady() const
    {
      return frame_aligner_.hasPx4Pose() && (!enable_frame_alignment_ || frame_aligner_.isAligned());
    }

    void maybeRestartPlannerTrajectory()
    {
      if (!planner_restart_pending_ || planner_startup_sequence_pending_ ||
          control_source_ != ControlSource::Planner || !has_auto_cmd_)
      {
        return;
      }

      if (!is_armed_ || !is_offboard_mode_ || !isPx4ExecutionReady())
      {
        return;
      }

      restart_trajectory_pub_->publish(Empty{});
      planner_restart_pending_ = false;
      RCLCPP_INFO(this->get_logger(), "requested planner trajectory restart after offboard became active");
    }

    bool handlePlannerStartupSequence()
    {
      if (!planner_startup_sequence_pending_)
      {
        return false;
      }

      if (!planner_takeoff_initialized_)
      {
        std::string error_message;
        if (!captureCurrentPose(error_message))
        {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              2000,
              "planner startup waiting for current pose: %s",
              error_message.c_str());
          return true;
        }

        manual_target_position_enu_[2] += takeoff_height_m_;
        manual_target_valid_ = true;
        planner_takeoff_target_z_ = manual_target_position_enu_[2];
        planner_takeoff_initialized_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "planner startup takeoff clearance armed to z=%.2f m before handing off to planner",
            planner_takeoff_target_z_);
      }

      constexpr float kPlannerTakeoffToleranceM = 0.15f;
      if (has_planner_odom_ && planner_odom_enu_[2] >= planner_takeoff_target_z_ - kPlannerTakeoffToleranceM)
      {
        planner_startup_sequence_pending_ = false;
        planner_restart_pending_ = true;
        RCLCPP_INFO(this->get_logger(), "planner startup takeoff clearance reached, handing off to planner");
        maybeRestartPlannerTrajectory();
        return false;
      }

      publishOffboardSetpoint(makeManualTargetCommand(), true);
      return true;
    }

    void publishOffboardSetpoint(const quadrotor_msgs::msg::PositionCommand &cmd, bool allow_mode_reassert)
    {
      const uint64_t stamp = nowMicros();
      if (!ensureTimestampReady("position setpoint", stamp))
      {
        return;
      }

      px4_msgs::msg::OffboardControlMode offboard_mode{};
      offboard_mode.timestamp = stamp;
      offboard_mode.position = true;
      offboard_mode.velocity = true;
      offboard_mode.acceleration = true;
      offboard_mode.attitude = false;
      offboard_mode.body_rate = false;
      offboard_mode.thrust_and_torque = false;
      offboard_mode.direct_actuator = false;
      offboard_mode_pub_->publish(offboard_mode);

      const auto pos_ned_raw = enuPositionToNed(
          static_cast<float>(cmd.position.x),
          static_cast<float>(cmd.position.y),
          static_cast<float>(cmd.position.z));

      px4_msgs::msg::TrajectorySetpoint setpoint{};
      setpoint.timestamp = stamp;
      setpoint.position = {pos_ned_raw[0], pos_ned_raw[1], pos_ned_raw[2]};
      if (enable_frame_alignment_ && frame_aligner_.isAligned())
      {
        const auto offset = frame_aligner_.getOffsetNed();
        setpoint.position[0] += offset[0];
        setpoint.position[1] += offset[1];
        setpoint.position[2] += offset[2];
      }
      std::array<float, 3> velocity_ned = {
          static_cast<float>(cmd.velocity.y),
          static_cast<float>(cmd.velocity.x),
          static_cast<float>(-cmd.velocity.z)};
      std::array<float, 3> acceleration_ned = {
          static_cast<float>(cmd.acceleration.y),
          static_cast<float>(cmd.acceleration.x),
          static_cast<float>(-cmd.acceleration.z)};

      const float velocity_norm_before = vectorNorm(velocity_ned);
      const float acceleration_norm_before = vectorNorm(acceleration_ned);
      const bool velocity_clamped = clampVectorNorm(velocity_ned, max_velocity_setpoint_mps_);
      const bool acceleration_clamped = clampVectorNorm(acceleration_ned, max_acceleration_setpoint_mps2_);

      if (velocity_clamped || acceleration_clamped)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "clamped planner feedforward (vel %.2f->%.2f m/s, acc %.2f->%.2f m/s^2)",
            velocity_norm_before,
            vectorNorm(velocity_ned),
            acceleration_norm_before,
            vectorNorm(acceleration_ned));
      }

      setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
      setpoint.acceleration = {acceleration_ned[0], acceleration_ned[1], acceleration_ned[2]};
      setpoint.jerk = {0.0f, 0.0f, 0.0f};
      setpoint.yaw = enuYawToNed(static_cast<float>(cmd.yaw));
      setpoint.yawspeed = static_cast<float>(-cmd.yaw_dot);
      trajectory_setpoint_pub_->publish(setpoint);

      if (allow_mode_reassert && setpoint_counter_ >= warmup_cycles_)
      {
        const int cycles_after_warmup = setpoint_counter_ - warmup_cycles_;
        constexpr int kCommandRetryIntervalCycles = 50;
        if (!isPx4ExecutionReady())
        {
          if (cycles_after_warmup % kCommandRetryIntervalCycles == 0)
          {
            RCLCPP_INFO(
                this->get_logger(),
                "waiting for PX4 local position/alignment before offboard enable (local_pos=%s alignment=%s)",
                frame_aligner_.hasPx4Pose() ? "true" : "false",
                (!enable_frame_alignment_ || frame_aligner_.isAligned()) ? "true" : "false");
          }
        }
        else if ((!is_offboard_mode_ || !is_armed_) && (cycles_after_warmup % kCommandRetryIntervalCycles == 0))
        {
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
          RCLCPP_INFO(
              this->get_logger(),
              "sent offboard+arm command (retry=%d, armed=%s, offboard=%s)",
              cycles_after_warmup / kCommandRetryIntervalCycles,
              is_armed_ ? "true" : "false",
              is_offboard_mode_ ? "true" : "false");
        }
      }

      setpoint_counter_ += 1;
    }

    void handleManualPoseCommand(const geometry_msgs::msg::PoseStamped &msg)
    {
      manual_target_position_enu_ = {
          static_cast<float>(msg.pose.position.x),
          static_cast<float>(msg.pose.position.y),
          static_cast<float>(msg.pose.position.z)};

      if (!isFiniteVector(manual_target_position_enu_))
      {
        RCLCPP_WARN(this->get_logger(), "ignored manual pose command with non-finite target");
        return;
      }

      const double quat_norm =
          (msg.pose.orientation.x * msg.pose.orientation.x) +
          (msg.pose.orientation.y * msg.pose.orientation.y) +
          (msg.pose.orientation.z * msg.pose.orientation.z) +
          (msg.pose.orientation.w * msg.pose.orientation.w);

      if (quat_norm > 1.0e-6)
      {
        manual_target_yaw_enu_ = quaternionToYaw(msg.pose.orientation);
      }
      else if (has_planner_yaw_)
      {
        manual_target_yaw_enu_ = planner_yaw_enu_;
      }
      else if (!manual_target_valid_ && has_auto_cmd_)
      {
        manual_target_yaw_enu_ = static_cast<float>(last_auto_cmd_.yaw);
      }

      manual_target_valid_ = true;
      const bool reset_warmup = control_source_ != ControlSource::Manual;
      setControlSource(ControlSource::Manual, "manual pose target", reset_warmup);
    }

    void handleNudgeCommand(const geometry_msgs::msg::Twist &msg)
    {
      if (!manual_target_valid_)
      {
        std::string error_message;
        if (!captureCurrentPose(error_message))
        {
          RCLCPP_WARN(this->get_logger(), "cannot apply nudge: %s", error_message.c_str());
          return;
        }
      }

      const float reference_yaw = has_planner_yaw_ ? planner_yaw_enu_ : manual_target_yaw_enu_;
      float delta_x = static_cast<float>(msg.linear.x);
      float delta_y = static_cast<float>(msg.linear.y);

      if (nudge_body_frame_)
      {
        const float cos_yaw = std::cos(reference_yaw);
        const float sin_yaw = std::sin(reference_yaw);
        const float body_x = delta_x;
        const float body_y = delta_y;
        delta_x = (cos_yaw * body_x) - (sin_yaw * body_y);
        delta_y = (sin_yaw * body_x) + (cos_yaw * body_y);
      }

      manual_target_position_enu_[0] += delta_x;
      manual_target_position_enu_[1] += delta_y;
      manual_target_position_enu_[2] += static_cast<float>(msg.linear.z);
      manual_target_yaw_enu_ = normalizeAngle(
          manual_target_yaw_enu_ + static_cast<float>(msg.angular.z));
      manual_target_valid_ = true;

      const bool reset_warmup = control_source_ != ControlSource::Manual;
      setControlSource(ControlSource::Manual, "manual nudge", reset_warmup);
    }

    bool handleTakeoffRequest(std::string &message)
    {
      std::string error_message;
      if (!captureCurrentPose(error_message))
      {
        message = error_message;
        return false;
      }

      manual_target_position_enu_[2] += takeoff_height_m_;
      manual_target_valid_ = true;
      setControlSource(ControlSource::Manual, "takeoff", true);
      message = "takeoff target armed";
      return true;
    }

    bool handleHoverRequest(std::string &message)
    {
      std::string error_message;
      if (!captureCurrentPose(error_message))
      {
        message = error_message;
        return false;
      }

      setControlSource(ControlSource::Manual, "hover", true);
      message = "hover target locked to current odometry";
      return true;
    }

    bool handleLandRequest(std::string &message)
    {
      if (!manual_target_valid_)
      {
        std::string error_message;
        if (!captureCurrentPose(error_message))
        {
          message = error_message;
          return false;
        }
      }

      setControlSource(ControlSource::Landing, "land", false);
      landing_command_sent_ = false;
      landing_retry_counter_ = 0;
      message = "landing requested";
      return true;
    }

    bool handleResumeAutoRequest(std::string &message)
    {
      if (!has_auto_cmd_)
      {
        message = "planner command has not been received yet";
        return false;
      }

      setControlSource(ControlSource::Planner, "resume auto", true);
      message = "planner control resumed";
      return true;
    }

    void handleLandingTimer()
    {
      constexpr int kLandRetryIntervalCycles = 50;

      if (!landing_command_sent_ || (!is_auto_land_mode_ && (landing_retry_counter_ % kLandRetryIntervalCycles == 0)))
      {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
        landing_command_sent_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "sent land command (retry=%d, armed=%s, auto_land=%s)",
            landing_retry_counter_ / kLandRetryIntervalCycles,
            is_armed_ ? "true" : "false",
            is_auto_land_mode_ ? "true" : "false");
      }

      if (!is_auto_land_mode_ && is_armed_)
      {
        quadrotor_msgs::msg::PositionCommand hold_cmd{};
        if (getActiveCommand(hold_cmd))
        {
          publishOffboardSetpoint(hold_cmd, false);
        }
      }

      landing_retry_counter_ += 1;
    }

    void handleVelocityCommand(const geometry_msgs::msg::Twist &msg)
    {
      last_velocity_cmd_ = msg;
      last_velocity_time_us_ = nowMicros();
      has_velocity_cmd_ = true;
      const bool reset_warmup = control_source_ != ControlSource::Velocity;
      setControlSource(ControlSource::Velocity, "velocity cmd", reset_warmup);
    }

    void handleVelocityTimer()
    {
      if (!has_velocity_cmd_)
      {
        return;
      }

      const uint64_t now_us = nowMicros();
      if (now_us == 0)
      {
        if (!ensureTimestampReady("velocity command timeout", now_us))
        {
          return;
        }
      }

      if (last_velocity_time_us_ == 0)
      {
        last_velocity_time_us_ = now_us;
        return;
      }

      const double elapsed_ms = static_cast<double>(now_us - last_velocity_time_us_) / 1000.0;
      if (elapsed_ms > velocity_timeout_ms_)
      {
        RCLCPP_WARN(
            this->get_logger(),
            "velocity command timeout (%.0f ms > %.0f ms), falling back to hover",
            elapsed_ms,
            velocity_timeout_ms_);
        has_velocity_cmd_ = false;
        std::string dummy;
        handleHoverRequest(dummy);
        return;
      }
      publishVelocitySetpoint(last_velocity_cmd_, true);
    }

    void publishVelocitySetpoint(const geometry_msgs::msg::Twist &cmd, bool allow_mode_reassert)
    {
      const uint64_t stamp = nowMicros();
      if (!ensureTimestampReady("velocity setpoint", stamp))
      {
        return;
      }

      px4_msgs::msg::OffboardControlMode offboard_mode{};
      offboard_mode.timestamp = stamp;
      offboard_mode.position = false;
      offboard_mode.velocity = true;
      offboard_mode.acceleration = false;
      offboard_mode.attitude = false;
      offboard_mode.body_rate = false;
      offboard_mode.thrust_and_torque = false;
      offboard_mode.direct_actuator = false;
      offboard_mode_pub_->publish(offboard_mode);

      std::array<float, 3> velocity_ned = {
          static_cast<float>(cmd.linear.y),
          static_cast<float>(cmd.linear.x),
          static_cast<float>(-cmd.linear.z)};
      const float velocity_norm_before = vectorNorm(velocity_ned);
      if (clampVectorNorm(velocity_ned, max_velocity_setpoint_mps_))
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "clamped velocity command feedforward %.2f->%.2f m/s",
            velocity_norm_before,
            vectorNorm(velocity_ned));
      }
      const float yaw_rate_ned = static_cast<float>(-cmd.angular.z);

      constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
      px4_msgs::msg::TrajectorySetpoint setpoint{};
      setpoint.timestamp = stamp;
      setpoint.position = {kNaN, kNaN, kNaN};
      setpoint.velocity = {velocity_ned[0], velocity_ned[1], velocity_ned[2]};
      setpoint.acceleration = {kNaN, kNaN, kNaN};
      setpoint.jerk = {kNaN, kNaN, kNaN};
      setpoint.yaw = kNaN;
      setpoint.yawspeed = yaw_rate_ned;
      trajectory_setpoint_pub_->publish(setpoint);

      if (allow_mode_reassert && setpoint_counter_ >= warmup_cycles_)
      {
        const int cycles_after_warmup = setpoint_counter_ - warmup_cycles_;
        constexpr int kCommandRetryIntervalCycles = 50;
        if (!isPx4ExecutionReady())
        {
          if (cycles_after_warmup % kCommandRetryIntervalCycles == 0)
          {
            RCLCPP_INFO(
                this->get_logger(),
                "waiting for PX4 local position/alignment before velocity offboard enable (local_pos=%s alignment=%s)",
                frame_aligner_.hasPx4Pose() ? "true" : "false",
                (!enable_frame_alignment_ || frame_aligner_.isAligned()) ? "true" : "false");
          }
        }
        else if ((!is_offboard_mode_ || !is_armed_) && (cycles_after_warmup % kCommandRetryIntervalCycles == 0))
        {
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
          RCLCPP_INFO(
              this->get_logger(),
              "sent offboard+arm command (retry=%d, armed=%s, offboard=%s)",
              cycles_after_warmup / kCommandRetryIntervalCycles,
              is_armed_ ? "true" : "false",
              is_offboard_mode_ ? "true" : "false");
        }
      }
      setpoint_counter_ += 1;
    }

    void timerCallback()
    {
      maybeRestartPlannerTrajectory();

      if (control_source_ == ControlSource::Planner && handlePlannerStartupSequence())
      {
        return;
      }

      if (control_source_ == ControlSource::Landing)
      {
        handleLandingTimer();
        return;
      }

      if (control_source_ == ControlSource::Velocity)
      {
        handleVelocityTimer();
        return;
      }

      quadrotor_msgs::msg::PositionCommand active_cmd{};
      if (!getActiveCommand(active_cmd))
      {
        return;
      }

      publishOffboardSetpoint(active_cmd, true);
    }

    std::string command_topic_;
    std::string manual_pose_topic_;
    std::string nudge_topic_;
    std::string command_frame_id_;
    std::string offboard_mode_topic_;
    std::string trajectory_setpoint_topic_;
    std::string vehicle_command_topic_;
    std::string planner_odom_topic_;
    std::string px4_local_position_topic_;
    std::string vehicle_status_topic_;
    std::string restart_trajectory_topic_;
    std::string takeoff_service_;
    std::string hover_service_;
    std::string land_service_;
    std::string resume_auto_service_;
    std::string world_name_;
    std::string gz_clock_topic_;
    double publish_rate_hz_{50.0};
    int warmup_cycles_{20};
    uint8_t target_system_{1};
    uint8_t target_component_{1};
    uint8_t source_system_{1};
    uint16_t source_component_{1};
    float takeoff_height_m_{1.0f};
    bool nudge_body_frame_{true};
    bool enable_frame_alignment_{true};
    bool lock_alignment_on_first_valid_{true};
    bool relock_on_local_position_reset_{true};
    bool require_planner_takeoff_clearance_{true};
    float max_velocity_setpoint_mps_{1.5f};
    float max_acceleration_setpoint_mps2_{2.0f};
    bool is_armed_{false};
    bool is_offboard_mode_{false};
    bool is_auto_land_mode_{false};
    bool planner_restart_pending_{false};
    bool planner_startup_sequence_pending_{false};
    bool planner_takeoff_initialized_{false};
    bool has_planner_odom_{false};
    bool has_planner_yaw_{false};
    std::array<float, 3> planner_odom_enu_{0.0f, 0.0f, 0.0f};
    float planner_yaw_enu_{0.0f};
    FrameAligner frame_aligner_;
    std::array<float, 3> manual_target_position_enu_{0.0f, 0.0f, 0.0f};
    float manual_target_yaw_enu_{0.0f};
    float planner_takeoff_target_z_{0.0f};
    bool manual_target_valid_{false};
    ControlSource control_source_{ControlSource::Planner};
    int setpoint_counter_{0};
    int landing_retry_counter_{0};
    bool landing_command_sent_{false};
    bool has_auto_cmd_{false};

    std::string velocity_topic_;
    double velocity_timeout_ms_{200.0};
    geometry_msgs::msg::Twist last_velocity_cmd_{};
    uint64_t last_velocity_time_us_{0};
    bool has_velocity_cmd_{false};

    quadrotor_msgs::msg::PositionCommand last_auto_cmd_{};
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr auto_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nudge_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr planner_odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<Empty>::SharedPtr restart_trajectory_pub_;
    rclcpp::Service<Trigger>::SharedPtr takeoff_srv_;
    rclcpp::Service<Trigger>::SharedPtr hover_srv_;
    rclcpp::Service<Trigger>::SharedPtr land_srv_;
    rclcpp::Service<Trigger>::SharedPtr resume_auto_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    gz::transport::Node gz_node_;
    std::atomic<uint64_t> latest_gz_clock_us_{0};
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Clock px4_timestamp_system_clock_{RCL_SYSTEM_TIME};
    Px4TimestampSource px4_timestamp_source_{Px4TimestampSource::System};
  };
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uav_bridge::OffboardBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
