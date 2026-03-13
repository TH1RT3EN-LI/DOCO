#include "uav_mode_supervisor/supervisor_logic.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <relative_position_fusion/srv/start_relative_tracking.hpp>
#include <uav_mode_supervisor/msg/supervisor_state.hpp>
#include <uav_mode_supervisor/srv/command_supervisor.hpp>
#include <uav_visual_landing/msg/landing_controller_state.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace uav_mode_supervisor
{

class UavModeSupervisorNode : public rclcpp::Node
{
  using CommandSupervisor = uav_mode_supervisor::srv::CommandSupervisor;
  using StartRelativeTracking = relative_position_fusion::srv::StartRelativeTracking;
  using SupervisorState = uav_mode_supervisor::msg::SupervisorState;
  using Trigger = std_srvs::srv::Trigger;

  enum class ActionType
  {
    StopTracking,
    StartTracking,
    StopVisualLanding,
    StartVisualLanding,
    PositionMode,
    Hold,
  };

  struct ActionStep
  {
    ActionType type;
    bool required{true};
    float tracking_height_m{0.0f};
  };

  struct PendingPlan
  {
    SupervisorLogic::ActionPlan plan;
    std::vector<ActionStep> actions;
    std::size_t next_action_index{0};
    bool in_flight{false};
  };

public:
  UavModeSupervisorNode()
  : Node("uav_mode_supervisor")
  {
    const auto config = LoadConfig();
    logic_ = std::make_unique<SupervisorLogic>(config);

    command_service_name_ = this->get_parameter("command_service").as_string();
    track_alias_service_name_ = this->get_parameter("track_alias_service").as_string();
    land_alias_service_name_ = this->get_parameter("land_alias_service").as_string();
    position_alias_service_name_ = this->get_parameter("position_alias_service").as_string();
    hold_alias_service_name_ = this->get_parameter("hold_alias_service").as_string();
    stop_alias_service_name_ = this->get_parameter("stop_alias_service").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    diagnostics_topic_ = this->get_parameter("diagnostics_topic").as_string();
    relocalize_requested_topic_ = this->get_parameter("relocalize_requested_topic").as_string();
    visual_landing_state_topic_ = this->get_parameter("visual_landing_state_topic").as_string();
    tracking_start_service_name_ = this->get_parameter("tracking_start_service").as_string();
    tracking_stop_service_name_ = this->get_parameter("tracking_stop_service").as_string();
    visual_landing_start_service_name_ =
      this->get_parameter("visual_landing_start_service").as_string();
    visual_landing_stop_service_name_ =
      this->get_parameter("visual_landing_stop_service").as_string();
    position_mode_service_name_ = this->get_parameter("position_mode_service").as_string();
    hold_service_name_ = this->get_parameter("hold_service").as_string();
    service_wait_timeout_s_ = this->get_parameter("service_wait_timeout_s").as_double();
    automatic_action_retry_s_ = this->get_parameter("automatic_action_retry_s").as_double();
    tick_rate_hz_ = std::max(1.0, this->get_parameter("tick_rate_hz").as_double());

    state_pub_ = this->create_publisher<SupervisorState>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10,
      std::bind(&UavModeSupervisorNode::OnDiagnostics, this, std::placeholders::_1));
    relocalize_requested_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      relocalize_requested_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UavModeSupervisorNode::OnRelocalizeRequested, this, std::placeholders::_1));
    visual_landing_state_sub_ =
      this->create_subscription<uav_visual_landing::msg::LandingControllerState>(
      visual_landing_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&UavModeSupervisorNode::OnVisualLandingState, this, std::placeholders::_1));

    tracking_start_client_ = this->create_client<StartRelativeTracking>(tracking_start_service_name_);
    tracking_stop_client_ = this->create_client<Trigger>(tracking_stop_service_name_);
    visual_landing_start_client_ = this->create_client<Trigger>(visual_landing_start_service_name_);
    visual_landing_stop_client_ = this->create_client<Trigger>(visual_landing_stop_service_name_);
    position_mode_client_ = this->create_client<Trigger>(position_mode_service_name_);
    hold_client_ = this->create_client<Trigger>(hold_service_name_);

    command_srv_ = this->create_service<CommandSupervisor>(
      command_service_name_,
      std::bind(
        &UavModeSupervisorNode::OnCommand, this, std::placeholders::_1,
        std::placeholders::_2));
    track_alias_srv_ = this->create_service<Trigger>(
      track_alias_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        response->success = AcceptCommand(
          "start_tracking", std::numeric_limits<float>::quiet_NaN(), response->message);
      });
    land_alias_srv_ = this->create_service<Trigger>(
      land_alias_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        response->success = AcceptCommand(
          "start_visual_landing", std::numeric_limits<float>::quiet_NaN(), response->message);
      });
    position_alias_srv_ = this->create_service<Trigger>(
      position_alias_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        response->success = AcceptCommand(
          "position", std::numeric_limits<float>::quiet_NaN(), response->message);
      });
    hold_alias_srv_ = this->create_service<Trigger>(
      hold_alias_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        response->success = AcceptCommand(
          "hold", std::numeric_limits<float>::quiet_NaN(), response->message);
      });
    stop_alias_srv_ = this->create_service<Trigger>(
      stop_alias_service_name_,
      [this](
        const std::shared_ptr<Trigger::Request> request,
        std::shared_ptr<Trigger::Response> response)
      {
        (void)request;
        response->success = AcceptCommand(
          "stop", std::numeric_limits<float>::quiet_NaN(), response->message);
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tick_rate_hz_));
    timer_ = this->create_wall_timer(period, std::bind(&UavModeSupervisorNode::OnTimer, this));

    PublishState();
    RCLCPP_INFO(
      this->get_logger(),
      "uav_mode_supervisor: command=%s track=%s land=%s position=%s hold=%s stop=%s state=%s fusion_diag=%s visual_state=%s",
      command_service_name_.c_str(), track_alias_service_name_.c_str(),
      land_alias_service_name_.c_str(), position_alias_service_name_.c_str(),
      hold_alias_service_name_.c_str(),
      stop_alias_service_name_.c_str(), state_topic_.c_str(), diagnostics_topic_.c_str(),
      visual_landing_state_topic_.c_str());
  }

private:
  SupervisorLogic::Config LoadConfig()
  {
    this->declare_parameter<std::string>("command_service", "/uav/mode_supervisor/command");
    this->declare_parameter<std::string>("track_alias_service", "/uav/mode/track");
    this->declare_parameter<std::string>("land_alias_service", "/uav/mode/land");
    this->declare_parameter<std::string>("position_alias_service", "/uav/mode/position");
    this->declare_parameter<std::string>("hold_alias_service", "/uav/mode/hold");
    this->declare_parameter<std::string>("stop_alias_service", "/uav/mode/stop");
    this->declare_parameter<std::string>("state_topic", "/uav/mode_supervisor/state");
    this->declare_parameter<std::string>("diagnostics_topic", "/relative_position/diagnostics");
    this->declare_parameter<std::string>(
      "relocalize_requested_topic", "/relative_position/relocalize_requested");
    this->declare_parameter<std::string>(
      "visual_landing_state_topic", "/uav/visual_landing/controller_state");
    this->declare_parameter<std::string>(
      "tracking_start_service", "/uav/relative_tracking/command/start");
    this->declare_parameter<std::string>(
      "tracking_stop_service", "/uav/relative_tracking/command/stop");
    this->declare_parameter<std::string>(
      "visual_landing_start_service", "/uav/visual_landing/command/start");
    this->declare_parameter<std::string>(
      "visual_landing_stop_service", "/uav/visual_landing/command/stop");
    this->declare_parameter<std::string>("position_mode_service", "/uav/control/command/position_mode");
    this->declare_parameter<std::string>("hold_service", "/uav/control/command/hold");
    this->declare_parameter<double>("service_wait_timeout_s", 1.0);
    this->declare_parameter<double>("automatic_action_retry_s", 1.0);
    this->declare_parameter<double>("tick_rate_hz", 10.0);
    this->declare_parameter<double>("default_tracking_height_m", 1.5);
    this->declare_parameter<bool>("allow_visual_preempt_tracking", true);
    this->declare_parameter<bool>("allow_tracking_preempt_visual_precommit", true);
    this->declare_parameter<bool>("auto_recover_tracking_on_visual_loss", true);
    this->declare_parameter<bool>("auto_hold_on_fusion_degrade", true);
    this->declare_parameter<std::vector<std::string>>(
      "visual_landing_commit_phases", std::vector<std::string>{"DESCEND_TRACK", "TERMINAL", "LAND"});
    this->declare_parameter<std::vector<std::string>>(
      "visual_landing_recover_on_phases", std::vector<std::string>{"HOLD_WAIT"});

    SupervisorLogic::Config config;
    config.default_tracking_height_m =
      static_cast<float>(this->get_parameter("default_tracking_height_m").as_double());
    config.allow_visual_preempt_tracking =
      this->get_parameter("allow_visual_preempt_tracking").as_bool();
    config.allow_tracking_preempt_visual_precommit =
      this->get_parameter("allow_tracking_preempt_visual_precommit").as_bool();
    config.auto_recover_tracking_on_visual_loss =
      this->get_parameter("auto_recover_tracking_on_visual_loss").as_bool();
    config.auto_hold_on_fusion_degrade =
      this->get_parameter("auto_hold_on_fusion_degrade").as_bool();
    config.visual_commit_phases =
      this->get_parameter("visual_landing_commit_phases").as_string_array();
    config.visual_recover_on_phases =
      this->get_parameter("visual_landing_recover_on_phases").as_string_array();
    return config;
  }

  void OnDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    SupervisorLogic::FusionStatus status = logic_->fusion_status();
    status.diagnostics_seen = !msg->status.empty();
    status.initialized = false;
    status.relocalize_requested = fallback_relocalize_requested_.value_or(true);
    status.reason = msg->status.empty() ? "diagnostics_missing" : msg->status.front().message;

    if (const auto initialized = FindDiagnosticValue(*msg, "initialized")) {
      status.initialized = ParseBool(*initialized, false);
    }
    if (const auto relocalize_requested = FindDiagnosticValue(*msg, "relocalize_requested")) {
      status.relocalize_requested = ParseBool(*relocalize_requested, true);
    }
    if (const auto reason = FindDiagnosticValue(*msg, "relocalization_reason")) {
      status.reason = *reason;
    }

    logic_->UpdateFusionStatus(status);
  }

  void OnRelocalizeRequested(const std_msgs::msg::Bool::SharedPtr msg)
  {
    fallback_relocalize_requested_ = msg->data;
    if (!logic_->fusion_status().diagnostics_seen) {
      auto status = logic_->fusion_status();
      status.relocalize_requested = msg->data;
      logic_->UpdateFusionStatus(status);
    }
  }

  void OnVisualLandingState(
    const uav_visual_landing::msg::LandingControllerState::SharedPtr msg)
  {
    SupervisorLogic::VisualLandingStatus status;
    status.state_seen = true;
    status.active = msg->active;
    status.target_detected = msg->target_detected;
    status.phase = msg->phase;
    logic_->UpdateVisualLandingStatus(status);
  }

  void OnCommand(
    const std::shared_ptr<CommandSupervisor::Request> request,
    std::shared_ptr<CommandSupervisor::Response> response)
  {
    response->success = AcceptCommand(
      request->command, request->tracking_target_height_m, response->message);
  }

  bool AcceptCommand(
    const std::string & command,
    float tracking_target_height_m,
    std::string & response_message)
  {
    if (pending_plan_.has_value()) {
      response_message = "another supervisor command is already in progress";
      return false;
    }

    const auto plan = logic_->BuildCommandPlan(command, tracking_target_height_m);
    if (!plan.accepted) {
      response_message = plan.message;
      logic_->RecordPlanFailure(plan, plan.message);
      PublishState();
      return false;
    }

    last_automatic_action_key_.clear();
    response_message = "command accepted: " + plan.message;
    QueuePlan(plan);
    return true;
  }

  void OnTimer()
  {
    if (!pending_plan_.has_value()) {
      MaybeQueueAutomaticPlan();
    }
    PublishState();
  }

  void MaybeQueueAutomaticPlan()
  {
    const auto automatic_plan = logic_->BuildAutomaticPlan();
    if (!automatic_plan.has_value()) {
      last_automatic_action_key_.clear();
      return;
    }

    const std::string action_key = automatic_plan->command_name;
    const rclcpp::Time now = this->now();
    if (
      action_key == last_automatic_action_key_ &&
      (now - last_automatic_action_time_).seconds() < automatic_action_retry_s_)
    {
      return;
    }

    last_automatic_action_key_ = action_key;
    last_automatic_action_time_ = now;
    QueuePlan(*automatic_plan);
  }

  void QueuePlan(const SupervisorLogic::ActionPlan & plan)
  {
    PendingPlan pending_plan;
    pending_plan.plan = plan;
    pending_plan.actions = BuildActions(plan);
    pending_plan.next_action_index = 0;
    pending_plan.in_flight = false;
    pending_plan_ = std::move(pending_plan);

    if (pending_plan_->actions.empty()) {
      CompletePendingPlan();
      return;
    }

    StartNextAction();
  }

  std::vector<ActionStep> BuildActions(const SupervisorLogic::ActionPlan & plan) const
  {
    const bool stop_steps_required =
      !plan.request_hold || plan.start_tracking || plan.start_visual_landing ||
      plan.request_position_mode;

    std::vector<ActionStep> actions;
    if (plan.stop_tracking) {
      actions.push_back(ActionStep{ActionType::StopTracking, stop_steps_required, 0.0f});
    }
    if (plan.stop_visual_landing) {
      actions.push_back(ActionStep{ActionType::StopVisualLanding, stop_steps_required, 0.0f});
    }
    if (plan.request_hold) {
      actions.push_back(ActionStep{ActionType::Hold, true, 0.0f});
    }
    if (plan.request_position_mode) {
      actions.push_back(ActionStep{ActionType::PositionMode, true, 0.0f});
    }
    if (plan.start_tracking) {
      actions.push_back(ActionStep{ActionType::StartTracking, true, plan.tracking_height_m});
    }
    if (plan.start_visual_landing) {
      actions.push_back(ActionStep{ActionType::StartVisualLanding, true, 0.0f});
    }
    return actions;
  }

  void StartNextAction()
  {
    if (!pending_plan_.has_value() || pending_plan_->in_flight) {
      return;
    }
    if (pending_plan_->next_action_index >= pending_plan_->actions.size()) {
      CompletePendingPlan();
      return;
    }

    pending_plan_->in_flight = true;
    const ActionStep step = pending_plan_->actions[pending_plan_->next_action_index];
    switch (step.type) {
      case ActionType::StopTracking:
        DispatchTriggerAction(
          tracking_stop_client_, tracking_stop_service_name_, "stop relative tracking");
        return;
      case ActionType::StartTracking:
        DispatchTrackingStart(step.tracking_height_m);
        return;
      case ActionType::StopVisualLanding:
        DispatchTriggerAction(
          visual_landing_stop_client_, visual_landing_stop_service_name_, "stop visual landing");
        return;
      case ActionType::StartVisualLanding:
        DispatchTriggerAction(
          visual_landing_start_client_, visual_landing_start_service_name_, "start visual landing");
        return;
      case ActionType::PositionMode:
        DispatchTriggerAction(
          position_mode_client_, position_mode_service_name_, "switch px4 position mode");
        return;
      case ActionType::Hold:
        DispatchTriggerAction(hold_client_, hold_service_name_, "request hold");
        return;
    }
  }

  void CompletePendingPlan()
  {
    if (!pending_plan_.has_value()) {
      return;
    }

    const auto previous_owner = logic_->owner();
    const auto plan = pending_plan_->plan;
    pending_plan_.reset();
    logic_->ApplySuccessfulPlan(plan);
    if (previous_owner != logic_->owner()) {
      RCLCPP_INFO(
        this->get_logger(), "control owner -> %s (%s)",
        SupervisorLogic::OwnerName(logic_->owner()), plan.message.c_str());
    }
    PublishState();
  }

  void FailPendingPlan(const std::string & error_message)
  {
    if (!pending_plan_.has_value()) {
      return;
    }

    const auto plan = pending_plan_->plan;
    pending_plan_.reset();
    logic_->RecordPlanFailure(plan, error_message);
    RCLCPP_WARN(this->get_logger(), "%s", error_message.c_str());
    PublishState();
  }

  void OnActionFinished(bool success, const std::string & message)
  {
    if (!pending_plan_.has_value()) {
      return;
    }

    const ActionStep step = pending_plan_->actions[pending_plan_->next_action_index];
    pending_plan_->in_flight = false;

    if (!success && step.required) {
      FailPendingPlan(ActionTypeName(step.type) + std::string(" failed: ") + message);
      return;
    }

    if (!success) {
      RCLCPP_WARN(
        this->get_logger(), "%s failed but continuing: %s",
        ActionTypeName(step.type).c_str(), message.c_str());
    }

    ++pending_plan_->next_action_index;
    StartNextAction();
  }

  void DispatchTrackingStart(float tracking_height_m)
  {
    if (!tracking_start_client_->wait_for_service(std::chrono::duration<double>(service_wait_timeout_s_))) {
      OnActionFinished(false, "relative tracking start service not ready");
      return;
    }

    auto request = std::make_shared<StartRelativeTracking::Request>();
    request->target_height_m = tracking_height_m;
    tracking_start_client_->async_send_request(
      request,
      [this](rclcpp::Client<StartRelativeTracking>::SharedFuture future)
      {
        try {
          const auto response = future.get();
          OnActionFinished(response->success, response->message);
        } catch (const std::exception & error) {
          OnActionFinished(false, error.what());
        }
      });
  }

  void DispatchTriggerAction(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & service_name,
    const std::string & action_name)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(service_wait_timeout_s_))) {
      OnActionFinished(false, action_name + " service not ready: " + service_name);
      return;
    }

    auto request = std::make_shared<Trigger::Request>();
    client->async_send_request(
      request,
      [this](rclcpp::Client<Trigger>::SharedFuture future)
      {
        try {
          const auto response = future.get();
          OnActionFinished(response->success, response->message);
        } catch (const std::exception & error) {
          OnActionFinished(false, error.what());
        }
      });
  }

  void PublishState()
  {
    SupervisorState msg;
    msg.header.stamp = this->now();
    msg.owner = SupervisorLogic::OwnerName(logic_->owner());
    msg.active_tracking_height_m = logic_->active_tracking_height_m();
    msg.last_command = logic_->last_command();
    msg.pending_command = pending_plan_.has_value() ? pending_plan_->plan.command_name : "";
    msg.command_in_progress = pending_plan_.has_value();
    msg.last_message = logic_->last_message();
    msg.fusion_diagnostics_seen = logic_->fusion_status().diagnostics_seen;
    msg.fusion_initialized = logic_->fusion_status().initialized;
    msg.fusion_relocalize_requested = logic_->fusion_status().relocalize_requested;
    msg.fusion_ready = logic_->FusionReady();
    msg.fusion_reason = logic_->fusion_status().reason;
    msg.visual_state_seen = logic_->visual_status().state_seen;
    msg.visual_active = logic_->visual_status().active;
    msg.visual_target_detected = logic_->visual_status().target_detected;
    msg.visual_phase = logic_->visual_status().phase;
    msg.visual_committed = logic_->VisualCommitted();
    msg.visual_capture_observed = logic_->visual_capture_observed();
    state_pub_->publish(msg);
  }

  static std::optional<std::string> FindDiagnosticValue(
    const diagnostic_msgs::msg::DiagnosticArray & array,
    const std::string & key_name)
  {
    for (const auto & status : array.status) {
      for (const auto & key_value : status.values) {
        if (key_value.key == key_name) {
          return key_value.value;
        }
      }
    }
    return std::nullopt;
  }

  static bool ParseBool(const std::string & text, bool default_value)
  {
    std::string normalized = text;
    std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char value) {return static_cast<char>(std::tolower(value));});
    if (normalized == "true" || normalized == "1") {
      return true;
    }
    if (normalized == "false" || normalized == "0") {
      return false;
    }
    return default_value;
  }

  static std::string ActionTypeName(ActionType type)
  {
    switch (type) {
      case ActionType::StopTracking:
        return "stop_tracking";
      case ActionType::StartTracking:
        return "start_tracking";
      case ActionType::StopVisualLanding:
        return "stop_visual_landing";
      case ActionType::StartVisualLanding:
        return "start_visual_landing";
      case ActionType::PositionMode:
        return "position_mode";
      case ActionType::Hold:
        return "hold";
    }
    return "unknown";
  }

  std::unique_ptr<SupervisorLogic> logic_;

  std::string command_service_name_;
  std::string track_alias_service_name_;
  std::string land_alias_service_name_;
  std::string position_alias_service_name_;
  std::string hold_alias_service_name_;
  std::string stop_alias_service_name_;
  std::string state_topic_;
  std::string diagnostics_topic_;
  std::string relocalize_requested_topic_;
  std::string visual_landing_state_topic_;
  std::string tracking_start_service_name_;
  std::string tracking_stop_service_name_;
  std::string visual_landing_start_service_name_;
  std::string visual_landing_stop_service_name_;
  std::string position_mode_service_name_;
  std::string hold_service_name_;
  double service_wait_timeout_s_{1.0};
  double automatic_action_retry_s_{1.0};
  double tick_rate_hz_{10.0};

  std::optional<bool> fallback_relocalize_requested_{};
  std::optional<PendingPlan> pending_plan_{};
  std::string last_automatic_action_key_;
  rclcpp::Time last_automatic_action_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<SupervisorState>::SharedPtr state_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr relocalize_requested_sub_;
  rclcpp::Subscription<uav_visual_landing::msg::LandingControllerState>::SharedPtr
    visual_landing_state_sub_;
  rclcpp::Service<CommandSupervisor>::SharedPtr command_srv_;
  rclcpp::Service<Trigger>::SharedPtr track_alias_srv_;
  rclcpp::Service<Trigger>::SharedPtr land_alias_srv_;
  rclcpp::Service<Trigger>::SharedPtr position_alias_srv_;
  rclcpp::Service<Trigger>::SharedPtr hold_alias_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_alias_srv_;
  rclcpp::Client<StartRelativeTracking>::SharedPtr tracking_start_client_;
  rclcpp::Client<Trigger>::SharedPtr tracking_stop_client_;
  rclcpp::Client<Trigger>::SharedPtr visual_landing_start_client_;
  rclcpp::Client<Trigger>::SharedPtr visual_landing_stop_client_;
  rclcpp::Client<Trigger>::SharedPtr position_mode_client_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_mode_supervisor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_mode_supervisor::UavModeSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
