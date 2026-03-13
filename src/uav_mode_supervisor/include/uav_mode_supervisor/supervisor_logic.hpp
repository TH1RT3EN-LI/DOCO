#pragma once

#include <optional>
#include <string>
#include <vector>

namespace uav_mode_supervisor
{

class SupervisorLogic
{
public:
  enum class Owner
  {
    Idle,
    RelativeTracking,
    VisualLanding,
    Hold,
  };

  enum class PlanKind
  {
    ManualCommand,
    AutomaticRecovery,
    AutomaticSafety,
    PassiveStateChange,
  };

  struct Config
  {
    float default_tracking_height_m{1.5f};
    bool allow_visual_preempt_tracking{true};
    bool allow_tracking_preempt_visual_precommit{true};
    bool auto_recover_tracking_on_visual_loss{true};
    bool auto_hold_on_fusion_degrade{true};
    std::vector<std::string> visual_commit_phases{"DESCEND_TRACK", "TERMINAL", "LAND"};
    std::vector<std::string> visual_recover_on_phases{"HOLD_WAIT"};
  };

  struct FusionStatus
  {
    bool diagnostics_seen{false};
    bool initialized{false};
    bool relocalize_requested{true};
    std::string reason{"diagnostics_missing"};
  };

  struct VisualLandingStatus
  {
    bool state_seen{false};
    bool active{false};
    bool target_detected{false};
    std::string phase{"READY"};
  };

  struct ActionPlan
  {
    bool accepted{false};
    PlanKind kind{PlanKind::ManualCommand};
    std::string command_name;
    bool stop_tracking{false};
    bool start_tracking{false};
    float tracking_height_m{0.0f};
    bool stop_visual_landing{false};
    bool start_visual_landing{false};
    bool request_position_mode{false};
    bool request_hold{false};
    Owner owner_after{Owner::Idle};
    std::string message;
  };

  SupervisorLogic();
  explicit SupervisorLogic(const Config & config);

  void UpdateFusionStatus(const FusionStatus & status);
  void UpdateVisualLandingStatus(const VisualLandingStatus & status);

  ActionPlan BuildCommandPlan(
    const std::string & command,
    float requested_tracking_height_m) const;

  std::optional<ActionPlan> BuildAutomaticPlan() const;

  void ApplySuccessfulPlan(const ActionPlan & plan);
  void RecordPlanFailure(const ActionPlan & plan, const std::string & error_message);

  Owner owner() const { return owner_; }
  const FusionStatus & fusion_status() const { return fusion_status_; }
  const VisualLandingStatus & visual_status() const { return visual_status_; }
  float active_tracking_height_m() const { return active_tracking_height_m_; }
  bool visual_capture_observed() const { return visual_capture_observed_; }
  bool FusionReady() const;
  bool VisualCommitted() const;
  const std::string & last_command() const { return last_command_; }
  const std::string & last_message() const { return last_message_; }

  static const char * OwnerName(Owner owner);

private:
  bool PhaseInSet(const std::string & phase, const std::vector<std::string> & phases) const;
  float ResolveTrackingHeight(float requested_tracking_height_m) const;
  static std::string NormalizeCommand(const std::string & command);

  Config config_;
  Owner owner_{Owner::Idle};
  FusionStatus fusion_status_{};
  VisualLandingStatus visual_status_{};
  float active_tracking_height_m_{1.5f};
  bool visual_capture_observed_{false};
  std::string last_command_{"idle"};
  std::string last_message_{"idle"};
};

}  // namespace uav_mode_supervisor
