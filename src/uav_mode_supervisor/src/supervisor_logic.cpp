#include "uav_mode_supervisor/supervisor_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace uav_mode_supervisor
{

namespace
{

constexpr float kHeightEpsilonM = 1.0e-3f;

}  // namespace

SupervisorLogic::SupervisorLogic()
: SupervisorLogic(Config{})
{
}

SupervisorLogic::SupervisorLogic(const Config & config)
: config_(config),
  active_tracking_height_m_(config.default_tracking_height_m)
{
}

void SupervisorLogic::UpdateFusionStatus(const FusionStatus & status)
{
  fusion_status_ = status;
}

void SupervisorLogic::UpdateVisualLandingStatus(const VisualLandingStatus & status)
{
  visual_status_ = status;
  if (
    visual_status_.active && visual_status_.state_seen &&
    visual_status_.phase != "READY" && visual_status_.phase != "HOLD_WAIT")
  {
    visual_capture_observed_ = true;
  }

  if (!visual_status_.active && owner_ != Owner::VisualLanding) {
    visual_capture_observed_ = false;
  }
}

bool SupervisorLogic::FusionReady() const
{
  return fusion_status_.diagnostics_seen && fusion_status_.initialized &&
         !fusion_status_.relocalize_requested;
}

bool SupervisorLogic::VisualCommitted() const
{
  return visual_status_.active && PhaseInSet(visual_status_.phase, config_.visual_commit_phases);
}

SupervisorLogic::ActionPlan SupervisorLogic::BuildCommandPlan(
  const std::string & command,
  float requested_tracking_height_m) const
{
  ActionPlan plan;
  plan.command_name = NormalizeCommand(command);

  if (
    plan.command_name == "start_tracking" || plan.command_name == "tracking" ||
    plan.command_name == "relative_tracking")
  {
    if (!FusionReady()) {
      plan.message = "fusion is not ready for relative tracking";
      return plan;
    }

    const float resolved_height = ResolveTrackingHeight(requested_tracking_height_m);
    plan.accepted = true;
    plan.kind = PlanKind::ManualCommand;
    plan.start_tracking = true;
    plan.tracking_height_m = resolved_height;
    plan.owner_after = Owner::RelativeTracking;
    plan.message = "relative tracking requested";

    switch (owner_) {
      case Owner::Idle:
      case Owner::Hold:
        return plan;
      case Owner::RelativeTracking:
        if (std::abs(active_tracking_height_m_ - resolved_height) <= kHeightEpsilonM) {
          plan.start_tracking = false;
          plan.message = "relative tracking already active at requested height";
        } else {
          plan.stop_tracking = true;
          plan.message = "relative tracking restart requested with new height";
        }
        return plan;
      case Owner::VisualLanding:
        if (VisualCommitted()) {
          plan.accepted = false;
          plan.start_tracking = false;
          plan.message = "visual landing is in commit phase and cannot be preempted";
          return plan;
        }
        if (!config_.allow_tracking_preempt_visual_precommit) {
          plan.accepted = false;
          plan.start_tracking = false;
          plan.message = "tracking preemption of visual landing is disabled";
          return plan;
        }
        plan.stop_visual_landing = visual_status_.active;
        plan.message = "preempting visual landing with relative tracking";
        return plan;
    }
  }

  if (
    plan.command_name == "start_visual_landing" || plan.command_name == "visual_landing" ||
    plan.command_name == "landing")
  {
    plan.accepted = true;
    plan.kind = PlanKind::ManualCommand;
    plan.start_visual_landing = true;
    plan.owner_after = Owner::VisualLanding;
    plan.message = "visual landing requested";

    switch (owner_) {
      case Owner::Idle:
      case Owner::Hold:
        return plan;
      case Owner::RelativeTracking:
        if (!config_.allow_visual_preempt_tracking) {
          plan.accepted = false;
          plan.start_visual_landing = false;
          plan.message = "visual landing preemption of relative tracking is disabled";
          return plan;
        }
        plan.stop_tracking = true;
        plan.message = "preempting relative tracking with visual landing";
        return plan;
      case Owner::VisualLanding:
        plan.start_visual_landing = false;
        plan.message = "visual landing already active";
        return plan;
    }
  }

  if (plan.command_name == "hold") {
    plan.accepted = true;
    plan.kind = PlanKind::ManualCommand;
    plan.stop_tracking = owner_ == Owner::RelativeTracking;
    plan.stop_visual_landing = owner_ == Owner::VisualLanding && visual_status_.active;
    plan.request_hold = true;
    plan.owner_after = Owner::Hold;
    plan.message = "hold requested";
    return plan;
  }

  if (
    plan.command_name == "position" || plan.command_name == "position_mode" ||
    plan.command_name == "px4_position")
  {
    plan.accepted = true;
    plan.kind = PlanKind::ManualCommand;
    plan.request_position_mode = true;
    plan.owner_after = Owner::Idle;
    plan.message = "px4 position mode requested";

    switch (owner_) {
      case Owner::Idle:
      case Owner::Hold:
        return plan;
      case Owner::RelativeTracking:
        plan.stop_tracking = true;
        plan.message = "preempting relative tracking with px4 position mode";
        return plan;
      case Owner::VisualLanding:
        if (VisualCommitted()) {
          plan.accepted = false;
          plan.request_position_mode = false;
          plan.message = "visual landing is in commit phase and cannot be preempted";
          return plan;
        }
        plan.stop_visual_landing = visual_status_.active;
        plan.message = "preempting visual landing with px4 position mode";
        return plan;
    }
  }

  if (plan.command_name == "stop") {
    plan.accepted = true;
    plan.kind = PlanKind::ManualCommand;
    plan.stop_tracking = owner_ == Owner::RelativeTracking;
    plan.stop_visual_landing = owner_ == Owner::VisualLanding && visual_status_.active;
    plan.request_hold = true;
    plan.owner_after = Owner::Idle;
    plan.message = "stop requested";
    return plan;
  }

  plan.message = "unsupported supervisor command";
  return plan;
}

std::optional<SupervisorLogic::ActionPlan> SupervisorLogic::BuildAutomaticPlan() const
{
  if (
    config_.auto_hold_on_fusion_degrade && !FusionReady() &&
    (owner_ == Owner::RelativeTracking || owner_ == Owner::VisualLanding))
  {
    ActionPlan plan;
    plan.accepted = true;
    plan.kind = PlanKind::AutomaticSafety;
    plan.command_name = "auto_hold_fusion_degraded";
    plan.stop_tracking = owner_ == Owner::RelativeTracking;
    plan.stop_visual_landing = owner_ == Owner::VisualLanding && visual_status_.active;
    plan.request_hold = true;
    plan.owner_after = Owner::Hold;
    plan.message = "fusion degraded, falling back to hold";
    return plan;
  }

  if (
    owner_ == Owner::VisualLanding && config_.auto_recover_tracking_on_visual_loss &&
    FusionReady() && visual_capture_observed_ && !VisualCommitted() && visual_status_.active &&
    PhaseInSet(visual_status_.phase, config_.visual_recover_on_phases))
  {
    ActionPlan plan;
    plan.accepted = true;
    plan.kind = PlanKind::AutomaticRecovery;
    plan.command_name = "auto_recover_tracking";
    plan.stop_visual_landing = true;
    plan.start_tracking = true;
    plan.tracking_height_m = active_tracking_height_m_;
    plan.owner_after = Owner::RelativeTracking;
    plan.message = "visual landing lost target before commit, returning to tracking";
    return plan;
  }

  if (
    owner_ == Owner::VisualLanding && visual_status_.state_seen && !visual_status_.active &&
    visual_status_.phase == "READY")
  {
    ActionPlan plan;
    plan.accepted = true;
    plan.kind = PlanKind::PassiveStateChange;
    plan.command_name = "visual_landing_inactive";
    plan.owner_after = Owner::Idle;
    plan.message = "visual landing became inactive";
    return plan;
  }

  return std::nullopt;
}

void SupervisorLogic::ApplySuccessfulPlan(const ActionPlan & plan)
{
  owner_ = plan.owner_after;
  if (plan.start_tracking) {
    active_tracking_height_m_ = ResolveTrackingHeight(plan.tracking_height_m);
  }
  if (plan.start_visual_landing) {
    visual_capture_observed_ = false;
  } else if (owner_ != Owner::VisualLanding) {
    visual_capture_observed_ = false;
  }
  last_command_ = plan.command_name;
  last_message_ = plan.message;
}

void SupervisorLogic::RecordPlanFailure(const ActionPlan & plan, const std::string & error_message)
{
  last_command_ = plan.command_name;
  last_message_ = error_message;
}

const char * SupervisorLogic::OwnerName(Owner owner)
{
  switch (owner) {
    case Owner::Idle:
      return "idle";
    case Owner::RelativeTracking:
      return "relative_tracking";
    case Owner::VisualLanding:
      return "visual_landing";
    case Owner::Hold:
      return "hold";
  }
  return "idle";
}

bool SupervisorLogic::PhaseInSet(
  const std::string & phase,
  const std::vector<std::string> & phases) const
{
  return std::find(phases.begin(), phases.end(), phase) != phases.end();
}

float SupervisorLogic::ResolveTrackingHeight(float requested_tracking_height_m) const
{
  if (!std::isfinite(requested_tracking_height_m) || requested_tracking_height_m <= 0.0f) {
    return config_.default_tracking_height_m;
  }
  return requested_tracking_height_m;
}

std::string SupervisorLogic::NormalizeCommand(const std::string & command)
{
  std::string normalized = command;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char value) {return static_cast<char>(std::tolower(value));});
  return normalized;
}

}  // namespace uav_mode_supervisor
