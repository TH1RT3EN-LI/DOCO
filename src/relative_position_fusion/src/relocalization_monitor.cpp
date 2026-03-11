#include "relative_position_fusion/relocalization_monitor.hpp"

#include <cmath>

namespace relative_position_fusion
{

RelocalizationMonitor::RelocalizationMonitor(const RelocalizationMonitorConfig & config)
: config_(config)
{
}

void RelocalizationMonitor::Reset()
{
  requested_ = true;
  enter_counter_ = 0;
  exit_counter_ = 0;
  degraded_counter_ = 0;
}

RelocalizationDecision RelocalizationMonitor::Update(const RelocalizationInput & input)
{
  RelocalizationDecision decision;

  if (!input.initialized) {
    requested_ = true;
    enter_counter_ = 0;
    exit_counter_ = 0;
    degraded_counter_ = 0;
    decision.requested = true;
    decision.reason = "uninitialized";
    return decision;
  }

  if (!input.critical_inputs_ready) {
    degraded_counter_ += 1;
    enter_counter_ = 0;
    exit_counter_ = 0;
    if (degraded_counter_ >= config_.enter_hold_cycles) {
      requested_ = true;
      decision.reason = "inputs_missing";
    } else {
      decision.reason = "inputs_missing_pending";
    }
    decision.requested = requested_;
    decision.degraded_counter = degraded_counter_;
    return decision;
  }

  degraded_counter_ = 0;

  if (!std::isfinite(input.rho_m)) {
    requested_ = true;
    enter_counter_ = 0;
    exit_counter_ = 0;
    decision.requested = true;
    decision.reason = "rho_invalid";
    return decision;
  }

  if (input.consecutive_gate_rejects >=
    config_.max_consecutive_gate_rejects_before_force_request)
  {
    requested_ = true;
    enter_counter_ = config_.enter_hold_cycles;
    exit_counter_ = 0;
    decision.requested = true;
    decision.reason = "gate_rejects";
    decision.enter_counter = enter_counter_;
    return decision;
  }

  if (requested_) {
    if (input.rho_m < config_.exit_threshold_m) {
      exit_counter_ += 1;
      if (exit_counter_ >= config_.exit_hold_cycles) {
        requested_ = false;
        enter_counter_ = 0;
        exit_counter_ = 0;
        decision.reason = "ok";
      } else {
        decision.reason = "recovery_pending";
      }
    } else {
      exit_counter_ = 0;
      decision.reason = "rho_high";
    }
  } else {
    if (input.rho_m > config_.enter_threshold_m) {
      enter_counter_ += 1;
      if (enter_counter_ >= config_.enter_hold_cycles) {
        requested_ = true;
        enter_counter_ = config_.enter_hold_cycles;
        exit_counter_ = 0;
        decision.reason = "rho_high";
      } else {
        decision.reason = "rho_high_pending";
      }
    } else {
      enter_counter_ = 0;
      decision.reason = "ok";
    }
  }

  decision.requested = requested_;
  decision.enter_counter = enter_counter_;
  decision.exit_counter = exit_counter_;
  decision.degraded_counter = degraded_counter_;
  return decision;
}

}  // namespace relative_position_fusion
