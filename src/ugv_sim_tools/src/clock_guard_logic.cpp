#include "ugv_sim_tools/clock_guard_logic.hpp"

namespace ugv_sim_tools
{

ClockGuardDecision evaluateClockGuard(
  std::optional<int64_t> last_ns,
  int64_t now_ns,
  const ClockGuardConfig & config)
{
  if (!last_ns.has_value() || now_ns >= *last_ns) {
    return ClockGuardDecision{ClockGuardAction::kForward, now_ns};
  }

  const bool allow_epoch_reset =
    config.allow_clock_reset &&
    *last_ns >= config.reset_newer_than_ns &&
    now_ns <= config.reset_older_than_ns;

  if (allow_epoch_reset) {
    return ClockGuardDecision{ClockGuardAction::kForwardReset, now_ns};
  }

  return ClockGuardDecision{ClockGuardAction::kDrop, last_ns};
}

}
