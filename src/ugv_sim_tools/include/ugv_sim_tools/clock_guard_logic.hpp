#pragma once

#include <cstdint>
#include <optional>

namespace ugv_sim_tools
{

struct ClockGuardConfig
{
  bool allow_clock_reset{true};
  int64_t reset_newer_than_ns{30LL * 1000LL * 1000LL * 1000LL};
  int64_t reset_older_than_ns{5LL * 1000LL * 1000LL * 1000LL};
};

enum class ClockGuardAction
{
  kForward,
  kForwardReset,
  kDrop,
};

struct ClockGuardDecision
{
  ClockGuardAction action{ClockGuardAction::kForward};
  std::optional<int64_t> next_last_ns;
};

ClockGuardDecision evaluateClockGuard(
  std::optional<int64_t> last_ns,
  int64_t now_ns,
  const ClockGuardConfig & config);

}
