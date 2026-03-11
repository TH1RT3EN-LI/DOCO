#pragma once

#include <string>

namespace relative_position_fusion
{

struct RelocalizationMonitorConfig
{
  double enter_threshold_m{1.0};
  double exit_threshold_m{0.7};
  int enter_hold_cycles{5};
  int exit_hold_cycles{10};
  int max_consecutive_gate_rejects_before_force_request{10};
};

struct RelocalizationInput
{
  bool initialized{false};
  bool critical_inputs_ready{false};
  double rho_m{0.0};
  int consecutive_gate_rejects{0};
};

struct RelocalizationDecision
{
  bool requested{true};
  std::string reason{"uninitialized"};
  int enter_counter{0};
  int exit_counter{0};
  int degraded_counter{0};
};

class RelocalizationMonitor
{
public:
  explicit RelocalizationMonitor(const RelocalizationMonitorConfig & config = {});

  void Reset();

  RelocalizationDecision Update(const RelocalizationInput & input);

private:
  RelocalizationMonitorConfig config_{};
  bool requested_{true};
  int enter_counter_{0};
  int exit_counter_{0};
  int degraded_counter_{0};
};

}  // namespace relative_position_fusion
