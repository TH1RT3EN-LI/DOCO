#pragma once

#include "relative_position_fusion/agent_planar_state.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <limits>
#include <string>

namespace relative_position_fusion
{

struct FusionDiagnostics
{
  bool initialized{false};
  bool measurement_available{false};
  bool measurement_used{false};
  bool gate_rejected{false};
  bool relocalize_requested{true};

  int consecutive_gate_rejects{0};

  double rho_m{0.0};
  double lambda_max_m2{0.0};
  double mahalanobis_d2{0.0};
  double measurement_pair_dt_s{std::numeric_limits<double>::quiet_NaN()};

  std::string filter_mode{"uninitialized"};
  std::string relocalization_reason{"uninitialized"};
  std::string measurement_phase{"bootstrap"};
  std::string measurement_pairing_mode{"nearest_buffer_pair"};

  AgentPlanarState uav_state;
  AgentPlanarState ugv_state;
};

class DiagnosticsPublisher
{
public:
  explicit DiagnosticsPublisher(
    rclcpp::Logger logger,
    std::string status_name = "relative_position_fusion",
    std::string hardware_id = "relative_position_fusion");

  diagnostic_msgs::msg::DiagnosticArray Build(
    const rclcpp::Time & stamp,
    const FusionDiagnostics & diagnostics) const;

private:
  rclcpp::Logger logger_;
  std::string status_name_;
  std::string hardware_id_;
};

}  // namespace relative_position_fusion
