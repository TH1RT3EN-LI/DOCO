#include "relative_position_fusion/diagnostics_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cmath>
#include <string>
#include <utility>

namespace relative_position_fusion
{

namespace
{

diagnostic_msgs::msg::KeyValue MakeKeyValue(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = std::move(key);
  key_value.value = std::move(value);
  return key_value;
}

std::string BoolString(bool value)
{
  return value ? "true" : "false";
}

std::string NumberString(double value)
{
  if (!std::isfinite(value)) {
    return "nan";
  }
  return std::to_string(value);
}

}  // namespace

DiagnosticsPublisher::DiagnosticsPublisher(
  rclcpp::Logger logger,
  std::string status_name,
  std::string hardware_id)
: logger_(std::move(logger)),
  status_name_(std::move(status_name)),
  hardware_id_(std::move(hardware_id))
{
}

diagnostic_msgs::msg::DiagnosticArray DiagnosticsPublisher::Build(
  const rclcpp::Time & stamp,
  const FusionDiagnostics & diagnostics) const
{
  (void)logger_;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = status_name_;
  status.hardware_id = hardware_id_;

  if (!diagnostics.initialized || diagnostics.relocalize_requested) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = diagnostics.relocalization_reason;
  } else if (diagnostics.gate_rejected) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "measurement rejected by gate";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "relative position fusion healthy";
  }

  status.values.reserve(24);
  status.values.push_back(MakeKeyValue("initialized", BoolString(diagnostics.initialized)));
  status.values.push_back(MakeKeyValue("mode", diagnostics.filter_mode));
  status.values.push_back(MakeKeyValue("relocalize_requested", BoolString(diagnostics.relocalize_requested)));
  status.values.push_back(MakeKeyValue("relocalization_reason", diagnostics.relocalization_reason));
  status.values.push_back(MakeKeyValue("measurement_available", BoolString(diagnostics.measurement_available)));
  status.values.push_back(MakeKeyValue("measurement_used", BoolString(diagnostics.measurement_used)));
  status.values.push_back(MakeKeyValue("gate_rejected", BoolString(diagnostics.gate_rejected)));
  status.values.push_back(MakeKeyValue("consecutive_gate_rejects", std::to_string(diagnostics.consecutive_gate_rejects)));
  status.values.push_back(MakeKeyValue("rho_m", NumberString(diagnostics.rho_m)));
  status.values.push_back(MakeKeyValue("lambda_max_m2", NumberString(diagnostics.lambda_max_m2)));
  status.values.push_back(MakeKeyValue("mahalanobis_d2", NumberString(diagnostics.mahalanobis_d2)));

  status.values.push_back(MakeKeyValue("uav_pose_age_s", NumberString(diagnostics.uav_state.pose_age_sec)));
  status.values.push_back(MakeKeyValue("uav_velocity_age_s", NumberString(diagnostics.uav_state.velocity_age_sec)));
  status.values.push_back(MakeKeyValue("ugv_pose_age_s", NumberString(diagnostics.ugv_state.pose_age_sec)));
  status.values.push_back(MakeKeyValue("ugv_velocity_age_s", NumberString(diagnostics.ugv_state.velocity_age_sec)));
  status.values.push_back(MakeKeyValue("uav_pose_source", diagnostics.uav_state.pose_source));
  status.values.push_back(MakeKeyValue("uav_velocity_source", diagnostics.uav_state.velocity_source));
  status.values.push_back(MakeKeyValue("ugv_pose_source", diagnostics.ugv_state.pose_source));
  status.values.push_back(MakeKeyValue("ugv_velocity_source", diagnostics.ugv_state.velocity_source));
  status.values.push_back(MakeKeyValue("uav_pose_valid", BoolString(diagnostics.uav_state.pose_valid)));
  status.values.push_back(MakeKeyValue("uav_velocity_valid", BoolString(diagnostics.uav_state.velocity_valid)));
  status.values.push_back(MakeKeyValue("ugv_pose_valid", BoolString(diagnostics.ugv_state.pose_valid)));
  status.values.push_back(MakeKeyValue("ugv_velocity_valid", BoolString(diagnostics.ugv_state.velocity_valid)));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(std::move(status));
  return array;
}

}  // namespace relative_position_fusion
