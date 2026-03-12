// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__TRAITS_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uav_visual_landing/msg/detail/landing_controller_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace uav_visual_landing
{

namespace msg
{

inline void to_flow_style_yaml(
  const LandingControllerState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << ", ";
  }

  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: target_detected
  {
    out << "target_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.target_detected, out);
    out << ", ";
  }

  // member: observation_age_s
  {
    out << "observation_age_s: ";
    rosidl_generator_traits::value_to_yaml(msg.observation_age_s, out);
    out << ", ";
  }

  // member: target_confidence
  {
    out << "target_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.target_confidence, out);
    out << ", ";
  }

  // member: height_source
  {
    out << "height_source: ";
    rosidl_generator_traits::value_to_yaml(msg.height_source, out);
    out << ", ";
  }

  // member: current_height_m
  {
    out << "current_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.current_height_m, out);
    out << ", ";
  }

  // member: range_valid
  {
    out << "range_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.range_valid, out);
    out << ", ";
  }

  // member: range_height_m
  {
    out << "range_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_height_m, out);
    out << ", ";
  }

  // member: cmd_vx
  {
    out << "cmd_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vx, out);
    out << ", ";
  }

  // member: cmd_vy
  {
    out << "cmd_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vy, out);
    out << ", ";
  }

  // member: cmd_vz
  {
    out << "cmd_vz: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vz, out);
    out << ", ";
  }

  // member: cmd_yaw_rate
  {
    out << "cmd_yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_yaw_rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LandingControllerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << "\n";
  }

  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: target_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.target_detected, out);
    out << "\n";
  }

  // member: observation_age_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "observation_age_s: ";
    rosidl_generator_traits::value_to_yaml(msg.observation_age_s, out);
    out << "\n";
  }

  // member: target_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.target_confidence, out);
    out << "\n";
  }

  // member: height_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height_source: ";
    rosidl_generator_traits::value_to_yaml(msg.height_source, out);
    out << "\n";
  }

  // member: current_height_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.current_height_m, out);
    out << "\n";
  }

  // member: range_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.range_valid, out);
    out << "\n";
  }

  // member: range_height_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range_height_m: ";
    rosidl_generator_traits::value_to_yaml(msg.range_height_m, out);
    out << "\n";
  }

  // member: cmd_vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vx, out);
    out << "\n";
  }

  // member: cmd_vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vy, out);
    out << "\n";
  }

  // member: cmd_vz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vz: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_vz, out);
    out << "\n";
  }

  // member: cmd_yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_yaw_rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LandingControllerState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace uav_visual_landing

namespace rosidl_generator_traits
{

[[deprecated("use uav_visual_landing::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uav_visual_landing::msg::LandingControllerState & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_visual_landing::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_visual_landing::msg::to_yaml() instead")]]
inline std::string to_yaml(const uav_visual_landing::msg::LandingControllerState & msg)
{
  return uav_visual_landing::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uav_visual_landing::msg::LandingControllerState>()
{
  return "uav_visual_landing::msg::LandingControllerState";
}

template<>
inline const char * name<uav_visual_landing::msg::LandingControllerState>()
{
  return "uav_visual_landing/msg/LandingControllerState";
}

template<>
struct has_fixed_size<uav_visual_landing::msg::LandingControllerState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<uav_visual_landing::msg::LandingControllerState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<uav_visual_landing::msg::LandingControllerState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__TRAITS_HPP_
