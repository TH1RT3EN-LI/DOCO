// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uav_visual_landing:msg/TargetObservation.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__TRAITS_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uav_visual_landing/msg/detail/target_observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace uav_visual_landing
{

namespace msg
{

inline void to_flow_style_yaml(
  const TargetObservation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: detected
  {
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << ", ";
  }

  // member: pose_valid
  {
    out << "pose_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_valid, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: pixel_err_u
  {
    out << "pixel_err_u: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_err_u, out);
    out << ", ";
  }

  // member: pixel_err_v
  {
    out << "pixel_err_v: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_err_v, out);
    out << ", ";
  }

  // member: err_u_norm
  {
    out << "err_u_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.err_u_norm, out);
    out << ", ";
  }

  // member: err_v_norm
  {
    out << "err_v_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.err_v_norm, out);
    out << ", ";
  }

  // member: yaw_err_rad
  {
    out << "yaw_err_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_err_rad, out);
    out << ", ";
  }

  // member: marker_span_px
  {
    out << "marker_span_px: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_span_px, out);
    out << ", ";
  }

  // member: reproj_err_px
  {
    out << "reproj_err_px: ";
    rosidl_generator_traits::value_to_yaml(msg.reproj_err_px, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TargetObservation & msg,
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

  // member: detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detected: ";
    rosidl_generator_traits::value_to_yaml(msg.detected, out);
    out << "\n";
  }

  // member: pose_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_valid, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: pixel_err_u
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_err_u: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_err_u, out);
    out << "\n";
  }

  // member: pixel_err_v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pixel_err_v: ";
    rosidl_generator_traits::value_to_yaml(msg.pixel_err_v, out);
    out << "\n";
  }

  // member: err_u_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_u_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.err_u_norm, out);
    out << "\n";
  }

  // member: err_v_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_v_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.err_v_norm, out);
    out << "\n";
  }

  // member: yaw_err_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_err_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_err_rad, out);
    out << "\n";
  }

  // member: marker_span_px
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "marker_span_px: ";
    rosidl_generator_traits::value_to_yaml(msg.marker_span_px, out);
    out << "\n";
  }

  // member: reproj_err_px
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reproj_err_px: ";
    rosidl_generator_traits::value_to_yaml(msg.reproj_err_px, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TargetObservation & msg, bool use_flow_style = false)
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
  const uav_visual_landing::msg::TargetObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  uav_visual_landing::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uav_visual_landing::msg::to_yaml() instead")]]
inline std::string to_yaml(const uav_visual_landing::msg::TargetObservation & msg)
{
  return uav_visual_landing::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uav_visual_landing::msg::TargetObservation>()
{
  return "uav_visual_landing::msg::TargetObservation";
}

template<>
inline const char * name<uav_visual_landing::msg::TargetObservation>()
{
  return "uav_visual_landing/msg/TargetObservation";
}

template<>
struct has_fixed_size<uav_visual_landing::msg::TargetObservation>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<uav_visual_landing::msg::TargetObservation>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<uav_visual_landing::msg::TargetObservation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__TRAITS_HPP_
