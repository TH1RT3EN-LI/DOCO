// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "uav_visual_landing/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "uav_visual_landing/msg/detail/landing_controller_state__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace uav_visual_landing
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_visual_landing
cdr_serialize(
  const uav_visual_landing::msg::LandingControllerState & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_visual_landing
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  uav_visual_landing::msg::LandingControllerState & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_visual_landing
get_serialized_size(
  const uav_visual_landing::msg::LandingControllerState & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_visual_landing
max_serialized_size_LandingControllerState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace uav_visual_landing

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_uav_visual_landing
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, uav_visual_landing, msg, LandingControllerState)();

#ifdef __cplusplus
}
#endif

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
