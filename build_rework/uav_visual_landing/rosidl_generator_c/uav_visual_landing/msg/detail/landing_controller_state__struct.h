// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'phase'
// Member 'height_source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LandingControllerState in the package uav_visual_landing.
typedef struct uav_visual_landing__msg__LandingControllerState
{
  std_msgs__msg__Header header;
  bool active;
  rosidl_runtime_c__String phase;
  bool target_detected;
  float observation_age_s;
  float target_confidence;
  rosidl_runtime_c__String height_source;
  float current_height_m;
  bool range_valid;
  float range_height_m;
  float cmd_vx;
  float cmd_vy;
  float cmd_vz;
  float cmd_yaw_rate;
} uav_visual_landing__msg__LandingControllerState;

// Struct for a sequence of uav_visual_landing__msg__LandingControllerState.
typedef struct uav_visual_landing__msg__LandingControllerState__Sequence
{
  uav_visual_landing__msg__LandingControllerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_visual_landing__msg__LandingControllerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_H_
