// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uav_visual_landing:msg/TargetObservation.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_H_
#define UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_H_

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

/// Struct defined in msg/TargetObservation in the package uav_visual_landing.
typedef struct uav_visual_landing__msg__TargetObservation
{
  std_msgs__msg__Header header;
  bool detected;
  bool pose_valid;
  float confidence;
  float pixel_err_u;
  float pixel_err_v;
  float err_u_norm;
  float err_v_norm;
  float yaw_err_rad;
  float marker_span_px;
  float reproj_err_px;
} uav_visual_landing__msg__TargetObservation;

// Struct for a sequence of uav_visual_landing__msg__TargetObservation.
typedef struct uav_visual_landing__msg__TargetObservation__Sequence
{
  uav_visual_landing__msg__TargetObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uav_visual_landing__msg__TargetObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_H_
