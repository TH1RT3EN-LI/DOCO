// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uav_visual_landing:msg/TargetObservation.idl
// generated code does not contain a copyright notice
#include "uav_visual_landing/msg/detail/target_observation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
uav_visual_landing__msg__TargetObservation__init(uav_visual_landing__msg__TargetObservation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    uav_visual_landing__msg__TargetObservation__fini(msg);
    return false;
  }
  // detected
  // pose_valid
  // confidence
  // pixel_err_u
  // pixel_err_v
  // err_u_norm
  // err_v_norm
  // yaw_err_rad
  // marker_span_px
  // reproj_err_px
  return true;
}

void
uav_visual_landing__msg__TargetObservation__fini(uav_visual_landing__msg__TargetObservation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detected
  // pose_valid
  // confidence
  // pixel_err_u
  // pixel_err_v
  // err_u_norm
  // err_v_norm
  // yaw_err_rad
  // marker_span_px
  // reproj_err_px
}

bool
uav_visual_landing__msg__TargetObservation__are_equal(const uav_visual_landing__msg__TargetObservation * lhs, const uav_visual_landing__msg__TargetObservation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // detected
  if (lhs->detected != rhs->detected) {
    return false;
  }
  // pose_valid
  if (lhs->pose_valid != rhs->pose_valid) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // pixel_err_u
  if (lhs->pixel_err_u != rhs->pixel_err_u) {
    return false;
  }
  // pixel_err_v
  if (lhs->pixel_err_v != rhs->pixel_err_v) {
    return false;
  }
  // err_u_norm
  if (lhs->err_u_norm != rhs->err_u_norm) {
    return false;
  }
  // err_v_norm
  if (lhs->err_v_norm != rhs->err_v_norm) {
    return false;
  }
  // yaw_err_rad
  if (lhs->yaw_err_rad != rhs->yaw_err_rad) {
    return false;
  }
  // marker_span_px
  if (lhs->marker_span_px != rhs->marker_span_px) {
    return false;
  }
  // reproj_err_px
  if (lhs->reproj_err_px != rhs->reproj_err_px) {
    return false;
  }
  return true;
}

bool
uav_visual_landing__msg__TargetObservation__copy(
  const uav_visual_landing__msg__TargetObservation * input,
  uav_visual_landing__msg__TargetObservation * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // detected
  output->detected = input->detected;
  // pose_valid
  output->pose_valid = input->pose_valid;
  // confidence
  output->confidence = input->confidence;
  // pixel_err_u
  output->pixel_err_u = input->pixel_err_u;
  // pixel_err_v
  output->pixel_err_v = input->pixel_err_v;
  // err_u_norm
  output->err_u_norm = input->err_u_norm;
  // err_v_norm
  output->err_v_norm = input->err_v_norm;
  // yaw_err_rad
  output->yaw_err_rad = input->yaw_err_rad;
  // marker_span_px
  output->marker_span_px = input->marker_span_px;
  // reproj_err_px
  output->reproj_err_px = input->reproj_err_px;
  return true;
}

uav_visual_landing__msg__TargetObservation *
uav_visual_landing__msg__TargetObservation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__TargetObservation * msg = (uav_visual_landing__msg__TargetObservation *)allocator.allocate(sizeof(uav_visual_landing__msg__TargetObservation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uav_visual_landing__msg__TargetObservation));
  bool success = uav_visual_landing__msg__TargetObservation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uav_visual_landing__msg__TargetObservation__destroy(uav_visual_landing__msg__TargetObservation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uav_visual_landing__msg__TargetObservation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uav_visual_landing__msg__TargetObservation__Sequence__init(uav_visual_landing__msg__TargetObservation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__TargetObservation * data = NULL;

  if (size) {
    data = (uav_visual_landing__msg__TargetObservation *)allocator.zero_allocate(size, sizeof(uav_visual_landing__msg__TargetObservation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uav_visual_landing__msg__TargetObservation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uav_visual_landing__msg__TargetObservation__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
uav_visual_landing__msg__TargetObservation__Sequence__fini(uav_visual_landing__msg__TargetObservation__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      uav_visual_landing__msg__TargetObservation__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

uav_visual_landing__msg__TargetObservation__Sequence *
uav_visual_landing__msg__TargetObservation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uav_visual_landing__msg__TargetObservation__Sequence * array = (uav_visual_landing__msg__TargetObservation__Sequence *)allocator.allocate(sizeof(uav_visual_landing__msg__TargetObservation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uav_visual_landing__msg__TargetObservation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uav_visual_landing__msg__TargetObservation__Sequence__destroy(uav_visual_landing__msg__TargetObservation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uav_visual_landing__msg__TargetObservation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uav_visual_landing__msg__TargetObservation__Sequence__are_equal(const uav_visual_landing__msg__TargetObservation__Sequence * lhs, const uav_visual_landing__msg__TargetObservation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uav_visual_landing__msg__TargetObservation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uav_visual_landing__msg__TargetObservation__Sequence__copy(
  const uav_visual_landing__msg__TargetObservation__Sequence * input,
  uav_visual_landing__msg__TargetObservation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uav_visual_landing__msg__TargetObservation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uav_visual_landing__msg__TargetObservation * data =
      (uav_visual_landing__msg__TargetObservation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uav_visual_landing__msg__TargetObservation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uav_visual_landing__msg__TargetObservation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uav_visual_landing__msg__TargetObservation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
