// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_visual_landing:msg/TargetObservation.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__uav_visual_landing__msg__TargetObservation __attribute__((deprecated))
#else
# define DEPRECATED__uav_visual_landing__msg__TargetObservation __declspec(deprecated)
#endif

namespace uav_visual_landing
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TargetObservation_
{
  using Type = TargetObservation_<ContainerAllocator>;

  explicit TargetObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->pose_valid = false;
      this->confidence = 0.0f;
      this->pixel_err_u = 0.0f;
      this->pixel_err_v = 0.0f;
      this->err_u_norm = 0.0f;
      this->err_v_norm = 0.0f;
      this->yaw_err_rad = 0.0f;
      this->marker_span_px = 0.0f;
      this->reproj_err_px = 0.0f;
    }
  }

  explicit TargetObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detected = false;
      this->pose_valid = false;
      this->confidence = 0.0f;
      this->pixel_err_u = 0.0f;
      this->pixel_err_v = 0.0f;
      this->err_u_norm = 0.0f;
      this->err_v_norm = 0.0f;
      this->yaw_err_rad = 0.0f;
      this->marker_span_px = 0.0f;
      this->reproj_err_px = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detected_type =
    bool;
  _detected_type detected;
  using _pose_valid_type =
    bool;
  _pose_valid_type pose_valid;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _pixel_err_u_type =
    float;
  _pixel_err_u_type pixel_err_u;
  using _pixel_err_v_type =
    float;
  _pixel_err_v_type pixel_err_v;
  using _err_u_norm_type =
    float;
  _err_u_norm_type err_u_norm;
  using _err_v_norm_type =
    float;
  _err_v_norm_type err_v_norm;
  using _yaw_err_rad_type =
    float;
  _yaw_err_rad_type yaw_err_rad;
  using _marker_span_px_type =
    float;
  _marker_span_px_type marker_span_px;
  using _reproj_err_px_type =
    float;
  _reproj_err_px_type reproj_err_px;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__detected(
    const bool & _arg)
  {
    this->detected = _arg;
    return *this;
  }
  Type & set__pose_valid(
    const bool & _arg)
  {
    this->pose_valid = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__pixel_err_u(
    const float & _arg)
  {
    this->pixel_err_u = _arg;
    return *this;
  }
  Type & set__pixel_err_v(
    const float & _arg)
  {
    this->pixel_err_v = _arg;
    return *this;
  }
  Type & set__err_u_norm(
    const float & _arg)
  {
    this->err_u_norm = _arg;
    return *this;
  }
  Type & set__err_v_norm(
    const float & _arg)
  {
    this->err_v_norm = _arg;
    return *this;
  }
  Type & set__yaw_err_rad(
    const float & _arg)
  {
    this->yaw_err_rad = _arg;
    return *this;
  }
  Type & set__marker_span_px(
    const float & _arg)
  {
    this->marker_span_px = _arg;
    return *this;
  }
  Type & set__reproj_err_px(
    const float & _arg)
  {
    this->reproj_err_px = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_visual_landing::msg::TargetObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_visual_landing::msg::TargetObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::TargetObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::TargetObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_visual_landing__msg__TargetObservation
    std::shared_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_visual_landing__msg__TargetObservation
    std::shared_ptr<uav_visual_landing::msg::TargetObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TargetObservation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detected != other.detected) {
      return false;
    }
    if (this->pose_valid != other.pose_valid) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->pixel_err_u != other.pixel_err_u) {
      return false;
    }
    if (this->pixel_err_v != other.pixel_err_v) {
      return false;
    }
    if (this->err_u_norm != other.err_u_norm) {
      return false;
    }
    if (this->err_v_norm != other.err_v_norm) {
      return false;
    }
    if (this->yaw_err_rad != other.yaw_err_rad) {
      return false;
    }
    if (this->marker_span_px != other.marker_span_px) {
      return false;
    }
    if (this->reproj_err_px != other.reproj_err_px) {
      return false;
    }
    return true;
  }
  bool operator!=(const TargetObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TargetObservation_

// alias to use template instance with default allocator
using TargetObservation =
  uav_visual_landing::msg::TargetObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__STRUCT_HPP_
