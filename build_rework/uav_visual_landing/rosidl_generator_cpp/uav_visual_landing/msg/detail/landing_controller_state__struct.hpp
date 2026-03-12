// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_

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
# define DEPRECATED__uav_visual_landing__msg__LandingControllerState __attribute__((deprecated))
#else
# define DEPRECATED__uav_visual_landing__msg__LandingControllerState __declspec(deprecated)
#endif

namespace uav_visual_landing
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandingControllerState_
{
  using Type = LandingControllerState_<ContainerAllocator>;

  explicit LandingControllerState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->phase = "";
      this->target_detected = false;
      this->observation_age_s = 0.0f;
      this->target_confidence = 0.0f;
      this->height_source = "";
      this->current_height_m = 0.0f;
      this->range_valid = false;
      this->range_height_m = 0.0f;
      this->cmd_vx = 0.0f;
      this->cmd_vy = 0.0f;
      this->cmd_vz = 0.0f;
      this->cmd_yaw_rate = 0.0f;
    }
  }

  explicit LandingControllerState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    phase(_alloc),
    height_source(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active = false;
      this->phase = "";
      this->target_detected = false;
      this->observation_age_s = 0.0f;
      this->target_confidence = 0.0f;
      this->height_source = "";
      this->current_height_m = 0.0f;
      this->range_valid = false;
      this->range_height_m = 0.0f;
      this->cmd_vx = 0.0f;
      this->cmd_vy = 0.0f;
      this->cmd_vz = 0.0f;
      this->cmd_yaw_rate = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _active_type =
    bool;
  _active_type active;
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phase_type phase;
  using _target_detected_type =
    bool;
  _target_detected_type target_detected;
  using _observation_age_s_type =
    float;
  _observation_age_s_type observation_age_s;
  using _target_confidence_type =
    float;
  _target_confidence_type target_confidence;
  using _height_source_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _height_source_type height_source;
  using _current_height_m_type =
    float;
  _current_height_m_type current_height_m;
  using _range_valid_type =
    bool;
  _range_valid_type range_valid;
  using _range_height_m_type =
    float;
  _range_height_m_type range_height_m;
  using _cmd_vx_type =
    float;
  _cmd_vx_type cmd_vx;
  using _cmd_vy_type =
    float;
  _cmd_vy_type cmd_vy;
  using _cmd_vz_type =
    float;
  _cmd_vz_type cmd_vz;
  using _cmd_yaw_rate_type =
    float;
  _cmd_yaw_rate_type cmd_yaw_rate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phase = _arg;
    return *this;
  }
  Type & set__target_detected(
    const bool & _arg)
  {
    this->target_detected = _arg;
    return *this;
  }
  Type & set__observation_age_s(
    const float & _arg)
  {
    this->observation_age_s = _arg;
    return *this;
  }
  Type & set__target_confidence(
    const float & _arg)
  {
    this->target_confidence = _arg;
    return *this;
  }
  Type & set__height_source(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->height_source = _arg;
    return *this;
  }
  Type & set__current_height_m(
    const float & _arg)
  {
    this->current_height_m = _arg;
    return *this;
  }
  Type & set__range_valid(
    const bool & _arg)
  {
    this->range_valid = _arg;
    return *this;
  }
  Type & set__range_height_m(
    const float & _arg)
  {
    this->range_height_m = _arg;
    return *this;
  }
  Type & set__cmd_vx(
    const float & _arg)
  {
    this->cmd_vx = _arg;
    return *this;
  }
  Type & set__cmd_vy(
    const float & _arg)
  {
    this->cmd_vy = _arg;
    return *this;
  }
  Type & set__cmd_vz(
    const float & _arg)
  {
    this->cmd_vz = _arg;
    return *this;
  }
  Type & set__cmd_yaw_rate(
    const float & _arg)
  {
    this->cmd_yaw_rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> *;
  using ConstRawPtr =
    const uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uav_visual_landing__msg__LandingControllerState
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uav_visual_landing__msg__LandingControllerState
    std::shared_ptr<uav_visual_landing::msg::LandingControllerState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandingControllerState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->phase != other.phase) {
      return false;
    }
    if (this->target_detected != other.target_detected) {
      return false;
    }
    if (this->observation_age_s != other.observation_age_s) {
      return false;
    }
    if (this->target_confidence != other.target_confidence) {
      return false;
    }
    if (this->height_source != other.height_source) {
      return false;
    }
    if (this->current_height_m != other.current_height_m) {
      return false;
    }
    if (this->range_valid != other.range_valid) {
      return false;
    }
    if (this->range_height_m != other.range_height_m) {
      return false;
    }
    if (this->cmd_vx != other.cmd_vx) {
      return false;
    }
    if (this->cmd_vy != other.cmd_vy) {
      return false;
    }
    if (this->cmd_vz != other.cmd_vz) {
      return false;
    }
    if (this->cmd_yaw_rate != other.cmd_yaw_rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandingControllerState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandingControllerState_

// alias to use template instance with default allocator
using LandingControllerState =
  uav_visual_landing::msg::LandingControllerState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__STRUCT_HPP_
