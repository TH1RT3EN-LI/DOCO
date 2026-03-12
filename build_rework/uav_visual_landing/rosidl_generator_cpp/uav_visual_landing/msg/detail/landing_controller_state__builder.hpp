// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_visual_landing:msg/LandingControllerState.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_visual_landing/msg/detail/landing_controller_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_visual_landing
{

namespace msg
{

namespace builder
{

class Init_LandingControllerState_cmd_yaw_rate
{
public:
  explicit Init_LandingControllerState_cmd_yaw_rate(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  ::uav_visual_landing::msg::LandingControllerState cmd_yaw_rate(::uav_visual_landing::msg::LandingControllerState::_cmd_yaw_rate_type arg)
  {
    msg_.cmd_yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vz
{
public:
  explicit Init_LandingControllerState_cmd_vz(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_yaw_rate cmd_vz(::uav_visual_landing::msg::LandingControllerState::_cmd_vz_type arg)
  {
    msg_.cmd_vz = std::move(arg);
    return Init_LandingControllerState_cmd_yaw_rate(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vy
{
public:
  explicit Init_LandingControllerState_cmd_vy(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vz cmd_vy(::uav_visual_landing::msg::LandingControllerState::_cmd_vy_type arg)
  {
    msg_.cmd_vy = std::move(arg);
    return Init_LandingControllerState_cmd_vz(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_cmd_vx
{
public:
  explicit Init_LandingControllerState_cmd_vx(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vy cmd_vx(::uav_visual_landing::msg::LandingControllerState::_cmd_vx_type arg)
  {
    msg_.cmd_vx = std::move(arg);
    return Init_LandingControllerState_cmd_vy(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_range_height_m
{
public:
  explicit Init_LandingControllerState_range_height_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_cmd_vx range_height_m(::uav_visual_landing::msg::LandingControllerState::_range_height_m_type arg)
  {
    msg_.range_height_m = std::move(arg);
    return Init_LandingControllerState_cmd_vx(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_range_valid
{
public:
  explicit Init_LandingControllerState_range_valid(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_range_height_m range_valid(::uav_visual_landing::msg::LandingControllerState::_range_valid_type arg)
  {
    msg_.range_valid = std::move(arg);
    return Init_LandingControllerState_range_height_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_current_height_m
{
public:
  explicit Init_LandingControllerState_current_height_m(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_range_valid current_height_m(::uav_visual_landing::msg::LandingControllerState::_current_height_m_type arg)
  {
    msg_.current_height_m = std::move(arg);
    return Init_LandingControllerState_range_valid(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_height_source
{
public:
  explicit Init_LandingControllerState_height_source(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_current_height_m height_source(::uav_visual_landing::msg::LandingControllerState::_height_source_type arg)
  {
    msg_.height_source = std::move(arg);
    return Init_LandingControllerState_current_height_m(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_target_confidence
{
public:
  explicit Init_LandingControllerState_target_confidence(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_height_source target_confidence(::uav_visual_landing::msg::LandingControllerState::_target_confidence_type arg)
  {
    msg_.target_confidence = std::move(arg);
    return Init_LandingControllerState_height_source(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_observation_age_s
{
public:
  explicit Init_LandingControllerState_observation_age_s(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_target_confidence observation_age_s(::uav_visual_landing::msg::LandingControllerState::_observation_age_s_type arg)
  {
    msg_.observation_age_s = std::move(arg);
    return Init_LandingControllerState_target_confidence(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_target_detected
{
public:
  explicit Init_LandingControllerState_target_detected(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_observation_age_s target_detected(::uav_visual_landing::msg::LandingControllerState::_target_detected_type arg)
  {
    msg_.target_detected = std::move(arg);
    return Init_LandingControllerState_observation_age_s(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_phase
{
public:
  explicit Init_LandingControllerState_phase(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_target_detected phase(::uav_visual_landing::msg::LandingControllerState::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return Init_LandingControllerState_target_detected(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_active
{
public:
  explicit Init_LandingControllerState_active(::uav_visual_landing::msg::LandingControllerState & msg)
  : msg_(msg)
  {}
  Init_LandingControllerState_phase active(::uav_visual_landing::msg::LandingControllerState::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_LandingControllerState_phase(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

class Init_LandingControllerState_header
{
public:
  Init_LandingControllerState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandingControllerState_active header(::uav_visual_landing::msg::LandingControllerState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LandingControllerState_active(msg_);
  }

private:
  ::uav_visual_landing::msg::LandingControllerState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_visual_landing::msg::LandingControllerState>()
{
  return uav_visual_landing::msg::builder::Init_LandingControllerState_header();
}

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__LANDING_CONTROLLER_STATE__BUILDER_HPP_
