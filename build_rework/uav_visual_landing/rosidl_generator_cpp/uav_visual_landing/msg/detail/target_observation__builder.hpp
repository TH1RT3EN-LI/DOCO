// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uav_visual_landing:msg/TargetObservation.idl
// generated code does not contain a copyright notice

#ifndef UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__BUILDER_HPP_
#define UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uav_visual_landing/msg/detail/target_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uav_visual_landing
{

namespace msg
{

namespace builder
{

class Init_TargetObservation_reproj_err_px
{
public:
  explicit Init_TargetObservation_reproj_err_px(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  ::uav_visual_landing::msg::TargetObservation reproj_err_px(::uav_visual_landing::msg::TargetObservation::_reproj_err_px_type arg)
  {
    msg_.reproj_err_px = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_marker_span_px
{
public:
  explicit Init_TargetObservation_marker_span_px(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_reproj_err_px marker_span_px(::uav_visual_landing::msg::TargetObservation::_marker_span_px_type arg)
  {
    msg_.marker_span_px = std::move(arg);
    return Init_TargetObservation_reproj_err_px(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_yaw_err_rad
{
public:
  explicit Init_TargetObservation_yaw_err_rad(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_marker_span_px yaw_err_rad(::uav_visual_landing::msg::TargetObservation::_yaw_err_rad_type arg)
  {
    msg_.yaw_err_rad = std::move(arg);
    return Init_TargetObservation_marker_span_px(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_err_v_norm
{
public:
  explicit Init_TargetObservation_err_v_norm(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_yaw_err_rad err_v_norm(::uav_visual_landing::msg::TargetObservation::_err_v_norm_type arg)
  {
    msg_.err_v_norm = std::move(arg);
    return Init_TargetObservation_yaw_err_rad(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_err_u_norm
{
public:
  explicit Init_TargetObservation_err_u_norm(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_err_v_norm err_u_norm(::uav_visual_landing::msg::TargetObservation::_err_u_norm_type arg)
  {
    msg_.err_u_norm = std::move(arg);
    return Init_TargetObservation_err_v_norm(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_pixel_err_v
{
public:
  explicit Init_TargetObservation_pixel_err_v(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_err_u_norm pixel_err_v(::uav_visual_landing::msg::TargetObservation::_pixel_err_v_type arg)
  {
    msg_.pixel_err_v = std::move(arg);
    return Init_TargetObservation_err_u_norm(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_pixel_err_u
{
public:
  explicit Init_TargetObservation_pixel_err_u(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_pixel_err_v pixel_err_u(::uav_visual_landing::msg::TargetObservation::_pixel_err_u_type arg)
  {
    msg_.pixel_err_u = std::move(arg);
    return Init_TargetObservation_pixel_err_v(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_confidence
{
public:
  explicit Init_TargetObservation_confidence(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_pixel_err_u confidence(::uav_visual_landing::msg::TargetObservation::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_TargetObservation_pixel_err_u(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_pose_valid
{
public:
  explicit Init_TargetObservation_pose_valid(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_confidence pose_valid(::uav_visual_landing::msg::TargetObservation::_pose_valid_type arg)
  {
    msg_.pose_valid = std::move(arg);
    return Init_TargetObservation_confidence(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_detected
{
public:
  explicit Init_TargetObservation_detected(::uav_visual_landing::msg::TargetObservation & msg)
  : msg_(msg)
  {}
  Init_TargetObservation_pose_valid detected(::uav_visual_landing::msg::TargetObservation::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_TargetObservation_pose_valid(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

class Init_TargetObservation_header
{
public:
  Init_TargetObservation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetObservation_detected header(::uav_visual_landing::msg::TargetObservation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TargetObservation_detected(msg_);
  }

private:
  ::uav_visual_landing::msg::TargetObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uav_visual_landing::msg::TargetObservation>()
{
  return uav_visual_landing::msg::builder::Init_TargetObservation_header();
}

}  // namespace uav_visual_landing

#endif  // UAV_VISUAL_LANDING__MSG__DETAIL__TARGET_OBSERVATION__BUILDER_HPP_
