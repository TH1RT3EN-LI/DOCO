#pragma once

#include <array>
#include <cmath>

namespace uav_bridge
{

enum class UavControlMode
{
  Hold,
  Takeoff,
  Position,
  Px4PositionHold,
  VelocityBody,
  Landing,
};

class UavControlModeTracker
{
public:
  UavControlMode mode() const
  {
    return mode_;
  }

  void requestHold()
  {
    mode_ = UavControlMode::Hold;
  }

  void requestTakeoff()
  {
    mode_ = UavControlMode::Takeoff;
  }

  void requestPosition()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::Position;
    }
  }

  void requestPx4PositionHold()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::Px4PositionHold;
    }
  }

  void requestVelocityBody()
  {
    if (mode_ != UavControlMode::Landing) {
      mode_ = UavControlMode::VelocityBody;
    }
  }

  void requestLanding()
  {
    mode_ = UavControlMode::Landing;
  }

  void onVelocityCommandTimeout()
  {
    if (mode_ == UavControlMode::VelocityBody) {
      mode_ = UavControlMode::Hold;
    }
  }

  void onLandingComplete()
  {
    if (mode_ == UavControlMode::Landing) {
      mode_ = UavControlMode::Hold;
    }
  }

private:
  UavControlMode mode_{UavControlMode::Hold};
};

inline bool isFiniteVelocityBodyCommand(
  float body_velocity_x_mps,
  float body_velocity_y_mps,
  float body_velocity_z_mps,
  float yaw_rate_radps)
{
  return std::isfinite(body_velocity_x_mps) &&
         std::isfinite(body_velocity_y_mps) &&
         std::isfinite(body_velocity_z_mps) &&
         std::isfinite(yaw_rate_radps);
}

inline bool hasValidVelocityEstimate(
  bool horizontal_velocity_valid,
  bool vertical_velocity_valid,
  const std::array<float, 3> & velocity_ned_mps)
{
  return horizontal_velocity_valid &&
         vertical_velocity_valid &&
         std::isfinite(velocity_ned_mps[0]) &&
         std::isfinite(velocity_ned_mps[1]) &&
         std::isfinite(velocity_ned_mps[2]);
}

inline bool isFiniteVelocitySetpoint(
  const std::array<float, 3> & velocity_ned_mps,
  float yaw_speed_ned_radps)
{
  return std::isfinite(velocity_ned_mps[0]) &&
         std::isfinite(velocity_ned_mps[1]) &&
         std::isfinite(velocity_ned_mps[2]) &&
         std::isfinite(yaw_speed_ned_radps);
}

}  // namespace uav_bridge
