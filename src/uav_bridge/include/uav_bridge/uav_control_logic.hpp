#pragma once

namespace uav_bridge
{

enum class UavControlMode
{
  Hold,
  Takeoff,
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

}  // namespace uav_bridge
