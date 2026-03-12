#pragma once

namespace uav_bridge
{

enum class UavControlMode
{
  Auto,
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

  bool requestAutoEnable(bool has_auto_command)
  {
    if (!has_auto_command || mode_ == UavControlMode::Landing) {
      return false;
    }
    mode_ = UavControlMode::Auto;
    return true;
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
  UavControlMode mode_{UavControlMode::Auto};
};

}  // namespace uav_bridge
