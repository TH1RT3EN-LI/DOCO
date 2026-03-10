#include "ugv_sim_tools/initial_pose_logic.hpp"

#include <algorithm>
#include <cmath>

namespace ugv_sim_tools
{

int clampPublishCount(int requested_count)
{
  return std::max(1, requested_count);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  const double half = 0.5 * yaw;

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(half);
  quaternion.w = std::cos(half);
  return quaternion;
}

std::array<double, 36> buildConservativeInitialPoseCovariance()
{
  constexpr double kPi = 3.14159265358979323846;
  std::array<double, 36> covariance{};
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[35] = std::pow(10.0 * kPi / 180.0, 2.0);
  return covariance;
}

InitialPoseStateMachine::InitialPoseStateMachine(const InitialPoseConfig & config)
: publish_count_(clampPublishCount(config.publish_count)),
  require_subscriber_(config.require_subscriber),
  wait_for_amcl_pose_(config.wait_for_amcl_pose),
  max_wait_sec_(config.max_wait_sec)
{
}

PublishTickResult InitialPoseStateMachine::tick(bool has_subscriber, double elapsed_sec)
{
  PublishTickResult result;

  if (done_) {
    result.phase = InitialPosePhase::kCompleted;
    result.published_count = published_count_;
    return result;
  }

  if (max_wait_sec_ > 0.0 && elapsed_sec > max_wait_sec_) {
    done_ = true;
    result.done_now = true;
    result.timed_out = true;
    result.phase = InitialPosePhase::kCompleted;
    result.published_count = published_count_;
    return result;
  }

  if (require_subscriber_ && !has_subscriber) {
    result.waiting_for_subscriber = true;
    result.phase = InitialPosePhase::kWaitingForSubscriber;
    result.published_count = published_count_;
    return result;
  }

  result.should_publish = true;
  ++published_count_;
  result.published_count = published_count_;

  if (!finished_initial_burst_ && published_count_ >= publish_count_) {
    finished_initial_burst_ = true;
    result.initial_burst_completed_now = true;
    if (!wait_for_amcl_pose_) {
      done_ = true;
      result.done_now = true;
      result.phase = InitialPosePhase::kCompleted;
      return result;
    }
  }

  result.phase = currentPhase();
  return result;
}

bool InitialPoseStateMachine::onAmclPoseReceived()
{
  if (done_ || amcl_pose_received_) {
    return false;
  }

  amcl_pose_received_ = true;
  done_ = true;
  return true;
}

bool InitialPoseStateMachine::isDone() const
{
  return done_;
}

int InitialPoseStateMachine::publishedCount() const
{
  return published_count_;
}

InitialPosePhase InitialPoseStateMachine::phase() const
{
  return currentPhase();
}

InitialPosePhase InitialPoseStateMachine::currentPhase() const
{
  if (done_) {
    return InitialPosePhase::kCompleted;
  }
  if (finished_initial_burst_ && wait_for_amcl_pose_ && !amcl_pose_received_) {
    return InitialPosePhase::kWaitingForAmclPose;
  }
  return InitialPosePhase::kPublishingBurst;
}

}
