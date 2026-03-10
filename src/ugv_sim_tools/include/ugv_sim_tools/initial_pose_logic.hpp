#pragma once

#include <array>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>

namespace ugv_sim_tools
{

enum class InitialPosePhase
{
  kPublishingBurst,
  kWaitingForSubscriber,
  kWaitingForAmclPose,
  kCompleted,
};

struct InitialPoseConfig
{
  int publish_count{5};
  bool require_subscriber{true};
  bool wait_for_amcl_pose{true};
  double max_wait_sec{120.0};
};

struct PublishTickResult
{
  bool should_publish{false};
  bool waiting_for_subscriber{false};
  bool initial_burst_completed_now{false};
  bool done_now{false};
  bool timed_out{false};
  int published_count{0};
  InitialPosePhase phase{InitialPosePhase::kPublishingBurst};
};

int clampPublishCount(int requested_count);

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

std::array<double, 36> buildConservativeInitialPoseCovariance();

class InitialPoseStateMachine
{
public:
  explicit InitialPoseStateMachine(const InitialPoseConfig & config);

  PublishTickResult tick(bool has_subscriber, double elapsed_sec);

  bool onAmclPoseReceived();

  bool isDone() const;

  int publishedCount() const;

  InitialPosePhase phase() const;

private:
  InitialPosePhase currentPhase() const;

  int publish_count_;
  bool require_subscriber_;
  bool wait_for_amcl_pose_;
  double max_wait_sec_;
  int published_count_{0};
  bool finished_initial_burst_{false};
  bool amcl_pose_received_{false};
  bool done_{false};
};

}
