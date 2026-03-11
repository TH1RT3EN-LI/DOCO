#include "relative_position_fusion/time_alignment_buffer.hpp"

#include <algorithm>

namespace relative_position_fusion
{

namespace
{

template<typename SampleT>
void PruneDequeImpl(
  std::deque<SampleT> * samples,
  const rclcpp::Time & newest_stamp,
  double buffer_length_sec)
{
  if (samples == nullptr || samples->empty()) {
    return;
  }

  while (!samples->empty()) {
    const double age_sec = (newest_stamp - samples->front().stamp).seconds();
    if (age_sec <= buffer_length_sec) {
      break;
    }
    samples->pop_front();
  }
}

template<typename SampleT>
std::optional<SampleT> LatestFreshSample(
  const std::deque<SampleT> & samples,
  const rclcpp::Time & now,
  double max_age_sec)
{
  for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
    const double age_sec = (now - it->stamp).seconds();
    if (age_sec < 0.0) {
      continue;
    }
    if (age_sec <= max_age_sec) {
      return *it;
    }
    break;
  }
  return std::nullopt;
}

}  // namespace

TimeAlignmentBuffer::TimeAlignmentBuffer(double buffer_length_sec)
: buffer_length_sec_(std::max(0.1, buffer_length_sec))
{
}

void TimeAlignmentBuffer::SetBufferLengthSec(double buffer_length_sec)
{
  std::scoped_lock lock(mutex_);
  buffer_length_sec_ = std::max(0.1, buffer_length_sec);
}

void TimeAlignmentBuffer::PushPose(const PoseSample2D & sample)
{
  std::scoped_lock lock(mutex_);
  if (!pose_samples_.empty() && sample.stamp <= pose_samples_.back().stamp) {
    pose_samples_.push_back(sample);
    std::sort(
      pose_samples_.begin(), pose_samples_.end(),
      [](const PoseSample2D & lhs, const PoseSample2D & rhs) { return lhs.stamp < rhs.stamp; });
  } else {
    pose_samples_.push_back(sample);
  }
  PruneDequeImpl(&pose_samples_, pose_samples_.back().stamp, buffer_length_sec_);
}

void TimeAlignmentBuffer::PushVelocity(const VelocitySample2D & sample)
{
  std::scoped_lock lock(mutex_);
  if (!velocity_samples_.empty() && sample.stamp <= velocity_samples_.back().stamp) {
    velocity_samples_.push_back(sample);
    std::sort(
      velocity_samples_.begin(), velocity_samples_.end(),
      [](const VelocitySample2D & lhs, const VelocitySample2D & rhs) { return lhs.stamp < rhs.stamp; });
  } else {
    velocity_samples_.push_back(sample);
  }
  PruneDequeImpl(&velocity_samples_, velocity_samples_.back().stamp, buffer_length_sec_);
}

std::optional<PoseSample2D> TimeAlignmentBuffer::LatestPose(
  const rclcpp::Time & now,
  double max_age_sec) const
{
  std::scoped_lock lock(mutex_);
  return LatestFreshSample(pose_samples_, now, max_age_sec);
}

std::optional<VelocitySample2D> TimeAlignmentBuffer::LatestVelocity(
  const rclcpp::Time & now,
  double max_age_sec) const
{
  std::scoped_lock lock(mutex_);
  return LatestFreshSample(velocity_samples_, now, max_age_sec);
}

std::optional<VelocitySample2D> TimeAlignmentBuffer::EstimateVelocityFromPoseHistory(
  const rclcpp::Time & now,
  double max_pose_age_sec,
  double min_dt_sec,
  double max_dt_sec,
  const std::string & fallback_source) const
{
  std::scoped_lock lock(mutex_);
  auto latest = LatestFreshSample(pose_samples_, now, max_pose_age_sec);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  for (auto it = pose_samples_.rbegin() + 1; it != pose_samples_.rend(); ++it) {
    const double dt_sec = (latest->stamp - it->stamp).seconds();
    if (dt_sec < min_dt_sec) {
      continue;
    }
    if (dt_sec > max_dt_sec) {
      break;
    }

    VelocitySample2D estimated;
    estimated.stamp = latest->stamp;
    estimated.velocity_global = (latest->position_global - it->position_global) / dt_sec;
    estimated.source = fallback_source;
    return estimated;
  }

  return std::nullopt;
}

template<typename SampleT>
void TimeAlignmentBuffer::PruneDeque(
  std::deque<SampleT> * samples,
  const rclcpp::Time & newest_stamp,
  double buffer_length_sec)
{
  PruneDequeImpl(samples, newest_stamp, buffer_length_sec);
}

template void TimeAlignmentBuffer::PruneDeque<PoseSample2D>(
  std::deque<PoseSample2D> * samples,
  const rclcpp::Time & newest_stamp,
  double buffer_length_sec);

template void TimeAlignmentBuffer::PruneDeque<VelocitySample2D>(
  std::deque<VelocitySample2D> * samples,
  const rclcpp::Time & newest_stamp,
  double buffer_length_sec);

}  // namespace relative_position_fusion
