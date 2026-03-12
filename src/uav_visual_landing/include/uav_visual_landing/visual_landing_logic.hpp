#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace uav_visual_landing
{

enum class ControllerPhase
{
  Ready,
  HoldWait,
  TrackAlign,
  HoldVerify,
  DescendTrack,
  Terminal,
  Land,
};

inline const char * phaseName(ControllerPhase phase)
{
  switch (phase) {
    case ControllerPhase::Ready:
      return "READY";
    case ControllerPhase::HoldWait:
      return "HOLD_WAIT";
    case ControllerPhase::TrackAlign:
      return "TRACK_ALIGN";
    case ControllerPhase::HoldVerify:
      return "HOLD_VERIFY";
    case ControllerPhase::DescendTrack:
      return "DESCEND_TRACK";
    case ControllerPhase::Terminal:
      return "TERMINAL";
    case ControllerPhase::Land:
      return "LAND";
  }
  return "UNKNOWN";
}

enum class HeightSource
{
  Odom,
  FlowRange,
};

inline const char * heightSourceName(HeightSource source)
{
  switch (source) {
    case HeightSource::Odom:
      return "ODOM";
    case HeightSource::FlowRange:
      return "FLOW_RANGE";
  }
  return "ODOM";
}

struct HeightConfig
{
  double timeout_s{0.15};
  float min_m{0.05f};
  float max_m{5.0f};
  float max_diff_m{0.35f};
};

struct HeightDecision
{
  bool height_valid{false};
  float height_m{0.0f};
  HeightSource height_source{HeightSource::Odom};
};

struct AlignmentConfig
{
  float align_enter_lateral_m{0.08f};
  float align_exit_lateral_m{0.05f};
  float align_enter_yaw_rad{0.08f};
  float align_exit_yaw_rad{0.06f};
};

struct MetricLateralError
{
  bool valid{false};
  float x_m{0.0f};
  float y_m{0.0f};
  float norm_m{0.0f};
  float x_rate_mps{0.0f};
  float y_rate_mps{0.0f};
};

struct RelativeTarget3D
{
  bool valid{false};
  bool yaw_valid{false};
  float x_m{0.0f};
  float y_m{0.0f};
  float z_m{0.0f};
  float yaw_err_rad{0.0f};
};

struct CommandRateLimitConfig
{
  float max_acc_xy_mps2{0.8f};
  float max_acc_z_mps2{0.5f};
  float max_acc_yaw_radps2{1.2f};
};

inline float clamp(float value, float limit)
{
  return std::max(-limit, std::min(value, limit));
}

inline float clampDelta(float target, float previous, float max_delta)
{
  if (max_delta <= 0.0f) {
    return target;
  }
  const float delta = target - previous;
  if (delta > max_delta) {
    return previous + max_delta;
  }
  if (delta < -max_delta) {
    return previous - max_delta;
  }
  return target;
}

inline float lerp(float a, float b, float t)
{
  return a + ((b - a) * t);
}

inline HeightDecision evaluateHeightDecision(
  float odom_height_m,
  bool has_height_measurement,
  float height_measurement_m,
  double height_age_s,
  const HeightConfig & config)
{
  HeightDecision decision;
  decision.height_m = odom_height_m;
  if (!has_height_measurement) {
    return decision;
  }
  if (!std::isfinite(height_measurement_m) || height_age_s < 0.0 ||
    height_age_s > config.timeout_s)
  {
    return decision;
  }
  if (height_measurement_m < config.min_m || height_measurement_m > config.max_m) {
    return decision;
  }
  if (std::isfinite(odom_height_m) &&
    std::abs(height_measurement_m - odom_height_m) > config.max_diff_m)
  {
    return decision;
  }
  decision.height_valid = true;
  decision.height_m = height_measurement_m;
  decision.height_source = HeightSource::FlowRange;
  return decision;
}

inline MetricLateralError computeMetricLateralError(
  float err_u_norm,
  float err_v_norm,
  float err_u_rate_norm_s,
  float err_v_rate_norm_s,
  float active_height_m)
{
  MetricLateralError error;
  if (!std::isfinite(err_u_norm) || !std::isfinite(err_v_norm) ||
    !std::isfinite(err_u_rate_norm_s) || !std::isfinite(err_v_rate_norm_s) ||
    !std::isfinite(active_height_m) || active_height_m <= 1.0e-6f)
  {
    return error;
  }

  error.valid = true;
  error.x_m = active_height_m * err_u_norm;
  error.y_m = active_height_m * err_v_norm;
  error.norm_m = std::hypot(error.x_m, error.y_m);
  error.x_rate_mps = active_height_m * err_u_rate_norm_s;
  error.y_rate_mps = active_height_m * err_v_rate_norm_s;
  return error;
}

inline bool hasFreshTrackingObservation(
  bool has_valid_observation,
  double observation_age_s,
  double observation_timeout_s,
  int consecutive_miss_count,
  int observation_miss_frames)
{
  if (!has_valid_observation || !std::isfinite(observation_age_s) ||
    !std::isfinite(observation_timeout_s) || observation_age_s < 0.0 ||
    observation_age_s > observation_timeout_s)
  {
    return false;
  }

  if (observation_miss_frames <= 0) {
    return true;
  }
  return consecutive_miss_count < observation_miss_frames;
}

inline RelativeTarget3D computeRelativeTarget3D(
  float err_u_norm,
  float err_v_norm,
  float depth_m,
  bool yaw_valid,
  float yaw_err_rad)
{
  RelativeTarget3D target;
  if (!std::isfinite(err_u_norm) || !std::isfinite(err_v_norm) ||
    !std::isfinite(depth_m) || depth_m <= 1.0e-6f)
  {
    return target;
  }

  target.valid = true;
  target.x_m = depth_m * err_u_norm;
  target.y_m = depth_m * err_v_norm;
  target.z_m = depth_m;
  target.yaw_valid = yaw_valid && std::isfinite(yaw_err_rad);
  target.yaw_err_rad = target.yaw_valid ? yaw_err_rad : 0.0f;
  return target;
}

inline MetricLateralError lateralErrorFromRelativeTarget(const RelativeTarget3D & target)
{
  MetricLateralError error;
  if (!target.valid || !std::isfinite(target.x_m) || !std::isfinite(target.y_m)) {
    return error;
  }

  error.valid = true;
  error.x_m = target.x_m;
  error.y_m = target.y_m;
  error.norm_m = std::hypot(target.x_m, target.y_m);
  return error;
}

inline bool advanceRelativeTarget(
  RelativeTarget3D & target,
  float body_vx_mps,
  float body_vy_mps,
  float body_vz_mps,
  float body_yaw_rate_radps,
  float dt_s)
{
  if (!target.valid || !std::isfinite(body_vx_mps) || !std::isfinite(body_vy_mps) ||
    !std::isfinite(body_vz_mps) || !std::isfinite(body_yaw_rate_radps) ||
    !std::isfinite(dt_s) || dt_s <= 1.0e-6f)
  {
    return false;
  }

  const float yaw_delta_rad = body_yaw_rate_radps * dt_s;
  const float cos_yaw = std::cos(yaw_delta_rad);
  const float sin_yaw = std::sin(yaw_delta_rad);
  const float rotated_x_m = (cos_yaw * target.x_m) - (sin_yaw * target.y_m);
  const float rotated_y_m = (sin_yaw * target.x_m) + (cos_yaw * target.y_m);

  target.x_m = rotated_x_m;
  target.y_m = rotated_y_m;
  target.x_m += body_vy_mps * dt_s;
  target.y_m += body_vx_mps * dt_s;
  target.z_m = std::max(0.0f, target.z_m + (body_vz_mps * dt_s));
  if (target.yaw_valid) {
    target.yaw_err_rad += yaw_delta_rad;
    while (target.yaw_err_rad > static_cast<float>(M_PI)) {
      target.yaw_err_rad -= static_cast<float>(2.0 * M_PI);
    }
    while (target.yaw_err_rad < -static_cast<float>(M_PI)) {
      target.yaw_err_rad += static_cast<float>(2.0 * M_PI);
    }
  }
  target.valid = std::isfinite(target.x_m) && std::isfinite(target.y_m) &&
    std::isfinite(target.z_m);
  return target.valid;
}

inline bool isAligned(
  const MetricLateralError & lateral_error,
  bool yaw_valid,
  float yaw_err_rad,
  bool in_window,
  const AlignmentConfig & config)
{
  if (!lateral_error.valid) {
    return false;
  }

  if (!yaw_valid || !std::isfinite(yaw_err_rad)) {
    if (in_window) {
      return lateral_error.norm_m <= config.align_exit_lateral_m;
    }
    return lateral_error.norm_m <= config.align_enter_lateral_m;
  }

  const float yaw_abs = std::abs(yaw_err_rad);
  if (in_window) {
    return lateral_error.norm_m <= config.align_exit_lateral_m &&
           yaw_abs <= config.align_exit_yaw_rad;
  }
  return lateral_error.norm_m <= config.align_enter_lateral_m &&
         yaw_abs <= config.align_enter_yaw_rad;
}

inline ControllerPhase nextPhaseOnTargetLoss(ControllerPhase phase)
{
  switch (phase) {
    case ControllerPhase::TrackAlign:
    case ControllerPhase::HoldVerify:
    case ControllerPhase::DescendTrack:
    case ControllerPhase::Terminal:
      return ControllerPhase::HoldWait;
    case ControllerPhase::Ready:
    case ControllerPhase::HoldWait:
    case ControllerPhase::Land:
      return phase;
  }
  return phase;
}

inline float computeClosedLoopVelocity(
  float target_position,
  float current_position,
  float current_velocity,
  float kp,
  float damping,
  float max_speed)
{
  return clamp(
    (kp * (target_position - current_position)) - (damping * current_velocity),
    max_speed);
}

inline bool computeLimitedRate(
  float current_value,
  float previous_value,
  float dt_s,
  float dt_min_s,
  float dt_max_s,
  float max_abs_rate,
  float & out_rate)
{
  if (!std::isfinite(current_value) || !std::isfinite(previous_value) || !std::isfinite(dt_s) ||
    dt_s < dt_min_s || dt_s > dt_max_s || dt_s <= 1.0e-6f)
  {
    return false;
  }
  out_rate = clamp((current_value - previous_value) / dt_s, max_abs_rate);
  return true;
}

inline int updateConsecutiveConditionCount(int current_count, bool condition)
{
  return condition ? (std::max(0, current_count) + 1) : 0;
}

inline bool meetsConsecutiveConditionCount(int current_count, int required_count)
{
  if (required_count <= 1) {
    return current_count > 0;
  }
  return current_count >= required_count;
}

inline void applyBodyRateLimit(
  float & vx,
  float & vy,
  float & vz,
  float & yaw_rate,
  float last_vx,
  float last_vy,
  float last_vz,
  float last_yaw_rate,
  float dt_s,
  const CommandRateLimitConfig & config)
{
  const float safe_dt = std::max(dt_s, 1.0e-3f);
  const float max_dxy = std::max(0.0f, config.max_acc_xy_mps2) * safe_dt;
  if (max_dxy > 0.0f) {
    const float dvx = vx - last_vx;
    const float dvy = vy - last_vy;
    const float dxy = std::sqrt((dvx * dvx) + (dvy * dvy));
    if (dxy > max_dxy && dxy > 1.0e-6f) {
      const float scale = max_dxy / dxy;
      vx = last_vx + (dvx * scale);
      vy = last_vy + (dvy * scale);
    }
  }

  const float max_dz = std::max(0.0f, config.max_acc_z_mps2) * safe_dt;
  vz = clampDelta(vz, last_vz, max_dz);

  const float max_dyaw = std::max(0.0f, config.max_acc_yaw_radps2) * safe_dt;
  yaw_rate = clampDelta(yaw_rate, last_yaw_rate, max_dyaw);
}

}  // namespace uav_visual_landing
