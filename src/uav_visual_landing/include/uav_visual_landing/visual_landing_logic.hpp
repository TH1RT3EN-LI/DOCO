#pragma once

#include <algorithm>
#include <cmath>

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

struct AlignmentConfig
{
  float enter_error{0.03f};
  float exit_error{0.02f};
  float enter_yaw{0.12f};
  float exit_yaw{0.09f};
};

struct RangeConfig
{
  double timeout_s{0.15};
  float min_m{0.05f};
  float max_m{5.0f};
  float max_diff_m{0.35f};
};

struct RangeDecision
{
  bool valid{false};
  float effective_height_m{0.0f};
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

inline bool isAligned(
  float err_u_norm,
  float err_v_norm,
  float yaw_err_rad,
  bool in_window,
  const AlignmentConfig & config)
{
  const float err_norm = std::sqrt((err_u_norm * err_u_norm) + (err_v_norm * err_v_norm));
  const float yaw_abs = std::abs(yaw_err_rad);
  if (in_window) {
    return err_norm <= config.exit_error && yaw_abs <= config.exit_yaw;
  }
  return err_norm <= config.enter_error && yaw_abs <= config.enter_yaw;
}

inline RangeDecision evaluateRangeDecision(
  float odom_height_m,
  bool has_range,
  float range_height_m,
  double range_age_s,
  const RangeConfig & config)
{
  RangeDecision decision;
  decision.effective_height_m = odom_height_m;
  if (!has_range) {
    return decision;
  }
  if (!std::isfinite(range_height_m) || range_age_s < 0.0 || range_age_s > config.timeout_s) {
    return decision;
  }
  if (range_height_m < config.min_m || range_height_m > config.max_m) {
    return decision;
  }
  if (std::isfinite(odom_height_m) && std::abs(range_height_m - odom_height_m) > config.max_diff_m) {
    return decision;
  }
  decision.valid = true;
  decision.effective_height_m = range_height_m;
  return decision;
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
  return clamp((kp * (target_position - current_position)) - (damping * current_velocity), max_speed);
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
