#include "ugv_controller_emulator/controller_emulator_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugv_controller_emulator
{

uint8_t xor_checksum(const uint8_t * data, std::size_t len) noexcept
{
  uint8_t checksum = 0;
  for (std::size_t i = 0; i < len; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

int16_t decode_i16_be(uint8_t high, uint8_t low) noexcept
{
  const auto value =
    static_cast<uint16_t>((static_cast<uint16_t>(high) << 8U) | static_cast<uint16_t>(low));
  return static_cast<int16_t>(value);
}

std::pair<uint8_t, uint8_t> encode_i16_be(int16_t value) noexcept
{
  const auto encoded = static_cast<uint16_t>(value);
  return {
    static_cast<uint8_t>((encoded >> 8U) & 0xFFU),
    static_cast<uint8_t>(encoded & 0xFFU)};
}

int16_t saturate_i16(double value) noexcept
{
  if (!std::isfinite(value)) {
    return 0;
  }

  const auto rounded = static_cast<long long>(std::llround(value));
  if (rounded > std::numeric_limits<int16_t>::max()) {
    return std::numeric_limits<int16_t>::max();
  }
  if (rounded < std::numeric_limits<int16_t>::min()) {
    return std::numeric_limits<int16_t>::min();
  }
  return static_cast<int16_t>(rounded);
}

double apply_limit(double current, double target, double max_delta) noexcept
{
  if (target > current + max_delta) {
    return current + max_delta;
  }
  if (target < current - max_delta) {
    return current - max_delta;
  }
  return target;
}

TwistState clamp_twist(
  const TwistState & target,
  const EmulatorParams & params) noexcept
{
  TwistState clamped = target;
  clamped.vx = std::clamp(clamped.vx, -params.max_vx, params.max_vx);
  clamped.vy = std::clamp(clamped.vy, -params.max_vy, params.max_vy);
  clamped.wz = std::clamp(clamped.wz, -params.max_wz, params.max_wz);
  return clamped;
}

TwistState step_twist(
  const TwistState & current,
  const TwistState & target,
  const EmulatorParams & params,
  SteadyTimePoint now,
  SteadyTimePoint last_applied) noexcept
{
  TwistState effective_target = target;
  if (params.cmd_timeout > 0.0) {
    const auto age = std::chrono::duration<double>(now - target.stamp).count();
    if (age > params.cmd_timeout) {
      effective_target = TwistState{};
    }
  }

  effective_target = clamp_twist(effective_target, params);

  double dt = std::chrono::duration<double>(now - last_applied).count();
  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.0;
  }

  TwistState next = current;
  next.vx = apply_limit(current.vx, effective_target.vx, params.acc_lim_xy * dt);
  next.vy = apply_limit(current.vy, effective_target.vy, params.acc_lim_xy * dt);
  next.wz = apply_limit(current.wz, effective_target.wz, params.acc_lim_wz * dt);
  next.stamp = now;
  return next;
}

std::array<uint8_t, RECEIVE_DATA_SIZE> build_feedback_frame(
  const EmulatorParams & params,
  const TwistState & applied,
  const std::optional<TwistState> & odom,
  SteadyTimePoint now) noexcept
{
  const TwistState * source = &applied;
  if (odom.has_value()) {
    const auto age = std::chrono::duration<double>(now - odom->stamp).count();
    if (age <= 0.5) {
      source = &odom.value();
    }
  }

  std::array<uint8_t, RECEIVE_DATA_SIZE> frame{};
  frame[0] = FRAME_HEADER;
  frame[1] = 0;

  const auto vx_mmps = saturate_i16(source->vx * 1000.0);
  const auto vy_mmps = saturate_i16(source->vy * 1000.0);
  const auto wz_mradps = saturate_i16(source->wz * 1000.0);
  const auto [vx_hi, vx_lo] = encode_i16_be(vx_mmps);
  const auto [vy_hi, vy_lo] = encode_i16_be(vy_mmps);
  const auto [wz_hi, wz_lo] = encode_i16_be(wz_mradps);
  frame[2] = vx_hi;
  frame[3] = vx_lo;
  frame[4] = vy_hi;
  frame[5] = vy_lo;
  frame[6] = wz_hi;
  frame[7] = wz_lo;

  const auto [ax_hi, ax_lo] = encode_i16_be(0);
  const auto [ay_hi, ay_lo] = encode_i16_be(0);
  const auto [az_hi, az_lo] = encode_i16_be(saturate_i16(G_REF * ACCEL_RATIO));
  const auto [gx_hi, gx_lo] = encode_i16_be(0);
  const auto [gy_hi, gy_lo] = encode_i16_be(0);
  const auto [gz_hi, gz_lo] = encode_i16_be(saturate_i16(source->wz / GYROSCOPE_RATIO));

  frame[8] = ax_hi;
  frame[9] = ax_lo;
  frame[10] = ay_hi;
  frame[11] = ay_lo;
  frame[12] = az_hi;
  frame[13] = az_lo;
  frame[14] = gx_hi;
  frame[15] = gx_lo;
  frame[16] = gy_hi;
  frame[17] = gy_lo;
  frame[18] = gz_hi;
  frame[19] = gz_lo;

  const auto [v_hi, v_lo] = encode_i16_be(saturate_i16(params.battery_voltage * 1000.0));
  frame[20] = v_hi;
  frame[21] = v_lo;
  frame[22] = xor_checksum(frame.data(), 22);
  frame[23] = FRAME_TAIL;
  return frame;
}

bool parse_command_frame(
  const std::array<uint8_t, SEND_DATA_SIZE> & frame,
  SteadyTimePoint stamp,
  TwistState * out) noexcept
{
  if (out == nullptr) {
    return false;
  }
  if (frame[0] != FRAME_HEADER || frame[SEND_DATA_SIZE - 1] != FRAME_TAIL) {
    return false;
  }
  if (xor_checksum(frame.data(), 9) != frame[9]) {
    return false;
  }

  out->vx = static_cast<double>(decode_i16_be(frame[3], frame[4])) * 0.001;
  out->vy = static_cast<double>(decode_i16_be(frame[5], frame[6])) * 0.001;
  out->wz = static_cast<double>(decode_i16_be(frame[7], frame[8])) * 0.001;
  out->stamp = stamp;
  return true;
}

std::optional<TwistState> CommandParser::push_byte(
  uint8_t byte,
  SteadyTimePoint stamp) noexcept
{
  if (rx_count_ == 0U) {
    if (byte != FRAME_HEADER) {
      return std::nullopt;
    }
    frame_buf_[0] = byte;
    rx_count_ = 1U;
    return std::nullopt;
  }

  frame_buf_[rx_count_] = byte;
  ++rx_count_;

  if (rx_count_ < SEND_DATA_SIZE) {
    return std::nullopt;
  }

  rx_count_ = 0U;

  TwistState decoded;
  if (!parse_command_frame(frame_buf_, stamp, &decoded)) {
    return std::nullopt;
  }
  return decoded;
}

void CommandParser::reset() noexcept
{
  rx_count_ = 0U;
}

}
