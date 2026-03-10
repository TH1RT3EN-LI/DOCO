#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

namespace ugv_controller_emulator
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTimePoint = SteadyClock::time_point;

inline constexpr uint8_t FRAME_HEADER = 0x7B;
inline constexpr uint8_t FRAME_TAIL = 0x7D;
inline constexpr std::size_t SEND_DATA_SIZE = 11;
inline constexpr std::size_t RECEIVE_DATA_SIZE = 24;

inline constexpr double GYROSCOPE_RATIO = 0.00026644;
inline constexpr double ACCEL_RATIO = 1671.84;
inline constexpr double G_REF = 9.80665;

struct TwistState
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  SteadyTimePoint stamp{};
};

struct EmulatorParams
{
  double cmd_timeout{0.5};
  double cmd_pub_hz{50.0};
  double feedback_hz{200.0};
  double max_vx{0.6};
  double max_vy{0.6};
  double max_wz{2.0};
  double acc_lim_xy{2.0};
  double acc_lim_wz{3.2};
  double battery_voltage{24.0};
};

uint8_t xor_checksum(const uint8_t * data, std::size_t len) noexcept;
int16_t decode_i16_be(uint8_t high, uint8_t low) noexcept;
std::pair<uint8_t, uint8_t> encode_i16_be(int16_t value) noexcept;
int16_t saturate_i16(double value) noexcept;
double apply_limit(double current, double target, double max_delta) noexcept;
TwistState clamp_twist(
  const TwistState & target,
  const EmulatorParams & params) noexcept;
TwistState step_twist(
  const TwistState & current,
  const TwistState & target,
  const EmulatorParams & params,
  SteadyTimePoint now,
  SteadyTimePoint last_applied) noexcept;
std::array<uint8_t, RECEIVE_DATA_SIZE> build_feedback_frame(
  const EmulatorParams & params,
  const TwistState & applied,
  const std::optional<TwistState> & odom,
  SteadyTimePoint now) noexcept;
bool parse_command_frame(
  const std::array<uint8_t, SEND_DATA_SIZE> & frame,
  SteadyTimePoint stamp,
  TwistState * out) noexcept;

class CommandParser
{
public:
  std::optional<TwistState> push_byte(uint8_t byte, SteadyTimePoint stamp) noexcept;
  void reset() noexcept;

private:
  std::array<uint8_t, SEND_DATA_SIZE> frame_buf_{};
  std::size_t rx_count_{0};
};

}
