#pragma once
//
// Parameter ids for the robosub controller — the shared vocabulary between
// firmware and ground station. These numbers are a WIRE CONTRACT: the dashboard
// hardcodes the same values, so never renumber an existing id; only append.
//
// Layout: gains are (channel << 4) | term, term 0=Kp 1=Ki 2=Kd. Non-gain
// parameters live at 0x40+.

#include <stdint.h>

namespace tardigrade {
namespace param {

// channels
inline constexpr uint16_t kDepth = 0x00;
inline constexpr uint16_t kYaw   = 0x10;
inline constexpr uint16_t kRoll  = 0x20;
inline constexpr uint16_t kPitch = 0x30;

// terms (added to a channel)
inline constexpr uint16_t kKp = 0;
inline constexpr uint16_t kKi = 1;
inline constexpr uint16_t kKd = 2;

// non-gain parameters
inline constexpr uint16_t kAuthority       = 0x40;  // autonomous output scale 0..1
inline constexpr uint16_t kDepthSetpoint   = 0x50;  // hold target, ENU z metres
inline constexpr uint16_t kHeadingSetpoint = 0x51;  // hold target, radians

}  // namespace param
}  // namespace tardigrade
