#pragma once
//
// EscPwm — one ESC channel driven by the ESP32's LEDC peripheral.
//
// An ESC (Electronic Speed Controller) sits between this board and a brushless
// motor. The ESP32 cannot drive such a motor directly — brushless motors need
// three-phase commutation at motor currents — so the ESC does that work and
// this firmware only supplies a low-power timing signal.
//
// The signal is the standard hobby-servo convention: a pulse repeated at a
// fixed frame rate, where 1000 us means stopped and 2000 us means full. LEDC
// generates it in hardware, so the pulse train is jitter-free and unaffected by
// whatever the CPU is doing.
//
// That last property is a double edge and is the reason HardwareWatchdog
// exists: if the CPU hangs, LEDC keeps emitting the last commanded pulse
// forever. Nothing in software can stop a motor once the software has stopped.

#include <stdint.h>

namespace tardigrade {

// Standard ESC pulse bounds, microseconds.
inline constexpr uint16_t kEscMinUs = 1000;
inline constexpr uint16_t kEscMaxUs = 2000;

// Marine bidirectional bounds (Blue Robotics T200 / Basic ESC and similar),
// where the STOP point is the middle of the range rather than the bottom.
inline constexpr uint16_t kThrusterMinUs = 1100;  // full reverse
inline constexpr uint16_t kThrusterStopUs = 1500;
inline constexpr uint16_t kThrusterMaxUs = 1900;  // full forward

// Which convention an output follows.
//
// This exists because "stopped" is a DIFFERENT PULSE WIDTH on the two vehicle
// types, and stopped is precisely what Safety writes on disarm, on watchdog
// trip, and on emergency stop. Driving a marine ESC with the unidirectional
// mapping would turn every one of those into full reverse thrust — the
// failsafe becoming the failure. The mode is therefore explicit at
// construction rather than inferred or defaulted.
enum class EscMode : uint8_t {
    Unidirectional,  // 1000 stop .. 2000 full. Propellers; negative rejected.
    Bidirectional,   // 1100 reverse .. 1500 stop .. 1900 forward. Thrusters.
};

// Frame rate. 50 Hz is the universally-compatible choice and the right default
// for unknown hardware. Most modern ESCs accept 400 Hz and benefit from it —
// raise this only once you know the parts, because an ESC fed a rate it does
// not support behaves erratically rather than failing cleanly.
inline constexpr uint32_t kEscFrameHz = 50;

// 16 bits over a 20 ms frame resolves ~0.3 us per step, far finer than any ESC
// distinguishes.
inline constexpr uint8_t kEscPwmBits = 16;

class EscPwm {
public:
    // `channel` must be unique per ESC; the ESP32 has 16 LEDC channels.
    // Immediately parks the output at this mode's stop pulse — an ESC must see
    // a valid idle at power-up both to arm and, more importantly, to not spin.
    bool begin(uint8_t pin, uint8_t channel, EscMode mode);

    // Raw pulse width, clamped to the active mode's bounds.
    void writeMicroseconds(uint16_t us);

    // Signed thrust, -1..+1, where 0 is stopped in BOTH modes.
    // In Unidirectional mode negatives are treated as 0, never as reverse.
    // NaN maps to 0.
    void writeNormalized(float value);

    // Park at this mode's stop pulse. Safe to call before begin().
    void stop();

    uint16_t lastMicroseconds() const { return last_us_; }
    uint16_t stopMicroseconds() const;

private:
    uint16_t minUs() const;
    uint16_t maxUs() const;

    uint8_t channel_ = 0;
    EscMode mode_ = EscMode::Unidirectional;
    uint16_t last_us_ = kEscMinUs;
    bool ready_ = false;
};

}  // namespace tardigrade
