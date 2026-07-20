#pragma once
//
// MotorManager — owns the ESC outputs and is the only thing that writes them.
//
// Per docs/architecture.md this module owns arming, disarming, idle throttle
// and the ESC update. It implements IMotorSink, so Safety can stop it without
// knowing anything about ESCs, PWM or pin assignments.
//
// It deliberately holds NO opinion about whether a command should be allowed —
// that decision belongs to Safety, in one place. MotorManager only carries out
// what it is given, and guarantees that "stopped" is always reachable.

#include "core/IMotorSink.h"
#include "core/types.h"
#include "drivers/EscPwm.h"

namespace tardigrade {

class MotorManager : public IMotorSink {
public:
    // `pins` is borrowed for the duration of begin() only. `count` is clamped
    // to kMaxMotors. `mode` must match the physical ESCs — see EscMode; the
    // wrong one puts full reverse thrust on every stop command.
    MotorManager(const uint8_t* pins, uint8_t count, EscMode mode);

    // Attaches every channel and parks it at minimum throttle. ESCs expect to
    // see minimum for a moment at power-up before they will respond to
    // anything; begin() holds that state so they arm predictably.
    bool begin();

    // IMotorSink. `value` is signed -1..+1; 0 is a full stop, not idle.
    void setMotor(uint8_t index, float value) override;
    void stopAll() override;

    uint8_t count() const { return count_; }

    // Last commanded pulse width for telemetry / bench debugging.
    uint16_t pulseUs(uint8_t index) const;

private:
    EscPwm esc_[kMaxMotors];
    uint8_t pins_[kMaxMotors] = {};
    EscMode mode_ = EscMode::Unidirectional;
    uint8_t count_ = 0;
    bool ready_ = false;
};

}  // namespace tardigrade
