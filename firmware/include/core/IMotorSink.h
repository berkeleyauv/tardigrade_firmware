#pragma once
//
// IMotorSink — where accepted motor commands land.
//
// Lives in core/ rather than beside either implementation because both Safety
// (which decides whether a command is allowed) and MotorManager (which carries
// it out) depend on it, and neither should depend on the other.

#include <stdint.h>

namespace tardigrade {

class IMotorSink {
public:
    virtual ~IMotorSink() = default;

    // `value` is normalized 0..1. Implementations must treat 0 as "stopped",
    // not "minimum spin".
    virtual void setMotor(uint8_t index, float value) = 0;

    // Immediately drive every output to its stopped state. Called by Safety on
    // disarm and on watchdog trip, so it must be safe to call at any time,
    // from any state, including before begin() has succeeded.
    virtual void stopAll() = 0;
};

}  // namespace tardigrade
