#pragma once
//
// IMixer — maps a body-frame ControlOutput wrench onto per-motor commands.
//
// This is where a vehicle's geometry lives: how each thruster/propeller
// contributes to each axis. The robosub mixer encodes the 8-thruster layout;
// a quadcopter mixer (later) encodes its four. The controller above and the
// MotorManager below both stay geometry-agnostic.

#include "core/types.h"

namespace tardigrade {

class IMixer {
public:
    virtual ~IMixer() = default;

    // Fill `out.value[0..count)` in normalized -1..+1, and set out.count.
    virtual void mix(const ControlOutput& wrench, MotorOutput& out) = 0;
};

}  // namespace tardigrade
