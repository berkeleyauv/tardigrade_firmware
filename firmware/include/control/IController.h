#pragma once
//
// IController — the seam between the estimate and the desired wrench.
//
// Takes the vehicle's current VehicleState and produces a ControlOutput wrench.
// The controller OWNS its setpoints (a hold target, operator input, a mission
// command) rather than receiving them here, so they can be adjusted out of band
// — captured on arm, or set live over the parameter path. Everything downstream
// — the mixer — sees only the wrench.

#include "core/types.h"

namespace tardigrade {

class IController {
public:
    virtual ~IController() = default;

    // Compute the control wrench for this tick. `dt` is seconds since the last
    // call. Must not block.
    virtual void update(const VehicleState& state, float dt,
                        ControlOutput& out) = 0;

    // Drop accumulated integrator state. Called on disarm and whenever a hold is
    // (re)engaged, so old error cannot kick the vehicle.
    virtual void reset() = 0;
};

}  // namespace tardigrade
