#pragma once
//
// RobosubController — depth hold, heading hold, and roll/pitch leveling.
//
// Four independent PID channels producing a body-frame wrench:
//
//   depth  -> force.z (heave)   holds a target z
//   yaw    -> torque.z          holds a target heading
//   roll   -> torque.x          holds level (target 0)
//   pitch  -> torque.y          holds level (target 0)
//
// Surge and sway are left at zero: this is a station-keeping controller, not a
// waypoint follower. Add them when there is a position/velocity command to
// track.
//
// It OWNS its hold target and its gains, and exposes both over IParameterSink so
// the ground station can tune live and step the setpoint. The target is either
// captured on arm (captureHold) or set over the parameter path.
//
// FRAME NOTE. Depth error is a WORLD-vertical quantity but the heave force is
// applied in the BODY frame. Holding roll and pitch near zero is what keeps the
// two aligned, so the leveling channels are not cosmetic — they are what makes
// depth hold behave. VehicleState.altitude_m is ENU z (up positive), so a depth
// target of 2 m below the surface is z = -2.0.

#include "control/IController.h"
#include "control/Pid.h"
#include "core/IParameterSink.h"
#include "core/types.h"

namespace tardigrade {

class RobosubController : public IController, public IParameterSink {
public:
    RobosubController();

    void update(const VehicleState& state, float dt, ControlOutput& out) override;
    void reset() override;

    // Snapshot the current depth and heading as the hold target, level. Called
    // on the arm rising edge — this is the "put it down, arm, it holds here"
    // workflow.
    void captureHold(const VehicleState& state);

    // Autonomous output scale, 0..1. A bring-up safety cap: keeps a bad gain
    // from slamming a thruster to full while tuning. Raise toward 1 with
    // confidence.
    float authority() const { return authority_; }

    // IParameterSink — live gain/authority/setpoint tuning. See Parameters.h.
    bool setParameter(uint16_t id, float value) override;
    uint16_t parameterCount() const override;
    bool parameterAt(uint16_t index, uint16_t& id, float& value) const override;

private:
    Pid* channel(uint16_t ch);              // maps a param channel to its Pid
    const Pid* channel(uint16_t ch) const;

    Pid depth_;
    Pid yaw_;
    Pid roll_;
    Pid pitch_;
    float depth_setpoint_ = 0.0f;    // ENU z, metres
    float heading_setpoint_ = 0.0f;  // radians
    float authority_ = 0.35f;
};

}  // namespace tardigrade
