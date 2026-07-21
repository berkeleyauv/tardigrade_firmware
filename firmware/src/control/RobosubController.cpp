#include "control/RobosubController.h"

#include <math.h>

#include "control/Parameters.h"

namespace tardigrade {

namespace {
// Shortest signed angular distance, wrapped to [-pi, pi]. Heading hold needs
// this or crossing +/-180 deg would command a near-full-circle turn.
inline float wrapPi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

RobosubController::RobosubController() {
    // Conservative starting gains — placeholders to be tuned in water. Integral
    // and output limits kept low so a bad gain during bring-up cannot slam a
    // thruster to full.
    depth_.setGains(2.0f, 0.10f, 1.5f);
    depth_.setIntegralLimit(0.25f);
    depth_.setOutputLimit(1.0f);

    yaw_.setGains(1.2f, 0.02f, 0.4f);
    yaw_.setIntegralLimit(0.15f);
    yaw_.setOutputLimit(1.0f);

    roll_.setGains(0.8f, 0.0f, 0.25f);
    roll_.setOutputLimit(0.6f);   // leveling should never dominate depth/heading
    pitch_.setGains(0.8f, 0.0f, 0.25f);
    pitch_.setOutputLimit(0.6f);
}

void RobosubController::captureHold(const VehicleState& state) {
    depth_setpoint_ = state.altitude_m;
    heading_setpoint_ = state.yaw_rad;
    reset();
}

void RobosubController::update(const VehicleState& state, float dt,
                               ControlOutput& out) {
    out.timestamp_us = state.timestamp_us;

    // Depth: error in world z (up positive). Derivative term is the estimator's
    // vertical velocity — no differentiation of a noisy depth signal.
    const float ez = depth_setpoint_ - state.altitude_m;
    out.force.z = depth_.update(ez, state.vertical_velocity_mps, dt);
    out.force.x = 0.0f;  // station-keeping: no surge command
    out.force.y = 0.0f;  // no sway command

    // Heading: wrapped error, damped by the yaw rate straight from the gyro.
    const float eyaw = wrapPi(heading_setpoint_ - state.yaw_rad);
    out.torque.z = yaw_.update(eyaw, state.angular_velocity.z, dt);

    // Leveling: hold roll/pitch at zero, damped by body rates. Keeps body-heave
    // aligned with world-vertical for depth hold.
    out.torque.x = roll_.update(0.0f - state.roll_rad, state.angular_velocity.x, dt);
    out.torque.y = pitch_.update(0.0f - state.pitch_rad, state.angular_velocity.y, dt);

    // Bring-up authority cap, applied uniformly so it scales effort without
    // distorting the mix balance between axes.
    out.force.x *= authority_;
    out.force.y *= authority_;
    out.force.z *= authority_;
    out.torque.x *= authority_;
    out.torque.y *= authority_;
    out.torque.z *= authority_;
}

void RobosubController::reset() {
    depth_.reset();
    yaw_.reset();
    roll_.reset();
    pitch_.reset();
}

Pid* RobosubController::channel(uint16_t ch) {
    switch (ch) {
        case param::kDepth: return &depth_;
        case param::kYaw:   return &yaw_;
        case param::kRoll:  return &roll_;
        case param::kPitch: return &pitch_;
        default:            return nullptr;
    }
}

const Pid* RobosubController::channel(uint16_t ch) const {
    return const_cast<RobosubController*>(this)->channel(ch);
}

bool RobosubController::setParameter(uint16_t id, float value) {
    if (id == param::kAuthority) {
        authority_ = clampf(value, 0.0f, 1.0f);
        return true;
    }
    if (id == param::kDepthSetpoint) {
        depth_setpoint_ = value;
        return true;
    }
    if (id == param::kHeadingSetpoint) {
        heading_setpoint_ = value;
        return true;
    }

    Pid* p = channel(id & 0xF0);
    if (p == nullptr) {
        return false;
    }
    switch (id & 0x0F) {
        case param::kKp: p->setKp(value); return true;
        case param::kKi: p->setKi(value); return true;
        case param::kKd: p->setKd(value); return true;
        default:         return false;
    }
}

uint16_t RobosubController::parameterCount() const {
    return 15;  // 4 channels x 3 gains + authority + 2 setpoints
}

bool RobosubController::parameterAt(uint16_t index, uint16_t& id,
                                    float& value) const {
    if (index >= parameterCount()) {
        return false;
    }
    if (index < 12) {
        const uint16_t channels[4] = {param::kDepth, param::kYaw,
                                      param::kRoll, param::kPitch};
        const uint16_t ch = channels[index / 3];
        const uint16_t term = index % 3;
        const Pid* p = channel(ch);
        id = ch | term;
        value = (term == param::kKp) ? p->kp()
              : (term == param::kKi) ? p->ki() : p->kd();
        return true;
    }
    switch (index) {
        case 12: id = param::kAuthority;       value = authority_;       return true;
        case 13: id = param::kDepthSetpoint;   value = depth_setpoint_;   return true;
        case 14: id = param::kHeadingSetpoint; value = heading_setpoint_; return true;
        default: return false;
    }
}

}  // namespace tardigrade
