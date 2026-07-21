#include "control/Pid.h"

namespace tardigrade {

namespace {
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

float Pid::update(float error, float measurement_rate, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    const float p = kp_ * error;
    const float d = -kd_ * measurement_rate;  // derivative on measurement

    // Tentative integral, clamped so its CONTRIBUTION cannot exceed i_limit_.
    float candidate = integral_ + error * dt;
    float i_term = ki_ * candidate;

    const float unsaturated = p + i_term + d;

    // Freeze integration if the output is already saturated and this error
    // would push it further into the rail. Otherwise the integrator winds up
    // against a limit it cannot act on and overshoots on release.
    const bool saturated_high = unsaturated > out_limit_ && error > 0.0f;
    const bool saturated_low = unsaturated < -out_limit_ && error < 0.0f;
    if (!saturated_high && !saturated_low) {
        integral_ = candidate;
    }

    // Clamp the integral term itself as a second guard.
    i_term = clampf(ki_ * integral_, -i_limit_, i_limit_);

    return clampf(p + i_term + d, -out_limit_, out_limit_);
}

}  // namespace tardigrade
