#include "estimator/ExternalEstimator.h"

#include <math.h>

namespace tardigrade {

ExternalEstimator::ExternalEstimator(const JetsonLink& link,
                                     uint32_t freshness_timeout_us)
    : link_(link), freshness_timeout_us_(freshness_timeout_us) {}

bool ExternalEstimator::begin() {
    // The transport (serial + parser) is owned and started elsewhere; there is
    // nothing to open here. Report ready so the boot sequence proceeds — health
    // then depends entirely on pose actually arriving.
    return true;
}

bool ExternalEstimator::update(uint32_t now_us) {
    const PoseSample& p = link_.latest();

    if (!p.valid) {
        // No pose has ever arrived. Nothing to publish; healthy() stays false.
        stale_ = true;
        return false;
    }

    // Age is measured against the ESP's own receipt time, never a Jetson
    // timestamp — the two clocks are unrelated.
    stale_ = (now_us - p.received_us) > freshness_timeout_us_;

    // Only advance the state on a genuinely new sample. Re-copying the same
    // pose every tick would be harmless but pointless; the freshness check
    // above already handles "no new data".
    const bool fresh_sample = !seq_started_ || (p.seq != last_seq_);
    if (!fresh_sample) {
        return false;
    }
    seq_started_ = true;
    last_seq_ = p.seq;

    state_.timestamp_us = p.received_us;
    state_.orientation = p.orientation;
    state_.angular_velocity = p.angular_velocity;

    // ENU world frame: z is up, so it maps straight onto altitude, and the z
    // component of linear velocity onto vertical velocity. (VehicleState has no
    // x/y position fields yet; the Pose frame carries them so the wire format
    // need not change when it does.)
    state_.altitude_m = p.position.z;
    state_.vertical_velocity_mps = p.linear_velocity.z;

    fillEulerFromQuaternion();

    state_.valid = !stale_;
    state_.altitude_valid = !stale_;
    return true;
}

void ExternalEstimator::fillEulerFromQuaternion() {
    const float w = state_.orientation.w, x = state_.orientation.x;
    const float y = state_.orientation.y, z = state_.orientation.z;

    state_.roll_rad = atan2f(2.0f * (w * x + y * z),
                             1.0f - 2.0f * (x * x + y * y));

    float sp = 2.0f * (w * y - z * x);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    state_.pitch_rad = asinf(sp);

    state_.yaw_rad = atan2f(2.0f * (w * z + x * y),
                            1.0f - 2.0f * (y * y + z * z));
}

bool ExternalEstimator::healthy() const {
    return state_.valid && !stale_;
}

}  // namespace tardigrade
