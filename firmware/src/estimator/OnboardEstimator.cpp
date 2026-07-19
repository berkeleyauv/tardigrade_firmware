#include "estimator/OnboardEstimator.h"

#include <math.h>

namespace tardigrade {

OnboardEstimator::OnboardEstimator(IImuSource& source,
                                   uint32_t freshness_timeout_us)
    : source_(source), freshness_timeout_us_(freshness_timeout_us) {}

bool OnboardEstimator::begin() {
    const bool imu_ok = source_.begin();
    if (vertical_ != nullptr) {
        // A failed range sensor is not fatal: attitude still flies, altitude
        // simply stays invalid and any altitude-hold mode must refuse to arm.
        vertical_->begin();
    }
    return imu_ok;
}

bool OnboardEstimator::update(uint32_t now_us) {
    ImuData imu;
    if (!source_.read(imu) || !imu.valid) {
        // No fresh sample this tick. Keep the last state, but note how long it
        // has been stale — past the timeout, healthy() trips Safety.
        // Never block, never guess.
        stale_ = (now_us - last_valid_us_) > freshness_timeout_us_;
        return false;
    }

    const Vec3& a = imu.accel;
    const float a_norm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

    // Seed from the first sample so the filter starts at roughly the right
    // attitude instead of spending seconds converging from identity.
    if (!seeded_) {
        if (a_norm > 0.0f) {
            const float roll = atan2f(a.y, a.z);
            const float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
            const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
            const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
            q0_ = cr * cp;
            q1_ = sr * cp;
            q2_ = cr * sp;
            q3_ = -sr * sp;
        }
        seeded_ = true;
        last_sample_us_ = imu.timestamp_us;
        last_valid_us_ = now_us;
        stale_ = false;
        updateEulerFromQuaternion();
        state_.timestamp_us = imu.timestamp_us;
        state_.angular_velocity = imu.gyro;
        state_.valid = true;
        return true;
    }

    // Integration step from the sensor's own timestamps, not the loop rate —
    // micros() wraps every ~71 min, but unsigned subtraction handles that.
    const uint32_t dt_us = imu.timestamp_us - last_sample_us_;
    last_sample_us_ = imu.timestamp_us;
    const float dt = dt_us * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
        // Implausible gap (first tick after a stall, or a wrap artifact).
        // Skip the integration rather than corrupt the quaternion.
        return false;
    }

    float gx = imu.gyro.x, gy = imu.gyro.y, gz = imu.gyro.z;

    // Only apply the gravity correction when the accelerometer is actually
    // measuring gravity. Under heavy linear acceleration ‖a‖ departs from g and
    // the vector is a bad attitude reference, so we coast on the gyro instead.
    if (a_norm > 0.0f) {
        const float ax = a.x / a_norm;
        const float ay = a.y / a_norm;
        const float az = a.z / a_norm;

        // Gravity direction implied by the current quaternion estimate.
        const float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
        const float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
        const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

        // Error is the cross product between measured and estimated gravity.
        const float ex = ay * vz - az * vy;
        const float ey = az * vx - ax * vz;
        const float ez = ax * vy - ay * vx;

        if (ki_ > 0.0f) {
            integral_error_.x += ki_ * ex * dt;
            integral_error_.y += ki_ * ey * dt;
            integral_error_.z += ki_ * ez * dt;
            gx += integral_error_.x;
            gy += integral_error_.y;
            gz += integral_error_.z;
        }

        gx += kp_ * ex;
        gy += kp_ * ey;
        gz += kp_ * ez;
    }

    // Integrate the quaternion with the corrected rate.
    const float half_dt = 0.5f * dt;
    const float dq0 = (-q1_ * gx - q2_ * gy - q3_ * gz) * half_dt;
    const float dq1 = (q0_ * gx + q2_ * gz - q3_ * gy) * half_dt;
    const float dq2 = (q0_ * gy - q1_ * gz + q3_ * gx) * half_dt;
    const float dq3 = (q0_ * gz + q1_ * gy - q2_ * gx) * half_dt;

    q0_ += dq0;
    q1_ += dq1;
    q2_ += dq2;
    q3_ += dq3;

    const float q_norm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (q_norm > 0.0f) {
        const float inv = 1.0f / q_norm;
        q0_ *= inv;
        q1_ *= inv;
        q2_ *= inv;
        q3_ *= inv;
    }

    state_.timestamp_us = imu.timestamp_us;
    state_.angular_velocity = imu.gyro;  // raw body rates, not the corrected ones
    updateEulerFromQuaternion();
    state_.valid = true;
    last_valid_us_ = now_us;
    stale_ = false;

    // Vertical channel runs AFTER attitude: it needs this tick's orientation to
    // remove tilt from the range reading and to project accel onto the world
    // vertical. The dependency never runs the other way.
    if (vertical_ != nullptr) {
        vertical_->update(now_us, state_.orientation, imu.accel,
                          imu.timestamp_us);
        state_.altitude_m = vertical_->altitude();
        state_.vertical_velocity_mps = vertical_->verticalVelocity();
        state_.altitude_valid = vertical_->healthy();
    }

    return true;
}

void OnboardEstimator::updateEulerFromQuaternion() {
    state_.orientation.w = q0_;
    state_.orientation.x = q1_;
    state_.orientation.y = q2_;
    state_.orientation.z = q3_;

    state_.roll_rad = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                             1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));

    // Clamp before asin: rounding can push the argument just past +/-1.
    float sin_pitch = 2.0f * (q0_ * q2_ - q3_ * q1_);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    state_.pitch_rad = asinf(sin_pitch);

    state_.yaw_rad = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                            1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_));
}

bool OnboardEstimator::healthy() const {
    return state_.valid && !stale_ && source_.healthy();
}

}  // namespace tardigrade
