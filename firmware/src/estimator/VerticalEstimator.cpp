#include "estimator/VerticalEstimator.h"

#include <math.h>

namespace tardigrade {

namespace {
// Two sensors that disagree by more than this are not both looking at the
// ground — one is over a vent, a step, or a surface it cannot read. Better to
// coast on the accelerometer than to average a good reading with a lie.
constexpr float kAgreementTolerance_m = 0.15f;
}  // namespace

VerticalEstimator::VerticalEstimator(IRangeSensor* const* sensors, uint8_t count)
    : sensors_(sensors),
      count_(count > kMaxRangeSensors ? kMaxRangeSensors : count) {}

bool VerticalEstimator::begin() {
    bool any = false;
    for (uint8_t i = 0; i < count_; ++i) {
        if (sensors_[i] != nullptr && sensors_[i]->begin()) {
            any = true;
        }
    }
    initialized_ = any;
    return any;
}

void VerticalEstimator::setNoise(float sigma_accel, float sigma_range) {
    if (sigma_accel > 0.0f) sigma_accel_ = sigma_accel;
    if (sigma_range > 0.0f) sigma_range_ = sigma_range;
}

bool VerticalEstimator::fuseRange(const Quat& orientation, float cos_tilt,
                                  float& height_out) {
    (void)orientation;

    float heights[kMaxRangeSensors];
    uint8_t n = 0;

    for (uint8_t i = 0; i < count_; ++i) {
        IRangeSensor* s = sensors_[i];
        if (s == nullptr) {
            continue;
        }
        RangeData r;
        if (!s->read(r) || !r.valid) {
            continue;
        }
        // A reading at the sensor's ceiling is a saturation, not a distance.
        if (r.range_m >= s->maxRange()) {
            continue;
        }
        heights[n++] = r.range_m * cos_tilt;  // boresight range -> height
    }

    if (n == 0) {
        return false;
    }
    if (n == 1) {
        height_out = heights[0];
        return true;
    }

    // Two readings: trust them only if they corroborate each other.
    const float spread = fabsf(heights[0] - heights[1]);
    if (spread > kAgreementTolerance_m) {
        return false;
    }
    height_out = 0.5f * (heights[0] + heights[1]);
    return true;
}

bool VerticalEstimator::update(uint32_t now_us, const Quat& orientation,
                               const Vec3& accel_body, uint32_t sample_us) {
    if (!initialized_) {
        return false;
    }

    // Gravity direction expressed in the body frame — the same vector Mahony
    // builds. Its z component is cos(tilt); the whole vector projects body
    // acceleration onto the world vertical.
    const float q0 = orientation.w, q1 = orientation.x;
    const float q2 = orientation.y, q3 = orientation.z;
    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q0 * q1 + q2 * q3);
    const float vz_dir = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // Accelerometer measures specific force: at rest it reads +g along "up",
    // so removing gravity leaves true vertical acceleration, + is up.
    const float accel_up =
        accel_body.x * vx + accel_body.y * vy + accel_body.z * vz_dir -
        kGravity_mps2;

    const float cos_tilt = vz_dir;

    if (!seeded_) {
        last_sample_us_ = sample_us;
        seeded_ = true;
        float h;
        if (cos_tilt >= min_cos_tilt_ && fuseRange(orientation, cos_tilt, h)) {
            z_ = h;
            vz_ = 0.0f;
            last_range_us_ = now_us;
            stale_ = false;
            return true;
        }
        return false;
    }

    const uint32_t dt_us = sample_us - last_sample_us_;
    last_sample_us_ = sample_us;
    const float dt = dt_us * 1e-6f;
    if (dt <= 0.0f || dt > 0.5f) {
        return false;  // implausible gap; don't corrupt the filter
    }

    // --- Predict -----------------------------------------------------------
    z_ += vz_ * dt + 0.5f * accel_up * dt * dt;
    vz_ += accel_up * dt;

    // Q from acceleration uncertainty driven through the motion model:
    // G = [dt^2/2, dt]', Q = G G' * sigma_accel^2
    const float sa2 = sigma_accel_ * sigma_accel_;
    const float dt2 = dt * dt;
    const float q00 = 0.25f * dt2 * dt2 * sa2;
    const float q01 = 0.5f * dt2 * dt * sa2;
    const float q11 = dt2 * sa2;

    p00_ += dt * (2.0f * p01_ + dt * p11_) + q00;
    p01_ += dt * p11_ + q01;
    p11_ += q11;

    // --- Correct -----------------------------------------------------------
    float height = 0.0f;
    const bool have_range =
        cos_tilt >= min_cos_tilt_ && fuseRange(orientation, cos_tilt, height);

    if (have_range) {
        const float r = sigma_range_ * sigma_range_;
        const float s = p00_ + r;
        if (s > 0.0f) {
            const float k0 = p00_ / s;
            const float k1 = p01_ / s;
            const float innovation = height - z_;

            z_ += k0 * innovation;
            vz_ += k1 * innovation;

            // P = (I - K H) P, with H = [1 0]
            const float new_p00 = p00_ * (1.0f - k0);
            const float new_p01 = p01_ * (1.0f - k0);
            const float new_p11 = p11_ - k1 * p01_;
            p00_ = new_p00;
            p01_ = new_p01;
            p11_ = new_p11;
        }
        last_range_us_ = now_us;
    }

    stale_ = (now_us - last_range_us_) > range_timeout_us_;
    return have_range;
}

}  // namespace tardigrade
