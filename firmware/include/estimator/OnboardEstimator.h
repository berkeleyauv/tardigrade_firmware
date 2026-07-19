#pragma once
//
// OnboardEstimator — the hopcopter's IStateEstimator.
//
// Reads raw accel/gyro from an IImuSource and runs a Mahony complementary
// filter to produce an attitude quaternion. The gyro supplies short-term
// angular rate (accurate but drifting); the accelerometer supplies a long-term
// gravity reference that corrects the drift. Mahony blends them with a simple
// PI controller on the error between measured and estimated gravity, which is
// far cheaper than an EKF and entirely adequate for a hopcopter.
//
// KNOWN LIMITATION — YAW DRIFTS. Gravity constrains roll and pitch but says
// nothing about rotation *about* the gravity vector, so yaw is unobservable
// from accel+gyro alone and will slowly wander. Fixing it needs the ICM-20948's
// magnetometer (a heading reference) fused in as a third input. Roll and pitch,
// which is what a stabilization loop actually needs, are fully observable.
//
// See docs/estimator.md.

#include "core/types.h"
#include "drivers/IImuSource.h"
#include "estimator/IStateEstimator.h"
#include "estimator/VerticalEstimator.h"

namespace tardigrade {

class OnboardEstimator : public IStateEstimator {
public:
    // `source` must outlive this estimator. `freshness_timeout_us` is how long
    // the state may go without a fresh sample before healthy() turns false and
    // Safety's sensor-timeout failsafe trips.
    explicit OnboardEstimator(IImuSource& source,
                              uint32_t freshness_timeout_us = 50000);

    bool begin() override;
    bool update(uint32_t now_us) override;
    const VehicleState& state() const override { return state_; }
    bool healthy() const override;

    // Mahony gains. kp sets how hard gravity pulls the estimate back (higher =
    // faster correction but more vibration bleed-through); ki trims residual
    // gyro bias the driver's calibration missed. Tune on the bench.
    void setGains(float kp, float ki) { kp_ = kp; ki_ = ki; }

    // Optionally attach the vertical channel. Left unset, altitude and vertical
    // velocity stay 0 and altitude_valid stays false — attitude is unaffected,
    // so a hopcopter without ToF still flies on this estimator. The pointer is
    // borrowed and must outlive this object.
    void setVerticalEstimator(VerticalEstimator* vertical) { vertical_ = vertical; }

private:
    // Re-derive roll/pitch/yaw from the quaternion for logging/telemetry.
    void updateEulerFromQuaternion();

    IImuSource& source_;
    VerticalEstimator* vertical_ = nullptr;
    VehicleState state_;

    // Filter state: orientation quaternion and the integral error term.
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;
    Vec3 integral_error_;

    float kp_ = 2.0f;
    float ki_ = 0.005f;

    uint32_t freshness_timeout_us_;
    uint32_t last_valid_us_ = 0;
    uint32_t last_sample_us_ = 0;
    bool seeded_ = false;
    bool stale_ = false;
};

}  // namespace tardigrade
