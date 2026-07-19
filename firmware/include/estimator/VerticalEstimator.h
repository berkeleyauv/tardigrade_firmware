#pragma once
//
// VerticalEstimator — altitude and vertical velocity for the hopcopter.
//
// A 2-state Kalman filter (altitude, vertical velocity) driven by the
// accelerometer and corrected by downward ToF readings. The accelerometer is
// fast but its double integration drifts without bound; the ToF is absolute but
// slow, noisy, and prone to dropouts. Each covers the other's weakness.
//
// Why a KF here when attitude uses Mahony: this problem genuinely IS coupled —
// a range measurement corrects altitude AND, through the covariance, velocity.
// A 2x2 filter costs a handful of multiplies with no matrix inversion, so the
// argument against a 7-state quaternion EKF simply doesn't apply.
//
// DEPENDS ON ATTITUDE, one direction only. A ToF measures along its boresight,
// so tilt must be divided out before the reading means "height" — and the
// world-frame vertical acceleration must be projected out of body-frame accel.
// Both come from the attitude estimate. Nothing here feeds back into it.
//
// See docs/estimator.md.

#include "core/types.h"
#include "drivers/IRangeSensor.h"

namespace tardigrade {

class VerticalEstimator {
public:
    // Sensors are borrowed, not owned, and must outlive this object. Two
    // downward units let a disagreement be detected; one leaves you trusting
    // whatever the single sensor claims.
    VerticalEstimator(IRangeSensor* const* sensors, uint8_t count);

    bool begin();

    // Advance the filter one tick. `orientation` and `accel_body` come from the
    // attitude estimate and the same IMU sample. Returns true if a range
    // measurement was fused this tick (false means it coasted on accel, which
    // is normal and not a fault).
    bool update(uint32_t now_us, const Quat& orientation,
                const Vec3& accel_body, uint32_t sample_us);

    float altitude() const { return z_; }
    float verticalVelocity() const { return vz_; }

    // False once the filter has coasted past the timeout with no usable range —
    // dead reckoning on accel alone goes bad quickly.
    bool healthy() const { return initialized_ && !stale_; }

    // sigma_accel: accelerometer noise/bias drift, m/s^2 — how fast the filter
    // loses confidence while coasting. sigma_range: ToF measurement noise, m —
    // how hard a reading pulls the estimate. Raise sigma_range if the altitude
    // twitches on rough surfaces.
    void setNoise(float sigma_accel, float sigma_range);

    // Reject a ToF reading when tilted past this. The cos() correction degrades
    // at steep angles and the beam may be hitting something other than the
    // ground directly below.
    void setMaxTiltCos(float cos_tilt) { min_cos_tilt_ = cos_tilt; }

private:
    // Combine the sensors into one trusted height, or report none available.
    bool fuseRange(const Quat& orientation, float cos_tilt, float& height_out);

    IRangeSensor* const* sensors_;
    uint8_t count_;

    // State and its covariance (symmetric, so p01 == p10).
    float z_ = 0.0f;
    float vz_ = 0.0f;
    float p00_ = 1.0f, p01_ = 0.0f, p11_ = 1.0f;

    float sigma_accel_ = 0.5f;
    float sigma_range_ = 0.03f;
    float min_cos_tilt_ = 0.7f;          // ~45 degrees
    uint32_t range_timeout_us_ = 500000;  // 0.5 s coasting before untrusted

    uint32_t last_sample_us_ = 0;
    uint32_t last_range_us_ = 0;
    bool initialized_ = false;
    bool seeded_ = false;
    bool stale_ = true;
};

}  // namespace tardigrade
