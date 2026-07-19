#pragma once
//
// IRangeSensor — driver-layer contract for a single-point ranging sensor
// (the hopcopter's downward-facing ToF units).
//
// Scope: chips that report a distance along their own boresight. Converting
// that to altitude needs the vehicle's attitude, which happens one layer up in
// VerticalEstimator — a driver never sees the tilt and never applies it.
//
// The sensor's own limits are part of the contract. A ToF that reports 2.4 m
// when its ceiling is 2.0 m is not measuring, it is guessing, and only the
// driver knows where that line falls. Hence maxRange().

#include "core/types.h"

namespace tardigrade {

class IRangeSensor {
public:
    virtual ~IRangeSensor() = default;

    // Bring the sensor up (bus config, ID check, timing budget, ranging mode).
    virtual bool begin() = 0;

    // Read the latest measurement. Returns false (and leaves `out.valid ==
    // false`) when no new reading is ready or the return was too weak to
    // trust. ToF units run far slower than the control loop, so "nothing new
    // this tick" is the common case and is NOT an error.
    virtual bool read(RangeData& out) = 0;

    // True once begin() succeeded and readings are arriving on schedule.
    virtual bool healthy() const = 0;

    // Largest distance this sensor can measure, metres. Readings at or beyond
    // it are rejected rather than fused — at hop apex the estimator must coast
    // on the accelerometer instead of believing a saturated range.
    virtual float maxRange() const = 0;
};

}  // namespace tardigrade
