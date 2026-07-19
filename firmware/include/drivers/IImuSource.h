#pragma once
//
// IImuSource — driver-layer contract for a RAW inertial sensor.
//
// Scope: chips that report raw accelerometer + gyroscope samples and rely on
// THIS firmware to fuse them into an attitude — specifically the hopcopter's
// SparkFun ICM-20948 9DoF IMU, wired to the ESP32 over I2C (Qwiic).
//
// Deliberately NOT implemented by the robosub path. There the Jetson fuses the
// VectorNav IMU and ZED stereo camera into a full pose estimate and pushes it
// to the ESP32 (over a ROS link), so a fused solution — not raw accel/gyro —
// arrives through the estimator layer, not here. Making that path pretend to be
// a raw IMU would force us to discard the Jetson's fusion and re-run a worse
// filter on the ESP32. See docs/estimator.md for the layering rationale.

#include "core/types.h"

namespace tardigrade {

class IImuSource {
public:
    virtual ~IImuSource() = default;

    // Bring the sensor up (bus config, WHO_AM_I check, ranges, etc.).
    // Returns false if the device is absent or misconfigured.
    virtual bool begin() = 0;

    // Read the latest sample. Returns false (and leaves `out.valid == false`)
    // if the read failed; callers must treat that as a sensor timeout.
    virtual bool read(ImuData& out) = 0;

    // True once begin() has succeeded and recent reads are healthy. Feeds
    // Safety's sensor-timeout logic.
    virtual bool healthy() const = 0;
};

}  // namespace tardigrade
