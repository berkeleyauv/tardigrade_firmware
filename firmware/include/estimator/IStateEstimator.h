#pragma once
//
// IStateEstimator — THE abstraction seam for supporting multiple vehicles.
//
// Every vehicle must produce a VehicleState; they just reach it differently:
//
//   * Hopcopter  -> OnboardEstimator: reads an IImuSource (raw accel/gyro from
//                   the ICM-20948 over I2C) and runs onboard fusion
//                   (Mahony/Madgwick) to compute attitude.
//
//   * Robosub    -> ExternalEstimator: the Jetson already fused the VectorNav
//                   IMU and ZED stereo camera into a pose estimate; this
//                   subscriber copies that fused solution (received over a ROS
//                   link) straight into VehicleState. No onboard fusion.
//
// The Controller, Safety, and Telemetry modules depend ONLY on this interface
// and on VehicleState — they never learn which vehicle they are running on.
// Selection happens once at boot (build flag / Parameter) via a factory.
//
// Timing contract: update() is PULL-BASED and MUST NOT block. The fixed-rate
// loop calls it every tick. If no fresh sample is available (common on the
// async Jetson ROS path), the estimator keeps its last state and reports
// healthy() == false once the data has gone stale, which trips Safety.

#include "core/types.h"

namespace tardigrade {

class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;

    // One-time setup (open the sensor/link, zero the filter). Returns false on
    // failure to initialize the underlying source.
    virtual bool begin() = 0;

    // Pull the newest available sensor data and recompute the estimate.
    // `now_us` is the current micros() from the scheduler. Returns true if the
    // state was advanced with fresh data this tick, false if it reused the
    // previous state (no new sample yet). Never blocks.
    virtual bool update(uint32_t now_us) = 0;

    // The current best estimate. Always readable; check .valid / healthy()
    // before acting on it.
    virtual const VehicleState& state() const = 0;

    // False when the estimate has gone stale past the freshness timeout or the
    // source failed. Consumed by Safety's sensor-timeout failsafe.
    virtual bool healthy() const = 0;
};

}  // namespace tardigrade
