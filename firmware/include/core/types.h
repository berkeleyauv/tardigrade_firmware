#pragma once
//
// Core data contracts for the Tardigrade flight controller.
//
// These structs are the shared vocabulary between modules. They contain no
// logic and no hardware knowledge — only plain data. Every module compiles
// against this header; changing a contract here is a deliberate, cross-cutting
// decision.
//
// See docs/data_model.md for the design intent behind each type.

#include <cstdint>

namespace tardigrade {

// ---------------------------------------------------------------------------
// Small math primitives
// ---------------------------------------------------------------------------

// A 3-axis vector. Body frame unless a field's comment says otherwise.
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Unit quaternion (w, x, y, z), body-to-world rotation. Identity by default.
struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// ---------------------------------------------------------------------------
// Fixed capacities
// ---------------------------------------------------------------------------

// Upper bound on motors/thrusters across every vehicle this firmware targets.
// Robosub uses up to 8 thrusters; hopcopter far fewer. Sized for the largest.
inline constexpr uint8_t kMaxMotors = 8;

// Upper bound on downward range sensors. The hopcopter carries two for
// redundancy — a single ToF over a dark or glossy patch returns garbage, and
// landing is the one phase where a bad altitude breaks hardware.
inline constexpr uint8_t kMaxRangeSensors = 2;

// Standard gravity. Shared so drivers and estimators cannot disagree.
inline constexpr float kGravity_mps2 = 9.80665f;

// ---------------------------------------------------------------------------
// Sensor samples
// ---------------------------------------------------------------------------

// Raw sample from a raw-IMU chip (accel + gyro). On the hopcopter this comes
// from the ICM-20948 over I2C, owned by an IImuSource driver and consumed by
// the onboard estimator. NOTE: the robosub path does NOT produce ImuData — the
// Jetson fuses the VectorNav IMU and ZED stereo camera into a full pose
// estimate and feeds VehicleState directly. See docs/estimator.md.
struct ImuData {
    uint32_t timestamp_us = 0;   // micros() when the sample was taken
    Vec3 accel;                  // m/s^2, body frame
    Vec3 gyro;                   // rad/s, body frame
    float temperature_c = 0.0f;  // optional; 0 if the chip doesn't report it
    bool valid = false;          // false => stale/failed read, do not trust
};

// One reading from a downward range sensor (ToF). Owned by an IRangeSensor
// driver, consumed by the vertical estimator. NOTE: `range_m` is the distance
// along the SENSOR'S OWN AXIS, not height — converting it to altitude requires
// the vehicle's tilt, which is why the vertical channel depends on attitude.
struct RangeData {
    uint32_t timestamp_us = 0;
    float range_m = 0.0f;
    uint8_t quality = 0;         // sensor-specific confidence, 0 = worst
    bool valid = false;          // false => out of range, weak return, or failed
};

// Battery health. Owned by the battery driver, consumed by Safety/Telemetry.
struct BatteryState {
    uint32_t timestamp_us = 0;
    float voltage_v = 0.0f;
    float current_a = 0.0f;      // 0 if not measured
    bool valid = false;
};

// ---------------------------------------------------------------------------
// Estimation / control pipeline
// ---------------------------------------------------------------------------

// Best estimate of the vehicle's state. This is the estimator's output and the
// controller's input — the single seam that both the onboard-fusion path and
// the external (Jetson/VectorNav) path must fill. See docs/estimator.md.
struct VehicleState {
    uint32_t timestamp_us = 0;   // time the estimate is valid for

    // Attitude. Quaternion is authoritative; Euler angles are a convenience
    // derived from it for logging/telemetry.
    Quat orientation;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    Vec3 angular_velocity;       // rad/s, body frame

    // Vertical channel, filled by VerticalEstimator. Both stay 0 on vehicles
    // with no range sensor. vertical_velocity_mps is carried explicitly rather
    // than left for the controller to differentiate: differentiating a noisy
    // altitude inside a loop that commands thrust amplifies exactly the noise
    // you least want there.
    float altitude_m = 0.0f;             // height above ground, tilt-compensated
    float vertical_velocity_mps = 0.0f;  // world frame, + is up

    bool valid = false;          // false => estimate is stale/untrustworthy
    bool altitude_valid = false; // vertical channel specifically; attitude can
                                 // be trusted while altitude is not
};

// What the flight-mode manager wants the vehicle to do. Controller input.
struct DesiredState {
    uint32_t timestamp_us = 0;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    Vec3 angular_velocity;       // rate targets (rate mode)
    float thrust = 0.0f;         // normalized collective thrust, 0..1

    // Altitude-hold targets. Used only in modes that close the vertical loop;
    // in manual/rate modes `thrust` is commanded directly and these are ignored.
    float altitude_m = 0.0f;
    float climb_rate_mps = 0.0f;
};

// Controller output: desired torques/force before mixing to motors.
struct ControlOutput {
    uint32_t timestamp_us = 0;
    Vec3 torque;                 // roll/pitch/yaw effort, normalized -1..1
    float thrust = 0.0f;         // normalized collective, 0..1
};

// Final per-motor commands after mixing. Only [0, count) are meaningful.
struct MotorOutput {
    uint32_t timestamp_us = 0;
    uint8_t count = 0;
    float value[kMaxMotors] = {};  // normalized 0..1 per motor
    bool armed = false;
};

}  // namespace tardigrade
