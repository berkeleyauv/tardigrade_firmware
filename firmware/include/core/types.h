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

// Upper bound on thrusters. The robosub uses up to 8.
inline constexpr uint8_t kMaxMotors = 8;

// ---------------------------------------------------------------------------
// Sensor samples
// ---------------------------------------------------------------------------

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

// Best estimate of the vehicle's state — the pose the Jetson fuses (VectorNav
// IMU + ZED stereo via the EKF) and delivers over the link. See docs/estimator.md.
struct VehicleState {
    uint32_t timestamp_us = 0;   // time the estimate is valid for

    // Attitude. Quaternion is authoritative; Euler angles are a convenience
    // derived from it for logging/telemetry.
    Quat orientation;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    Vec3 angular_velocity;       // rad/s, body frame

    // Depth channel (ENU z from the Jetson EKF). vertical_velocity_mps is
    // carried explicitly rather than left for the controller to differentiate:
    // differentiating a noisy depth inside a loop that commands thrust
    // amplifies exactly the noise you least want there.
    float altitude_m = 0.0f;             // ENU z, metres (negative = deeper)
    float vertical_velocity_mps = 0.0f;  // world frame, + is up

    bool valid = false;          // false => estimate is stale/untrustworthy
    bool altitude_valid = false; // depth channel specifically
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

// Controller output: a body-frame wrench (force + torque) before mixing to
// motors. This is the general 6-DOF form the mixer maps onto thrusters.
//
// The quadcopter is the degenerate case: it uses force.z as collective thrust
// and leaves force.x/y at zero, because propellers can only push along one body
// axis. The robosub uses all six. One contract, so the mixer interface and the
// controller seam are identical for both vehicles.
struct ControlOutput {
    uint32_t timestamp_us = 0;
    Vec3 force;   // body frame: x=surge, y=sway, z=heave. Normalized -1..1.
    Vec3 torque;  // body frame: x=roll, y=pitch, z=yaw.  Normalized -1..1.
};

// Final per-motor commands after mixing. Only [0, count) are meaningful.
//
// SIGNED, -1..+1, with 0 meaning STOPPED on every vehicle. The hopcopter's
// propellers only ever spin one way, so it simply never commands a negative;
// the robosub's thrusters are reversible and need the full range.
//
// One signed contract rather than a per-vehicle convention, because the
// alternative puts "stopped" at a different number depending on the airframe —
// and the value that means stop is the value Safety writes on every disarm,
// every watchdog trip, and every emergency stop. Getting that wrong inverts the
// failsafe into a maximum-thrust command, so it is worth spending a bit on the
// hopcopter to keep the meaning of zero identical everywhere.
struct MotorOutput {
    uint32_t timestamp_us = 0;
    uint8_t count = 0;
    float value[kMaxMotors] = {};  // -1..+1 per motor; 0 = stopped
    bool armed = false;
};

}  // namespace tardigrade
