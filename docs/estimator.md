# Estimator

## Purpose

Turn the Jetson's fused pose into the robot's `VehicleState` (orientation
quaternion, roll/pitch/yaw, angular velocity, depth, vertical velocity).

The estimator is the only module that knows where the pose came from. Safety
and Telemetry consume `VehicleState` and never learn the source.

## Transitional note

This document describes the **current** on-ESP path: the ESP receives the
Jetson's fused pose and holds it in `VehicleState` for the on-ESP controller.
Control is migrating to a Jetson ROS node (see
`tardigrade_ws/docs/jetson_control_architecture.md`), and once that lands the
Jetson no longer needs to send pose to the ESP at all — it already has it. At
that point `ExternalEstimator`, `JetsonLink`, the `Pose` frame, and this
document retire; the ESP becomes a pure actuator with no `VehicleState`
concept. Until then, this is the accurate, working path, and nothing here
should be treated as legacy prematurely.

## Why fusion happens on the Jetson, not the ESP

The pose comes from the VectorNav IMU + ZED stereo camera, fused by
`robot_localization`'s EKF **on the Jetson**. The ESP never sees raw sensor
data — only the finished pose. Re-running fusion on the ESP would mean
discarding the EKF's visual odometry and reconstructing a worse estimate from
scratch, for no benefit: the Jetson's filter is better than anything the ESP32
can run, and it's already running.

This is why the abstraction seam is `IStateEstimator` → `VehicleState`, not a
raw-sensor interface — the ESP only ever needs to consume a finished estimate.

## Interface

- **`IStateEstimator`** (`firmware/include/estimator/IStateEstimator.h`) —
  produces `VehicleState`. `update()` is pull-based and must never block; the
  fixed-rate loop calls it every tick.

## Implementation

```
JetsonLink ──► ExternalEstimator (passthrough) ──► VehicleState
```

- **`ExternalEstimator`** — receives the fused pose from `JetsonLink` and
  copies it straight into `VehicleState`. No fusion, no filtering.

## Timing and health

- The Jetson path is **asynchronous** — pose messages arrive at the EKF's
  publish rate (the `robot_localization` filter in `tardigrade_ws` runs at
  **30 Hz**, independent of the ESP's loop rate). When no new message is
  available, the estimator reuses its last `VehicleState`.
- `ExternalEstimator` tracks `last_valid_us`. If
  `now_us - last_valid_us > freshness_timeout`, `healthy()` returns `false`,
  which trips `Safety`'s sensor-timeout failsafe.

## Link protocol

The Jetson → ESP pose stream rides the ground-link's binary protocol: the
Jetson publishes the fused pose on a ROS topic, a Jetson-side bridge repacks
it into a compact `Pose` frame, and `JetsonLink` decodes it on the ESP side.

`JetsonLink` owns that transport and `ExternalEstimator` stays transport-
agnostic: it consumes whatever pose the link last delivered and applies the
`last_valid_us` freshness check above.

See **[ros_link.md](ros_link.md)** for how DDS, XRCE-DDS, and micro-ROS
relate, which process runs on which machine, the bandwidth/QoS constraints,
and why a plain serial bridge was chosen over micro-ROS.
