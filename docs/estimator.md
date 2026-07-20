# Estimator

## Purpose

Turn sensor data into the vehicle's best estimate of its own state
(`VehicleState`: orientation quaternion, roll/pitch/yaw, angular velocity,
and — later — altitude).

The estimator is the only module that knows how attitude is computed. The
Controller, Safety, and Telemetry modules consume `VehicleState` and never
learn where it came from.

## The core decision: abstract at the estimator output, not the IMU

Tardigrade must run on two very different vehicles:

| | Hopcopter | Robosub |
|---|---|---|
| Sensor | SparkFun ICM-20948 9DoF IMU (accel + gyro) | VectorNav IMU + ZED stereo camera |
| Connection | Chip → ESP32 (I2C / Qwiic) | Sensors → Jetson → ESP32 (ROS link) |
| Where fusion happens | **On the ESP32** | **On the Jetson**, fusing IMU + visual odometry |
| What the ESP32 receives | Raw accel/gyro | A fully-fused pose estimate |

These differ in **where sensor fusion happens**, not just in which chip is
used. That is why the abstraction seam is `IStateEstimator` → `VehicleState`,
**not** the raw IMU.

If we abstracted at the raw-IMU level instead, the Robosub path would have to
discard the Jetson's fused pose and re-run a crude Mahony filter on the ESP32,
using only the VectorNav's raw output and throwing away the ZED's visual
odometry entirely — worse accuracy, added latency, and pointless, since the
Jetson's filter is far better than anything the ESP32 can run.

## Interfaces

Two layered contracts, matching the Drivers-vs-Estimator split in
[architecture.md](architecture.md):

- **`IImuSource`** (driver layer, `firmware/include/drivers/IImuSource.h`) —
  raw accelerometer + gyroscope chips only (the hopcopter's ICM-20948).
  Produces `ImuData`. The Robosub / Jetson path does **not** implement this.

- **`IStateEstimator`** (estimator layer,
  `firmware/include/estimator/IStateEstimator.h`) — produces `VehicleState`.
  This is the seam both vehicles satisfy.

## Implementations

```
Hopcopter:  Icm20948 (IImuSource) ──► OnboardEstimator (Mahony) ──► VehicleState
Robosub:    JetsonLink ──► ExternalEstimator (passthrough)      ──► VehicleState
```

- **`OnboardEstimator`** — reads an `IImuSource` (ICM-20948), runs onboard
  fusion (Mahony or Madgwick), fills `VehicleState`.
- **`ExternalEstimator`** — receives the fused pose estimate from the Jetson
  link and copies the solution straight into `VehicleState`. No onboard fusion.

Selection happens once at boot via a factory keyed off a build flag
(`-D VEHICLE_HOPCOPTER` / `-D VEHICLE_ROBOSUB`) or a `Parameter`.

## Timing and health

- `update()` is **pull-based and must never block.** The fixed-rate loop calls
  it every tick.
- The I2C path delivers a fresh sample on (nearly) every tick. The Jetson path
  is **asynchronous** — pose messages arrive at the EKF's publish rate
  (the robot_localization filter in tardigrade_ws runs at **30 Hz**, far slower
  than the control loop and slower than the VectorNav's own 800 Hz), independent
  of the loop. When no new message is available, the estimator reuses its last
  `VehicleState`.
- Each estimator tracks `last_valid_us`. If
  `now_us - last_valid_us > freshness_timeout`, `healthy()` returns `false`,
  which trips the Safety module's sensor-timeout failsafe. Both vehicles use
  this same mechanism — a payoff of the shared interface.

## Link protocol (Robosub)

The Jetson → ESP32 pose stream rides a **ROS link**: the Jetson publishes the
fused pose on a topic and the ESP32 subscribes as a micro-ROS node, handing each
message to `ExternalEstimator`.

`JetsonLink` owns that transport and `ExternalEstimator` stays transport-
agnostic: it consumes whatever pose the link last delivered and applies the same
`last_valid_us` freshness check as the hopcopter path. The estimator seam only
requires that a fused pose lands in `VehicleState`, so the transport choice —
micro-ROS or a leaner custom bridge — stays cheap to reverse.

See **[ros_link.md](ros_link.md)** for how DDS, XRCE-DDS, and micro-ROS relate,
which process runs on which machine, the bandwidth and QoS constraints, and the
fallback option.

## Build order

Implement `OnboardEstimator` first and close the hopcopter control loop before
building `ExternalEstimator` / `JetsonLink`. The Robosub path reuses the same
`IStateEstimator` contract, so nothing is wasted by doing it second — and the
controller is proven before the Jetson link is added as a variable.
