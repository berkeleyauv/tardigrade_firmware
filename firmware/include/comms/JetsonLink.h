#pragma once
//
// JetsonLink — receives the fused pose the Jetson streams to the robosub.
//
// The Jetson runs the robot_localization EKF (ZED visual odometry + VectorNav
// IMU) and publishes nav_msgs/Odometry. A bridge node on the Jetson reframes
// each message into a Protocol Pose frame and injects it onto the same serial
// link the operator commands use. This class is handed decoded Pose frames and
// keeps the newest one, timestamped on arrival.
//
// It is deliberately the ONLY place that knows pose arrives over a wire.
// ExternalEstimator reads a plain PoseSample and never learns the transport, so
// swapping this for a micro-ROS subscriber later would touch nothing else. See
// docs/ros_link.md.

#include "core/types.h"

namespace tardigrade {

// One fused pose, already unpacked from the wire. Frames follow the Pose frame
// layout in Protocol.h (ENU world frame, body-to-world quaternion).
struct PoseSample {
    uint16_t seq = 0;
    Vec3 position;            // world frame (ENU), metres
    Quat orientation;         // body-to-world, normalized
    Vec3 linear_velocity;     // world frame, m/s
    Vec3 angular_velocity;    // body frame, rad/s
    uint32_t received_us = 0; // ESP micros() at decode — freshness is measured
                              // against this, NOT any Jetson timestamp, since
                              // the two clocks are unrelated.
    bool valid = false;
};

class JetsonLink {
public:
    // Decode a Pose frame payload (as delivered by the parser) into the latest
    // sample, stamping it with `now_us`. Returns false and changes nothing if
    // the payload is the wrong length. Dropped sequence numbers are counted.
    bool onPoseFrame(const uint8_t* payload, uint8_t len, uint32_t now_us);

    const PoseSample& latest() const { return latest_; }
    bool everReceived() const { return latest_.valid; }
    uint32_t drops() const { return drops_; }

private:
    PoseSample latest_;
    uint16_t last_seq_ = 0;
    uint32_t drops_ = 0;
    bool seq_started_ = false;
};

}  // namespace tardigrade
