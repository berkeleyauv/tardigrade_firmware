#pragma once
//
// Safety — arming state and the ground-link deadman watchdog.
//
// THE FAILURE THIS EXISTS TO PREVENT: a motor is spinning under host command,
// and the host goes away. USB gets bumped, the ground station freezes, the OS
// decides to install an update. Nothing arrives to say "stop", because the
// thing that would have said it is gone. Without a watchdog the motor keeps
// running against a listener that no longer exists.
//
// So the watchdog is driven by ABSENCE, not by events, and that dictates the
// shape of the API: update() MUST be called every loop tick, not only when a
// packet arrives. A timeout checked exclusively on receipt can never fire on a
// dead link — the one case it is built for.
//
// Safety owns the motor sink so that tripping is not advisory. It stops the
// motors itself rather than setting a flag and trusting somebody to read it.

#include "core/types.h"

namespace tardigrade {

// Where motor commands land. MotorManager will implement this once ESCs exist;
// until then anything that can print or drive a pin will do.
class IMotorSink {
public:
    virtual ~IMotorSink() = default;
    virtual void setMotor(uint8_t index, float value) = 0;
    virtual void stopAll() = 0;
};

enum class DisarmReason : uint8_t {
    Commanded  = 0,  // the host asked
    LinkLost   = 1,  // watchdog expired
    NeverArmed = 2,
};

class Safety {
public:
    // `link_timeout_us` is how long the vehicle tolerates silence while armed.
    // Short enough that a runaway is brief, long enough to survive ordinary
    // host scheduling jitter; the ground station should heartbeat several times
    // faster than this.
    Safety(IMotorSink& motors, uint32_t link_timeout_us = 300000);

    // Call EVERY tick. Trips the watchdog when the link has gone quiet.
    void update(uint32_t now_us);

    // Feed the watchdog. Any valid frame counts as proof of life, not just
    // Heartbeat — a host actively commanding motors is plainly still there.
    void noteTraffic(uint32_t now_us);

    // Arming refuses while the link is already considered lost: arming into
    // silence would produce a vehicle that is live and uncommandable.
    bool arm(uint32_t now_us);
    void disarm(DisarmReason reason);

    // Gate for motor-test commands. Returns false and leaves motors untouched
    // unless armed and the link is alive. `value` is clamped to the test cap.
    bool commandMotor(uint8_t index, float value, uint8_t motor_count);

    bool armed() const { return armed_; }
    bool linkLost() const { return link_lost_; }
    DisarmReason lastDisarmReason() const { return last_disarm_reason_; }

    // Ceiling on bench motor-test throttle. Full authority belongs to the
    // controller, not to a slider.
    void setTestThrottleLimit(float limit) { test_limit_ = limit; }

private:
    IMotorSink& motors_;
    uint32_t link_timeout_us_;
    uint32_t last_traffic_us_ = 0;
    float test_limit_ = 0.30f;
    DisarmReason last_disarm_reason_ = DisarmReason::NeverArmed;
    bool armed_ = false;
    bool link_lost_ = true;  // no host has spoken yet
};

}  // namespace tardigrade
