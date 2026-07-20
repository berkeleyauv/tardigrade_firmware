#include "safety/Safety.h"

namespace tardigrade {

Safety::Safety(IMotorSink& motors, uint32_t link_timeout_us)
    : motors_(motors), link_timeout_us_(link_timeout_us) {}

void Safety::update(uint32_t now_us) {
    if (!armed_) {
        // Disarmed: staleness is irrelevant and would only produce noise. The
        // link is judged again at arm() time.
        return;
    }

    // Unsigned subtraction, so the ~71 minute micros() wrap is handled.
    if ((now_us - last_traffic_us_) > link_timeout_us_) {
        link_lost_ = true;
        disarm(DisarmReason::LinkLost);
    }
}

void Safety::noteTraffic(uint32_t now_us) {
    last_traffic_us_ = now_us;
    link_lost_ = false;
}

bool Safety::arm(uint32_t now_us) {
    if (link_lost_) {
        return false;
    }
    // Re-datum the watchdog on the arm itself. Otherwise a command that arrived
    // just before a long stall would leave the vehicle armed with most of its
    // timeout already spent.
    last_traffic_us_ = now_us;
    armed_ = true;
    return true;
}

void Safety::disarm(DisarmReason reason) {
    armed_ = false;
    last_disarm_reason_ = reason;
    // Stop first, remember why second. Disarming must be an action, not a note.
    motors_.stopAll();
}

bool Safety::commandMotor(uint8_t index, float value, uint8_t motor_count) {
    if (!armed_ || link_lost_) {
        return false;
    }
    if (index >= motor_count || index >= kMaxMotors) {
        return false;
    }
    if (!(value >= 0.0f)) {
        return false;  // also rejects NaN, which would sail past value < 0
    }
    if (value > test_limit_) {
        value = test_limit_;
    }
    motors_.setMotor(index, value);
    return true;
}

}  // namespace tardigrade
