#include "motors/MotorManager.h"

#include <Arduino.h>

namespace tardigrade {

namespace {
// How long to hold minimum throttle at startup so the ESCs recognise it and
// arm. Shorter than this and some ESCs sit refusing commands; they are not
// broken, they simply never saw a valid idle.
constexpr uint32_t kEscArmHoldMs = 2000;
}  // namespace

MotorManager::MotorManager(const uint8_t* pins, uint8_t count, EscMode mode)
    : mode_(mode), count_(count > kMaxMotors ? kMaxMotors : count) {
    for (uint8_t i = 0; i < count_; ++i) {
        pins_[i] = pins[i];
    }
}

bool MotorManager::begin() {
    bool all_ok = true;
    for (uint8_t i = 0; i < count_; ++i) {
        // LEDC channels are allocated in pairs sharing a timer; using every
        // other channel keeps each ESC on its own timer and avoids one channel
        // silently retuning another's frequency.
        if (!esc_[i].begin(pins_[i], static_cast<uint8_t>(i * 2), mode_)) {
            all_ok = false;
        }
    }
    ready_ = all_ok;

    // Every output is already parked at its stop pulse by EscPwm::begin().
    // Hold it there long enough for the ESCs to accept it as idle.
    delay(kEscArmHoldMs);
    return all_ok;
}

void MotorManager::setMotor(uint8_t index, float value) {
    if (index >= count_) {
        return;
    }
    esc_[index].writeNormalized(value);
}

void MotorManager::stopAll() {
    // Deliberately not gated on ready_: stopping must work in every state,
    // including a partially failed begin(). EscPwm::stop() is safe when its
    // channel was never attached.
    for (uint8_t i = 0; i < count_; ++i) {
        esc_[i].stop();
    }
}

uint16_t MotorManager::pulseUs(uint8_t index) const {
    return index < count_ ? esc_[index].lastMicroseconds() : 0;
}

}  // namespace tardigrade
