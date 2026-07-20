#include "drivers/EscPwm.h"

#include <Arduino.h>

namespace tardigrade {

namespace {

// Frame period in microseconds, and the LEDC duty value representing it.
constexpr uint32_t kFrameUs = 1000000UL / kEscFrameHz;
constexpr uint32_t kFullScale = 1UL << kEscPwmBits;

// Pulse width -> LEDC duty. Done in 32-bit integers: at 16 bits and a 20 ms
// frame the intermediate product reaches ~1.3e8, which overflows 16-bit math
// silently and would put the ESC somewhere unintended.
inline uint32_t usToDuty(uint16_t us) {
    return (static_cast<uint32_t>(us) * kFullScale) / kFrameUs;
}

}  // namespace

uint16_t EscPwm::minUs() const {
    return mode_ == EscMode::Bidirectional ? kThrusterMinUs : kEscMinUs;
}

uint16_t EscPwm::maxUs() const {
    return mode_ == EscMode::Bidirectional ? kThrusterMaxUs : kEscMaxUs;
}

uint16_t EscPwm::stopMicroseconds() const {
    return mode_ == EscMode::Bidirectional ? kThrusterStopUs : kEscMinUs;
}

bool EscPwm::begin(uint8_t pin, uint8_t channel, EscMode mode) {
    channel_ = channel;
    mode_ = mode;
    last_us_ = stopMicroseconds();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(pin, kEscFrameHz, kEscPwmBits)) {
        return false;
    }
    channel_ = pin;  // core 3.x addresses channels by pin
#else
    ledcSetup(channel_, kEscFrameHz, kEscPwmBits);
    ledcAttachPin(pin, channel_);
#endif

    ready_ = true;
    // Park at minimum before returning. Any window in which the output is
    // undefined is a window in which a motor might spin.
    stop();
    return true;
}

void EscPwm::writeMicroseconds(uint16_t us) {
    const uint16_t lo = minUs(), hi = maxUs();
    if (us < lo) us = lo;
    if (us > hi) us = hi;
    last_us_ = us;
    if (ready_) {
        ledcWrite(channel_, usToDuty(us));
    }
}

void EscPwm::writeNormalized(float value) {
    // Inverted test so NaN lands on the stopped branch. Comparing `value != 0`
    // or `value < 0` would let NaN through to the arithmetic below and put an
    // arbitrary pulse width on the wire.
    if (!(value > 0.0f) && !(value < 0.0f)) {
        stop();
        return;
    }
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;

    if (mode_ == EscMode::Bidirectional) {
        // Symmetric about the stop point, so 0 is genuinely neutral.
        const float span = (value > 0.0f) ? (kThrusterMaxUs - kThrusterStopUs)
                                          : (kThrusterStopUs - kThrusterMinUs);
        writeMicroseconds(
            static_cast<uint16_t>(kThrusterStopUs + value * span));
        return;
    }

    // Unidirectional: a propeller cannot be commanded backwards. Negative is
    // treated as stopped rather than mirrored, so a sign error on the hopcopter
    // fails safe instead of silently spinning up.
    if (value < 0.0f) {
        stop();
        return;
    }
    writeMicroseconds(static_cast<uint16_t>(
        kEscMinUs + value * (kEscMaxUs - kEscMinUs)));
}

void EscPwm::stop() {
    writeMicroseconds(stopMicroseconds());
}

}  // namespace tardigrade
