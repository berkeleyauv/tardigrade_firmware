#include "drivers/Vl53l0x.h"

#include <Arduino.h>
#include <Wire.h>

namespace tardigrade {

namespace {
// Ranging period. ~50 Hz matches the ToF slot in docs/scheduler.md and is about
// as fast as this part manages without accuracy falling apart.
constexpr uint16_t kRangingPeriod_ms = 20;
constexpr uint32_t kTimingBudget_us = 20000;

// The VL53L0X reports ~8190 mm when it sees nothing. That is a sentinel, not a
// distance, and must never reach the filter.
constexpr uint16_t kNoTarget_mm = 8000;

// Time for the chip to boot after XSHUT is released.
constexpr uint32_t kBootDelay_ms = 10;
}  // namespace

Vl53l0x::Vl53l0x(int xshut_pin, uint8_t address, float max_range_m)
    : xshut_pin_(xshut_pin), address_(address), max_range_m_(max_range_m) {}

void Vl53l0x::standby() {
    if (xshut_pin_ < 0) {
        return;
    }
    pinMode(xshut_pin_, OUTPUT);
    digitalWrite(xshut_pin_, LOW);
    initialized_ = false;
}

bool Vl53l0x::bringUp() {
    Wire.begin();  // idempotent; removes any ordering dependency on the IMU

    if (xshut_pin_ >= 0) {
        pinMode(xshut_pin_, OUTPUT);
        digitalWrite(xshut_pin_, HIGH);
        delay(kBootDelay_ms);
    }

    // Adafruit's begin() readdresses the device when the requested address
    // differs from the 0x29 power-on default.
    if (!dev_.begin(address_, false, &Wire)) {
        initialized_ = false;
        return false;
    }

    dev_.setMeasurementTimingBudgetMicroSeconds(kTimingBudget_us);
    if (!dev_.startRangeContinuous(kRangingPeriod_ms)) {
        initialized_ = false;
        return false;
    }

    last_ok_ms_ = millis();
    initialized_ = true;
    return true;
}

bool Vl53l0x::begin() {
    // Already up — beginAll() ran during setup. Re-running the reset dance here
    // would drop this sensor back to 0x29 and collide with its neighbour, so
    // this must stay idempotent: VerticalEstimator::begin() calls it blindly.
    if (initialized_) {
        return true;
    }

    // Single-sensor path. With two on the bus use beginAll(), which holds the
    // others in reset while this one is readdressed.
    standby();
    delay(kBootDelay_ms);
    return bringUp();
}

bool Vl53l0x::beginAll(Vl53l0x* const* sensors, uint8_t count) {
    if (sensors == nullptr || count == 0) {
        return false;
    }

    // Everything into reset FIRST — while any sensor is awake at 0x29 it will
    // answer to traffic meant for the one being configured.
    for (uint8_t i = 0; i < count; ++i) {
        if (sensors[i] != nullptr) {
            sensors[i]->standby();
        }
    }
    delay(kBootDelay_ms);

    // Then one at a time. Each keeps its new address when the next wakes.
    bool all_ok = true;
    for (uint8_t i = 0; i < count; ++i) {
        if (sensors[i] == nullptr || !sensors[i]->bringUp()) {
            all_ok = false;
        }
    }
    return all_ok;
}

bool Vl53l0x::read(RangeData& out) {
    out.valid = false;
    if (!initialized_) {
        return false;
    }
    if (!dev_.isRangeComplete()) {
        return false;  // no new sample yet — expected, not an error
    }

    const uint16_t mm = dev_.readRangeResult();
    const uint8_t status = dev_.readRangeStatus();

    // A completed measurement proves the chip is alive even when it saw
    // nothing, so health and validity are tracked separately here.
    last_ok_ms_ = millis();
    out.timestamp_us = micros();

    if (status != 0 || mm == 0 || mm >= kNoTarget_mm) {
        return false;  // weak or absent return; out.valid stays false
    }

    out.range_m = mm * 0.001f;
    out.quality = 255;
    out.valid = true;
    return true;
}

bool Vl53l0x::healthy() const {
    return initialized_ && (millis() - last_ok_ms_) < kRangeHealthTimeoutMs;
}

}  // namespace tardigrade
