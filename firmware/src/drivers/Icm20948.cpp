#include "drivers/Icm20948.h"

#include <Arduino.h>
#include <Wire.h>

namespace tardigrade {

namespace {
// The SparkFun library reports acceleration in milli-g and rotation in
// degrees/second; ImuData wants m/s^2 and rad/s.
constexpr float kMilliGToMps2 = kGravity_mps2 / 1000.0f;
}  // namespace

Icm20948::Icm20948(uint8_t ad0, int sda, int scl)
    : ad0_(ad0), sda_(sda), scl_(scl) {}

bool Icm20948::begin() {
    if (sda_ >= 0 && scl_ >= 0) {
        Wire.begin(sda_, scl_);
    } else {
        Wire.begin();
    }
    Wire.setClock(400000);  // 400 kHz fast-mode I2C

    // The chip can need a moment after power-up; give it a few attempts before
    // declaring it absent.
    bool found = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        imu_.begin(Wire, ad0_);
        if (imu_.status == ICM_20948_Stat_Ok) {
            found = true;
            break;
        }
        delay(10);
    }
    if (!found) {
        initialized_ = false;
        return false;
    }

    // Start from a known state — the chip wakes in low-power mode.
    imu_.swReset();
    delay(250);
    imu_.sleep(false);
    imu_.lowPower(false);
    imu_.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                       ICM_20948_Sample_Mode_Continuous);

    // Full-scale ranges. The chip defaults to +/-250 dps and +/-2 g, both far
    // too narrow for flight: a hopcopter exceeds 250 dps on ordinary attitude
    // corrections (and certainly on a tumble), and airframe vibration alone
    // saturates 2 g. A clipped sample is a lie the estimator cannot detect, so
    // we trade resolution for headroom.
    ICM_20948_fss_t fss;
    fss.g = dps2000;
    fss.a = gpm8;
    imu_.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    // Low-pass the worst of the propeller/frame vibration before it aliases
    // into the control loop.
    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.g = gyr_d196bw6_n229bw8;
    dlpcfg.a = acc_d111bw4_n136bw;
    imu_.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
    imu_.enableDLPF(ICM_20948_Internal_Acc, true);
    imu_.enableDLPF(ICM_20948_Internal_Gyr, true);

    if (imu_.status != ICM_20948_Stat_Ok) {
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    return true;
}

bool Icm20948::calibrateGyro(uint16_t samples) {
    if (!initialized_) {
        return false;
    }

    gyro_bias_ = Vec3{};  // measure the raw rate, not the already-corrected one

    double sx = 0.0, sy = 0.0, sz = 0.0;
    uint16_t collected = 0;
    const uint32_t deadline_ms = millis() + 5000;

    while (collected < samples && millis() < deadline_ms) {
        if (!imu_.dataReady()) {
            continue;
        }
        imu_.getAGMT();
        sx += imu_.gyrX() * DEG_TO_RAD;
        sy += imu_.gyrY() * DEG_TO_RAD;
        sz += imu_.gyrZ() * DEG_TO_RAD;
        ++collected;
    }

    if (collected == 0) {
        return false;
    }

    gyro_bias_.x = static_cast<float>(sx / collected);
    gyro_bias_.y = static_cast<float>(sy / collected);
    gyro_bias_.z = static_cast<float>(sz / collected);
    return collected == samples;
}

bool Icm20948::read(ImuData& out) {
    out.valid = false;
    if (!initialized_ || !imu_.dataReady()) {
        // Not an error — just nothing new since the last poll.
        return false;
    }

    imu_.getAGMT();  // refresh accel/gyro/mag/temp registers from the chip

    out.timestamp_us = micros();
    out.accel.x = imu_.accX() * kMilliGToMps2;
    out.accel.y = imu_.accY() * kMilliGToMps2;
    out.accel.z = imu_.accZ() * kMilliGToMps2;
    out.gyro.x = imu_.gyrX() * DEG_TO_RAD - gyro_bias_.x;
    out.gyro.y = imu_.gyrY() * DEG_TO_RAD - gyro_bias_.y;
    out.gyro.z = imu_.gyrZ() * DEG_TO_RAD - gyro_bias_.z;
    out.temperature_c = imu_.temp();
    out.valid = true;

    last_ok_ms_ = millis();
    return true;
}

bool Icm20948::healthy() const {
    return initialized_ && (millis() - last_ok_ms_) < kImuHealthTimeoutMs;
}

}  // namespace tardigrade
