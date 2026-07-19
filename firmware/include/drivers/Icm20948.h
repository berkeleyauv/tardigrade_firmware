#pragma once
//
// Icm20948 — IImuSource driver for the hopcopter's SparkFun 9DoF ICM-20948
// (Qwiic) breakout, wired to the ESP32 over I2C.
//
// It wraps the SparkFun Arduino library and converts that library's units
// (accel in milli-g, gyro in deg/s) into the SI units ImuData promises
// (accel in m/s^2, gyro in rad/s). This is the ONLY file that knows the
// ICM-20948 exists; everything above it sees a plain IImuSource.
//
// See docs/estimator.md for where this sits in the pipeline.

#include "drivers/IImuSource.h"
#include "ICM_20948.h"

namespace tardigrade {

class Icm20948 : public IImuSource {
public:
    // `ad0` selects the I2C address via the board's ADR jumper:
    //   1 => 0x69 (SparkFun default), 0 => 0x68.
    // `sda` / `scl` default to the board's standard I2C pins when left at -1.
    explicit Icm20948(uint8_t ad0 = 1, int sda = -1, int scl = -1);

    bool begin() override;
    bool read(ImuData& out) override;
    bool healthy() const override;

    // Average `samples` gyro readings and store them as a zero-rate bias that
    // read() subtracts from every subsequent sample. THE VEHICLE MUST BE
    // COMPLETELY STILL for the duration. Uncorrected bias integrates straight
    // into attitude drift, so the estimator depends on this having been run.
    // Returns false if it timed out before collecting a full set.
    bool calibrateGyro(uint16_t samples = 500);

    // The bias found by calibrateGyro(), rad/s. Zero until it runs.
    const Vec3& gyroBias() const { return gyro_bias_; }

private:
    ICM_20948_I2C imu_;
    uint8_t ad0_;
    int sda_;
    int scl_;
    Vec3 gyro_bias_;
    uint32_t last_ok_ms_ = 0;
    bool initialized_ = false;
};

// How long without a successful read before the chip counts as failed. A fast
// control loop polls faster than the sensor produces samples, so an occasional
// "not ready" tick is normal and must NOT read as a fault.
inline constexpr uint32_t kImuHealthTimeoutMs = 100;

}  // namespace tardigrade
