#include <Arduino.h>
#include "comms/CommandLink.h"
#include "drivers/Icm20948.h"
#include "estimator/OnboardEstimator.h"
#include "safety/Safety.h"

using namespace tardigrade;

// Stand-in until MotorManager and the ESC driver exist. Prints instead of
// driving hardware, so the command path and watchdog can be exercised on a
// bench with nothing connected and nothing able to spin.
class PrintMotorSink : public IMotorSink {
public:
    void setMotor(uint8_t index, float value) override {
        Serial.printf("[motor] %u -> %.3f\n", index, value);
    }
    void stopAll() override { Serial.println("[motor] ALL STOP"); }
};

constexpr uint8_t kMotorCount = 4;
PrintMotorSink motors;
Safety safety(motors);
CommandLink command_link(Serial, safety, kMotorCount);

// Hopcopter IMU on the default I2C pins (ESP32: SDA=21, SCL=22),
// SparkFun default address (AD0=1 => 0x69).
Icm20948 imu;
OnboardEstimator estimator(imu);

// Attitude is estimated as fast as samples arrive; printing is throttled so the
// serial link doesn't become the bottleneck.
constexpr uint32_t kPrintIntervalMs = 100;
uint32_t last_print_ms = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Tardigrade FC starting...");

    if (!imu.begin()) {
        Serial.println("ICM-20948 not found - check Qwiic wiring and ADR jumper");
        return;
    }
    Serial.println("ICM-20948 online (+/-2000 dps, +/-8 g)");

    Serial.println("Calibrating gyro - HOLD STILL...");
    if (imu.calibrateGyro()) {
        const Vec3& b = imu.gyroBias();
        Serial.printf("gyro bias [rad/s]=% .4f % .4f % .4f\n", b.x, b.y, b.z);
    } else {
        Serial.println("gyro calibration incomplete - bias may be inaccurate");
    }

    Serial.println("Estimator running - tilt the board");
}

void loop() {
    const uint32_t now_us = micros();

    // Pull-based and non-blocking: run the filter every pass, print rarely.
    estimator.update(now_us);

    // Drain the link first, then judge it. update() must run unconditionally —
    // the watchdog fires on the ABSENCE of traffic, so gating it on a received
    // packet would guarantee it never trips on the link loss it exists for.
    command_link.update(now_us, estimator.state());
    safety.update(now_us);

    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms < kPrintIntervalMs) {
        return;
    }
    last_print_ms = now_ms;

    const VehicleState& s = estimator.state();
    Serial.printf(
        "roll=% 7.2f  pitch=% 7.2f  yaw=% 7.2f  [deg]  healthy=%d  armed=%d  "
        "link=%s  crc_err=%lu\n",
        s.roll_rad * RAD_TO_DEG,
        s.pitch_rad * RAD_TO_DEG,
        s.yaw_rad * RAD_TO_DEG,
        estimator.healthy(),
        safety.armed(),
        safety.linkLost() ? "lost" : "ok",
        (unsigned long)command_link.crcErrors());
}
