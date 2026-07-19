#include <Arduino.h>
#include "drivers/Icm20948.h"
#include "estimator/OnboardEstimator.h"

using namespace tardigrade;

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
    // Pull-based and non-blocking: run the filter every pass, print rarely.
    estimator.update(micros());

    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms < kPrintIntervalMs) {
        return;
    }
    last_print_ms = now_ms;

    const VehicleState& s = estimator.state();
    Serial.printf("roll=% 7.2f  pitch=% 7.2f  yaw=% 7.2f  [deg]   healthy=%d\n",
                  s.roll_rad * RAD_TO_DEG,
                  s.pitch_rad * RAD_TO_DEG,
                  s.yaw_rad * RAD_TO_DEG,
                  estimator.healthy());
}
