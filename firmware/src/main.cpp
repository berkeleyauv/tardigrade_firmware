#include <Arduino.h>
#include "comms/CommandLink.h"
#include "drivers/Icm20948.h"
#include "estimator/OnboardEstimator.h"
#include "motors/MotorManager.h"
#include "safety/HardwareWatchdog.h"
#include "safety/Safety.h"

using namespace tardigrade;

// ESC signal pins. Avoids the strapping pins (0, 2, 12, 15), which affect boot
// mode if something drives them, and the input-only range (34-39).
constexpr uint8_t kMotorPins[] = {25, 26, 27, 33};
constexpr uint8_t kMotorCount = sizeof(kMotorPins);

// Hopcopter: propellers on unidirectional ESCs. The robosub's reversible
// thrusters need EscMode::Bidirectional — the two conventions put "stopped" at
// different pulse widths, so this must match the physical hardware.
MotorManager motors(kMotorPins, kMotorCount, EscMode::Unidirectional);
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

    if (HardwareWatchdog::resetWasWatchdog()) {
        // A silent reboot mid-test looks like a glitch unless something says
        // otherwise. This is the firmware telling on itself.
        Serial.println("!! previous boot ended in a WATCHDOG RESET !!");
    }

    // Motors first, before anything that can fail or block. Until the ESC pins
    // are driven they float, and a floating ESC input is undefined. begin()
    // parks every channel at minimum throttle and holds it there long enough
    // for the ESCs to accept it as idle.
    Serial.printf("Arming %u ESC outputs (holding minimum)...\n", kMotorCount);
    if (!motors.begin()) {
        Serial.println("ESC output setup FAILED - check pins");
    }

    if (imu.begin()) {
        Serial.println("ICM-20948 online (+/-2000 dps, +/-8 g)");
        Serial.println("Calibrating gyro - HOLD STILL...");
        if (imu.calibrateGyro()) {
            const Vec3& b = imu.gyroBias();
            Serial.printf("gyro bias [rad/s]=% .4f % .4f % .4f\n", b.x, b.y, b.z);
        } else {
            Serial.println("gyro calibration incomplete - bias may be inaccurate");
        }
    } else {
        // Not fatal, and NOT a reason to skip the rest of setup: the link and
        // both watchdogs still need to come up so the vehicle stays
        // commandable and stoppable without an estimate.
        Serial.println("ICM-20948 not found - check Qwiic wiring and ADR jumper");
    }

    // Watchdog LAST. Gyro calibration and the ESC hold block for seconds by
    // design; arming earlier would trip it before the control loop ever runs.
    if (HardwareWatchdog::begin(1)) {
        Serial.println("hardware watchdog armed (1 s)");
    } else {
        Serial.println("hardware watchdog FAILED TO ARM");
    }

    Serial.println("ready - tilt the board, or connect the ground station");
}

void loop() {
    const uint32_t now_us = micros();

    // Feed the hardware watchdog once per pass. Reaching this line is the
    // proof of life it is waiting for; if the loop hangs anywhere below, the
    // chip resets and the ESCs lose their signal.
    HardwareWatchdog::feed();

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
        "link=%s  esc=[%u %u %u %u]us  crc_err=%lu\n",
        s.roll_rad * RAD_TO_DEG,
        s.pitch_rad * RAD_TO_DEG,
        s.yaw_rad * RAD_TO_DEG,
        estimator.healthy(),
        safety.armed(),
        safety.linkLost() ? "lost" : "ok",
        motors.pulseUs(0), motors.pulseUs(1),
        motors.pulseUs(2), motors.pulseUs(3),
        (unsigned long)command_link.crcErrors());
}
