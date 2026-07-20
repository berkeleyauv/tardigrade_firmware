#include <Arduino.h>
#include "comms/CommandLink.h"
#include "estimator/IStateEstimator.h"
#include "motors/MotorManager.h"
#include "safety/HardwareWatchdog.h"
#include "safety/Safety.h"

// Vehicle selection happens ONCE here, at compile time, via a build flag — the
// factory promised in docs/estimator.md. Everything below the IStateEstimator
// seam (Safety, CommandLink, the control loop) is identical for both vehicles;
// only the estimator, the motor hardware, and the pose source differ.
//
//   default             -> hopcopter (onboard Mahony fusion, I2C IMU)
//   -D VEHICLE_ROBOSUB   -> robosub  (external pose from the Jetson EKF)

using namespace tardigrade;

#if defined(VEHICLE_ROBOSUB)
#include "comms/JetsonLink.h"
#include "estimator/ExternalEstimator.h"

// Thruster pins and order from tardigrade_ws esp_thruster_map.json. No onboard
// I2C IMU on the sub, so the I2C pins are free to drive thrusters.
constexpr uint8_t kMotorPins[] = {21, 19, 27, 18, 5, 14, 12, 26};
constexpr EscMode kEscMode = EscMode::Bidirectional;  // reversible thrusters
const char* const kVehicleName = "ROBOSUB";

JetsonLink jetson;
ExternalEstimator vehicle_estimator(jetson);
#else
#include "drivers/Icm20948.h"
#include "estimator/OnboardEstimator.h"

// ESC pins avoiding the strapping pins (0, 2, 12, 15) and input-only 34-39.
constexpr uint8_t kMotorPins[] = {25, 26, 27, 33};
constexpr EscMode kEscMode = EscMode::Unidirectional;  // propellers
const char* const kVehicleName = "HOPCOPTER";

Icm20948 imu;  // I2C, SparkFun default address 0x69
OnboardEstimator vehicle_estimator(imu);
#endif

constexpr uint8_t kMotorCount = sizeof(kMotorPins);

MotorManager motors(kMotorPins, kMotorCount, kEscMode);
Safety safety(motors);
CommandLink command_link(Serial, safety, kMotorCount);

// The rest of the firmware sees only this. Neither the loop nor any consumer
// knows which vehicle produced the estimate.
IStateEstimator& estimator = vehicle_estimator;

constexpr uint32_t kPrintIntervalMs = 100;
uint32_t last_print_ms = 0;

// Per-vehicle sensor bring-up, kept out of the common flow below.
static void beginSensors() {
#if defined(VEHICLE_ROBOSUB)
    command_link.setJetsonLink(&jetson);
    Serial.println("waiting for Jetson pose over the ROS link...");
#else
    if (imu.begin()) {
        Serial.println("ICM-20948 online (+/-2000 dps, +/-8 g)");
        Serial.println("Calibrating gyro - HOLD STILL...");
        if (imu.calibrateGyro()) {
            const Vec3& b = imu.gyroBias();
            Serial.printf("gyro bias [rad/s]=% .4f % .4f % .4f\n",
                          b.x, b.y, b.z);
        } else {
            Serial.println("gyro calibration incomplete - bias inaccurate");
        }
    } else {
        // Not fatal: the link and both watchdogs still come up so the vehicle
        // stays commandable and stoppable without an estimate.
        Serial.println("ICM-20948 not found - check wiring and ADR jumper");
    }
#endif
    estimator.begin();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("Tardigrade FC starting (%s)...\n", kVehicleName);

    if (HardwareWatchdog::resetWasWatchdog()) {
        // A silent reboot mid-test looks like a glitch unless something says so.
        Serial.println("!! previous boot ended in a WATCHDOG RESET !!");
    }

    // Motors first, before anything that can fail or block. Floating ESC pins
    // are undefined; begin() parks every channel at its stop pulse and holds it
    // long enough for the ESCs to accept it as idle.
    Serial.printf("Arming %u ESC outputs (holding stop)...\n", kMotorCount);
    if (!motors.begin()) {
        Serial.println("ESC output setup FAILED - check pins");
    }

    beginSensors();

    // Watchdog LAST: gyro calibration and the ESC hold block for seconds by
    // design and would trip it before the control loop ever runs.
    if (HardwareWatchdog::begin(1)) {
        Serial.println("hardware watchdog armed (1 s)");
    } else {
        Serial.println("hardware watchdog FAILED TO ARM");
    }

    Serial.println("ready - connect the ground station");
}

void loop() {
    const uint32_t now_us = micros();

    // Feed the hardware watchdog once per pass. Reaching this line is the proof
    // of life it waits for; a hang anywhere below resets the chip and the ESCs
    // lose their signal.
    HardwareWatchdog::feed();

    // Pull-based and non-blocking every pass.
    estimator.update(now_us);

    // Drain the link, then judge it. update() runs unconditionally — the
    // deadman fires on ABSENCE of traffic, so gating it on a received packet
    // would guarantee it never trips on the link loss it exists for.
    command_link.update(now_us, estimator.state());
    safety.update(now_us);

    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms < kPrintIntervalMs) {
        return;
    }
    last_print_ms = now_ms;

    const VehicleState& s = estimator.state();
    Serial.printf(
        "%s rpy=% 7.2f % 7.2f % 7.2f [deg] alt=% .2f healthy=%d armed=%d "
        "link=%s crc_err=%lu\n",
        kVehicleName,
        s.roll_rad * RAD_TO_DEG, s.pitch_rad * RAD_TO_DEG,
        s.yaw_rad * RAD_TO_DEG, s.altitude_m,
        estimator.healthy(), safety.armed(),
        safety.linkLost() ? "lost" : "ok",
        (unsigned long)command_link.crcErrors());
}
