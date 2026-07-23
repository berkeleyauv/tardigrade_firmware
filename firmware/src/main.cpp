#include <Arduino.h>
#include "comms/CommandLink.h"
#include "comms/JetsonLink.h"
#include "control/RobosubController.h"
#include "estimator/ExternalEstimator.h"
#include "estimator/IStateEstimator.h"
#include "mixer/RobosubMixer.h"
#include "motors/MotorManager.h"
#include "safety/HardwareWatchdog.h"
#include "safety/Safety.h"

// Robosub flight controller. The Jetson fuses the VectorNav IMU + ZED stereo
// (robot_localization EKF) into a pose and streams it here; this firmware holds
// depth + heading and drives the thrusters, with independent failsafes.
//
// NOTE: control is slated to move to a Jetson ROS node (see
// tardigrade_ws/docs/jetson_control_architecture.md). Until that lands, the
// on-ESP RobosubController stays as the working control path. Manual per-
// thruster bench testing does not depend on it.

using namespace tardigrade;

// Thruster pins and order from tardigrade_ws esp_thruster_map.json.
constexpr uint8_t kMotorPins[] = {21, 19, 27, 18, 5, 14, 12, 26};
constexpr uint8_t kMotorCount = sizeof(kMotorPins);

JetsonLink jetson;
ExternalEstimator vehicle_estimator(jetson);
RobosubController controller;
RobosubMixer mixer;

MotorManager motors(kMotorPins, kMotorCount, EscMode::Bidirectional);
Safety safety(motors);
CommandLink command_link(Serial, safety, kMotorCount);

// The loop and its consumers see only the interface, never the concrete type.
IStateEstimator& estimator = vehicle_estimator;

// Latches true the first time the estimate is healthy. The sensor-timeout
// failsafe only fires after this — otherwise a bench sub with no Jetson (never
// healthy) would disarm itself the instant it armed, blocking manual thruster
// checks.
bool ever_healthy = false;
bool prev_armed = false;
uint32_t last_ctrl_us = 0;

constexpr uint32_t kPrintIntervalMs = 100;
uint32_t last_print_ms = 0;

// Estimate + setpoints -> mixed thruster commands.
static void controlStep(uint32_t now_us) {
    const bool armed = safety.armed();
    const bool healthy = estimator.healthy();
    const VehicleState& s = estimator.state();

    // Arm rising edge: capture "hold here" — submerge, level off, arm, and it
    // holds THIS depth and heading. (Setpoints can be stepped over the
    // parameter path.)
    if (armed && !prev_armed) {
        controller.captureHold(s);
        last_ctrl_us = now_us;
        Serial.printf("HOLD engaged: depth z=% .2f m  heading=% .1f deg\n",
                      s.altitude_m, s.yaw_rad * RAD_TO_DEG);
    }
    prev_armed = armed;

    if (armed && healthy) {
        const float dt = (now_us - last_ctrl_us) * 1e-6f;
        last_ctrl_us = now_us;
        if (dt <= 0.0f || dt > 0.2f) {
            return;  // implausible gap; skip rather than kick the integrators
        }
        ControlOutput wrench;
        controller.update(s, dt, wrench);  // authority applied inside
        MotorOutput mo;
        mixer.mix(wrench, mo);
        for (uint8_t i = 0; i < mo.count; ++i) {
            motors.setMotor(i, mo.value[i]);
        }
    } else {
        // Not holding: keep integrators clean so the next arm starts fresh.
        controller.reset();
        last_ctrl_us = now_us;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Tardigrade FC starting (ROBOSUB)...");

    if (HardwareWatchdog::resetWasWatchdog()) {
        Serial.println("!! previous boot ended in a WATCHDOG RESET !!");
    }

    // Motors first: floating ESC pins are undefined. begin() parks every channel
    // at its stop pulse and holds it long enough for the ESCs to accept idle.
    Serial.printf("Arming %u ESC outputs (holding stop)...\n", kMotorCount);
    if (!motors.begin()) {
        Serial.println("ESC output setup FAILED - check pins");
    }

    command_link.setJetsonLink(&jetson);
    command_link.setParameterSink(&controller);  // live PID tuning
    controller.loadFromFlash();  // overlay saved gains onto compiled defaults
    estimator.begin();
    Serial.println("waiting for Jetson pose over the ROS link...");

    // Watchdog LAST: the ESC idle hold blocks for seconds.
    if (HardwareWatchdog::begin(1)) {
        Serial.println("hardware watchdog armed (1 s)");
    } else {
        Serial.println("hardware watchdog FAILED TO ARM");
    }

    Serial.println("ready - connect the ground station");
}

void loop() {
    const uint32_t now_us = micros();
    HardwareWatchdog::feed();

    estimator.update(now_us);
    command_link.update(now_us, estimator.state());
    safety.update(now_us);

    // Sensor-timeout failsafe: once the estimate has been healthy, losing it
    // while armed disarms.
    if (estimator.healthy()) {
        ever_healthy = true;
    } else if (safety.armed() && ever_healthy) {
        safety.disarm(DisarmReason::SensorTimeout);
    }

    controlStep(now_us);

    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms < kPrintIntervalMs) {
        return;
    }
    last_print_ms = now_ms;

    const VehicleState& s = estimator.state();
    Serial.printf(
        "ROBOSUB rpy=% 7.2f % 7.2f % 7.2f [deg] alt=% .2f healthy=%d armed=%d "
        "link=%s crc_err=%lu\n",
        s.roll_rad * RAD_TO_DEG, s.pitch_rad * RAD_TO_DEG,
        s.yaw_rad * RAD_TO_DEG, s.altitude_m,
        estimator.healthy(), safety.armed(),
        safety.linkLost() ? "lost" : "ok",
        (unsigned long)command_link.crcErrors());
}
