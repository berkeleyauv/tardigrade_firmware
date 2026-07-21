#include <Arduino.h>
#include "comms/CommandLink.h"
#include "estimator/IStateEstimator.h"
#include "motors/MotorManager.h"
#include "safety/HardwareWatchdog.h"
#include "safety/Safety.h"

// Vehicle selection happens ONCE here, at compile time, via a build flag — the
// factory promised in docs/estimator.md. Everything below the IStateEstimator
// seam (Safety, CommandLink, the control loop) is identical for both vehicles;
// only the estimator, the motor hardware, the pose source, and the control law
// differ.
//
//   default             -> hopcopter (onboard Mahony fusion, I2C IMU)
//   -D VEHICLE_ROBOSUB   -> robosub  (external pose from the Jetson EKF)

using namespace tardigrade;

#if defined(VEHICLE_ROBOSUB)
#include "comms/JetsonLink.h"
#include "control/RobosubController.h"
#include "estimator/ExternalEstimator.h"
#include "mixer/RobosubMixer.h"

// Thruster pins and order from tardigrade_ws esp_thruster_map.json. No onboard
// I2C IMU on the sub, so the I2C pins are free to drive thrusters.
constexpr uint8_t kMotorPins[] = {21, 19, 27, 18, 5, 14, 12, 26};
constexpr EscMode kEscMode = EscMode::Bidirectional;  // reversible thrusters
const char* const kVehicleName = "ROBOSUB";

JetsonLink jetson;
ExternalEstimator vehicle_estimator(jetson);
RobosubController controller;
RobosubMixer mixer;

// The controller owns its hold target and its output authority cap; both are
// live-tunable over the parameter path. See RobosubController / Parameters.h.
bool prev_armed = false;
uint32_t last_ctrl_us = 0;
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

// Latches true the first time the estimate is healthy. The sensor-timeout
// failsafe only fires after this — otherwise a bench robosub with no Jetson
// (never healthy) would disarm itself the instant it armed, blocking manual
// thruster checks.
bool ever_healthy = false;

constexpr uint32_t kPrintIntervalMs = 100;
uint32_t last_print_ms = 0;

static void beginSensors() {
#if defined(VEHICLE_ROBOSUB)
    command_link.setJetsonLink(&jetson);
    command_link.setParameterSink(&controller);  // live PID tuning
    controller.loadFromFlash();  // overlay saved gains onto compiled defaults
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
        Serial.println("ICM-20948 not found - check wiring and ADR jumper");
    }
#endif
    estimator.begin();
}

// Per-vehicle control law: estimate + setpoints -> mixed thruster commands.
// The hopcopter's controller is not built yet, so its step is currently empty.
static void controlStep(uint32_t now_us) {
#if defined(VEHICLE_ROBOSUB)
    const bool armed = safety.armed();
    const bool healthy = estimator.healthy();
    const VehicleState& s = estimator.state();

    // Arm rising edge: capture "hold here". This is what makes the tuning
    // workflow work — submerge, level off, arm, and it holds THIS depth and
    // heading. (The setpoint can then be stepped live over the parameter path.)
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
#else
    (void)now_us;  // hopcopter controller: TODO
#endif
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.printf("Tardigrade FC starting (%s)...\n", kVehicleName);

    if (HardwareWatchdog::resetWasWatchdog()) {
        Serial.println("!! previous boot ended in a WATCHDOG RESET !!");
    }

    // Motors first: floating ESC pins are undefined. begin() parks every channel
    // at its stop pulse and holds it long enough for the ESCs to accept idle.
    Serial.printf("Arming %u ESC outputs (holding stop)...\n", kMotorCount);
    if (!motors.begin()) {
        Serial.println("ESC output setup FAILED - check pins");
    }

    beginSensors();

    // Watchdog LAST: gyro calibration and the ESC hold block for seconds.
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
    // while armed disarms. Both vehicles use the one mechanism.
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
        "%s rpy=% 7.2f % 7.2f % 7.2f [deg] alt=% .2f healthy=%d armed=%d "
        "link=%s crc_err=%lu\n",
        kVehicleName,
        s.roll_rad * RAD_TO_DEG, s.pitch_rad * RAD_TO_DEG,
        s.yaw_rad * RAD_TO_DEG, s.altitude_m,
        estimator.healthy(), safety.armed(),
        safety.linkLost() ? "lost" : "ok",
        (unsigned long)command_link.crcErrors());
}
