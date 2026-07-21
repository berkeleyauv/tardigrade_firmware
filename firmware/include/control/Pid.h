#pragma once
//
// Pid — one scalar PID channel. Reused for every controlled axis.
//
// Two deliberate choices, both about avoiding self-inflicted noise and windup:
//
//   * DERIVATIVE ON MEASUREMENT, not on error. The caller passes the measured
//     RATE of the process variable (which the estimator already provides:
//     angular_velocity for attitude, vertical_velocity for depth). This avoids
//     differentiating a noisy position signal, and avoids "derivative kick" — a
//     spike in D the instant a setpoint changes, which differentiating the
//     error would produce. It is the whole reason VehicleState carries those
//     velocities.
//
//   * ANTI-WINDUP by clamping the integrator AND freezing it while the output
//     is saturated. A submerged thruster spends a lot of time saturated; without
//     this the integral marches off and the vehicle overshoots hard when it
//     finally catches the setpoint.

namespace tardigrade {

class Pid {
public:
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

    // Individual accessors for live tuning (one gain per SetParameter message).
    void setKp(float v) { kp_ = v; }
    void setKi(float v) { ki_ = v; }
    void setKd(float v) { kd_ = v; }
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

    // Clamp on the integral TERM (ki*integral), in output units. Keep it well
    // below the output limit so the integrator can only ever trim, not dominate.
    void setIntegralLimit(float limit) { i_limit_ = limit; }

    // Clamp on the returned output, in normalized effort (-1..1 for our axes).
    void setOutputLimit(float limit) { out_limit_ = limit; }

    // error = setpoint - measurement.
    // measurement_rate = d(measurement)/dt, in the same units per second.
    // dt seconds since the last call.
    float update(float error, float measurement_rate, float dt);

    // Zero the integrator. Call when disarming or re-engaging a hold, so stale
    // accumulated error cannot kick the vehicle on the next arm.
    void reset() { integral_ = 0.0f; }

    float integral() const { return integral_; }

private:
    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float integral_ = 0.0f;
    float i_limit_ = 0.3f;
    float out_limit_ = 1.0f;
};

}  // namespace tardigrade
