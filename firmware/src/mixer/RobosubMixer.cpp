#include "mixer/RobosubMixer.h"

namespace tardigrade {

namespace {
// Columns: surge, sway, heave, roll, pitch, yaw — the ControlOutput axes.
// Rows: thrusters 0..7, matching kMotorPins order (slots 1..8 of the JSON).
struct MixRow {
    float surge, sway, heave, roll, pitch, yaw;
};

constexpr MixRow kMix[RobosubMixer::kThrusters] = {
    // slot 1 front_left_vertical  (pin 21)
    { 0.0f,  0.0f,  1.0f,  1.0f, -1.0f,  0.0f},
    // slot 2 front_right_vertical (pin 19)
    { 0.0f,  0.0f,  1.0f, -1.0f, -1.0f,  0.0f},
    // slot 3 back_left_vectored   (pin 27)
    { 1.0f,  1.0f,  0.0f,  0.0f,  0.0f, -1.0f},
    // slot 4 front_right_vectored (pin 18)
    { 1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f},
    // slot 5 front_left_vectored  (pin 5)
    { 1.0f, -1.0f,  0.0f,  0.0f,  0.0f, -1.0f},
    // slot 6 back_left_vertical   (pin 14)
    { 0.0f,  0.0f,  1.0f,  1.0f,  1.0f,  0.0f},
    // slot 7 back_right_vectored  (pin 12)
    { 1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f},
    // slot 8 back_right_vertical  (pin 26)
    { 0.0f,  0.0f,  1.0f, -1.0f,  1.0f,  0.0f},
};

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

void RobosubMixer::mix(const ControlOutput& w, MotorOutput& out) {
    out.timestamp_us = w.timestamp_us;
    out.count = kThrusters;
    for (uint8_t i = 0; i < kThrusters; ++i) {
        const MixRow& m = kMix[i];
        const float v = m.surge * w.force.x + m.sway * w.force.y +
                        m.heave * w.force.z + m.roll * w.torque.x +
                        m.pitch * w.torque.y + m.yaw * w.torque.z;
        out.value[i] = clampf(v, -1.0f, 1.0f);
    }
}

}  // namespace tardigrade
