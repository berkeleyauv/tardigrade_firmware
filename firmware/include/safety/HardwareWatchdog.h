#pragma once
//
// HardwareWatchdog — the ESP32's Task Watchdog Timer (TWDT), wrapped.
//
// WDT = Watchdog Timer: a counter in silicon that runs independently of the
// CPU. Firmware must periodically reset it ("feed" it). If the count ever
// expires, the chip resets itself.
//
// WHY THIS IS NOT REDUNDANT WITH Safety. Every other watchdog in this firmware
// — Safety's link deadman, the driver health timeouts, the estimator staleness
// flags — is code that runs inside loop(). If loop() itself stops advancing,
// none of them execute, and every one of them fails silently and
// simultaneously. Plausible causes on this hardware are not exotic: an I2C
// slave holding SDA low locks up the Wire library, and three devices share
// that bus.
//
// That matters specifically because the ESCs are driven by the LEDC PERIPHERAL,
// which generates its waveform in hardware. A hung CPU does not stop the pulse
// train — LEDC keeps emitting the last duty cycle indefinitely, so a motor
// commanded to 30%% stays at 30%% forever with nothing left running that could
// countermand it. Only a watchdog outside the CPU's control can break that.
//
// The recovery chain is: TWDT expires -> chip resets -> GPIOs revert to
// high-impedance inputs -> the ESC loses its signal -> the ESC stops the motor.
//
// TRADE-OFF, stated plainly: the action is a full reset. On a bench that is
// exactly right. In flight it means falling out of the sky — but the
// alternative is an aircraft with uncommandable motors, which is worse. A
// hung flight controller has already failed; this only decides how.

#include <stdint.h>

namespace tardigrade {

class HardwareWatchdog {
public:
    // Start watching the calling task. Call this at the END of setup(), after
    // gyro calibration and sensor bring-up: those block for seconds by design
    // and would trip the watchdog before the control loop ever starts.
    //
    // The ESP-IDF 4.x API takes whole seconds, so `timeout_s` is the real
    // granularity available. Returns false if the timer could not be armed.
    static bool begin(uint32_t timeout_s = 1);

    // Call every loop iteration. Cheap — a register write.
    static void feed();

    // True once begin() has succeeded. feed() is a no-op before then, so it is
    // safe to call unconditionally.
    static bool active();

    // Was the last boot caused by a watchdog expiry rather than a normal power
    // cycle? Worth surfacing: a vehicle that silently reboots mid-test looks
    // like a glitch unless something reports why.
    static bool resetWasWatchdog();
};

}  // namespace tardigrade
