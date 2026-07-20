#include "safety/HardwareWatchdog.h"

#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

namespace tardigrade {

namespace {
bool g_active = false;
}

bool HardwareWatchdog::begin(uint32_t timeout_s) {
    if (timeout_s == 0) {
        timeout_s = 1;
    }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Arduino core 3.x / ESP-IDF 5.x: config struct, milliseconds.
    const esp_task_wdt_config_t cfg = {
        .timeout_ms = timeout_s * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_err_t err = esp_task_wdt_init(&cfg);
#else
    // Arduino core 2.x / ESP-IDF 4.x: seconds, panic-on-expiry flag.
    esp_err_t err = esp_task_wdt_init(timeout_s, true);
#endif

    // The framework may have initialised the TWDT already. That is not a
    // failure — the timer exists either way, which is all we need.
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return false;
    }

    // Subscribe the calling task (loopTask). Until a task is added, the timer
    // watches nothing.
    err = esp_task_wdt_add(nullptr);
    if (err != ESP_OK && err != ESP_ERR_INVALID_ARG) {
        return false;
    }

    g_active = true;
    esp_task_wdt_reset();
    return true;
}

void HardwareWatchdog::feed() {
    if (g_active) {
        esp_task_wdt_reset();
    }
}

bool HardwareWatchdog::active() {
    return g_active;
}

bool HardwareWatchdog::resetWasWatchdog() {
    const esp_reset_reason_t reason = esp_reset_reason();
    return reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT ||
           reason == ESP_RST_WDT;
}

}  // namespace tardigrade
