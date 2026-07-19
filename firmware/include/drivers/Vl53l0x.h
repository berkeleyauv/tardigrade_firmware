#pragma once
//
// Vl53l0x — IRangeSensor driver for the hopcopter's downward VL53L0X ToF
// breakouts.
//
// THE ADDRESS PROBLEM. Every VL53L0X powers up at I2C address 0x29, and the
// chip has no address pins — the address can only be changed in software, by
// talking to it at 0x29. With two on one bus they collide immediately. The fix
// is the XSHUT pin: hold every sensor in reset, then release them ONE AT A
// TIME, reassigning each to a unique address before waking the next. That
// sequencing is what beginAll() exists to do; wire XSHUT on each board to its
// own GPIO or this driver cannot work.
//
// Reads are non-blocking. The sensor produces samples far slower than the
// control loop, so read() returning false usually means "nothing new yet",
// which the IRangeSensor contract treats as normal rather than a fault.

#include "drivers/IRangeSensor.h"

#include "Adafruit_VL53L0X.h"

namespace tardigrade {

class Vl53l0x : public IRangeSensor {
public:
    // `xshut_pin` must be wired to this board's XSHUT. `address` is the unique
    // I2C address to assign (anything free that isn't 0x29 — 0x30, 0x31, ...).
    //
    // `max_range_m` defaults deliberately LOW. The VL53L0X is specified to 2 m,
    // but that is a best case on a matte target in dim light; over dark or
    // glossy floors, or outdoors, usable range collapses well below it. The
    // estimator rejects readings at or past this value, so setting it too high
    // means feeding the filter garbage at exactly the moment it is least able
    // to detect that. Raise it only after measuring your own surfaces.
    Vl53l0x(int xshut_pin, uint8_t address, float max_range_m = 1.2f);

    bool begin() override;
    bool read(RangeData& out) override;
    bool healthy() const override;
    float maxRange() const override { return max_range_m_; }

    // Hold the sensor in reset so a neighbour can be readdressed safely.
    void standby();

    // Bring up several VL53L0Xs sharing one bus, resolving the 0x29 collision.
    // Use this instead of calling begin() on each when you have more than one.
    static bool beginAll(Vl53l0x* const* sensors, uint8_t count);

private:
    bool bringUp();

    Adafruit_VL53L0X dev_;
    int xshut_pin_;
    uint8_t address_;
    float max_range_m_;
    uint32_t last_ok_ms_ = 0;
    bool initialized_ = false;
};

// Without a completed measurement for this long, the chip counts as failed.
// Generous next to the ~20 ms ranging period so ordinary jitter is not a fault.
inline constexpr uint32_t kRangeHealthTimeoutMs = 200;

}  // namespace tardigrade
