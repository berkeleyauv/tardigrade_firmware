# Hardware Reference

Single source of truth for **wiring and bring-up**: which pin does what, which
peripheral each driver claims, and the fixed parameters of each bus.

The authoritative values live in code as named constants — this table can drift,
so the "Source" column names the file that actually decides each one. When they
disagree, the code wins and this doc is the bug.

Items marked **⚠ planned** are designed and built but not yet instantiated in
`main.cpp`; treat their pins/addresses as proposals until wired.

## On the "HAL"

This project has **no hand-written hardware abstraction layer**, and that is
deliberate — see the note under Modules in [architecture.md](architecture.md).
The Arduino-ESP32 framework (`Wire`, `ledc*`, `pinMode`) is the HAL. Drivers call
it directly; portability across MCUs is not a project goal, and the
`IImuSource` / `IRangeSensor` / `IMotorSink` interfaces already provide the
swap-the-sensor abstraction that matters. The empty `src/hal/` directory is a
leftover from the workspace transfer, not pending work.

## Target

| | |
|---|---|
| MCU | ESP32 (`esp32dev`) |
| Framework | Arduino-ESP32 core 2.0.17 (ESP-IDF 4.4) |
| C++ standard | C++17 (see [platformio.ini](../firmware/platformio.ini)) |
| USB serial | 115200 baud — carries the ground link AND human-readable logs |

## Pin allocation

| GPIO | Function | Notes | Source |
|---|---|---|---|
| 21 | I²C SDA | shared bus (IMU + ToF) | `Icm20948.cpp` |
| 22 | I²C SCL | shared bus | `Icm20948.cpp` |
| 25 | ESC 0 signal | motor/thruster 0 | `main.cpp` |
| 26 | ESC 1 signal | | `main.cpp` |
| 27 | ESC 2 signal | | `main.cpp` |
| 33 | ESC 3 signal | | `main.cpp` |
| 16 | ToF A XSHUT | ⚠ planned | example only |
| 17 | ToF B XSHUT | ⚠ planned | example only |

**Pin choices are constrained, not arbitrary.** Avoid the strapping pins
(0, 2, 12, 15) — driving them at boot changes the ESP32's boot mode. GPIO 34–39
are input-only and cannot drive an ESC or an XSHUT line. The ESC pins were
chosen to steer clear of both. Robosub needs 8 ESC pins; only 4 are assigned
today.

## I²C bus

One shared bus at **400 kHz** (fast mode). Every device hangs off GPIO 21/22.

| Device | Address | How set | Source |
|---|---|---|---|
| ICM-20948 IMU | `0x69` | AD0 pin high (SparkFun default) | `Icm20948` |
| VL53L0X ToF A | `0x30` ⚠ planned | reassigned at boot (see below) | `Vl53l0x` |
| VL53L0X ToF B | `0x31` ⚠ planned | reassigned at boot | `Vl53l0x` |

### The VL53L0X address-collision procedure

Every VL53L0X powers up at `0x29` and has no address pins, so two on one bus
collide. `Vl53l0x::beginAll()` resolves it with the XSHUT lines:

1. Hold **all** sensors in reset (XSHUT low).
2. Release one, reassign it to a unique address over I²C.
3. Only then release the next.

This is why each ToF needs its own XSHUT GPIO, and why `beginAll()` must be used
instead of calling `begin()` per sensor. `begin()` is idempotent so a later
call cannot drop a sensor back to `0x29`.

## ESC / PWM

Driven by the LEDC peripheral. Signal only — the ESC does the motor power.

| Parameter | Value | Source |
|---|---|---|
| Frame rate | 50 Hz | `EscPwm.h` (`kEscFrameHz`) |
| Resolution | 16-bit | `EscPwm.h` (`kEscPwmBits`) |
| LEDC channels | 0, 2, 4, 6 | `MotorManager.cpp` — every other channel, one timer each |
| Startup idle hold | 2000 ms at stop | `MotorManager.cpp` (`kEscArmHoldMs`) |

Two pulse conventions, selected by `EscMode` at construction. Getting this wrong
turns every stop command into full reverse — see the note on `MotorOutput` in
[types.h](../firmware/include/core/types.h).

| Mode | Reverse | Stop | Forward | Use |
|---|---|---|---|---|
| `Unidirectional` | — | 1000 µs | 2000 µs | Hopcopter propellers |
| `Bidirectional` | 1100 µs | 1500 µs | 1900 µs | Robosub thrusters |

`MotorOutput.value` is signed **−1..+1** (0 = stopped) for both; the hopcopter
never commands a negative.

## Fixed timing constants

Collected because they interact — the health timeouts must be looser than the
sensor sample periods, or a healthy sensor reads as failed (see the failsafe
layering table in [architecture.md](architecture.md)).

| Constant | Value | Meaning | Source |
|---|---|---|---|
| I²C clock | 400 kHz | fast-mode | `Icm20948.cpp` |
| ToF ranging period | 20 ms (~50 Hz) | per-sensor sample rate | `Vl53l0x.cpp` |
| ToF timing budget | 20 ms | integration time | `Vl53l0x.cpp` |
| IMU health timeout | 100 ms | silence ⇒ IMU unhealthy | `Icm20948.h` |
| ToF health timeout | 200 ms | silence ⇒ ToF unhealthy | `Vl53l0x.h` |
| Link deadman | 300 ms | host silence ⇒ disarm | `Safety.h` |
| Hardware watchdog | 1 s | loop hang ⇒ chip reset | `main.cpp` |

## Fixed capacities

| Constant | Value | Source |
|---|---|---|
| `kMaxMotors` | 8 | `types.h` — sized for the robosub |
| `kMaxRangeSensors` | 2 | `types.h` — hopcopter's two downward ToFs |

## Protocols

Documented elsewhere; pointers so this stays the one place you start:

- **Ground link** (framing, message types, CRC) —
  [communication.md](communication.md); authoritative definition in
  [Protocol.h](../firmware/include/comms/Protocol.h).
- **Jetson → ESP32 pose link** (DDS / XRCE-DDS / micro-ROS) —
  [ros_link.md](ros_link.md).
