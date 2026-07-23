# Hardware Reference

Single source of truth for **wiring and bring-up**: which pin does what, which
peripheral each driver claims, and the fixed parameters of each bus.

The authoritative values live in code as named constants — this table can drift,
so the "Source" column names the file that actually decides each one. When they
disagree, the code wins and this doc is the bug.

## On the "HAL"

This project has **no hand-written hardware abstraction layer**, and that is
deliberate — see the note under Modules in [architecture.md](architecture.md).
The Arduino-ESP32 framework (`Wire`, `ledc*`, `pinMode`) is the HAL. Drivers call
it directly; portability across MCUs is not a project goal, and the
`IMotorSink` interface already provides the abstraction that matters. The empty
`src/hal/` directory is a leftover from the workspace transfer, not pending
work.

## Target

| | |
|---|---|
| MCU | ESP32 (`esp32dev`) |
| Framework | Arduino-ESP32 core 2.0.17 (ESP-IDF 4.4) |
| C++ standard | C++17 (see [platformio.ini](../firmware/platformio.ini)) |
| USB serial | 115200 baud — carries the ground link AND human-readable logs |

## Pin allocation

Eight thruster/ESC signal pins, order and geometry from `tardigrade_ws`'s
`esp_thruster_map.json`:

| GPIO | Function | Notes | Source |
|---|---|---|---|
| 21 | ESC 0 signal | thruster slot 1 (front_left_vertical) | `main.cpp` |
| 19 | ESC 1 signal | slot 2 (front_right_vertical) | `main.cpp` |
| 27 | ESC 2 signal | slot 3 (back_left_vectored) | `main.cpp` |
| 18 | ESC 3 signal | slot 4 (front_right_vectored) | `main.cpp` |
| 5 | ESC 4 signal | slot 5 (front_left_vectored) | `main.cpp` |
| 14 | ESC 5 signal | slot 6 (back_left_vertical) | `main.cpp` |
| 12 | ESC 6 signal | slot 7 (back_right_vectored) | `main.cpp` |
| 26 | ESC 7 signal | slot 8 (back_right_vertical) | `main.cpp` |

**Pin choices are constrained, not arbitrary.** Avoid the strapping pins
(0, 2, 12, 15) and GPIO 34–39 (input-only, cannot drive an ESC signal). GPIO 12
is a strapping pin (sets flash voltage) but is safe here because the ESC only
reads it well after boot; it's used because the 8-thruster layout needs every
available pin.

## ESC / PWM

Driven by the LEDC peripheral. Signal only — the ESC does the motor power.

| Parameter | Value | Source |
|---|---|---|
| Frame rate | 50 Hz | `EscPwm.h` (`kEscFrameHz`) |
| Resolution | 16-bit | `EscPwm.h` (`kEscPwmBits`) |
| LEDC channels | 0, 2, 4, 6, 8, 10, 12, 14 | `MotorManager.cpp` — every other channel, one timer each |
| Startup idle hold | 2000 ms at stop | `MotorManager.cpp` (`kEscArmHoldMs`) |
| Pulse convention | `EscMode::Bidirectional` | reversible thrusters — see below |

Getting the pulse convention wrong turns every stop command into full reverse
— see the note on `MotorOutput` in
[types.h](../firmware/include/core/types.h).

| Mode | Reverse | Stop | Forward |
|---|---|---|---|
| `Bidirectional` (in use) | 1100 µs | 1500 µs | 1900 µs |

`MotorOutput.value` is signed **−1..+1**, 0 = stopped.

## Fixed timing constants

Collected because they interact — the health timeouts must be looser than the
sensor/link update periods, or a healthy link reads as failed (see the failsafe
layering table in [architecture.md](architecture.md)).

| Constant | Value | Meaning | Source |
|---|---|---|---|
| Pose freshness timeout | 100 ms | Jetson pose gone stale ⇒ `ExternalEstimator` unhealthy | `ExternalEstimator.h` |
| Link deadman | 300 ms | host silence ⇒ disarm | `Safety.h` |
| Hardware watchdog | 1 s | loop hang ⇒ chip reset | `main.cpp` |

## Fixed capacities

| Constant | Value | Source |
|---|---|---|
| `kMaxMotors` | 8 | `types.h` — sized for the robosub's 8 thrusters |

## Protocols

Documented elsewhere; pointers so this stays the one place you start:

- **Ground link** (framing, message types, CRC) —
  [communication.md](communication.md); authoritative definition in
  [Protocol.h](../firmware/include/comms/Protocol.h).
- **Jetson → ESP32 pose link** (DDS / XRCE-DDS / micro-ROS, why a plain serial
  bridge was chosen) — [ros_link.md](ros_link.md). Transitional — see the note
  in [estimator.md](estimator.md); control (and the pose link) is migrating to
  the Jetson.
