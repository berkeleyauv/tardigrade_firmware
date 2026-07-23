Tardigrade is a modular embedded flight controller framework for esp-based systems. It provides reusable sensor interfaces, state estimation, control, communication, and safety while remaining independent of any specific robot

High-level architecture

                User Commands
                      │
                      ▼
              Flight Mode Manager
                      │
                      ▼
              Desired Vehicle State
                      │
                      ▼
                Controller (PID)
                      │
                      ▼
                    Mixer
                      │
                      ▼
                Motor Manager
                      │
                      ▼
                    ESCs


Sensor flow

IMU
ToF
Battery
GPS (future)
Camera (future)

        │
        ▼
     Drivers
        │
        ▼
    State Estimator
        │
        ▼
   Vehicle State


Modules

    - HAL
        - Purpose: Hiding the esp
        - this package own GPIO, PWM, SPI, UART, I2C, Timers, etc.
        - STATUS: intentionally NOT a hand-written module. The Arduino-ESP32
          framework (Wire, ledc*, pinMode) already fills this role, and
          portability to a non-Arduino target is not a project goal. Drivers
          call the framework directly. A HAL abstracts across CHIPS (hiding
          register maps); it is not plug-and-play and does not auto-detect what
          is wired where — that knowledge is fixed, because the mixer depends on
          which motor is on which pin. The swap-the-sensor abstraction we
          actually need lives in the driver interfaces (IMotorSink), not here.
          The empty src/hal/ directory is a transfer leftover, not pending
          work. See docs/hardware.md.
    - Drivers
        - Purpose: Understanding the hardware
        - owns IMU, TOF, ESC, Battery, LED, etc.
    - Estimator
        - Purpose: Turn noisy sensor data into the robot's best estimate of it's state
        - Input: TOF, IMU, future sensors....
        - Output: roll, pitch, yaw, quaternion, angular vel, altitude
    - Controller
        - Purpose: Compare desired state versus actual state and compute corrections
    - Mixer
        - Purpose: Convert desired forces into motor outputs
        - different robots would use different mixers
    - Motor Manager
        - Purpose: Actually send motor outputs
        - owns arming, disarming, idle throttling, esc update
    - Communication
        - Purpose: connect flight controller to the outide world
        - Interfaces: USB, Wifi, Bluetooth, ESP-NOW
        - owns commands and telemetry
    - Safety
        - Purpose: preventing bad things
        - owns kill switch, watchdog, battery failsafe, sensor timeout, emergency stop
    - Parameters
        - Store configuration
        - ROLL_P, ROLL_I, ROLL_D, IMU_OFFSET_X, MOTOR_IDLE, BATTERY_WARNING


Failsafe layering (watchdogs)

A watchdog detects that something has STOPPED HAPPENING and acts on it. The
defining property is that it is triggered by absence, not by an event: something
must keep proving it is alive, and silence is the failure signal.

There is no single watchdog. Each layer answers a different question about a
different subject, and collapsing them would either ground the vehicle for a
harmless condition or miss a real fault.

| Layer | Detects | Window | Action |
|---|---|---|---|
| `ExternalEstimator` (`stale_`) | Jetson pose gone stale | 100 ms | `healthy()` false |
| `Safety` link deadman | ground station gone | 300 ms | disarm + stop motors |
| `HardwareWatchdog` (TWDT) | the control loop itself hung | 1 s | reset the chip |

(Transitional: once control moves to the Jetson — see
tardigrade_ws/docs/jetson_control_architecture.md — the pose link and
`ExternalEstimator`'s row retire with it; the ESP keeps the deadman and
watchdog regardless, since those protect the actuator layer directly.)

Two rules follow from the shape of this, and both are easy to break by
accident:

1. Every one of these must be POLLED, not checked on receipt. A timeout
   evaluated only when data arrives can never fire on a link that has gone
   silent, which is the single case it exists for. This is why
   `Safety::update()` is called unconditionally every tick.

2. Health is not validity. A sensor that answers promptly with "I see nothing"
   is healthy; its reading is merely unusable. Hovering above the ToF ceiling
   must not be reported as a failed sensor, or the vehicle failsafes at the top
   of every hop.

The last row is different in kind from the other two. Both of them are code
inside loop(); if loop() stops advancing they fail silently and simultaneously.
Plausible causes are ordinary, not exotic — the serial ISR wedging, a stalled
write, or any of the usual ways an embedded loop can hang.

That matters because the ESCs are driven by the LEDC peripheral, which generates
its waveform in hardware. A hung CPU does not stop the pulse train: LEDC keeps
emitting the last commanded duty cycle indefinitely, so a motor at 30% stays at
30% with nothing left running that could countermand it. Software cannot stop a
motor once the software has stopped.

Only the hardware watchdog sits outside the CPU's control, and its recovery path
is physical rather than logical:

    loop() hangs -> TWDT expires -> chip resets -> GPIOs revert to high-Z
                 -> ESC loses its signal -> motor stops

The trade-off is stated rather than hidden: the action is a full reset. On a
bench that is correct. In flight it means falling, but the alternative is an
aircraft with uncommandable motors. A hung flight controller has already failed;
the watchdog only decides how.

Ordering constraint: arm the hardware watchdog LAST in setup(). Gyro
calibration and the ESC idle hold block for seconds by design and would trip it
before the control loop ever runs.