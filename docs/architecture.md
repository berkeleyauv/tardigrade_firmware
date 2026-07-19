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