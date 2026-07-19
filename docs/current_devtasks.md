Step 1
HAL
Reason:
Everything depends on it.
Implement:
GPIO
PWM
I2C
SPI
UART
No robot logic.

Step 2
Drivers
Implement:
IMU
ESC
Battery
ToF
At the end:
You should be able to print sensor data.

Step 3
Scheduler
Implement a deterministic scheduler.
No RTOS yet.

Step 4
Data Structures
Actually write:
ImuData
VehicleState
DesiredState
ControlOutput
MotorOutput
Notice these were already designed.

Step 5
Estimator
Implement:
Mahony
or
Madgwick
Input:
ImuData
Output:
VehicleState

Step 6
Motor Manager
Control ESCs.
Test every motor individually.

Step 7
Mixer
Convert:
Desired torques
↓
Motor outputs

Step 8
PID Controller
Now connect:
VehicleState
DesiredState
↓
ControlOutput

Step 9
Safety
Implement:
Arm/disarm
Watchdog
Sensor timeout
Battery monitoring

Step 10
Communication
Now the laptop can:
Arm
Disarm
Read telemetry
Tune PID
Start/stop logging

Step 11
First stabilization test
This is the first time everything comes together.
Clamp the frame to a bench.
Tilt it by hand.
The controller should command the correct motors to oppose the motion.

