ImuData

Purpose:

Raw sensor sample from the IMU.

Owner:

IMU Driver

Consumers:

Estimator

Fields:

Timestamp
Accelerometer (x, y, z)
Gyroscope (x, y, z)
Temperature (optional)
VehicleState

Purpose:

Best estimate of the robot's state.

Owner:

Estimator

Consumers:

Controller
Telemetry
Safety

Fields:

Roll
Pitch
Yaw
Quaternion
Angular velocity
Altitude (future)

Do this for:

DesiredState
ControlOutput
MotorOutput
BatteryState
Parameter
Packet