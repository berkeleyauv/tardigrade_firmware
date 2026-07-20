Scope: the ground link (laptop <-> vehicle) — commands and telemetry.

Implemented in firmware/include/comms/Protocol.h. Host-side clients and how to
run them: tools/README.md (WebSerial dashboard, Python console).

The Robosub's onboard Jetson <-> ESP32 pose link is a separate channel with its
own transport and constraints; see ros_link.md.


Transport

USB Serial (v1)

Future:

Wi-Fi
Bluetooth
ESP-NOW


Packet Layout
Header
Type
Length
Payload
CRC


Commands
ARM
DISARM
SET_MODE
SET_TARGET
SET_PARAMETER
CALIBRATE
GET_STATE


Telemetry
Timestamp
Orientation
Battery
Motor outputs
Flight mode
Status flags