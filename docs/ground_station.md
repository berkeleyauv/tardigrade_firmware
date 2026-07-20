# Ground Station

The dashboard ([tools/dashboard.html](../tools/dashboard.html)) shows attitude,
heading, ToF, and telemetry, and drives arm/disarm, per-motor tests, and (later)
PID tuning. It runs two ways depending on where the ESP is plugged in.

## Which mode do I want?

| | Local USB | Remote / headless |
|---|---|---|
| ESP plugged into | the machine at your desk | the Jetson (you SSH in) |
| Transport | WebSerial (browser → USB) | WebSocket (browser → backend → serial) |
| Who can view | only that machine | anyone on the network, by URL |
| Browser | Chrome / Edge only | **any** browser |
| Command | open the file | run `gcs_server.py` |

The Jetson is headless, so the ESP-on-Jetson case is the **remote** mode. That is
the one you asked for and the main path below.

## Remote mode (headless Jetson, anyone with the URL)

The backend owns the one serial port and relays raw wire bytes to every browser,
so the page reuses the exact parser it would over USB — only the byte source
changes. WebSerial cannot do this: it only reaches a *local* USB port, which is
why a plain file won't work when the ESP is on a different machine.

**One-time setup on the Jetson:**

```bash
pip install aiohttp pyserial
```

**Every session — hopcopter, or robosub without pose injection:**

```bash
cd tardigrade_firmware/tools
python3 gcs_server.py --serial /dev/ttyUSB0
```

**Robosub — also inject the EKF pose onto the same link:**

```bash
source /opt/ros/humble/setup.bash          # so rclpy + nav_msgs are importable
source ~/tardigrade_ws/install/setup.bash  # for the odometry topic
python3 gcs_server.py --serial /dev/ttyUSB0 --ros
```

`--ros` subscribes to `/tardigrade/state/odometry/filtered` and streams `Pose`
frames to the ESP **whether or not any browser is connected** — so this one
command covers autonomy and the operator dashboard together. (For pose injection
with no web stack at all, `pose_bridge.py --serial /dev/ttyUSB0` does only that.)

**Then, from any machine on the network:**

```
http://<jetson-hostname>:8080/
```

e.g. `http://tardigrade-jetson.local:8080/` or `http://192.168.1.42:8080/`. Click
**Connect (server)** — the page finds the backend automatically at the host it
was served from. Multiple people can open it at once; all see the same telemetry,
any can send commands.

### Finding the address / opening the port

- Hostname: run `hostname` on the Jetson; on the same LAN try `<name>.local`.
- IP: `hostname -I` on the Jetson.
- `--serial` device: `ls /dev/ttyUSB* /dev/ttyACM*` (often `/dev/ttyUSB0`).
- Change the web port with `--http-port 9000`; check the health endpoint at
  `http://<host>:8080/status`.
- If a browser cannot reach it, the Jetson firewall is the usual cause:
  `sudo ufw allow 8080/tcp`.

Access is unauthenticated and unencrypted — fine for a lab LAN. Do not expose it
to the open internet; if you need remote access, tunnel over SSH:
`ssh -L 8080:localhost:8080 jetson` then browse `http://localhost:8080/`.

## Local mode (ESP on your own machine)

Open [tools/dashboard.html](../tools/dashboard.html) directly in Chrome or Edge,
or serve it (`python -m http.server` in `tools/`, then
`http://localhost:8000/dashboard.html`). Click **Connect** and pick the ESP's
port. WebSerial is Chromium-only.

## Demo mode

Either mode has a **Demo** button: synthetic telemetry through the real render
path, no hardware. Good for trying the UI or developing it.

## Safety behaviour to know

- The page heartbeats **only while armed.** Closing or even backgrounding the tab
  starves the firmware deadman, which disarms on its own (~300 ms). In remote
  mode the *browser* stops heartbeating, so this still holds per-viewer — but
  note the backend does not itself heartbeat, by design.
- Disconnect and page-close send an explicit DISARM first.
- Motor-test sliders cap at 30%, matching `Safety::test_limit_`.

## Command-line client

[tools/gcs.py](../tools/gcs.py) is a terminal client and protocol reference for
scripted tests (`--selftest` needs no hardware). All host tools share the wire
implementation in [tools/tardigrade_protocol.py](../tools/tardigrade_protocol.py).
