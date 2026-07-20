# Ground-station tools

Host-side tools that speak the wire protocol from
[docs/communication.md](../docs/communication.md)
(`firmware/include/comms/Protocol.h`). Two clients, same protocol:

| Tool | What it is | Use it for |
|---|---|---|
| `dashboard.html` | WebSerial dashboard (browser) | Live instruments, motor test, bench monitoring |
| `gcs.py` | Python console | Scripted tests, protocol selftest, quick bench pokes |

**One at a time.** The serial port is exclusive — close the PlatformIO monitor
and `gcs.py` before connecting the dashboard, and vice versa. A "port busy /
failed to open" error means something else still holds the port.

---

## dashboard.html

Requires **Chrome or Edge** (WebSerial is Chromium-only; Firefox/Safari will
show an unsupported banner). No install, no dependencies.

### Running it

Two options:

1. **Open the file directly** — double-click `tools/dashboard.html` or drag it
   into Chrome. No server involved; `file://` pages count as a secure context
   in Chromium, so WebSerial works.

2. **Serve it over localhost** (fallback if your browser refuses WebSerial on
   `file://`). From the **repo root**:

   ```
   python -m http.server 8000
   ```

   then browse to <http://localhost:8000/tools/dashboard.html>.
   Stop the server with `Ctrl+C`. This is a plain static file server — the
   dashboard has no backend; all logic runs in the page.

### Using it

- **Demo** — synthetic telemetry through the real render path. Inspect every
  instrument with no hardware attached. Nothing is transmitted.
- **Connect** — Chrome shows a port picker; choose the ESP32 (115200 baud is
  set by the page). Telemetry starts immediately via `GET_STATE` polling.
- Tabs: **HOPCOPTER** (attitude, heading, ToF A/B, altitude/v-speed, 4 motor
  sliders, live charts) and **ROBOSUB** (heading, pose, 8 thruster sliders).
- **ARM** before motor sliders do anything; every command is acknowledged in
  the Events log, including the refusal reason when the firmware says no.
- The **Raw serial text** panel shows the firmware's printf output, which
  shares the port with the binary protocol.

### Safety behaviour (mirrors the firmware — do not "fix" it)

- Heartbeats are sent **only while armed**. Closing or backgrounding the tab
  starves the firmware deadman (300 ms) and the vehicle disarms itself.
  Backgrounding counts because browsers throttle timers in hidden tabs.
- Disconnecting and closing the page both send DISARM first.
- Motor sliders cap at 30%, matching `Safety::test_limit_`. Raising the cap is
  a firmware decision, not a UI edit.

**First test with real hardware:** arm from the dashboard, then kill the tab.
The firmware should print `[motor] ALL STOP` on its own within ~300 ms. That
watchdog is the reason it is safe to have a UI commanding motors at all.

---

## gcs.py

Python 3; needs `pyserial` for hardware use (`pip install pyserial`), nothing
for the selftest.

```
python tools/gcs.py --selftest              # protocol round-trip, no hardware
python tools/gcs.py --port COM5             # interactive console
python tools/gcs.py --port COM5 --motor 2 --value 0.15   # one-shot spin test
```

Interactive commands: `arm`, `disarm`, `m <index> <0..1>`, `state`, `quit`.

Like the dashboard, it heartbeats only while armed — `Ctrl+C` while a motor is
spinning is a deliberate way to prove the firmware watchdog fires.

`gcs.py` is also the protocol's **reference implementation**: its selftest pins
the CRC to the standard CCITT-FALSE check value (`"123456789"` → `0x29B1`). If
you change the protocol, change `Protocol.h`/`.cpp`, `gcs.py`, and
`dashboard.html` together, and keep the selftest passing.
