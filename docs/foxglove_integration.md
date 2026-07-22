# Foxglove Integration — Architecture & Message Spec

## Scope

How Foxglove fits around the existing ESP ↔ Jetson bridge: every topic,
parameter, and service; who talks to whom; and how multiple simultaneous
operators/observers see the same live system.

This spec assumes the decision already made: **the ESP is not changing.**
No micro-ROS, no firmware changes, no protocol changes. Foxglove becomes the
full operator interface — observability, tuning, **and control**
(arm/disarm/motor test) — by extending the existing Jetson bridge node.

**Safety model:** the sub has a **physical kill switch**, and that is the
safety backstop for live testing — not a software presence mechanism. Earlier
drafts of this spec kept command authority tied to the webapp's browser-tab
heartbeat specifically to guard against an operator disappearing unnoticed.
With a physical cutoff someone can reach at all times during a live test,
that software guard is a convenience, not the thing keeping people safe, so
control is free to move into ROS/Foxglove. See §"Safety model" below for the
full reasoning and the one procedural rule this still implies.

The webapp ([ground_station.md](ground_station.md)) is retained during the
transition — it's built and works today — and retired once the Foxglove
control path (this spec) is built and proven. See
[ros_link.md](ros_link.md) for why the ESP↔Jetson link is a custom serial
protocol rather than ROS, and [pid_tuning.md](pid_tuning.md) for the tuning
workflow this extends.

## Three-tier picture

```
 ┌───────────┐   custom serial    ┌────────────────────┐   ROS 2 (DDS)   ┌─────────────────────┐
 │   ESP32   │◄──── protocol ────►│   Jetson bridge     │◄───────────────►│  N x Foxglove       │
 │(firmware, │  [A7][5E][TYPE]... │   node (rclpy)      │  topics /       │  clients            │
 │ no ROS,   │                    │   + WebSocket hub   │  parameters /   │  (any laptop, any   │
 │ unchanged)│                    │                     │  services       │  number, own layout,│
 └───────────┘                    └────────────────────┘                  │  incl. arm/disarm)  │
                                          ▲                                └─────────────────────┘
                                          │ WebSocket (transitional)
                                          ▼
                                   ┌──────────────┐
                                   │   Webapp     │  ← bench fallback during the
                                   │(dashboard.html)│    transition; retired once
                                   └──────────────┘    Foxglove control is proven
```

The ESP side of the bridge is exactly what's already built. Everything new
in this document lives in the Jetson bridge node and above it.

## What doesn't change

- Firmware, `Protocol.h`, every message type, `Safety`, the hardware
  watchdog, the sensor-timeout failsafe — untouched.
- The ESP never speaks ROS.
- `gcs_server.py`'s `SerialHub` (owns the one serial port, fans out to
  WebSocket clients) — unchanged, only extended.

## The Jetson bridge node — responsibilities

| Responsibility | Status |
|---|---|
| Own the serial port | existing (`SerialHub`) |
| Subscribe to EKF odometry, forward as `Pose` frames to the ESP | existing (`--ros`) |
| Relay WebSocket bytes ⇄ serial for the webapp (bench fallback) | existing |
| Decode ESP `State` frames and publish as a ROS topic | **new** |
| Expose PID gains/authority/setpoints as ROS 2 parameters; push edits to the ESP | **new** |
| Expose Save/Reset as ROS services | **new** |
| Expose Arm/Disarm as a ROS service | **new** |
| Expose motor test as a ROS topic | **new** |
| Publish link health as diagnostics | **new** |

## ROS interface catalog

### Topics published (Jetson → ROS graph → Foxglove)

| Topic | Type | Rate | Source |
|---|---|---|---|
| `/tardigrade/esp/state` | `tardigrade_interfaces/EspState` (new, below) | ~20 Hz | decoded `State` frames |
| `/tardigrade/esp/diagnostics` | `diagnostic_msgs/DiagnosticArray` | on change / few Hz | CRC errors, pose drops, disarm reason |

### Topics subscribed (ROS graph → Jetson → ESP)

| Topic | Type | Rate | Sink |
|---|---|---|---|
| `/tardigrade/state/odometry/filtered` | `nav_msgs/Odometry` | 30 Hz | repacked into `Pose` frames — unchanged, see [ros_link.md](ros_link.md) |
| `/tardigrade/thrusters/test_cmd` | `std_msgs/Float32MultiArray` | on change | 8 elements, index = thruster, value -1..+1 → individual `SetMotor` frames |

### ROS 2 parameters (declared by the bridge, editable via Foxglove's Parameters panel)

Mirrors [`control/Parameters.h`](../firmware/include/control/Parameters.h)
directly, one parameter per gain:

```
depth.kp   depth.ki   depth.kd
yaw.kp     yaw.ki     yaw.kd
roll.kp    roll.ki    roll.kd
pitch.kp   pitch.ki   pitch.kd
authority
depth_setpoint
heading_setpoint
```

A parameter-change callback on the bridge builds a `SetParameter` frame and
writes it to the ESP — the same translation pattern already proven for pose,
run in the opposite direction.

### Services

| Service | Type | Action |
|---|---|---|
| `/tardigrade/save_parameters` | `std_srvs/Trigger` | sends `SaveParameters` (`0x09`) |
| `/tardigrade/reset_parameters` | `std_srvs/Trigger` | sends `ResetParameters` (`0x0A`), then re-syncs the ROS parameter values to the restored defaults |
| `/tardigrade/set_armed` | `tardigrade_interfaces/SetArmed` | sends `Arm` (`0x02`) or `Disarm` (`0x03`) depending on the request's `armed` field |

`SetArmed.srv` already exists in `tardigrade_interfaces` (from the earlier
PX4 architecture) — `bool armed → bool success, string message`. Reusing it
rather than inventing a new type.

## Message draft

```
# tardigrade_interfaces/msg/EspState.msg
# Mirrors the State (0x80) telemetry frame in Protocol.h.
builtin_interfaces/Time stamp
float32 roll               # rad
float32 pitch              # rad
float32 yaw                # rad
float32 depth               # m, ENU z (VehicleState.altitude_m)
float32 vertical_velocity   # m/s
bool armed
bool state_valid
bool altitude_valid
bool link_ok
bool pose_ok
```

Lives in the existing `tardigrade_interfaces` package alongside
`RobotStatus.msg` — no new package needed. `DiagnosticArray` is a standard
message type; no custom definition required for link health (keys like
`crc_errors`, `pose_drops`, `last_disarm_reason`).

## Data flow walkthroughs

**Telemetry out:**
```
ESP → State frame (serial) → bridge decodes (tp.decode_state, already built)
    → publish EspState on /tardigrade/esp/state
    → every connected Foxglove client sees it live
    → rosbag records it if a recording is running
```

**Live tuning:**
```
Foxglove Parameters panel: operator edits depth.kp
    → ROS parameter-change callback fires on the bridge
    → bridge builds a SetParameter frame (id=0x00, value)
    → serial → ESP → RobosubController::setParameter() → live immediately
    → ESP Acks; bridge logs it
```

**Save / Reset:**
```
Foxglove Service Call panel → /tardigrade/save_parameters (Trigger)
    → bridge sends SaveParameters (0x09) → ESP writes NVS → Acks
    → bridge returns Trigger.Response(success=True)
```

**Arm / Disarm (new):**
```
Foxglove Service Call panel → /tardigrade/set_armed (armed=true)
    → bridge sends Arm (0x02) → ESP's Safety::arm() → Acks
    → bridge returns SetArmed.Response(success, message)
```

**Motor test (new):**
```
Foxglove Publish panel → /tardigrade/thrusters/test_cmd, index 3 set to 0.15
    → bridge builds a SetMotor frame (index=3, value=150) → serial → ESP
    → Safety::commandMotor() clamps to the test authority cap, applies it
```

**A subtlety worth building correctly:** the ESP is the actual source of
truth for gain values, not the bridge's ROS parameter cache. If the webapp
(or a direct `SetParameter`) changes a gain, or `ResetParameters` restores
defaults, the bridge's ROS-side parameter mirror has to be refreshed from
what the ESP reports back — via `Parameter` frames — rather than assuming
its own last-written value is still current. Two paths can write the same
logical value; only the ESP's own state should be trusted as ground truth.

## Safety model: physical kill switch, not a software presence beacon

Earlier drafts of this spec required command authority to stay tied to the
webapp's browser-tab heartbeat, so that an operator disappearing (closed tab,
crashed browser, dead network) would reliably disarm the vehicle within
~300 ms — a software deadman. Building an equivalent inside Foxglove turned
out to be genuinely uncertain: Foxglove ships as both a browser tab (Foxglove
Web, whose tab-lifecycle throttling is the same mechanism the webapp already
relies on) and a standalone Electron app (Foxglove Desktop, whose
window/panel lifecycle when minimized or unfocused is not something we've
verified behaves the same way). Moving command traffic through more hops
(`foxglove_bridge` → ROS graph → this bridge node) also means more places a
"the operator is gone" signal could fail to propagate, independent of whether
the presence mechanism itself works.

**None of that matters given the physical kill switch.** It's a hardware
cutoff, reachable without any software or network layer working correctly —
it is the actual backstop, and the software deadman was only ever a
convenience layered on top of it. With that in place, the added-hops and
unverified-beacon concerns don't block moving control into Foxglove.

**The one rule this implies, not optional:** the kill switch is the backstop
*because a human is positioned to reach it*. **Every live test — bench or
water, webapp or Foxglove — runs with someone explicitly on kill-switch
duty.** This is now a procedural rule, not a software property; it belongs in
[bench_checklist.md](bench_checklist.md) and [pid_tuning.md](pid_tuning.md)
as an explicit pre-test check, not just this doc.

**What's dropped from scope as a result:** the custom Foxglove
presence-beacon extension panel, and the "does Desktop vs. Web behave
differently" investigation. Neither is being built. **What's unaffected:**
the hardware watchdog and the sensor-timeout failsafe stay exactly as built —
they're independent of this decision and still fire regardless of what's on
the operator's screen.

## Example Foxglove layouts

Each teammate opens whichever layout matches their role; all connect to the
same live bridge (or the same `.mcap` recording for review).

**Pilot/tuning layout:** Parameters panel (gains + setpoints) · Service Call
buttons for Arm/Disarm/Save/Reset · Publish panel or slider widget for motor
test · Plot (depth setpoint vs. measured) · Plot (heading setpoint vs.
measured) · Diagnostics panel (link health).

**Observer/telemetry layout:** 3D panel (orientation) · Plot (roll/pitch/yaw)
· Plot (depth/vertical velocity) · Diagnostics panel · Raw Messages panel
(`EspState`) for full visibility. Read-only — no service/publish panels.

**Playback/review layout:** identical panels to Observer, data source swapped
for a recorded `.mcap` — no live connection, no Jetson required.

## What's new to build vs. already exists

| Piece | Status |
|---|---|
| ESP firmware / protocol | unchanged |
| Webapp | retained as bench fallback during the transition; retired once this spec is proven |
| Pose delivery to ESP | already built (`JetsonLink`, `ExternalEstimator`, `gcs_server.py --ros`) |
| `EspState.msg` | new, small |
| Bridge: decode `State` → publish `EspState` | new, small — reuses `tp.decode_state` |
| Bridge: gains as ROS parameters + change callback | new, moderate |
| Bridge: sync ROS parameters from the ESP (not just to it) | new, moderate — the subtlety above |
| Bridge: save/reset services | new, small |
| Bridge: `/tardigrade/set_armed` service | new, small — reuses existing `SetArmed.srv` |
| Bridge: motor-test topic | new, small |
| Bridge: diagnostics publishing | new, small |
| Foxglove layouts (pilot/observer/playback) | new, no code — saved layout configs only |
| Software presence-beacon panel | **dropped** — physical kill switch supersedes it |

## Open decisions before building

- `std_msgs/Float32MultiArray` for motor test vs. a custom typed message —
  leaning standard type for less boilerplate
- Final `EspState` field set — worth adding the PID P/I/D breakdown once
  that firmware/telemetry addition exists (see the tuning-diagnostics
  discussion — not yet built)
- `DiagnosticArray` (standard, more tooling) vs. a custom diagnostics message
  (more explicit typing)
- Telemetry publish rate — match the existing 20 Hz `GetState` poll, or
  slow it for smaller bag files
- Confirm kill-switch-duty is written into the bench checklist and tuning
  guide as an explicit pre-test step, not just implied by this doc
