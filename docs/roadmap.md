# Tardigrade Delivery Roadmap

Target: **all four deliverables done by ~Jul 30, 2026** (~10 days from Jul 20).
Assumes multiple pool sessions/week and focused team time. D4 is new scope
added after the original three; it runs on a parallel track and does not
push the target out — see the schedule below for why.

## Deliverables

| | Deliverable | Artifact |
|---|---|---|
| **D1** | Flashable robosub firmware | `.pio/build/robosub/firmware.bin` (`pio run -e robosub -t upload`) |
| **D2** | Networked webapp | `tools/gcs_server.py` + `tools/dashboard.html` — retained as a bench/fallback tool during the transition; primary control moves to Foxglove once D4's control surface is proven |
| **D3** | PID tuner + guide | dashboard PID panel + [pid_tuning.md](pid_tuning.md) (moves to Foxglove's Parameters panel once D4 lands — see below) |
| **D4** | Foxglove observability, tuning **and control** layer | extended Jetson bridge node + `EspState.msg` + Foxglove layouts — see [foxglove_integration.md](foxglove_integration.md) |

## Status: what's already built

Everything below is **written and compiles / passes its own test**, not yet
proven on real hardware unless stated otherwise. This is a large amount of
finished work — what's left is bring-up, tuning, and (for D4) wiring the
bridge extensions, not new design.

**Firmware** — drivers (`Icm20948`, `Vl53l0x`, `EscPwm`), estimators
(`OnboardEstimator`/Mahony, `VerticalEstimator`/Kalman, `ExternalEstimator`),
`Safety` (arming + operator deadman), `HardwareWatchdog`, `MotorManager`,
the full wire protocol (`Protocol.h`, `PacketParser`, `CommandLink`,
`JetsonLink`), the PID/mixer control chain (`Pid`, `RobosubController` with
flash persistence via NVS, `RobosubMixer` — decoupling verified in Python),
and the compile-time vehicle factory (`hopcopter`/`robosub` build envs, both
green).

**Ground station** — `dashboard.html` (both vehicle tabs, live instruments,
motor sliders, PID panel with the live constructor-code snippet, demo mode,
dual WebSerial/WebSocket transport), `gcs_server.py` (serial hub, WebSocket
relay, `--ros` pose injection), `pose_bridge.py`, and the shared
`tardigrade_protocol.py` (self-test passing, cross-language wire compat
confirmed against the firmware and the browser).

**Docs** — `architecture.md`, `hardware.md`, `ros_link.md`,
`ground_station.md`, `pid_tuning.md`, `bench_checklist.md`, and the new
`foxglove_integration.md` spec.

**Verified on real hardware so far: only the ICM-20948 + Mahony tilt test.**
Everything else — ESCs, watchdog trip, ToF, the robosub control chain
driving real thrusters, flash save/reset on real NVS, the WebSocket path
live, and all of D4 — is compile-/protocol-/browser-verified only. That's
the actual state of the remaining work.

## Safety model change (affects D4's scope)

The sub has a **physical kill switch**. That's now the documented safety
backstop for live testing, not a software presence mechanism — so control
(arm/disarm/motor test) is moving into Foxglove alongside tuning and
telemetry, not staying webapp-only. See
[foxglove_integration.md](foxglove_integration.md#safety-model-physical-kill-switch-not-a-software-presence-beacon)
for the full reasoning. The one thing this makes non-optional: **someone is
on kill-switch duty for every live test**, webapp or Foxglove. That belongs
in [bench_checklist.md](bench_checklist.md) and [pid_tuning.md](pid_tuning.md)
as an explicit step.

## Schedule — two parallel tracks

**Track A (hardware bring-up)** is the original plan, unchanged.
**Track B (Foxglove/bridge)** is new, mostly Jetson-side software, and can
run alongside Track A without blocking it — a teammate can build it while
others do bench/pool work.

### Track A — hardware bring-up

| Phase | Window | Definition of done |
|---|---|---|
| **1 — Dry bench** | Jul 21–22 | Robosub flashes + boots; every thruster spins the correct direction; watchdog reset, deadman disarm, and sensor-timeout all observed firing; webapp connects locally. See [bench_checklist.md](bench_checklist.md). |
| **2 — Network + Jetson** | Jul 22–24 | `gcs_server.py --ros` on the Jetson; a second laptop opens `http://<jetson>:8080/` over the LAN and sees live data; real EKF pose makes `ExternalEstimator` report `healthy`. |
| **3 — Water + tuning** | Jul 24–29 | Depth signal validated; leveling→heading→depth tuned per [pid_tuning.md](pid_tuning.md); gains saved to flash and holding across a reset. |
| **4 — Finalize** | Jul 29–30 | Tuned gains pasted into `RobosubController::applyDefaults()` and committed; final `firmware.bin` cut. |

### Track B — Foxglove / bridge (new)

| Phase | Window | Depends on | Definition of done |
|---|---|---|---|
| **F1 — Bridge core (read-only)** | Jul 21–23 | nothing — starts immediately, parallel with Phase 1 | `EspState.msg` added to `tardigrade_interfaces`; bridge decodes `State` frames and publishes `/tardigrade/esp/state`; a minimal Foxglove layout shows live data off the same bench ESP as Phase 1. No water needed. |
| **F2 — Tuning + control surface** | Jul 23–25 | **Phase 1 complete** | Gains as ROS 2 parameters with the ESP-is-source-of-truth sync; save/reset services; `/tardigrade/set_armed` service *plus* the bridge sending continuous Heartbeat to the ESP while armed (without this, the vehicle auto-disarms ~300ms after every Foxglove arm call — see foxglove_integration.md); motor-test topic; a pilot/tuning layout. Target: ready *before* Phase 3 starts, so it's usable during real tuning, not built after the fact. |
| **F3 — Multi-client validation + kill-switch rehearsal** | Jul 25–27 | F2, Phase 2's network path | Two laptops connect simultaneously with different layouts and both see the same live data; arm/disarm/motor-test confirmed working correctly and repeatably via Foxglove; kill switch physically rehearsed with someone designated to hold it. |
| **F4 — Recording + finalize + webapp retirement** | Jul 27–30 | F2, F3 | At least one real tuning session recorded via rosbag during Phase 3; playback confirmed on a laptop with no Jetson attached; the three standard layouts (pilot/observer/playback) saved for the team; webapp marked as retired-but-available-as-fallback. Converges with Phase 4. |

**Why F1 and F2 have different starting points.** F1 is read-only telemetry
— it doesn't touch anything safety-critical, so it runs alongside Phase 1
rather than waiting for it. F2 adds new control code (arm/disarm/motor-test)
— that's deliberately sequenced *after* Phase 1 verifies the hardware, so
any bug found while building/testing F2 is unambiguously a bridge-code
problem, not a hardware-wiring problem hiding underneath a brand-new UI.

**If Track B slips, Track A is unaffected.** The webapp still works as a
fallback until F2/F3 are proven — nothing in D1–D3 depends on D4 landing on
time.

## Critical path & risks

- **Phase 1 gates everything on Track A.** The most likely slip is a bad ESC
  or an inverted thruster found on the bench. The checklist front-loads
  exactly these.
- **Depth sensor (decide in Phase 2).** Depth comes from the EKF `z`, fed by
  ZED visual odometry, which degrades underwater. If depth hold is jittery in
  Phase 3 the estimator is the likely cause, and a pressure sensor fused into
  the EKF is a bigger job than tuning. Decide whether the depth signal is
  trustworthy *before* going wet, not during.
- **Kill switch confirmation, not a software design question anymore.**
  Before retiring the webapp's arm/disarm role, confirm the kill switch is
  tested and someone is explicitly designated to hold it during every
  Foxglove-driven live test. This replaces the earlier open question about
  software presence detection — it's now a procedural checklist item, not an
  architecture decision.
- **Hopcopter controller is not built.** Out of scope for these four
  deliverables; noted so it isn't mistaken for done.

## What each deliverable needs to be "done"

- **D1:** boots, holds depth+heading in water, all failsafes proven, tuned
  gains in git.
- **D2:** a teammate on the same network opens the URL, sees live telemetry,
  can arm and tune — available as the fallback tool through the transition.
- **D3:** someone can follow the guide, tune both axes, and land the values
  in `applyDefaults()` — via the webapp during the transition, via Foxglove's
  Parameters panel once D4 lands.
- **D4:** the team can open Foxglove on any laptop, see live telemetry,
  arm/disarm and test motors, tune gains through the Parameters panel with
  save/reset working, and replay a recorded session. The webapp is retired as
  the *required* tool once this is proven; the physical kill switch is the
  safety backstop for live testing throughout.
