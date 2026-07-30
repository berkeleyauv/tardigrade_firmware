# Tardigrade Delivery Roadmap

Target: **D1–D4 done by ~Jul 30, 2026** (~10 days from Jul 20). D5 (Jetson
control migration) is new scope layered on top — see its own window below;
completing it fully is a stretch against Jul 30, and that's stated honestly
rather than folded into the same deadline as everything else.

Scope note: **hopcopter is out of this repository.** It's a separate vehicle
in a separate repo going forward. Everything below is robosub-only.

## Deliverables

| | Deliverable | Artifact |
|---|---|---|
| **D1** | Flashable robosub firmware | `.pio/build/esp32dev/firmware.bin` (`pio run -t upload`) |
| **D2** | Networked webapp | `tools/gcs_server.py` + `tools/dashboard.html` — transitional bench/fallback tool; primary control moves to Foxglove once D4's control surface is proven |
| **D3** | PID tuner + guide | today: dashboard PID panel + [pid_tuning.md](pid_tuning.md). Once **D5** lands: Foxglove's Parameters panel talking to native ROS params on the Jetson controller node, gains in a versioned YAML — no ESP round-trip at all. |
| **D4** | Foxglove observability, tuning **and control** layer | extended Jetson bridge node + `EspState.msg` + Foxglove layouts — see [foxglove_integration.md](foxglove_integration.md) |
| **D5** | Jetson-side control (new) | PID + mixer as ROS nodes, gains in a versioned YAML, sim parity — see `tardigrade_ws/docs/jetson_control_architecture.md` |

## Status: what's already built

Everything below is **written and compiles / passes its own test**, not yet
proven on real hardware unless stated otherwise.

**Firmware (robosub only, single build env)** — drivers (`EscPwm`), `Safety`
(arming + operator deadman), `HardwareWatchdog`, `MotorManager`, the full wire
protocol (`Protocol.h`, `PacketParser`, `CommandLink`, `JetsonLink`), the
pose-consumption path (`ExternalEstimator`), and the PID/mixer control chain
(`Pid`, `RobosubController` with flash persistence via NVS, `RobosubMixer` —
decoupling verified in Python). The control chain is **transitional**: it
migrates to the Jetson under D5, at which point it's removed from firmware —
see the transitional notes in [estimator.md](estimator.md) and
`jetson_control_architecture.md`.

**Ground station** — `dashboard.html` (live instruments, motor sliders, PID
panel with the live constructor-code snippet, demo mode, dual
WebSerial/WebSocket transport), `gcs_server.py` (serial hub, WebSocket relay,
`--ros` pose injection), `pose_bridge.py`, and the shared
`tardigrade_protocol.py` (self-test passing, cross-language wire compat
confirmed against the firmware and the browser).

**Docs** — `architecture.md`, `hardware.md`, `ros_link.md`,
`ground_station.md`, `pid_tuning.md`, `bench_checklist.md`,
`foxglove_integration.md`, and (in `tardigrade_ws`) `esp_bridge.py` (F1, ROS
telemetry republish), `EspState.msg`, `jetson_control_architecture.md`.

**Verified on real hardware (Jul 29–30 bench session):** all 8 ESCs/thrusters
spin correctly by slot; the operator-link deadman disarms within 300 ms of
`esp_bridge` dying; the sensor/pose-timeout failsafe disarms correctly
(bench-tested via a temporary synthetic-pose hook in `esp_bridge` — see
`tardigrade_ws` README); disarmed motor commands are rejected; and the full
Jetson-side mixer chain (`thruster_mixer` → `esp_bridge` →
`/tardigrade/thrusters/cmd`) drives real thrusters correctly through surge,
sway, heave, and yaw. Three thrusters (slots 2, 6, 7) were found with
reversed polarity during bench testing and fixed in
`tardigrade_ws/config/esp_thruster_map.json` — see that repo's commit
history. A real telemetry bug was also found and fixed: `ExternalEstimator`
was freezing `state_valid`/`altitude_valid` at their last value instead of
tracking staleness live (fixed in this repo).

**Still not verified on hardware:** the hardware watchdog trip (needs a
deliberate firmware hang + reflash, not yet done), flash save/reset of PID
gains, the webapp/WebSocket path (`dashboard.html`/`gcs_server.py` — today's
session used the Foxglove/ROS path exclusively), and the on-Jetson
`depth_attitude_controller` PID itself (only the mixer half of the Track C1
chain was exercised — no real EKF pose was fed to it, only direct `Twist`
commands via `/tardigrade/cmd_vel`). D5's PID convergence and D2's webapp
fallback are therefore both still unproven claims, not confirmed regressions.

## Safety model

The sub has a **physical kill switch** — the documented safety backstop for
live testing, not a software presence mechanism. Control (arm/disarm/motor
test) is moving into Foxglove alongside tuning and telemetry, not staying
webapp-only. See
[foxglove_integration.md](foxglove_integration.md#safety-model-physical-kill-switch-not-a-software-presence-beacon).
The one thing this makes non-optional: **someone is on kill-switch duty for
every live test**, webapp or Foxglove — belongs in
[bench_checklist.md](bench_checklist.md) and [pid_tuning.md](pid_tuning.md) as
an explicit step. This is unaffected by D5 — the deadman, watchdog, and
authority cap stay on the ESP regardless of where the controller runs, and if
anything the ESP becomes a *cleaner* independent safety layer guarding
against bugs in the Jetson-side controller.

## Schedule — three tracks

**Track A (hardware bring-up)** — bench and pool work, unchanged in shape.
**Track B (Foxglove/bridge)** — mostly Jetson-side software, parallel to A.
**Track C (Jetson control migration, new)** — also mostly Jetson-side
software; simplifies Track B's F2 once it lands (gains become native ROS
params with no ESP round-trip).

### Track A — hardware bring-up

| Phase | Window | Definition of done |
|---|---|---|
| **1 — Dry bench** | Jul 21–22 | Robosub flashes + boots; every thruster spins the correct direction (**done Jul 30** — 3 reversed thrusters found + fixed); deadman disarm and sensor-timeout observed firing (**done Jul 30**); watchdog reset observed firing (**not done**); webapp connects locally (**not done** — this session used Foxglove/ROS exclusively). See [bench_checklist.md](bench_checklist.md). |
| **2 — Network + Jetson** | Jul 22–24 | `gcs_server.py --ros` on the Jetson; a second laptop opens `http://<jetson>:8080/` over the LAN and sees live data; real EKF pose makes `ExternalEstimator` report `healthy`. |
| **3 — Water + tuning** | Jul 24–29 | Depth signal validated; leveling→heading→depth tuned per [pid_tuning.md](pid_tuning.md); gains saved to flash and holding across a reset (on-ESP controller — see D5 for where tuning moves next). |
| **4 — Finalize** | Jul 29–30 | Tuned gains pasted into `RobosubController::applyDefaults()` and committed; final `firmware.bin` cut. |

### Track B — Foxglove / bridge

| Phase | Window | Depends on | Definition of done |
|---|---|---|---|
| **F1 — Bridge core (read-only)** | Jul 21–23 | nothing — parallel with Phase 1 | `EspState.msg` + `esp_bridge` node publish `/tardigrade/esp/state`; a minimal Foxglove layout shows live data off the bench ESP. **Done Jul 30** — verified on real hardware, live telemetry confirmed in Foxglove. |
| **F2 — Control surface** | Jul 23–25 | **Phase 1 complete** | `/tardigrade/set_armed` service + continuous Heartbeat-while-armed; motor-test topic; a pilot layout. **Set_armed, heartbeat, and motor-test all verified on real hardware Jul 30** (see `esp_bridge.py`'s F2 subset); no saved pilot layout yet, panels were ad hoc. Tuning-parameter sync is now moot — Track C1 landed first (see below), so gains live as Jetson ROS params, not ESP round-trips. |
| **F3 — Multi-client validation + kill-switch rehearsal** | Jul 25–27 | F2, Phase 2's network path | Two laptops connect simultaneously with different layouts and both see the same live data; arm/disarm/motor-test confirmed working via Foxglove; kill switch physically rehearsed. |
| **F4 — Recording + finalize + webapp retirement** | Jul 27–30 | F2, F3 | A real tuning session recorded via rosbag; playback confirmed with no Jetson attached; the standard layouts saved; webapp marked retired-but-available-as-fallback. |

### Track C — Jetson control migration (new)

| Phase | Window | Definition of done |
|---|---|---|
| **C1 — Controller + mixer nodes** | Jul 23–27 | PID controller and mixer run as Jetson ROS nodes on `/tardigrade/thrusters/cmd`, gains as ROS params loaded from a YAML. Runs alongside the existing ESP controller — not yet load-bearing. **Mixer half verified on real hardware Jul 30** (`thruster_mixer` driving real thrusters correctly via `/tardigrade/cmd_vel`, all axes). **Controller half (`depth_attitude_controller`) still unverified** — no real EKF pose fed to it yet, only direct `Twist` commands bypassing the PID entirely. |
| **C2 — Sim validation** | Jul 26–29 | Same nodes drive `tardigrade_sim`; tuned in sim, gains committed to the YAML. |
| **C3 — Real-robot cutover** (stretch) | after Jul 29 | Jetson controller output validated against the proven on-ESP baseline from Track A; once trusted, `esp_bridge` extended to accept `/tardigrade/thrusters/cmd` directly; ESP stripped to the safe-actuator role (`RobosubController`, `ExternalEstimator`, `JetsonLink`, the `Pose` frame removed). |

C1/C2 are realistic within the Jul 30 window; **C3 is honestly a stretch** — it
needs a proven hardware baseline from Track A first (the same
"verify-with-a-known-good-tool" principle as F2's Phase-1 dependency, one
level deeper), so it can't start in earnest until Track A is solid, and it's
the kind of cutover you don't rush the day before a deadline.

**Track dependencies, plainly:** F1 needs nothing. F2 needs Phase 1. C1/C2
need nothing (can start immediately). If C1/C2 land before F2, F2's tuning
scope simplifies — no ESP-parameter-sync work, since gains are just Jetson
ROS params by then. **If any track slips, the others are unaffected** — the
webapp still works as a fallback, and the on-ESP controller still works until
Track C's replacement is proven.

## Critical path & risks

- **Phase 1 gates everything on Track A**, and now also gates Track C's C3.
  The most likely slip is a bad ESC or an inverted thruster found on the
  bench. The checklist front-loads exactly these. **This materialized Jul
  30** — 3 of 8 thrusters had reversed polarity, found and fixed via
  systematic per-thruster + per-axis bench testing. No ESC failures.
- **Depth sensor (decide in Phase 2).** Depth comes from the EKF `z`, fed by
  ZED visual odometry, which degrades underwater. If depth hold is jittery in
  Phase 3 the estimator is the likely cause, and a pressure sensor fused into
  the EKF is a bigger job than tuning. Decide whether the depth signal is
  trustworthy *before* going wet, not during.
- **Kill switch confirmation** — procedural, not architectural. Confirm it's
  tested and someone is explicitly designated to hold it during every live
  test, webapp or Foxglove.
- **Don't let C1/C2 and Track A drift apart.** The Jetson controller in sim
  and the on-ESP controller in the water are two implementations of the same
  control law during the transition. If their gains diverge, "which one is
  right" becomes a real question — keep the sim YAML and
  `RobosubController::applyDefaults()` in sync until C3 makes one of them
  obsolete.
- **Hopcopter is out of scope for this repository entirely** — not "not built
  yet," genuinely a separate vehicle in a separate repo now.

## What each deliverable needs to be "done"

- **D1:** boots, holds depth+heading in water, all failsafes proven, tuned
  gains in git.
- **D2:** a teammate on the same network opens the URL, sees live telemetry,
  can arm and tune — available as the fallback tool through the transition.
- **D3:** someone can follow the guide, tune both axes, and land the values
  in a versioned place — `applyDefaults()` during the transition, the gains
  YAML once D5 lands.
- **D4:** the team can open Foxglove on any laptop, see live telemetry,
  arm/disarm and test motors, tune gains, and replay a recorded session. The
  webapp is retired as the *required* tool once this is proven; the physical
  kill switch is the safety backstop for live testing throughout.
- **D5:** the same controller + mixer code, with the same gains, drives both
  the simulator and the real robot. Tuning happens once, transfers everywhere.
