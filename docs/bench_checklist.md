# Phase 1 — Dry Bench Bring-Up Checklist

Robosub, out of water. Goal: prove the firmware is safe and every thruster is
correct **before** the sub ever gets wet. Work top to bottom — the ordering is
deliberate: failsafes are proven before any thruster is allowed to spin.

## Safety preamble — read first

- **Thrusters need water for cooling.** Run them dry only in brief taps (a second
  or two). Do not hold throttle on a dry T200.
- **Power from a bench supply with a current limit** if you have one — a miswired
  ESC trips the limit instead of cooking something.
- **Keep a hand on the disarm** (panel DISARM, or just pull power). Props/guards
  clear of fingers and cables.
- Nothing in Stage 3+ proceeds until Stage 2 (failsafes) passes.

## Stage 0 — Prep

- [ ] Firmware builds clean: `pio run -e robosub`
- [ ] ESCs wired to pins **21, 19, 27, 18, 5, 14, 12, 26** (thrusters 0–7 = slots
      1–8 of `esp_thruster_map.json`)
- [ ] ESC power rail separate from the ESP logic supply; common ground
- [ ] Bench supply current limit set (if used)

## Stage 1 — Flash & boot

- [ ] Flash: `pio run -e robosub -t upload`
- [ ] Open the monitor: `pio device monitor -b 115200`
- [ ] Boot banner reads `Tardigrade FC starting (ROBOSUB)...`
- [ ] `Arming 8 ESC outputs (holding stop)...` prints; ESCs beep their arm tone
      during the ~2 s hold
- [ ] `hardware watchdog armed (1 s)` prints
- [ ] `ready - connect the ground station` prints
- [ ] Status line streams: `ROBOSUB rpy=... healthy=0 armed=0 link=lost`
      (`healthy=0` is expected with no Jetson pose)

## Stage 2 — Prove the failsafes (BEFORE thrusters spin)

### Hardware watchdog
- [ ] Temporarily add `while (1) {}` inside `loop()` in `main.cpp`, flash
- [ ] Board resets within ~1 s and reboots
- [ ] After reboot, `!! previous boot ended in a WATCHDOG RESET !!` prints
- [ ] **Remove the `while(1)` and reflash** before continuing

### Operator deadman
- [ ] Connect the webapp (Stage 3 can be done first if easier), ARM
- [ ] Kill the link (close the tab / pull USB)
- [ ] Within ~300 ms the status line flips to `armed=0`, and disarm is logged

### Sensor-timeout (optional on bench, full check in Phase 2)
- [ ] Note: with no Jetson, `healthy=0`, so arming works but the controller does
      not drive thrusters. Full sensor-timeout-while-armed test happens in
      Phase 2 once pose is flowing and then cut.

## Stage 3 — Ground station

- [ ] Local connect works: open `tools/dashboard.html` in Chrome → **Connect**,
      pick the port (or run `python tools/gcs_server.py --serial <port>` and open
      `http://localhost:8080/`)
- [ ] Telemetry updates; `crc err` stays 0
- [ ] ARM / DISARM from the panel reflects in the status line
- [ ] Robosub tab shows 8 thruster sliders

## Stage 4 — Thruster direction (brief taps only)

With no Jetson pose the controller does not run, so the sliders drive each
thruster **individually** — this is the check that catches inverted wiring. ARM
first. Expected thrust direction is from the mix in `esp_thruster_map.json`:

| Idx | Pin | Name | + command should thrust |
|---|---|---|---|
| 0 | 21 | front_left_vertical | up (heave +) |
| 1 | 19 | front_right_vertical | up |
| 2 | 27 | back_left_vectored | forward (surge +) |
| 3 | 18 | front_right_vectored | forward |
| 4 | 5 | front_left_vectored | forward |
| 5 | 14 | back_left_vertical | up |
| 6 | 12 | back_right_vectored | forward |
| 7 | 26 | back_right_vertical | up |

- [ ] For each thruster 0–7: nudge its slider to ~+15%, confirm it **spins** and
      pushes in the direction above (feel the flow / watch the prop). Return to 0.
- [ ] Confirm reverse: a negative slider reverses thrust (bidirectional ESCs)
- [ ] Any thruster that spins the wrong way / pushes backwards → flip its sign in
      `esp_thruster_map.json` **and** [RobosubMixer.cpp](../firmware/src/mixer/RobosubMixer.cpp),
      reflash, re-check
- [ ] DISARM; confirm all thrusters go to neutral (stop)

## Stage 5 — Sign-off

- [ ] `while(1)` watchdog test code removed and reflashed
- [ ] All 8 thrusters verified correct direction
- [ ] Watchdog + deadman both observed firing
- [ ] Webapp connects and commands cleanly
- [ ] Ready for Phase 2 (Jetson + network) — see [roadmap.md](roadmap.md)

> If anything here fails, it is far cheaper to fix on the bench than in the water.
> A wrong-direction thruster in Phase 3 makes the controller drive *away* from the
> setpoint and looks like a tuning problem — catch it here.
