# PID Tuning (Robosub)

The robosub holds depth, heading, and level via four PID channels
([RobosubController](../firmware/include/control/RobosubController.h)). Gains and
setpoints are tunable live from the dashboard's **PID tuning** panel — no
reflash between iterations.

## How live tuning works

```
dashboard field  --SetParameter(id,value)-->  CommandLink  -->  RobosubController.setParameter()
dashboard connect --GetParameters-->  CommandLink  --Parameter frames-->  dashboard fields
```

On connect the panel reads every current value back, so the fields start where
the firmware actually is. Editing a field sends it immediately. The parameter
ids are a wire contract shared by [Parameters.h](../firmware/include/control/Parameters.h)
and the dashboard — never renumber, only append.

| Parameter | id | Units |
|---|---|---|
| depth / yaw / roll / pitch Kp,Ki,Kd | `(channel<<4)\|term`, channel depth=0x00 yaw=0x10 roll=0x20 pitch=0x30, term Kp=0 Ki=1 Kd=2 | gains |
| authority | 0x40 | 0..1 output scale |
| depth setpoint | 0x50 | ENU z, metres (−ve = deeper) |
| heading setpoint | 0x51 | radians on the wire; the panel shows degrees |

## The testing process

### Dry bench first
1. **Thruster directions.** With no Jetson pose the controller does not run
   (it gates on a healthy estimate), so the thruster sliders drive each unit
   directly. Confirm a heave command pushes all four verticals the same way and
   yaw turns the right direction. Fix any inverted thruster in
   `esp_thruster_map.json` **and** [RobosubMixer.cpp](../firmware/src/mixer/RobosubMixer.cpp)
   before going wet — a sign error makes the controller drive away from the
   setpoint.
2. **Depth signal.** Move the sub up/down by hand, watch `alt` track it.

### In water — one axis at a time, inner loops first
Order: **roll/pitch leveling → heading → depth.** Outer loops assume the inner
ones stabilize. (Most subs are passively stable in roll/pitch — buoyancy above
mass — so leveling may need almost nothing.)

Per axis:
1. Lower to depth, settle level and still.
2. **Arm** — captures the current depth and heading as the hold target (error
   zeroed here, automatically).
3. **Perturb and watch recovery** on the target-vs-actual charts:
   - slow return → raise **Kp**
   - overshoot / oscillation → raise **Kd** (clean: it is rate-based, fed by the
     estimator's velocity, not a differentiated position)
   - settles with a standing offset (buoyancy) → add a little **Ki**
4. Or step the setpoint field and watch the step response directly.

Classic method: `Ki=Kd=0`, raise `Kp` to the edge of oscillation, back off ~40%,
add `Kd` to damp, add `Ki` last and sparingly.

## Safety while tuning

- **Authority** starts at 0.35 so a bad gain cannot slam a thruster to full.
  Raise toward 1.0 as confidence grows.
- **Trim slightly positive buoyancy** so any failsafe (operator deadman, sensor
  timeout, hardware watchdog — all active) makes it **surface**, not sink.
- **Depth comes from the EKF `z`** (ZED visual odometry). Underwater, VO can
  degrade; jittery depth hold is more likely the sensor than the gains. A
  pressure/depth sensor fused into the EKF is the robust fix.
- The sensor-timeout failsafe disarms if pose is lost while armed, which cuts
  the controller and neutralises the thrusters.

## Where the gains live

Defaults are set in the `RobosubController` constructor. Live changes are **not
yet persisted** — they reset to those defaults on reboot. When you settle on
good values, copy them into the constructor (persistent parameter storage is a
later addition).
