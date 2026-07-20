# The Jetson ↔ ESP32 ROS Link (Robosub)

## Scope

How the Robosub's fused pose estimate gets from the Jetson to the ESP32.

This is **not** the ground link. [communication.md](communication.md) covers the
laptop↔vehicle channel (arm, disarm, tune, telemetry). This document covers the
onboard link between the companion computer and the flight controller, which
carries one thing: the pose estimate the Jetson produces by fusing the VectorNav
IMU with the ZED stereo camera. See [estimator.md](estimator.md) for why that
fusion happens on the Jetson rather than the ESP32.

## Three names that get confused

DDS, XRCE-DDS, and micro-ROS are three different layers. They are routinely used
interchangeably and they are not the same thing.

### DDS

**Data Distribution Service** — an OMG standard for publish/subscribe
middleware, and the foundation ROS 2 is built on. Three properties define it:

- **Data-centric pub/sub.** Typed topics, not RPC or byte streams.
- **No broker.** Peers discover each other via multicast (SPDP announces
  participants, SEDP announces their endpoints). There is no `roscore` to lose —
  this is the headline change from ROS 1.
- **Rich QoS.** Reliability, durability, history depth, deadline, liveliness,
  negotiated per endpoint pair.

Its wire protocol is **RTPS** (Real-Time Publish-Subscribe), typically over UDP.
Implementations include Fast DDS (eProsima, the ROS 2 default), Cyclone DDS, and
RTI Connext.

ROS 2 topics *are* DDS topics and ROS 2 QoS maps onto DDS QoS; the `rmw` layer
only abstracts which vendor is underneath.

**Why a microcontroller cannot run it:** chatty multicast discovery, per-peer
state for every participant discovered, a full UDP/IP stack, and dynamic
allocation throughout. That is a Linux-class budget, not an ESP32 one.

### XRCE-DDS

**DDS-XRCE** ("eXtremely Resource-Constrained Environments") is a separate OMG
standard that does *not* shrink DDS. It defines a **client/agent split**:

- **Client** (the MCU) — a compact binary protocol over any transport: serial,
  UDP, CAN. Performs no discovery and holds no DDS entities.
- **Agent** (a Linux host) — a full DDS participant that creates and owns real
  DDS entities *on the client's behalf*.

```
MCU ──XRCE protocol──► Agent ──real DDS/RTPS──► DDS databus ◄── ROS 2 nodes
```

The MCU is a proxy participant. Everything on the databus sees an ordinary DDS
endpoint; the Agent manufactures that appearance. eProsima's implementation is
**Micro XRCE-DDS** (Client + Agent).

### micro-ROS

A port of the **ROS 2 client stack** to microcontrollers, layered on top of an
XRCE-DDS client:

```
your code
rclc                  ← C client library (the rclcpp/rclpy equivalent)
rcl                   ← the SAME C library desktop ROS 2 uses
rmw_microxrcedds      ← middleware abstraction, swapped for the MCU
Micro XRCE-DDS Client
transport (serial / UDP / CAN)
```

Because `rcl` is shared with desktop ROS 2, topics, QoS, and message types
behave as expected. Execution uses `rclc_executor` — static and deterministic,
with entity counts and history depth bounded at compile time, so there is no
dynamic allocation in the steady state.

**The relationship in one line:** micro-ROS *uses* XRCE-DDS as its transport;
XRCE-DDS does *not* require micro-ROS.

## Which box runs what

```
ESP32                                   Jetson
─────                                   ──────
micro-ROS client stack                  micro-ROS Agent  (NOT micro-ROS —
  rclc / rcl / rmw_microxrcedds           it is the XRCE-DDS agent, an
  Micro XRCE-DDS Client                   ordinary Linux ROS 2 process)
  serial transport                      VectorNav + ZED fusion node
                                        the rest of the ROS 2 graph
```

**micro-ROS runs on the ESP32.** The Jetson runs the Agent and normal ROS 2
nodes. The phrase "run the micro-ROS agent" is misleading: the agent contains no
micro-ROS code. "micro-ROS" properly names the client-side stack and tooling.

## Contrast with the PX4 / Pixhawk setup

Worth stating explicitly, because the team has used XRCE-DDS on a Pixhawk and
the resemblance is misleading. PX4's `uxrce_dds_client` **is not micro-ROS.**

| | Pixhawk / PX4 | ESP32 / Tardigrade |
|---|---|---|
| On the MCU | `uxrce_dds_client` — bridges **uORB** straight to XRCE | **micro-ROS**: rclc + rcl + rmw |
| ROS client library onboard? | **No** | **Yes** — nodes, subscriptions, executor |
| Wire protocol | XRCE-DDS | XRCE-DDS *(identical)* |
| On the companion computer | `MicroXRCEAgent` | `micro_ros_agent` *(same agent)* |

PX4 takes a shortcut: it already has its own middleware (uORB), so it translates
uORB↔XRCE and never runs ROS code onboard. micro-ROS instead ports the real ROS
client stack down to the MCU.

The practical consequence: **the Agent is shared.** One Jetson can host a PX4
client and the ESP32 simultaneously, with both appearing as ordinary nodes on a
single ROS 2 graph.

## Constraints that drive the design

**Serial bandwidth is a hard ceiling.** Approximate, for one pose message
including CDR and XRCE framing:

| Message | ~Size | @115200 | @921600 |
|---|---|---|---|
| `PoseStamped` | ~100 B | ~115 Hz | ~920 Hz |
| `Odometry` | ~600 B | ~19 Hz | ~150 Hz |

`Odometry` carries two 6×6 float64 covariance matrices (576 bytes) that the ESP32
discards immediately. Prefer `PoseStamped`, run 921600 or native USB CDC, and
consider a custom slim `.msg` of `float32` fields if the high end of the
200–800 Hz target in [estimator.md](estimator.md) is actually needed.

**Use BEST_EFFORT QoS, not the default RELIABLE.** For a high-rate pose stream
the newest sample is the only one that matters; retransmitting a stale pose
spends bandwidth delivering data the controller has already moved past.

**The Agent is a hard dependency.** No Agent process, no data. If it dies the
session drops and the client must detect it and re-establish. This is exactly
what `ExternalEstimator`'s freshness timeout exists for — the link vanishing has
to degrade into a Safety failsafe, never a hang.

**Never block the control loop.** `IStateEstimator::update()` is pull-based and
must not block, so `rclc_executor_spin_some` runs with a zero timeout — or,
preferably, micro-ROS runs in its own FreeRTOS task that writes the latest pose
into a shared buffer the estimator reads. At a 1 kHz loop the separate task is
cleaner: it keeps serial I/O jitter out of the control path entirely.

## Where this lands in the firmware

```
VectorNav + ZED → Jetson fusion node → /pose topic
                                          │ micro-ROS
                                          ▼
                    JetsonLink (owns session, entities, reconnect)
                                          │
                                          ▼
                   ExternalEstimator (passthrough) → VehicleState
```

`JetsonLink` is the only module that knows any of this exists.
`ExternalEstimator` stays transport-agnostic: it copies whatever pose the link
last delivered and applies the same `last_valid_us` freshness check as the
hopcopter path.

## Fallback

If micro-ROS proves too heavy or too fiddly on the ESP32, the alternative is a
PX4-style bridge: a hand-rolled serial protocol into a Jetson-side node that
republishes to ROS. Leaner, but the serialization and versioning become ours to
maintain.

Because `JetsonLink` is the only place that would change, this decision stays
cheap to reverse — which is the reason to start with micro-ROS rather than
designing around a custom protocol up front.
