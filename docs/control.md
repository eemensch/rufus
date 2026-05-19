# Control interface for agents

This document specifies the topic contract a strategy uses to drive
a simulated agent and to read its state. The contract is the same
across platforms; the platform-specific differences live in how the
adapter clamps the command and what the limits in `Capability` mean.

Today only the rover adapter is implemented; quad and fixed-wing
adapters (Stages 2 and 3) will use the same contract.

## Topic contract

For an agent in namespace `<ns>` (default `''`, i.e. root):

| Direction | Topic                  | Type                              | QoS               |
|-----------|------------------------|-----------------------------------|-------------------|
| in        | `<ns>/cmd_vel`         | `geometry_msgs/msg/TwistStamped`  | reliable, depth 10 |
| out       | `<ns>/state`           | `rufus_sim_msgs/msg/AgentState`      | reliable, depth 10 |
| out       | `<ns>/capability`      | `rufus_sim_msgs/msg/Capability`      | reliable, **TRANSIENT_LOCAL** |

Subscribers to `capability` must use **`TRANSIENT_LOCAL`** durability
to receive the latched message; with the default `VOLATILE` they
get nothing if they subscribe after the adapter publishes.

## `cmd_vel` semantics

`TwistStamped`. Body frame. The adapter reads:

- `twist.linear.x` — forward speed (body x).
- `twist.angular.z` — yaw rate about body z.

All other fields are ignored by the rover adapter today and silently
dropped. They will be honoured for richer platforms:

| Field             | Rover (skid)         | Quadrotor           | Fixed-wing                  |
|-------------------|----------------------|---------------------|-----------------------------|
| linear.x          | forward speed        | body forward speed  | airspeed V                  |
| linear.y          | ignored              | body lateral speed  | ignored                     |
| linear.z          | ignored              | body vertical speed | climb rate (m/s, +up)       |
| angular.z         | yaw rate             | yaw rate            | turn rate ψ̇ (rad/s, ENU/CCW)|
| angular.x, .y     | ignored              | ignored             | ignored                     |

**Every platform's contract is its kinematic-model control
set, in body frame.** Each adapter takes exactly the inputs the
pursuit-evasion differential game constrains directly — no
carrot/heading-hold/velocity-vector indirection:

- **Rover** (Dubins car): `linear.x` = forward speed,
  `angular.z` = yaw rate, coupled by `|ω| ≤ v/R_min`.
- **Quad** (holonomic): `linear.{x,y,z}` body velocity,
  `angular.z` = yaw rate.
- **Fixed-wing** (Dubins airplane): `linear.x` = airspeed V,
  `angular.z` = turn rate ψ̇, `linear.z` = climb rate, with the
  speed-coupled set `V ∈ [v_min, v_max]`,
  `|ψ̇| ≤ g·tan(bank_max)/V`, `|climb| ≤ V·sin(γ_max)`.

`angular.z` (a turn/yaw rate) is a velocity input as much as
`linear.x` is; the adapter does not synthesise it from a
heading or velocity vector and does not hide it. This is what
lets a min-max be solved over each platform's exact admissible
set. `header.frame_id` should be `<ns>/base_link`.

**Adapter-internal frame plumbing.** Strategies always talk to
the adapter in body frame for rover and quad; what the adapter
forwards to the FCU differs by platform.

- **Rover:** publishes a body-frame `mavros_msgs/PositionTarget`
  on `mavros/setpoint_raw/local` with
  `coordinate_frame = MAV_FRAME_BODY_NED` (velocity + yaw_rate
  valid; position, accel, yaw ignored — the same velocity-level
  setpoint, type_mask 1479). No rotation. This is deliberate:
  ArduRover infers drive direction from `is_negative(packet.vx)`
  (`Rover/GCS_MAVLink_Rover.cpp`); in a world frame that is the
  NED-north sign, so a rover not heading due north drives
  backward (S1.4 baseline: `step_vx` cmd +0.5 → steady −0.5).
  Body frame makes `packet.vx` body-forward, so the sign is
  correct, and ArduRover routes to
  `set_desired_turn_rate_and_speed` — the unicycle (speed,
  turn-rate) mode that handles straight, arc and spin-in-place
  uniformly.
- **Quad:** rotates `linear.x`/`linear.y` into world ENU using
  the latest `mavros/local_position/pose` yaw before publishing
  to `mavros/setpoint_velocity/cmd_vel_unstamped`, because that
  plugin is configured `mav_frame: LOCAL_NED` (a *world* frame).
  Without it, an iris spawned at yaw=90° given body `(0.5,0,0)`
  would be told to fly 0.5 m/s east not along its body-x,
  the canonical 5.66 m divergence over an 8 s `step_vx`.
- **Fixed-wing:** `_project` clips `(V, ψ̇, climb)` to the
  Dubins-airplane admissible set (`dubins_airplane_clip`). The
  realization layer is the `MAV_CMD_GUIDED_CHANGE_*` slew
  triplet, NOT a `SET_POSITION_TARGET` velocity setpoint:
  ArduPlane's `handle_set_position_target_local_ned` is an
  altitude-only stub (verified in `GCS_MAVLink_Plane.cpp`), so a
  velocity setpoint is a silent no-op. Because GUIDED HEADING is
  heading- not rate-addressable, the adapter integrates the
  commanded `ψ̇` into a heading target (re-seeded from pose while
  `cmd_vel` is stale); SPEED takes `V`, ALTITUDE a target alt +
  climb rate from `climb`. ArduPlane independently caps the
  heading slew at the same coordinated-turn accel limit, so the
  FCU and the clip agree on `|ψ̇| ≤ g·tan(bank_max)/V`.

`header.stamp` is recorded by the adapter as the command's arrival
time, used for the `cmd_timeout_s` check (default 1 s; on stale
input the adapter sends zeros).
`header.frame_id` is informational; recommend `<ns>/base_link`.

### Rover specifics

The Aion R1 in `r1_rover.param` plus the project-owned override
`rufus_sim_bringup/config/r1_rover_tune.parm` has, at the time of
writing:

- `WP_SPEED = 2.0 m/s`           → `Capability.v_max = 2.0`
                                   (raised from upstream 1.0)
- `ATC_STR_RAT_MAX = 120 deg/s`  → `Capability.yaw_rate_max ≈ 2.09 rad/s`
- `ATC_TURN_MAX_G = 0.6`         → `lateral_accel_max ≈ 5.89 m/s²`

The adapter clamps `linear.x` to `[-v_max, v_max]` (skid-steer can
reverse) and `angular.z` to `[-yaw_rate_max, yaw_rate_max]` before
forwarding to the FCU. Saturation is reported via
`AgentState.saturation`.

**Do not command pure yaw on the rover — it is kinematically
infeasible via GUIDED (re-corrected 2026-05-18, firmware-
grounded; this reinstates the original guidance and reverses
the mid-session "pure yaw is supported" note).** ArduRover's
GUIDED/steering path enforces a minimum turn radius
(`TURN_RADIUS` = 0.9 m): the achievable yaw rate is
speed-coupled, `|ω| ≤ v / R_min`, so `(linear.x = 0,
angular.z ≠ 0)` asks for turn radius 0 and is outside the
model. True pivot-in-place exists in ArduRover but is an
AUTO-mode / `AR_WPNav` feature, unreachable from the
velocity/turn-rate setpoints this stack sends. A pure-yaw
command does make the rover flail (large dead time, overshoot,
worse at small ω), but that is out-of-model behaviour, not
fidelity. Strategies must command yaw **coupled with forward
speed** satisfying `v ≥ |ω|·R_min` (an arc); the tightest
feasible turn is radius `R_min`. The `imu`-sourced
`twist.angular` (B1) and the body-frame setpoint fix remain
valid and unrelated. Full diagnosis: `plan.md` Stage 1 (S1.3/
S1.4) and `CLAIMS.md` C4/C6/C7.

### Quadrotor specifics

The iris under the default `copter.parm` tune (ArduCopter 4.8-dev)
has:

- `WP_SPD = 10.0 m/s`         → `Capability.v_max = 10.0`
- `WP_SPD_UP = 2.5 m/s`       → `Capability.vz_max_up = 2.5`
- `WP_SPD_DN = 1.5 m/s`       → `Capability.vz_max_down = 1.5`
- `ATC_RATE_Y_MAX = 0`        → fallback
                                `Capability.yaw_rate_max ≈ 1.57 rad/s`
                                (90 deg/s firmware default)
- `ATC_ANGLE_MAX = 30 deg`    → `Capability.bank_angle_max ≈ 0.524 rad`,
                                `lateral_accel_max ≈ 5.66 m/s²`

The adapter clamps the horizontal-velocity vector
`(linear.x, linear.y)` to `v_max` in 2-norm (preserves the
commanded heading when saturating), clips `linear.z` to the
asymmetric vertical envelope `[-vz_max_down, vz_max_up]`, and
clamps `angular.z` to `[-yaw_rate_max, yaw_rate_max]`. Saturation
flags follow the rover pattern (`linear_velocity` for the
horizontal cap, `climb_rate` for the vertical cap,
`angular_velocity` for yaw).

ArduCopter in GUIDED on the ground does **not** honour velocity
setpoints. The adapter handles this with a `TAKING_OFF` bring-up
state: after arming it issues `MAV_CMD_NAV_TAKEOFF` to
`takeoff_altitude_m` (default 5 m) and gates `READY` on local
altitude exceeding `takeoff_alt_threshold_m` (default 4 m). For
the strategy contract this means: when an adapter publishes
`READY`, the iris is already airborne and tracking velocity
setpoints, regardless of platform.

The parameter names above (`WP_SPD*`, `ATC_ANGLE_MAX`) are valid
for ArduCopter 4.7+. The legacy names `WPNAV_SPEED*` (cm/s) and
`ANGLE_MAX` (cdeg) were retired in the SI-suffix migration; the
adapter would need those substituted back if you pin to AP 4.6 or
earlier.

### Fixed-wing specifics

The zephyr under the default `gazebo-zephyr.parm` tune
(ArduPlane 4.6-dev) has:

- `AIRSPEED_MIN = 9 m/s`        → `Capability.v_min = 9.0`
- `AIRSPEED_CRUISE = 12 m/s`    → used for `yaw_rate_at_cruise`
- `AIRSPEED_MAX = 22 m/s`       → `Capability.v_max = 22.0`
- `ROLL_LIMIT_DEG = 45 deg`     → `Capability.bank_angle_max ≈ 0.785 rad`,
                                   `lateral_accel_max ≈ 9.81 m/s²`
- `PTCH_LIM_MAX_DEG = 20 deg`   → `Capability.climb_angle_max ≈ 0.349 rad`,
                                   `vz_max_up ≈ 7.5 m/s` at `v_max`
- `PTCH_LIM_MIN_DEG = -25 deg`  → `vz_max_down ≈ 9.3 m/s` at `v_max`

`Capability.yaw_rate_max` is reported at cruise airspeed:
`g·tan(bank_max) / AIRSPEED_CRUISE ≈ 0.82 rad/s`. For a
strategy that needs the rate at a different speed, compute
`lateral_accel_max / V` on demand.

The strategy commands the Dubins-airplane control inputs
`(V, ψ̇, climb)` directly; `_project` (pure helper
`dubins_airplane_clip`) clips them to the speed-coupled
admissible set, with V clipped first because the ψ̇ and climb
bounds use the clipped V:

```
V       = clip(linear.x, v_min, v_max)
psidot  = clip(angular.z, -g·tan(bank_max)/V, +g·tan(bank_max)/V)
climb   = clip(linear.z, -V·sin(descent_angle), +V·sin(climb_angle))
```

The clipped `(V, ψ̇, climb)` drives the `GUIDED_CHANGE_*`
realization (the adapter integrates `ψ̇` into the heading
target; see the fixed-wing plumbing bullet above). Saturation
flags reflect which clip bounds were active:

- `airspeed`     — `linear.x` was outside `[v_min, v_max]`.
- `turn_rate`    — `|angular.z|` exceeded `g·tan(bank_max)/V`
                   (at the *clipped* airspeed).
- `climb_rate`   — `|linear.z|` exceeded `V·sin(γ_max)` (at the
                   clipped airspeed; up vs. descent angle by
                   sign).
- `linear_velocity` / `angular_velocity` — always `false` for
                   fixed-wing (those flags belong to the
                   body-frame velocity platforms).

### Takeoff (fixed-wing)

ArduPlane in GUIDED on the ground does not auto-launch via
`MAV_CMD_NAV_TAKEOFF` in the gz JSON SITL setup. The adapter
takes the canonical zephyr recipe documented in
`external/ardupilot_gazebo/README.md`: `mode fbwa` → `arm` →
`rc 3 1800`. The non-obvious gate is `MAV_GCS_SYSID`. AP
defaults that to 255 but MAVROS sends with sysid 1; without the
match, AP silently drops every `rc/override` message. The
adapter sets `MAV_GCS_SYSID=1` early in bring-up. Once the
local altitude crosses `takeoff_alt_threshold_m` (default 40 m),
the adapter releases the override and switches mode to GUIDED.
For a strategy, `READY` always means "armed, airborne, and
tracking velocity setpoints" — the same contract as the quad.

### Cruise fallback (fixed-wing)

A fixed-wing cannot satisfy a hover, so `cmd_vel` timeouts must
not decay to a zero command as they do for rover and quad. The
adapter's fallback while no `cmd_vel` is arriving is the control
triple `(V = AIRSPEED_CRUISE, ψ̇ = 0, climb = 0)`, and the
heading integrator is re-seeded from the current pose each tick
while stale, so `ψ̇ = 0` holds the actual heading rather than a
drifted integral. This keeps the plane airborne and straight at
cruise while a strategy is paused, restarted, or otherwise
quiet; on resume the integrator continues from where the plane
actually is.

## `state` semantics

`rufus_sim_msgs/AgentState` carries the agent's pose-and-twist snapshot
together with role and platform metadata.

```
std_msgs/Header header        # frame_id "map", ENU
string  agent_id               # e.g. "R0"
uint8   role                   # 0 NEUTRAL, 1 PURSUER, 2 EVADER
uint8   platform               # 0 ROVER, 1 QUADROTOR, 2 FIXED_WING
geometry_msgs/Pose  pose       # ENU world frame
geometry_msgs/Twist twist      # body frame
SaturationFlags saturation     # which limits were active on last cmd
```

`pose.position` is in the local ENU frame (origin at the FCU's home
location). `pose.orientation` is the body-to-world quaternion.
`twist.linear` is body-frame velocity; `twist.angular` is body-frame
angular rate (including yaw rate `twist.angular.z`).

The adapter publishes `state` at 50 Hz (configurable via the
`state_rate_hz` parameter).

## Admissible control set (per platform)

Each platform's strategy contract is the control-input set of
its kinematic model: the variables a pursuit-evasion min-max is
solved over, constrained directly. This is the audited mapping
from each control variable to its bound, the `Capability` field
that carries the bound, and the source autopilot parameter the
bound is derived from (recorded verbatim in `Capability.source`
at episode setup). The adapter re-enforces every bound at
runtime and raises the matching `SaturationFlags` bit.

Symbols: `v` body-forward / `V` airspeed (m/s); `ω` body yaw
rate, `ψ̇` turn rate (rad/s); `climb` vertical velocity
`ż = linear.z` (m/s, +up); `γ_up` / `γ_dn` max flight-path
(climb) angle up/down (rad); `R_min` minimum turn radius (m);
`g` = 9.81 m/s²; `φ_max` max bank (rad).

### Rover — Dubins car (skid-steer)

| `cmd_vel` field | symbol | bound | `Capability` field | source param |
|---|---|---|---|---|
| `linear.x` | `v` | `v ∈ [−v_max, v_max]` | `v_max` (`v_min=−v_max`) | `WP_SPEED`, `CRUISE_SPEED`, `GUID_SPEED_MAX` (max of) |
| `angular.z` | `ω` | `|ω| ≤ v_max·(180/π)`-cap **and** `|ω| ≤ |v|/R_min` | `yaw_rate_max`; `min_turn_radius` | `ATC_STR_RAT_MAX` (deg/s→rad); `TURN_RADIUS` |

The binding constraint is the **coupling** `|ω| ≤ |v|/R_min`:
ArduRover's GUIDED/steering path holds `TURN_RADIUS` as the
tightest arc even on a differential chassis (pure pivot is
AUTO-only), so `(v=0, ω≠0)` is infeasible. `R_min` defaults to
0.9 m (ArduRover `Parameters.cpp`; also set in
`r1_rover.param`). `lateral_accel_max` carries
`max(ATC_TURN_MAX_G·g, yaw_rate_max·v_max)` for strategies that
prefer the accel form.

An episode may override `min_turn_radius` (writes `TURN_RADIUS`)
**widen-only**: the value must be `≥` the controller-deliverable
native `TURN_RADIUS`. A larger R_min is a strictly more
restrictive admissible set the vehicle always realizes (just
turn gently); a smaller one would claim a tighter turn than the
underlying controller delivers, so it is rejected
(`ParameterOverrideError`), not silently clamped.

### Quadrotor — holonomic (simple-motion)

| `cmd_vel` field | symbol | bound | `Capability` field | source param |
|---|---|---|---|---|
| `linear.x`, `.y` | `v` | `‖(vx,vy)‖ ≤ v_max` | `v_max` (`v_min=−v_max`) | `WP_SPD` |
| `linear.z` | `climb` | `climb ∈ [−vz_max_down, vz_max_up]` | `vz_max_up`, `vz_max_down` | `WP_SPD_UP`, `WP_SPD_DN` |
| `angular.z` | `ω` | `|ω| ≤ yaw_rate_max` | `yaw_rate_max` | `ATC_RATE_Y_MAX` (deg/s→rad) |

Holonomic: yaw is decoupled from translation, so
`min_turn_radius = 0`. `bank_angle_max`/`lateral_accel_max`
(`ATC_ANGLE_MAX`, `g·tan`) are informational, not contract
bounds.

### Fixed-wing — Dubins airplane

| `cmd_vel` field | symbol | bound | `Capability` field | source param |
|---|---|---|---|---|
| `linear.x` | `V` | `V ∈ [v_min, v_max]` | `v_min`, `v_max` | `AIRSPEED_MIN`, `AIRSPEED_MAX` |
| `angular.z` | `ψ̇` | `|ψ̇| ≤ lateral_accel_max / V` (= `g·tan(φ_max)/V`) | `lateral_accel_max` | `ROLL_LIMIT_DEG` (→ `g·tan`) |
| `linear.z` | `climb` | `−V·sin(γ_dn) ≤ climb ≤ V·sin(γ_up)` | `climb_angle_max` (= `γ_up`); `vz_max_up`/`vz_max_down` (at `v_max`) | `PTCH_LIM_MAX_DEG` (`γ_up`), `PTCH_LIM_MIN_DEG` (`γ_dn`) |

Both the ψ̇ and climb bounds are **speed-coupled** (use the
clipped `V`); `min_turn_radius = 0` because the turn radius
`V²/lateral_accel_max` is not a single constant. `yaw_rate_max`
is `lateral_accel_max / AIRSPEED_CRUISE`, the cruise-speed value
only (a convenience scalar; the real bound is the speed-coupled
form above).

## `capability` semantics

Latched, episode-level descriptor of the velocity-command envelope.
Pulled from FCU parameters at adapter bring-up. Strategies read it
once at start and use it for a-priori feasibility checks.

```
std_msgs/Header header
string  agent_id
uint8   platform                # mirrors AgentState.PLATFORM_*

# Speed envelope (m/s).
float64 v_max
float64 v_min                   # rover/quad: -v_max; fixed-wing: >0

# Vertical envelope (m/s, positive up).
float64 vz_max_up               # 0 for rover
float64 vz_max_down             # 0 for rover

# Heading and turn limits.
float64 yaw_rate_max            # rad/s, body yaw rate
float64 lateral_accel_max       # m/s^2; ψ̇_vel(V) = lateral_accel_max / V
float64 bank_angle_max          # rad; informational (fixed-wing, quad)
float64 climb_angle_max         # rad; informational (fixed-wing)
float64 min_turn_radius         # m; rover |ω|≤|v|/R_min. 0 = n/a

string  source                  # audit trail of FCU param values
```

The `source` field carries which underlying autopilot parameters were
read and what values they held, e.g.

```
WP_SPEED=1.0; CRUISE_SPEED=0.992; GUID_SPEED_MAX=nan;
ATC_STR_RAT_MAX=120.0; ATC_TURN_MAX_G=0.6; TURN_RADIUS=0.9
```

This is essential for reproducibility of any tracking-error report.

## `SaturationFlags` semantics

Reported on every `AgentState` snapshot. `true` means the corresponding
limit was active when the last `cmd_vel` was projected onto the
feasible setpoint.

```
bool linear_velocity      # |linear.x| > v_max
bool angular_velocity     # |angular.z| > yaw_rate_max
bool airspeed             # fixed-wing: |V| outside [v_min, v_max]
bool turn_rate            # fixed-wing: |ψ̇| > g·tan(bank_max)/V
bool climb_rate           # fixed-wing/quad: |vz| outside vertical envelope
```

A strategy that wants to back-pressure on limits should subscribe to
`state` and watch these flags.

## Sending a command manually

With the chain up and the adapter in `READY`:

```bash
source /opt/ros/jazzy/setup.bash
source $PROJ/ros2_ws/install/setup.bash

# Drive forward at 0.5 m/s for 5 seconds.
ros2 topic pub --rate 10 /cmd_vel \
    geometry_msgs/msg/TwistStamped \
    '{header: {frame_id: "base_link"},
      twist: {linear: {x: 0.5}, angular: {z: 0.0}}}' &
PID=$!
sleep 5
kill $PID
```

Watch the rover move via `ros2 topic echo /state`:

```bash
ros2 topic echo /state | grep -A3 position
```

Or read the latched capability:

```bash
ros2 topic echo --qos-reliability reliable \
    --qos-durability transient_local --once /capability
```

## Writing a strategy node

Minimal Python skeleton. Subscribes to opponent state, computes a
command, publishes to the agent's `cmd_vel`. Save as
`scripts/strategy_pure_pursuit.py` (or build into a `rufus_sim_strategies`
package later).

```python
#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
)
from geometry_msgs.msg import TwistStamped
from rufus_sim_msgs.msg import AgentState, Capability


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2*(qw*qz + qx*qy),
                      1 - 2*(qy*qy + qz*qz))


class PurePursuitPursuer(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('self_ns', '/P0')
        self.declare_parameter('target_ns', '/E0')
        sn = self.get_parameter('self_ns').value
        tn = self.get_parameter('target_ns').value

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._self_state = None
        self._target_state = None
        self._cap = None

        self._pub = self.create_publisher(
            TwistStamped, f'{sn}/cmd_vel', 10)
        self.create_subscription(
            AgentState, f'{sn}/state',
            lambda m: setattr(self, '_self_state', m), 10)
        self.create_subscription(
            AgentState, f'{tn}/state',
            lambda m: setattr(self, '_target_state', m), 10)
        self.create_subscription(
            Capability, f'{sn}/capability',
            lambda m: setattr(self, '_cap', m), latched)

        self.create_timer(0.05, self._tick)  # 20 Hz

    def _tick(self):
        if not (self._self_state and self._target_state and self._cap):
            return
        s = self._self_state.pose.position
        t = self._target_state.pose.position
        psi = yaw_from_quat(
            self._self_state.pose.orientation.x,
            self._self_state.pose.orientation.y,
            self._self_state.pose.orientation.z,
            self._self_state.pose.orientation.w,
        )
        # bearing to target in world frame
        bearing = math.atan2(t.y - s.y, t.x - s.x)
        err = math.atan2(math.sin(bearing - psi),
                         math.cos(bearing - psi))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = f'{self._self_state.agent_id}/base_link'
        cmd.twist.linear.x = self._cap.v_max
        cmd.twist.angular.z = max(
            -self._cap.yaw_rate_max,
            min(self._cap.yaw_rate_max, 2.0 * err),
        )
        self._pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(PurePursuitPursuer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Two things this skeleton illustrates:

1. **Wait for `capability` before commanding.** A strategy that issues
   commands above the agent's envelope just gets clamped, and may
   produce surprising tracking. Reading `capability` once on bring-up
   is the contract.
2. **Use the latched QoS profile when subscribing to `capability`.**
   Otherwise nothing arrives.

For richer strategies see `scripts/rover_bench.py`, which drives a
sequence of prescribed `(vx, wz)` trajectories and integrates a
parallel kinematic unicycle for tracking-error analysis.

## Multi-agent

Each agent runs its own SITL + MAVROS + adapter chain, namespaced
by `agent_id` (e.g. `/R0`, `/R1`). A strategy node addresses an
agent by writing to `<agent_id>/cmd_vel` and reading from
`<agent_id>/state` and `<agent_id>/capability`. The contract on
each topic is identical to the single-agent case; nothing about
the message schemas is N-aware.

Bring-up is driven from a YAML manifest in
`rufus_sim_worlds/config/agents/`. The same file is consumed at
build time by `generate_world.py` (to produce the gz world and
per-instance vehicle SDFs) and at launch time by
`rufus_sim_bringup/multi_agent_sim.launch.py` (to spawn one MAVROS +
one adapter per agent). To run a different scenario, write a new
manifest and pass it as `manifest:=<abs path>` — the message
contract above does not change. See [`setup.md`](setup.md) for
the manifest schema and the recipe for adding agents, and
[`operations.md`](operations.md) for the full bring-up sequence.

The Stage 4 deliverable is independent commandability: a
strategy that publishes only to `/R0/cmd_vel` must move only R0,
not R1, even though both rovers share one gz physics world. This
is verified at the end of the multi-agent bring-up section in
[`operations.md`](operations.md).

## Episode-level wrapping (`rufus_sim_game`)

`rufus_sim_game/episode_runner` consumes the per-agent contract
above and adds three game-level topics:

- `/game/state` (`rufus_sim_msgs/GameState`): the current
  `AgentState` for every agent in the manifest, plus the list
  of termination predicate ids whose dwell holds this tick.
- `/game/role_assignments` (`rufus_sim_msgs/RoleAssignment`,
  TRANSIENT_LOCAL latched): one message per agent. Critically,
  this is published **only after every agent's `ready_when`
  predicate has held for its dwell**, i.e. after the warmup
  phase. Strategies wait for the latched message and only then
  start publishing `/<id>/cmd_vel` — until then the runner may
  itself be driving an agent toward its `initial_position`
  waypoint.
- `/game/termination_event` (`rufus_sim_msgs/TerminationEvent`,
  TRANSIENT_LOCAL latched): emitted once when the first
  termination predicate fires or when `duration_s` elapses.

The full DSL, the four-phase runner state machine, the
per-platform parameter override menu, and the rover-only
1v1 smoke test live in [`episodes.md`](episodes.md).

## Strategies (`rufus_sim_strategies`)

A strategy is a Python `Strategy` subclass that maps a
per-tick `Measurement` to a `Twist` on `/<agent_id>/cmd_vel`.
The strategy runs inside a per-agent `strategy_runner` ROS
node, gated on the `/game/role_assignments` hand-off latch so
it never fights the runner's warmup driver. Episode YAMLs
reference strategies by name through a registry; the package
ships `pure_pursuit_pursuer`, `constant_bearing_evader`, and
`lead_pursuer` as reference implementations covering the
memoryless and stateful patterns.

Full ABC, `Measurement` schema, registry mechanics, episode
YAML extension, and recording recipe live in
[`strategies.md`](strategies.md). Terminology there follows
control conventions: `Measurement` for the input bundle,
`control` for the per-tick method, `Twist` for the control
output.

## Frame and unit conventions

- **World frame**: ENU (`x`=east, `y`=north, `z`=up). Origin at the
  FCU's home position. `header.frame_id = "map"`.
- **Body frame**: forward = `+x`, left = `+y`, up = `+z`. Yaw rate
  is `+ω` for counter-clockwise as viewed from above (right-hand rule
  about `+z`).
- **Angles**: radians.
- **Distances**: metres.
- **Time**: ROS time (sim time when `--use_sim_time` is set; wall
  clock otherwise).

ArduPilot internally uses NED + degrees in many places. MAVROS
performs the ENU↔NED conversion; everything our adapter publishes is
ENU. Strategies should never see NED.
