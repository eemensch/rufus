# Episode definitions

An episode is a YAML file describing a pursuit-evasion game over
a fixed agent manifest. It is consumed by
`rufus_sim_game/episode_runner`, which loads it, applies any
per-agent FCU parameter overrides, drives each agent to its
configured initial position during a warmup phase, and then
evaluates the listed termination predicates each tick.

The runner publishes:

- `/game/state` (`rufus_sim_msgs/GameState`): the current AgentStates
  plus the list of predicate ids whose dwell window has closed in
  this tick. Published in every phase, including warmup
  (`sim_time = 0` until the game clock starts).
- `/game/role_assignments` (`rufus_sim_msgs/RoleAssignment`,
  TRANSIENT_LOCAL latched): one message per agent, *published
  only after every agent's `ready_when` has held for its dwell*.
  This is the hand-off signal — strategies must wait for it
  before publishing their own cmd_vel; until then, the runner
  owns each agent's `cmd_vel` if it is driving the agent toward
  an initial waypoint.
- `/game/termination_event` (`rufus_sim_msgs/TerminationEvent`,
  TRANSIENT_LOCAL latched): one message at the moment the first
  termination predicate's dwell completes, or at the moment
  elapsed sim time reaches `duration_s`
  (`outcome=timeout`).

The episode runner does not bring up the SITLs, MAVROS, or
adapters. Bring those up first per [`operations.md`](operations.md);
then start the runner via the launch file documented at the
bottom of this page. To run an episode many times across a
parameter sweep instead of by hand, see
[`evaluation.md`](evaluation.md).

## Anatomy of an episode YAML

```yaml
schema_version: 1                       # only `1` supported today
name: rover_capture_1v1                 # episode_id in published msgs
description: |
  Free-form text. Not interpreted by the runner.

manifest: package://rufus_sim_worlds/config/agents/two_rovers.yaml
                                        # the agent manifest the
                                        # SITL chain was brought
                                        # up against. Resolved
                                        # via the ament index for
                                        # `package://` URIs;
                                        # otherwise treated as a
                                        # filesystem path
                                        # (absolute, or relative
                                        # to the YAML's directory).

agents:                                 # per-agent episode-level
  R0:                                   # config. Every agent_id
    role: pursuer                       # listed must exist in the
    parameters:                         # manifest. Every field
      high_level: { v_max: 2.0 }        # under the agent_id is
    initial_position: [3.0, 0.0, 0.05]  # optional.
    ready_when:
      expr: "(R0.x - 3)**2 + (R0.y)**2 < 0.5**2"
      dwell_s: 0.3
  R1:
    role: evader

duration_s: 60.0                        # hard timeout; emits
                                        # TerminationEvent with
                                        # outcome=timeout.

tick_rate_hz: 50.0                      # optional. The runner
                                        # ticks the predicate
                                        # engine, the warmup
                                        # driver, and /game/state
                                        # at this rate; each
                                        # strategy_runner consumes
                                        # /game/state and publishes
                                        # /<id>/cmd_vel once per
                                        # tick. See "Tick rate"
                                        # below for the supported
                                        # range.

termination:                            # ordered list of named
  - id: capture                         # predicates. First to
    expr: "(R0.x - R1.x)**2 + (R0.y - R1.y)**2 < 0.5**2"
    dwell_s: 0.5                        # complete its dwell wins;
    outcome: pursuer_win                # ties broken by listed
                                        # order.

  - id: evader_escape
    expr: "R1.x**2 + R1.y**2 > 50**2"
    dwell_s: 0.0
    outcome: evader_win
```

Required fields: `schema_version`, `name`, `manifest`,
`duration_s`. `agents`, `termination`, and `tick_rate_hz` are
optional. An episode with no predicates simply runs to timeout;
one with no `tick_rate_hz` ticks at the launch-level default
(50 Hz).

## Per-agent fields under `agents:`

`role` — pursuer | evader | neutral. Pure label. The runner
publishes it in `/game/role_assignments` once the warmup is
done, and in `AgentState.role` when republishing GameState. It
plays no role in predicate evaluation.

`parameters` — optional FCU parameter overrides. See [Agent
parameter reference](#agent-parameter-reference) below for the
precise per-platform menu of `high_level` knobs and how each
maps to the underlying FCU parameter. The `fcu` sub-block writes
named FCU parameters directly, bypassing the translation. Both
sub-blocks coexist; `high_level` is expanded first, then `fcu`
overrides land on top, so an episode can pin one component via
`high_level` and a niche related parameter via `fcu` in the same
block.

`initial_position` — optional `[x, y, z]` waypoint in the world
ENU frame. When set, the runner publishes `/<id>/cmd_vel` to
drive the agent to that point during the warmup phase, using a
per-platform pursuit rule. Plane agents drive at 70 % of their
captured `Capability.v_max` (or 12 m/s as a fallback) toward the
waypoint as a world-frame velocity, then stop publishing once
`ready_when` latches so the adapter's cruise fallback takes
over and the plane loiters near the waypoint.

`ready_when` — optional polynomial inequality (same DSL as
termination predicates) that must hold for `dwell_s` seconds
before the runner declares this agent ready. If `initial_position`
is set without an explicit `ready_when`, the runner synthesises
the tolerance check
`(<id>.x-tx)**2 + (<id>.y-ty)**2 + (<id>.z-tz)**2 < 0.5**2`
with `dwell_s = 0`. If neither field is set, the agent is ready
as soon as its first `AgentState` arrives (the implicit "no
warmup" case — fits rovers that spawn in place and quads/planes
where you want the game to start the moment they finish takeoff).

## Predicate grammar

Each `expr` (in both `termination` and `ready_when`) is a
boolean expression over polynomial inequalities in the allowed
agent state symbols:

```
expr   := rel
        | expr & expr        # conjunction
        | expr | expr        # disjunction
        | ~expr              # negation
        | (expr)
rel    := poly OP poly
OP     := < | <= | > | >= | == | !=
poly   := <numeric literal>
        | <agent_id>.<component>
        | poly + poly | poly - poly | poly * poly | poly / poly
        | poly ** <integer>
        | (poly)
        | -poly
```

The boolean operators are Python's bitwise `&`, `|`, `~` — the
keyword forms `and`, `or`, `not` cannot be used because
`Relational.__bool__` raises (sympy refuses to coerce a symbolic
inequality to a Python bool, which is what `and`/`or`/`not`
require). Chained comparisons (`0 < x < 5`) are rejected for
the same reason.

Each leaf inequality must be polynomial: at every `rel` site,
`lhs - rhs` must be a polynomial in the symbols that appear.
`sympy.Poly` enforces this per-leaf.

Allowed `<component>` values are
`x, y, z, vx, vy, vz, psi, qw, qx, qy, qz`:

| component  | meaning                                 | frame                   |
|------------|-----------------------------------------|-------------------------|
| `x, y, z`  | position                                | world (ENU)             |
| `vx,vy,vz` | linear velocity                         | body                    |
| `psi`      | yaw extracted from quaternion           | world (rotation about z)|
| `qw,qx,qy,qz` | orientation quaternion               | world                   |

**Position semantics need a footnote.** The AgentState message
nominally carries pose in the world frame, but the per-platform
adapter currently fills it from the FCU's `local_position/pose`,
which is *each agent's own EKF home* — under multi-agent that
collapses to (0, 0) for every agent until they move. The runner
sidesteps this by also subscribing to gz's
`/world/<world>/dynamic_pose/info` (bridged to a
`tf2_msgs/TFMessage` called `/game/world_pose`) and overriding
`pose` with the ground-truth world pose before evaluating
predicates and before republishing the agent in `/game/state`.
The `episode.launch.py` brings up that bridge automatically.

After parsing, `lhs - rhs` must be a polynomial in the free
symbols (verified with `sympy.Poly`); anything that produces a
non-integer or negative `Pow` exponent — `sqrt`, `1/x`, etc. —
is rejected. Transcendental functions (`sin`, `cos`, `exp`, ...)
are not in the allowed name set and parse-time-fail.

Constant predicates (`1 < 2`) and predicates with no agent state
symbols are rejected: they cannot trigger on any agent state.

### Boolean composition

Inequalities combine with `&` (AND), `|` (OR), `~` (NOT) inside
a single predicate string. Two common Stage 7 patterns:

```yaml
# multi-pursuer capture — any pursuer within 0.5 m of the evader.
- id: capture
  expr: >
    ((R0.x - E0.x)**2 + (R0.y - E0.y)**2 < 0.5**2) |
    ((R1.x - E0.x)**2 + (R1.y - E0.y)**2 < 0.5**2)
  dwell_s: 0.5
  outcome: pursuer_win

# multi-evader escape — fires only when *all* evaders leave.
- id: all_evaders_escape
  expr: >
    (E0.x**2 + E0.y**2 > 50**2) &
    (E1.x**2 + E1.y**2 > 50**2) &
    (E2.x**2 + E2.y**2 > 50**2)
  outcome: evader_win
```

The engine recurses through `And`/`Or`/`Not` to find the leaf
inequalities and applies the polynomial-form check per leaf.

Three things to watch:

- **No keyword operators.** `and`, `or`, `not` cannot be used
  in predicate strings — `Relational.__bool__` raises in sympy.
  Use `&`, `|`, `~`.
- **Chained comparisons rejected** for the same reason.
  `0 < R0.x < 5` parses through Python's `and`; write
  `(R0.x > 0) & (R0.x < 5)`.
- **Margin is informational for compound predicates.**
  `CompiledPredicate.margin(values)` returns `lhs - rhs` for an
  atomic predicate; for compounds it returns 0.0 because there
  is no single signed margin. Split a compound into named atoms
  if you need numeric diagnostics.

## Phases of an episode run

```
        [param_setup]   FCU parameter writes for every override in
              |         `agents.<id>.parameters`. Skipped if no
              v         overrides. /game/state publishes with
                        sim_time=0 and active_predicates=[].
        [warmup]        For each agent that has initial_position,
              |         the runner publishes /<id>/cmd_vel to
              |         drive it there. Each agent's ready_when
              v         is polled every tick. /game/state
                        publishes with sim_time=0.
        [running]       /game/role_assignments published.
              |         Strategies take over; runner stops
              |         publishing cmd_vel. The duration_s
              v         clock starts; termination predicates
                        evaluated each tick.
        [terminated]    /game/termination_event published.
                        /game/state continues; predicates no
                        longer evaluated.
```

Per-platform warmup driving rule:

| platform | rule                                                                        |
|----------|-----------------------------------------------------------------------------|
| rover    | pure-pursuit on 2D `(x, y)`; body `linear.x` modulated by `cos(heading_err)`, `angular.z` proportional to `heading_err`. `z` ignored. |
| quad     | proportional control on world-frame `(x, y, z)` error, rotated into body frame using current yaw. Speed capped at 2 m/s. |
| plane    | world-ENU velocity equal to a unit vector toward the target scaled by `0.7 · Capability.v_max` (12 m/s if Capability hasn't arrived yet). The fixed-wing adapter projects this onto airspeed + heading + climb. Once `ready_when` latches the runner stops publishing for that plane and the adapter's cruise fallback loiters it near the waypoint. |

Dwell windows compare against sim time, not wall time. With
`use_sim_time:=true` (the default for any chain brought up via
`multi_agent_sim.launch.py`), `--speedup > 1` does not distort
either `dwell_s` or `duration_s`.

## Tick rate

`tick_rate_hz` controls how often the runner ticks the predicate
engine and publishes `/game/state`, and therefore how often each
`strategy_runner` calls its `Strategy.control` and publishes
`/<id>/cmd_vel`. Default 50 Hz; explicit YAML value overrides
the launch default.

Supported range: **0.5 Hz to 200 Hz**, validated at episode
load (out-of-range values raise `EpisodeLoadError`).

- **Lower bound (0.5 Hz).** ArduPilot GUIDED's setpoint
  inactivity timeout is ~3 s. Below ~0.4 Hz, the FCU sees the
  setpoint stream as stale, falls back to its safety mode
  (RTL/LAND), and the agent stops responding to the strategy.
  Sub-Hz rates can still make sense for slow rovers under
  extreme `--speedup`, but stay above the timeout.
- **Upper bound (200 Hz).** gz physics step is 1 ms (1 kHz)
  and SITL accepts MAVLink setpoints faster than that, but the
  full path
  `Strategy.control → /<id>/cmd_vel → adapter → MAVROS
  setpoint plugin → MAVLink TCP → SITL` accumulates per-stage
  latency that becomes audible above ~100 Hz under our
  4-MAVROS plugin set
  ([`rufus_sim_bringup/config/mavros_pluginlists.yaml`](https://github.com/...)).
  100 Hz is the practical ceiling for the chain we ship; 200 Hz
  is the hard cap so the runner doesn't accept obviously bad
  values.

Choose your tick rate by what the strategy needs, not by what
the chain can do. A pure-pursuit pursuer that just heads at
the target's current position works fine at 10 Hz; a strategy
that integrates a Kalman filter or chases fast-moving
adversaries may want 50–100 Hz. Higher rates buy nothing if
the strategy doesn't actually compute new commands.

## Outcome strings

`outcome` is a free string that ends up verbatim in the
`TerminationEvent.outcome` field. The conventions, defined as
constants in `rufus_sim_msgs/TerminationEvent.msg`, are:

| outcome string  | meaning                                        |
|-----------------|------------------------------------------------|
| `pursuer_win`   | a pursuer-favourable predicate fired           |
| `evader_win`    | an evader-favourable predicate fired           |
| `draw`          | a structurally-mutual predicate fired          |
| `timeout`       | reserved for the runner's `duration_s` event   |

Anything else is permitted; downstream tooling may not understand
non-conventional values.

## Agent parameter reference

The `parameters: high_level:` block under each agent uses the
canonical names from `rufus_sim_msgs/Capability.msg`. The runner
translates them to FCU parameters at episode start (see
`rufus_sim_game/agent_params.py`). The translation tables in this
section and that module are kept in sync; `agent_params.py` is
the source of truth for what the runner actually does.

Values in `high_level:` are in SI units matching the
corresponding `Capability` field (m/s, rad/s, rad, etc.); the
translation handles unit conversions to whatever ArduPilot
expects.

### `rover` (`PLATFORM_ROVER`)

| high_level option | unit  | FCU param(s)              | FCU unit |
|-------------------|-------|---------------------------|----------|
| `v_max`           | m/s   | `WP_SPEED`, `CRUISE_SPEED` | m/s      |
| `yaw_rate_max`    | rad/s | `ATC_STR_RAT_MAX`         | deg/s    |
| `lateral_accel_max` | m/s² | `ATC_TURN_MAX_G`         | g        |

Notes:

- Both `WP_SPEED` and `CRUISE_SPEED` are written for `v_max`
  because rover_adapter takes `max(WP_SPEED, CRUISE_SPEED,
  GUID_SPEED_MAX)` when computing the published Capability;
  setting just one of the three would not bound the envelope
  if any of the others is higher.
- `GUID_SPEED_MAX` is intentionally not written from
  `high_level: v_max` — it is rejected on some ArduRover builds
  and the override would fail-loudly there. Set it via the
  `fcu:` sub-block if you need it.

### `quad` (`PLATFORM_QUADROTOR`)

| high_level option | unit  | FCU param(s)         | FCU unit |
|-------------------|-------|----------------------|----------|
| `v_max`           | m/s   | `WP_SPD`             | m/s      |
| `vz_max_up`       | m/s   | `WP_SPD_UP`          | m/s      |
| `vz_max_down`     | m/s   | `WP_SPD_DN`          | m/s      |
| `yaw_rate_max`    | rad/s | `ATC_RATE_Y_MAX`     | deg/s    |
| `bank_angle_max`  | rad   | `ATC_ANGLE_MAX`      | deg      |

The `WP_SPD*` parameter family was renamed from `WPNAV_SPEED*`
(and the units changed from cm/s to m/s) in the ArduPilot 4.7+
SI-suffix migration. Older episode YAMLs that named
`WPNAV_SPEED` directly under `fcu:` will silently no-op against
4.7+ builds.

### `plane` (`PLATFORM_FIXED_WING`)

| high_level option | unit  | FCU param(s)        | FCU unit |
|-------------------|-------|---------------------|----------|
| `v_min`           | m/s   | `AIRSPEED_MIN`      | m/s      |
| `v_cruise`        | m/s   | `AIRSPEED_CRUISE`   | m/s      |
| `v_max`           | m/s   | `AIRSPEED_MAX`      | m/s      |
| `bank_angle_max`  | rad   | `ROLL_LIMIT_DEG`    | deg      |
| `climb_angle_max` | rad   | `PTCH_LIM_MAX_DEG`  | deg      |
| `climb_angle_min` | rad   | `PTCH_LIM_MIN_DEG`  | deg      |

`v_cruise` is informational rather than a hard envelope bound:
ArduPlane targets that airspeed between waypoints, but the
flight envelope is `[AIRSPEED_MIN, AIRSPEED_MAX]`.
`climb_angle_min` is the descent pitch limit (negative number;
the value passed in is the magnitude in radians, written as a
negative-degrees value).

### Setting parameters via `fcu:`

For anything not in the high_level menu (rover steering D-gain,
plane TECS tunables, sensor offsets, ...), use the `fcu:`
sub-block:

```yaml
parameters:
  R0:
    high_level: { v_max: 1.5 }
    fcu:
      ATC_STR_ACC_MAX: 60.0     # extra steering accel cap
      WP_SPEED: 1.4             # if you want to override what
                                # high_level wrote, this lands
                                # afterwards.
```

Names must match the FCU parameter exactly (post-AP-4.7-rename
where applicable); values are written as `PARAMETER_DOUBLE`
regardless of underlying integer/double typing — ArduPilot
coerces.

## Bringing up an episode

The chain (gz, SITL per agent, MAVROS, adapters) must already be
running per [`operations.md`](operations.md). With the chain up:

```bash
ros2 launch rufus_sim_game episode.launch.py \
    episode_path:=$PROJ/ros2_ws/install/rufus_sim_game/share/rufus_sim_game/config/episodes/rover_capture_1v1.yaml
```

That launch reads the episode YAML, follows its `manifest:`
reference to discover the world name, brings up a
`ros_gz_bridge parameter_bridge` from
`/world/<world>/dynamic_pose/info` to `/game/world_pose`, and
spawns `rufus_sim_game/episode_runner` configured with the same
episode path and `world_pose_topic:=/game/world_pose`.

To watch the result:

```bash
ros2 topic echo /game/state                # 50 Hz; high volume
ros2 topic echo /game/role_assignments     # one per agent, latched
ros2 topic echo /game/termination_event    # one-shot, latched
```

A reference dummy strategy that closes the loop — a pure-pursuit
pursuer driving R0 toward R1, gated on the role_assignments
hand-off — lives at `scripts/dummy_pursuer.py` and is the smoke
test for Stage 5.

## Limitations

- **No keyword operators in expressions.** `and`/`or`/`not` are
  not valid predicate operators because sympy refuses to coerce
  symbolic Relationals to Python booleans. Use `&`, `|`, `~`.
- **World-pose bridge — done.** `ros_gz_bridge`'s
  `gz.msgs.Pose_V → tf2_msgs/TFMessage` converter drops the
  per-pose `name`, which broke worlds with nested `<include>`s
  (e.g. iris's `iris_with_standoffs` child) for an earlier
  index-based fallback. The runner now uses a custom Python
  bridge (`rufus_sim_game.world_pose_bridge`) that subscribes to
  the gz topic via `gz.transport13`, copies the `name` field
  into `TransformStamped.child_frame_id`, and republishes the
  message; the `episode_runner` matches transforms by id.
- **Twist frames.** `vx, vy, vz` are body-frame (per
  `AgentState.msg`); world-frame velocities require trig of
  `psi` and are therefore non-polynomial. Predicates needing
  world-frame speeds will have to wait for either v2's broader
  expression set or for a runner-side derivative of position.
- **Capability resync race.** The runner republishes
  `/<id>/capability` on the same topic the adapter latched at
  startup. With TRANSIENT_LOCAL durability and two publishers,
  a new subscriber can receive both messages — adapter's first,
  runner's second — and must pick the one with the later
  header timestamp (or the one whose `source` includes
  `episode_override=...`). The cleaner v2 fix is for the
  adapter to expose a refresh service the runner calls, so
  there is only ever one latched message at a time.
