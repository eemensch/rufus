# Strategies

A *strategy* is a Python object that maps the simulator's view
of the world to a control command for one agent. The package
`rufus_sim_strategies` ships:

- The `Strategy` ABC.
- A `Measurement` dataclass that bundles one tick's view.
- A name-keyed registry (`rufus_sim_strategies.registry`) that
  episode YAMLs reference by string.
- A per-agent `strategy_runner` ROS node that hosts one
  `Strategy` instance, mediates the hand-off contract, and
  publishes `/<agent_id>/cmd_vel`.
- Three reference strategies — `PurePursuitPursuer`,
  `ConstantBearingEvader`, `LeadPursuer` — which double as
  worked examples of memoryless and stateful patterns.

Terminology follows control-systems convention: the per-tick
input bundle is a `Measurement`, the method that computes the
next command is `control`, and the returned `Twist` is the
control output. This deliberately departs from the more common
ML-RL phrasing (`Observation`, `select_action`) so the rest of
the codebase reads as a control system rather than an agent
loop. The ROS message field on the wire stays `Twist` and the
topic stays `cmd_vel` — those names are part of the platform
contract documented in [`control.md`](control.md).

## The `Strategy` ABC

```python
class Strategy(ABC):
    def __init__(self, *, agent_id: str, params: dict | None = None):
        ...
    def reset(self) -> None: ...        # default no-op
    @abstractmethod
    def control(self, measurement: Measurement) -> Twist: ...
```

Subclass and implement `control`. Override `reset` if your
strategy needs episode-start setup that depends on the agent_id
or params (the place to zero integrators, clear cached
measurements, seed PRNGs, ...). `__init__` is rarely overridden;
the runtime calls it with the agent_id and params dict pulled
from the episode YAML, then immediately calls `reset` before any
`control`.

The runtime never inspects fields on the strategy object. State
on `self` is fully private; anything stashed in `__init__`,
`reset`, or any prior `control` call is preserved on the next
call. This is how stateful strategies are supported — there is
no special `state_dict` mechanism and no save/load hook.

### `Measurement` fields

| field                | type                     | meaning                                                         |
|----------------------|--------------------------|-----------------------------------------------------------------|
| `sim_time_s`         | `float`                  | seconds since the runner's game clock started (after warmup)    |
| `agents`             | `Mapping[str, AgentState]`| every agent in the episode, keyed by `agent_id` for O(1) lookup |
| `my_state`           | `AgentState`             | convenience reference to `agents[self.agent_id]`                |
| `my_capability`      | `Capability`             | latched envelope (post-override if the episode used `parameters:`)|
| `active_predicates`  | `tuple[str, ...]`        | termination predicate ids whose dwell holds this tick           |
| `episode_id`         | `str`                    | the episode YAML's `name`                                       |

The dataclass is frozen, so a strategy can stash a reference
without worrying about external mutation between ticks (useful
for finite-differencing target velocities).

### What the strategy returns

A `geometry_msgs/Twist` in the agent's `cmd_vel` frame:

- rover, quad: body frame (`linear.x` = forward, `angular.z`
  = yaw rate).
- plane: world ENU velocity vector
  (`linear.{x, y, z}` = world-frame components).

Use `measurement.my_capability.platform` (compared against
`AgentState.PLATFORM_*` constants) to dispatch when one
Strategy class supports multiple platforms. The reference
strategies all do this; see `reference.py`.

## The `strategy_runner` lifecycle

One process per agent. The launch in
`rufus_sim_strategies/launch/episode_with_strategies.launch.py`
spawns one `strategy_runner` per agent that has a `strategy:`
block in the episode YAML.

The runner's tick sequence:

1. Subscribe to `/game/role_assignments` (TRANSIENT_LOCAL),
   `/game/state`, and `/<agent_id>/capability`
   (TRANSIENT_LOCAL). Stay silent (publish nothing).
2. When `/game/role_assignments` arrives with this runner's
   `agent_id`, set `_handed_off = True`. The strategy is still
   not invoked — wait for one more `/game/state`.
3. When the next `/game/state` arrives, build a
   `Measurement`, call `Strategy.reset()` exactly once
   (`_started = True`), then call `Strategy.control(m)` and
   publish the returned `Twist` (header-stamped) on
   `/<agent_id>/cmd_vel`.
4. On every subsequent `/game/state`, build a fresh
   `Measurement` and call `Strategy.control(m)`. The strategy
   runs at the runner's tick rate (50 Hz by default).

The hand-off contract — wait for the latched
`role_assignments` *and* the next `/game/state` — guarantees
strategies don't fight the runner's warmup driver and don't
operate on stale (warmup-phase) measurements.

If `Strategy.control` raises, the runner logs the exception and
publishes a zero `Twist` for that tick. This keeps the chain
alive while a buggy strategy is debugged; the agent stops
moving rather than the whole episode crashing.

## The registry

`rufus_sim_strategies.registry` maps `name -> Strategy class`.
Episode YAMLs name strategies by string; the runner looks up
the class through this registry.

Reference strategies are registered via side effect when
`rufus_sim_strategies` is imported (`__init__.py` imports
`reference`, which calls `register(...)` for each class).

Adding a new strategy in this package: define the class
anywhere under `rufus_sim_strategies/`, import it from
`__init__.py`, and call `register('your_name', YourStrategy)`.

Adding a strategy from an external package is a v2 follow-up;
for now the registry is in-process and v1 ships only the
references.

## Episode YAML strategy block

Per-agent extension to the existing schema (see
[`episodes.md`](episodes.md)):

```yaml
agents:
  R0:
    role: pursuer
    strategy:
      type: pure_pursuit_pursuer       # registry key
      params:                           # passed to Strategy.params
        target: R1
        v_factor: 1.0
        k_psi: 2.0
  R1:
    role: evader
    strategy:
      type: constant_bearing_evader
      params:
        threat: R0
        bearing_offset: 3.14159265
        v_factor: 0.95
```

Agents without a `strategy:` block receive no commands from any
`strategy_runner`; their `cmd_vel` is whoever else is
publishing (typically nothing once the warmup driver hands off,
so the agent goes idle/loiters). This is the path to keep an
agent passive in an episode, e.g. a stationary witness.

## Reference strategies

### `pure_pursuit_pursuer` — memoryless

Drive at the target's *current* position. Required param:
`target` (agent_id). Optional: `v_factor` (default 1.0),
`k_psi` (default 2.0), `k_pos` (default 0.7).

Per-platform behaviour:

- rover: heading toward target's (x, y); body `linear.x`
  modulated by `cos(heading_err)`, `angular.z` proportional to
  the wrapped heading error and capped at `yaw_rate_max`.
- quad: 3D world-frame proportional control on the position
  error, rotated into body frame using current yaw, capped at
  `v_factor * v_max` horizontally and `vz_max_up/down`
  vertically. Yaw aimed along the horizontal velocity.
- plane: world-ENU velocity equal to the unit vector toward
  the target, scaled by `max(v_min, v_factor * v_max)`. The
  altitude error becomes `vz_world` and the
  fixed-wing adapter projects to airspeed/heading/climb.

### `lead_pursuer` — stateful

Same per-platform pursuit as above, but the target point is
*lead* by `lead_time_s * v_target`, where `v_target` is a
finite-differenced estimate of the target's world-frame
velocity. The strategy stashes the previous `Measurement` in
`self._prev` and computes `dt` from the two `sim_time_s`
values. On the very first tick `_prev is None` so the strategy
collapses to memoryless pure pursuit; from the second tick on
it leads.

This is the canonical example of a stateful strategy:
internal state on `self`, no extra contract with the runtime.
Override `reset` to clear `_prev` (the runtime calls
`reset` exactly once per episode, so this is automatic when
the strategy is reused across episodes — currently moot since
each episode spawns a fresh strategy_runner process, but the
contract is in place for future runners that pool processes).

Required param: `target`. Optional: `lead_time_s` (default
1.0), and the same `v_factor`/`k_psi`/`k_pos` knobs as
`PurePursuitPursuer`.

### `constant_bearing_evader` — memoryless

Maintain a fixed angle `bearing_offset` from the LOS to a named
threat. Default `bearing_offset = π` flees directly away;
`±π/2` produces orbit-style escape; arbitrary offsets are
accepted. Required param: `threat`. Optional: `bearing_offset`
(default π), `v_factor`, `k_psi`, `k_pos`, `flee_distance_m`
(default 100, sets the virtual waypoint distance for quad/plane).

Per-platform: rover/plane heading directly to the desired
world-frame angle; quad uses a virtual waypoint at
`flee_distance_m` ahead in the desired direction.

## Writing a stateful strategy

The pattern is straightforward — store on `self`, read on the
next `control` call:

```python
class CrudeKalmanPursuer(Strategy):
    def reset(self):
        self._target = self.params['target']
        self._x_hat = None     # Kalman state estimate
        self._P = None          # covariance
        self._last_t = None

    def control(self, m):
        z = ...   # build measurement vector from m.agents[self._target]
        if self._x_hat is None:
            self._x_hat = z
            self._P = ...
        else:
            dt = m.sim_time_s - self._last_t
            # predict + update on self._x_hat, self._P
            ...
        self._last_t = m.sim_time_s
        # use self._x_hat to derive a setpoint, build a Twist
        return twist
```

No serialisation/restore hooks are needed; the runtime never
serialises the strategy. If you need to record a strategy's
internal state for replay, log it on a custom topic from inside
`control` — that is opaque to the runtime and ends up in the
rosbag2 stream just like any other publisher.

## Recording episodes

`episode_with_strategies.launch.py` defaults to recording an
episode rosbag2 to `/tmp/rufus_sim_bags/<episode_name>_<timestamp>`.
Override with the `bag_dir:=` launch argument or disable with
`record_bag:=false`.

Captured topics: `/game/state`, `/game/role_assignments`,
`/game/termination_event`, `/clock`, and per agent
`/<id>/state`, `/<id>/cmd_vel`, `/<id>/capability`.

`ros2 bag play` against this bag reproduces the
strategy-side commands and the runner's published GameState.
Replaying *into* a fresh chain to reproduce the trajectory is a
Stage 6 follow-up — strict bit-equal determinism with SITL is
non-trivial (see `plan.md`).
