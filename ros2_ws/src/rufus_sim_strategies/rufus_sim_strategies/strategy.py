"""Pursuit-evasion strategy ABC.

Terminology: this module follows control-systems convention.
The per-tick input bundle is a `Measurement` (what the agent
"sees"); the method that maps a measurement to an action is
`control` (compute the next control input); the action itself
is the control output, returned as a `geometry_msgs/Twist` in
the platform's `cmd_vel` frame. Where ROS conventions force
the word "action" or "command" elsewhere (topic name
`cmd_vel`, message field `Twist`), we keep those names.

A `Strategy` instance owns one agent's commands across a single
episode. The runtime `strategy_runner` node:

  1. Waits for the runner's `/game/role_assignments` latch
     (the runner publishes that only after every agent's
     `ready_when` has held for its dwell — the warmup is over).
  2. Waits for the next `/game/state` after the latch.
  3. Calls `Strategy.reset()` exactly once.
  4. On every subsequent `/game/state`, builds a `Measurement`,
     calls `Strategy.control(measurement)`, and publishes the
     returned `Twist` (header-stamped) on `/<agent_id>/cmd_vel`.

Strategies are fully opaque to the runtime: anything the
implementation stores on `self` between ticks is private. The
only contract the runtime depends on is the methods below.

Stateful strategies are first-class. The runtime calls
`control` once per tick and writes nothing else to the
`Strategy` instance — anything the strategy stashes on `self`
inside `__init__`, `reset`, or any prior `control` call is
preserved on the next call. Use this to maintain integrator
state, cached observations for finite-differencing target
velocity, neural-net hidden state, last-action history, or
any other internal model. `LeadPursuer` (in `reference.py`)
is a small worked example.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Mapping

from geometry_msgs.msg import Twist
from rufus_sim_msgs.msg import AgentState, Capability


@dataclass(frozen=True)
class Measurement:
    """One tick's measurement passed to a Strategy's `control`.

    `agents` is keyed by `agent_id` so subclasses can look up any
    agent in O(1) without iterating the GameState. `my_state` is
    a convenience reference to `agents[self.agent_id]`.

    `sim_time_s` is seconds since the runner's game clock
    started, i.e. seconds since the warmup-to-running
    transition; the strategy never sees a `sim_time_s` of 0
    from the warmup phase.

    `active_predicates` lists termination predicate ids whose
    dwell currently holds — strategies can use this to gate
    last-ditch maneuvers (e.g., bail out when a capture
    predicate is one tick from firing).

    Frozen so a strategy can stash a reference (for
    finite-differencing across ticks, replay, etc.) without
    worrying about external mutation between ticks.
    """

    sim_time_s: float
    agents: Mapping[str, AgentState]
    my_state: AgentState
    my_capability: Capability
    active_predicates: tuple[str, ...]
    episode_id: str


class Strategy(ABC):
    """Base class for pursuit-evasion strategies.

    Subclass and implement `control`. `__init__` and `reset`
    are optional override points for state setup.

    The runtime calls methods in this order, exactly once per
    episode:

      strategy = StrategyClass(agent_id=..., params={...})
      strategy.reset()
      while not terminated:
          twist = strategy.control(measurement)
          # runtime publishes twist on /<agent_id>/cmd_vel

    The returned `Twist` is the control output, in the agent's
    `cmd_vel` frame, which depends on the platform read off
    `measurement.my_capability`:

      - rover, quad: body frame.
        `linear.x` = forward speed (m/s),
        `angular.z` = yaw rate (rad/s).
      - plane: world ENU velocity vector.
        `linear.{x, y, z}` = world-frame components (m/s).

    Use `measurement.my_capability.platform` (compared against
    `AgentState.PLATFORM_*` constants) to dispatch when one
    Strategy class supports multiple platforms.
    """

    def __init__(self, *, agent_id: str, params: dict | None = None):
        self.agent_id = agent_id
        self.params = dict(params or {})

    def reset(self) -> None:
        """Hook called once before the first `control` call.

        Default no-op. Subclasses can override to initialise
        per-episode state (PRNG seeds, integrator memory,
        cached previous measurement, ...).
        """

    @abstractmethod
    def control(self, measurement: Measurement) -> Twist:
        """Return the control output for this tick.

        `measurement.my_state` is guaranteed to be present; the
        runtime would not call `control` otherwise.
        """
