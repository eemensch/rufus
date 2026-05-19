"""Episode runner: load an episode YAML, set per-agent FCU
parameter overrides, drive each agent to its initial position
during a warmup phase, then evaluate polynomial termination
predicates over per-agent state and emit GameState +
TerminationEvent.

The runner takes one ROS parameter, `episode_path`, pointing at an
episode YAML. It loads the manifest the episode references,
subscribes to `/<agent_id>/state` for every agent, and ticks at
`tick_rate_hz` (default 50 Hz).

Phases
------

Each tick advances the runner through one of four phases:

  param_setup  Issue `mavros/<ns>/param/set` calls for every
               override in the episode's per-agent
               `parameters:` block. Skipped immediately if no
               overrides. Predicate evaluation and warmup
               driving are gated until all calls return.

  warmup       For each agent that has an `initial_position`,
               publish `/<id>/cmd_vel` aimed at that target.
               Per-platform driving rules are documented inline
               below. Each agent's `ready_when` predicate is
               polled every tick; when every agent's
               `ready_when` has held for its dwell, the runner
               latches `/game/role_assignments` (one per agent)
               and transitions to `running`. Strategies must
               wait for that latched topic before publishing
               their own cmd_vel — that is the hand-off
               contract.

  running      The `duration_s` clock starts. Termination
               predicates are evaluated each tick; first to
               fire wins, ties broken by config order.
               `/game/state` carries `sim_time = elapsed`;
               `active_predicates` lists the predicate ids
               whose dwell holds this tick.

  terminated   `/game/termination_event` has been published.
               `/game/state` continues so consumers can confirm
               the final configuration; no more predicates are
               evaluated and the runner never emits a second
               event.

Sim time semantics
------------------

The runner reads `now()` from its own clock, which respects the
standard ROS `use_sim_time` parameter. With `use_sim_time:=true`
(the default for any chain brought up via
`multi_agent_sim.launch.py`), all timestamps and the dwell
accumulators are in gz sim time, so `--speedup > 1` does not
distort `dwell_s` or `duration_s`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, Quaternion, Twist, TwistStamped
from mavros_msgs.srv import ParamSetV2
from rufus_sim_msgs.msg import (
    AgentState, Capability, GameState, RoleAssignment,
    TerminationEvent,
)
from rcl_interfaces.msg import ParameterType, ParameterValue
from tf2_msgs.msg import TFMessage

from .agent_params import (
    ParameterOverrideError, apply_high_level_to_capability,
    translate,
)
from .predicate_engine import (
    DEFAULT_COMPONENTS, CompiledPredicate, DwellTimer, PredicateError,
    compile_predicate,
)


_ROLE_NAME_TO_INT = {
    'neutral': AgentState.ROLE_NEUTRAL,
    'pursuer': AgentState.ROLE_PURSUER,
    'evader': AgentState.ROLE_EVADER,
}

_SCHEMA_VERSION_SUPPORTED = 1

# Auto-tolerance for ready_when when only `initial_position` is
# provided: agent is "ready" when it is within this Euclidean
# distance of the target (and has been for the auto dwell).
_AUTO_TOL_M = 0.5
_AUTO_DWELL_S = 0.0

# Drive gains for the warmup phase. Tuned by feel for stable
# pursuit on rover and steady descent for quad; episodes that
# need different gains can tighten ready_when instead.
_ROVER_K_PSI = 2.0
_ROVER_V_FWD = 1.0
_QUAD_K_POS = 0.7
_QUAD_V_MAX = 2.0


class EpisodeLoadError(RuntimeError):
    pass


# --------------------------------------------------------------------
# Per-agent episode-level config
# --------------------------------------------------------------------


@dataclass
class _AgentConfig:
    role: int = AgentState.ROLE_NEUTRAL
    high_level_overrides: dict[str, float] = field(default_factory=dict)
    fcu_overrides: dict[str, float] = field(default_factory=dict)
    initial_position: Optional[tuple[float, float, float]] = None
    ready_predicate: Optional[CompiledPredicate] = None
    ready_dwell: Optional[DwellTimer] = None


# --------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------


def _yaw_from_quaternion(qw: float, qx: float, qy: float,
                        qz: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _agent_values(state: AgentState) -> dict[str, float]:
    qw = state.pose.orientation.w
    qx = state.pose.orientation.x
    qy = state.pose.orientation.y
    qz = state.pose.orientation.z
    return {
        'x': state.pose.position.x,
        'y': state.pose.position.y,
        'z': state.pose.position.z,
        # Twist is in the body frame per AgentState.msg.
        'vx': state.twist.linear.x,
        'vy': state.twist.linear.y,
        'vz': state.twist.linear.z,
        'psi': _yaw_from_quaternion(qw, qx, qy, qz),
        'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
    }


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _resolve_path(reference: str, base: Path) -> Path:
    """Resolve a manifest reference from an episode YAML.

    `package://<pkg>/<relpath>` looks up `<pkg>`'s install
    `share/` directory via the ament index; everything else is
    treated as a filesystem path, taken absolute as-is or made
    absolute against `base` (the episode YAML's directory).
    """
    if reference.startswith('package://'):
        rest = reference[len('package://'):]
        if '/' not in rest:
            raise EpisodeLoadError(
                f"manifest reference {reference!r}: "
                f"package:// URI must include a path"
            )
        pkg, rel = rest.split('/', 1)
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError as e:
            raise EpisodeLoadError(
                f"manifest reference {reference!r}: package "
                f"{pkg!r} not on the ament index (workspace not "
                f"sourced or package not installed)"
            ) from e
        return (Path(share) / rel).resolve()
    p = Path(reference)
    return p if p.is_absolute() else (base / p).resolve()


def _auto_ready_expr(agent_id: str,
                     initial_position: tuple[float, float, float]
                     ) -> str:
    x, y, z = initial_position
    return (
        f"({agent_id}.x - {x})**2 + "
        f"({agent_id}.y - {y})**2 + "
        f"({agent_id}.z - {z})**2 < {_AUTO_TOL_M ** 2}"
    )


# --------------------------------------------------------------------
# EpisodeRunner
# --------------------------------------------------------------------


class EpisodeRunner(Node):

    def __init__(self) -> None:
        super().__init__('episode_runner')

        self.declare_parameter('episode_path', '')
        self.declare_parameter('tick_rate_hz', 50.0)
        # See the multi-agent pose discussion in module docstring.
        # When set, the runner overrides AgentState.pose with the
        # corresponding world-frame transform; required for any
        # episode that has more than one agent in a shared world.
        self.declare_parameter('world_pose_topic', '')

        episode_path = self.get_parameter(
            'episode_path').get_parameter_value().string_value
        if not episode_path:
            raise EpisodeLoadError(
                'episode_path parameter is required '
                '(YAML file describing the episode)'
            )
        episode_path_abs = Path(episode_path).resolve()
        if not episode_path_abs.is_file():
            raise EpisodeLoadError(
                f'episode_path does not exist: {episode_path_abs}'
            )

        episode = yaml.safe_load(episode_path_abs.read_text())
        self._validate_schema(episode, episode_path_abs)
        self._episode_id: str = episode['name']
        self._duration_s: float = float(episode['duration_s'])
        # Episode-specified tick rate overrides the launch-level
        # default; clamped to a sane range (the FCU's GUIDED
        # inactivity timeout sits at ~3 s, so anything below
        # ~0.5 Hz risks the FCU disarming between commands; the
        # adapter+MAVROS+gz pipeline accumulates latency above
        # ~100 Hz under our 4-MAVROS plugin set, so 200 Hz is
        # the documented hard cap. Use 50 Hz unless you know
        # what you're doing.).
        self._yaml_tick_rate_hz: float | None = (
            float(episode['tick_rate_hz'])
            if 'tick_rate_hz' in episode else None
        )
        if (self._yaml_tick_rate_hz is not None
                and not 0.5 <= self._yaml_tick_rate_hz <= 200.0):
            raise EpisodeLoadError(
                f"{episode_path_abs}: tick_rate_hz must be in "
                f"[0.5, 200] Hz; got "
                f"{self._yaml_tick_rate_hz}"
            )

        manifest_path = _resolve_path(
            episode['manifest'], episode_path_abs.parent)
        if not manifest_path.is_file():
            raise EpisodeLoadError(
                f"manifest referenced by episode "
                f"{episode_path_abs} not found: {manifest_path}"
            )
        manifest = yaml.safe_load(manifest_path.read_text())
        self._agents_manifest: dict[str, dict] = {
            a['id']: a for a in manifest['agents']}
        self._agent_ids: list[str] = list(self._agents_manifest)

        self._configs: dict[str, _AgentConfig] = self._build_agent_configs(
            episode.get('agents') or {})

        self._predicates: list[CompiledPredicate] = []
        self._dwells: dict[str, DwellTimer] = {}
        for entry in episode.get('termination') or []:
            try:
                p = compile_predicate(
                    pred_id=entry['id'],
                    expr_str=entry['expr'],
                    dwell_s=float(entry.get('dwell_s', 0.0)),
                    outcome=entry['outcome'],
                    agent_ids=self._agent_ids,
                    components=DEFAULT_COMPONENTS,
                )
            except (KeyError, ValueError) as e:
                raise EpisodeLoadError(
                    f"failed to compile predicate {entry!r}: {e}"
                ) from e
            self._predicates.append(p)
            self._dwells[p.id] = DwellTimer(p.dwell_s)

        # Service clients per MAVROS namespace touched by the
        # parameters block, plus a flat list of pending writes.
        self._param_clients: dict[str, object] = {}
        self._params_to_set: list[tuple[str, str, str, float]] = []
        for aid, cfg in self._configs.items():
            for name, value in cfg.fcu_overrides.items():
                ns = (
                    f"/mavros_"
                    f"{int(self._agents_manifest[aid]['instance'])}"
                )
                if ns not in self._param_clients:
                    self._param_clients[ns] = self.create_client(
                        ParamSetV2, f'{ns}/param/set')
                self._params_to_set.append((aid, ns, name, value))
        self._param_pending_count: int = len(self._params_to_set)

        latched_qos = QoSProfile(
            depth=max(len(self._agent_ids), 1),
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        event_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._pub_state = self.create_publisher(
            GameState, '/game/state', 10)
        self._pub_roles = self.create_publisher(
            RoleAssignment, '/game/role_assignments', latched_qos)
        self._pub_event = self.create_publisher(
            TerminationEvent, '/game/termination_event', event_qos)

        # Capability resync. The adapter publishes /<id>/capability
        # TRANSIENT_LOCAL at startup with values derived from FCU
        # params *before* any episode-level override. After our
        # param_setup phase the FCU obeys the new params, but the
        # latched Capability message strategies see still reflects
        # the originals. Subscribe once to capture the adapter's
        # message, then republish a patched copy on the same topic
        # so a strategy subscribing later sees the correct envelope.
        cap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._latest_capability: dict[str, Capability] = {}
        self._capability_overrides_published: bool = False
        self._pub_capability: dict[str, object] = {}
        # Subscribe to every agent's Capability — the warmup
        # driver also reads it to scale cruise speed for plane
        # agents. Republish only for agents whose episode YAML
        # carries a `high_level` override (the resync target).
        for aid, cfg in self._configs.items():
            self.create_subscription(
                Capability, f'/{aid}/capability',
                lambda msg, captured_aid=aid:
                self._on_capability(captured_aid, msg),
                cap_qos,
            )
            if cfg.high_level_overrides:
                self._pub_capability[aid] = self.create_publisher(
                    Capability, f'/{aid}/capability', cap_qos)

        # cmd_vel publishers for warmup-driven agents.
        self._pub_cmd: dict[str, object] = {
            aid: self.create_publisher(
                TwistStamped, f'/{aid}/cmd_vel', 10)
            for aid, cfg in self._configs.items()
            if cfg.initial_position is not None
        }

        # AgentState subscriptions.
        self._latest: dict[str, AgentState | None] = {
            aid: None for aid in self._agent_ids}
        for aid in self._agent_ids:
            self.create_subscription(
                AgentState, f'/{aid}/state',
                lambda msg, captured_aid=aid: self._on_agent_state(
                    captured_aid, msg),
                10,
            )

        # World-pose ground-truth (TFMessage from gz dynamic_pose).
        self._world_pose: dict[str, tuple[Point, Quaternion]] = {}
        world_pose_topic = self.get_parameter(
            'world_pose_topic').get_parameter_value().string_value
        if world_pose_topic:
            self.create_subscription(
                TFMessage, world_pose_topic,
                self._on_world_pose, 10,
            )
            self.get_logger().info(
                f"using world-pose topic {world_pose_topic!r} as "
                f"ground-truth pose; AgentState pose will be "
                f"overridden in /game/state"
            )
        self._world_pose_required = bool(world_pose_topic)

        # Phase machine.
        self._phase: str = (
            'param_setup' if self._params_to_set
            else 'warmup' if self._has_warmup_work()
            else 'running'
        )
        self._param_setup_dispatched = False
        self._start_time_s: float | None = None
        self._roles_published: bool = False

        # Resolution order: episode YAML wins over the
        # launch-level ROS parameter, which in turn defaults to
        # 50 Hz. The launch wires the YAML value through to the
        # parameter as well, so the two should agree in normal
        # use; an explicit YAML override is the authoritative
        # source.
        tick_hz = (
            self._yaml_tick_rate_hz
            if self._yaml_tick_rate_hz is not None
            else float(self.get_parameter(
                'tick_rate_hz').get_parameter_value().double_value)
        )
        if not 0.5 <= tick_hz <= 200.0:
            raise EpisodeLoadError(
                f'tick_rate_hz must be in [0.5, 200] Hz, got '
                f'{tick_hz}'
            )
        self._tick_rate_hz = tick_hz
        self.create_timer(1.0 / tick_hz, self._tick)

        self.get_logger().info(
            f"episode {self._episode_id!r} loaded "
            f"(tick {self._tick_rate_hz:g} Hz): "
            f"{len(self._agent_ids)} agents, "
            f"{len(self._predicates)} termination predicates, "
            f"{sum(1 for c in self._configs.values() if c.ready_predicate)} "
            f"ready_when predicates, "
            f"{sum(1 for c in self._configs.values() if c.initial_position)} "
            f"initial_position waypoints, "
            f"duration {self._duration_s} s"
        )

    # ------------------------------------------------------------------
    # YAML parsing
    # ------------------------------------------------------------------

    def _build_agent_configs(self, agents_block: dict
                             ) -> dict[str, _AgentConfig]:
        configs = {aid: _AgentConfig() for aid in self._agent_ids}
        for aid, entry in agents_block.items():
            if aid not in configs:
                raise EpisodeLoadError(
                    f"agents block refers to unknown agent_id "
                    f"{aid!r} (manifest: {self._agent_ids})"
                )
            cfg = configs[aid]
            entry = entry or {}

            role_str = entry.get('role')
            if role_str is not None:
                if role_str not in _ROLE_NAME_TO_INT:
                    raise EpisodeLoadError(
                        f"agent {aid!r}: unknown role "
                        f"{role_str!r}; valid roles: "
                        f"{sorted(_ROLE_NAME_TO_INT)}"
                    )
                cfg.role = _ROLE_NAME_TO_INT[role_str]

            params_block = entry.get('parameters')
            if params_block:
                platform = self._agents_manifest[aid].get('type')
                try:
                    cfg.fcu_overrides = translate(
                        platform=platform,
                        high_level=params_block.get('high_level'),
                        fcu=params_block.get('fcu'),
                    )
                except ParameterOverrideError as e:
                    raise EpisodeLoadError(
                        f"agent {aid!r} parameters block: {e}"
                    ) from e
                # Stash the user-facing high_level dict for the
                # post-param-set Capability re-publish, where we
                # patch the Capability message in the canonical
                # field names rather than the FCU param names.
                cfg.high_level_overrides = dict(
                    params_block.get('high_level') or {})

            ip = entry.get('initial_position')
            if ip is not None:
                if (not isinstance(ip, (list, tuple))
                        or len(ip) != 3):
                    raise EpisodeLoadError(
                        f"agent {aid!r}: initial_position must be "
                        f"a 3-element [x, y, z] list, got {ip!r}"
                    )
                cfg.initial_position = (
                    float(ip[0]), float(ip[1]), float(ip[2]))

            ready_block = entry.get('ready_when')
            if ready_block:
                expr = ready_block.get('expr')
                if not isinstance(expr, str) or not expr.strip():
                    raise EpisodeLoadError(
                        f"agent {aid!r} ready_when: expr must be a "
                        f"non-empty string"
                    )
                dwell_s = float(ready_block.get('dwell_s', 0.0))
                try:
                    cfg.ready_predicate = compile_predicate(
                        pred_id=f'{aid}.ready',
                        expr_str=expr,
                        dwell_s=dwell_s,
                        outcome='ready',
                        agent_ids=self._agent_ids,
                        components=DEFAULT_COMPONENTS,
                    )
                except (PredicateError, ValueError) as e:
                    raise EpisodeLoadError(
                        f"agent {aid!r} ready_when: {e}"
                    ) from e
                cfg.ready_dwell = DwellTimer(dwell_s)
            elif cfg.initial_position is not None:
                # No explicit ready_when but a target was given;
                # synthesise a tolerance check around the target.
                expr = _auto_ready_expr(aid, cfg.initial_position)
                cfg.ready_predicate = compile_predicate(
                    pred_id=f'{aid}.ready',
                    expr_str=expr,
                    dwell_s=_AUTO_DWELL_S,
                    outcome='ready',
                    agent_ids=self._agent_ids,
                    components=DEFAULT_COMPONENTS,
                )
                cfg.ready_dwell = DwellTimer(_AUTO_DWELL_S)
        return configs

    def _has_warmup_work(self) -> bool:
        return any(c.ready_predicate is not None
                   or c.initial_position is not None
                   for c in self._configs.values())

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_agent_state(self, agent_id: str,
                        msg: AgentState) -> None:
        self._latest[agent_id] = msg

    def _on_capability(self, agent_id: str,
                       msg: Capability) -> None:
        # Only ever store the *first* message per agent — later
        # ones may be our own resync echo (TRANSIENT_LOCAL means
        # we receive what we publish). Single-shot is enough since
        # the adapter publishes once at startup.
        self._latest_capability.setdefault(agent_id, msg)

    def _on_world_pose(self, msg: TFMessage) -> None:
        # The custom `rufus_sim_game/world_pose_bridge` fills
        # `child_frame_id` with the gz pose `name` field, so we
        # match each agent_id to the transform whose
        # child_frame_id equals it. Transforms for child links
        # (iris_with_standoffs, base_link, rotors, ...) come
        # through with their own names and are silently
        # ignored.
        for tf in msg.transforms:
            aid = tf.child_frame_id
            if aid not in self._latest:
                continue
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            position = Point(x=translation.x, y=translation.y,
                             z=translation.z)
            self._world_pose[aid] = (position, rotation)

    # ------------------------------------------------------------------
    # Phase: param_setup
    # ------------------------------------------------------------------

    def _try_dispatch_param_setup(self) -> None:
        if self._param_setup_dispatched:
            return
        if any(self._latest[aid] is None
               for aid in self._agent_ids):
            return
        if not all(c.service_is_ready()
                   for c in self._param_clients.values()):
            return
        self._param_setup_dispatched = True
        self.get_logger().info(
            f'dispatching {len(self._params_to_set)} parameter '
            f'override(s) across '
            f'{len(self._param_clients)} mavros namespace(s)'
        )
        for aid, ns, name, value in self._params_to_set:
            req = ParamSetV2.Request()
            req.force_set = True
            req.param_id = name
            req.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            )
            fut = self._param_clients[ns].call_async(req)
            fut.add_done_callback(
                lambda f, a=aid, n=name, v=value:
                self._on_param_set_done(a, n, v, f)
            )

    def _on_param_set_done(self, agent_id: str, name: str,
                           value: float, fut) -> None:
        try:
            res = fut.result()
            if res.success:
                self.get_logger().info(
                    f"param set {agent_id} {name}={value}: ok"
                )
            else:
                self.get_logger().error(
                    f"param set {agent_id} {name}={value}: "
                    f"FCU rejected (success=false)"
                )
        except Exception as e:
            self.get_logger().error(
                f"param set {agent_id} {name}={value} "
                f"exception: {e}"
            )
        self._param_pending_count -= 1
        if self._param_pending_count == 0:
            self.get_logger().info('all parameter overrides applied')
            self._phase = (
                'warmup' if self._has_warmup_work() else 'running'
            )
            # Publish patched Capability messages now that the FCU
            # has accepted the overrides; the adapter's earlier
            # latched Capability would otherwise leave strategies
            # with stale envelope values.
            self._try_publish_capability_overrides()

    def _try_publish_capability_overrides(self) -> None:
        if self._capability_overrides_published:
            return
        if self._phase == 'param_setup':
            return  # not yet safe to publish
        for aid, cfg in self._configs.items():
            if not cfg.high_level_overrides:
                continue
            if aid not in self._latest_capability:
                # Wait until the adapter has published its baseline
                # Capability; we patch in place rather than
                # synthesise from scratch so unrelated fields
                # (saturation envelope, source string, ...) carry
                # over.
                return
        for aid, cfg in self._configs.items():
            if not cfg.high_level_overrides:
                continue
            patched = Capability()
            patched.header = self._latest_capability[aid].header
            patched.agent_id = self._latest_capability[aid].agent_id
            patched.platform = self._latest_capability[aid].platform
            patched.v_max = self._latest_capability[aid].v_max
            patched.v_min = self._latest_capability[aid].v_min
            patched.vz_max_up = self._latest_capability[aid].vz_max_up
            patched.vz_max_down = (
                self._latest_capability[aid].vz_max_down)
            patched.yaw_rate_max = (
                self._latest_capability[aid].yaw_rate_max)
            patched.lateral_accel_max = (
                self._latest_capability[aid].lateral_accel_max)
            patched.bank_angle_max = (
                self._latest_capability[aid].bank_angle_max)
            patched.climb_angle_max = (
                self._latest_capability[aid].climb_angle_max)
            # Carry the adapter-read native TURN_RADIUS through:
            # apply_high_level_to_capability uses it as the
            # widen-only floor, and it must survive when no
            # min_turn_radius override is given.
            patched.min_turn_radius = (
                self._latest_capability[aid].min_turn_radius)
            patched.source = (
                self._latest_capability[aid].source
                + f'; episode_override={cfg.high_level_overrides}'
            )
            platform = self._agents_manifest[aid].get('type')
            try:
                apply_high_level_to_capability(
                    platform, cfg.high_level_overrides, patched)
            except ParameterOverrideError as e:
                self.get_logger().error(
                    f"capability resync for {aid!r} failed: {e}"
                )
                continue
            patched.header.stamp = self.get_clock().now().to_msg()
            self._pub_capability[aid].publish(patched)
            self.get_logger().info(
                f"published Capability override for {aid!r}: "
                f"{cfg.high_level_overrides}"
            )
        self._capability_overrides_published = True

    # ------------------------------------------------------------------
    # Phase: warmup
    # ------------------------------------------------------------------

    def _warmup_tick(self, now,
                     resolved: dict[str, AgentState],
                     values: dict[str, dict[str, float]]) -> bool:
        """Drive each agent toward its initial position and check
        ready_when. Returns True iff every agent's ready_when has
        held for its dwell — that is the trigger to transition to
        `running`.
        """
        all_ready = True
        now_s = now.nanoseconds * 1e-9
        for aid, cfg in self._configs.items():
            agent_ready = True
            if cfg.ready_predicate is not None:
                try:
                    satisfied = cfg.ready_predicate.evaluate(values)
                except Exception:
                    satisfied = False
                held, _ = cfg.ready_dwell.update(now_s, satisfied)
                if not held:
                    agent_ready = False
            # Drive only while the agent is not yet ready. Planes
            # can't hover, so once an agent's ready_when has
            # latched we stop publishing cmd_vel and let the
            # platform's adapter fallback (cruise/loiter for
            # plane, hover for quad, idle for rover) hold it.
            if (cfg.initial_position is not None
                    and not agent_ready):
                self._publish_warmup_cmd(aid, resolved[aid])
            if not agent_ready:
                all_ready = False
        return all_ready

    def _publish_warmup_cmd(self, agent_id: str,
                            state: AgentState) -> None:
        target = self._configs[agent_id].initial_position
        platform = self._agents_manifest[agent_id].get('type')
        if platform == 'rover':
            twist = self._rover_pursuit_cmd(state, target)
            frame = 'base_link'
        elif platform == 'quad':
            twist = self._quad_pursuit_cmd(state, target)
            frame = 'base_link'
        elif platform == 'plane':
            twist = self._plane_pursuit_cmd(
                state, target,
                self._latest_capability.get(agent_id))
            # fixed_wing_adapter consumes cmd_vel as world-frame
            # ENU velocity (per docs/control.md fixed-wing
            # specifics), so the frame_id signals world frame.
            frame = 'map'
        else:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist = twist
        self._pub_cmd[agent_id].publish(msg)

    @staticmethod
    def _rover_pursuit_cmd(state: AgentState,
                           target: tuple[float, float, float]):
        dx = target[0] - state.pose.position.x
        dy = target[1] - state.pose.position.y
        bearing = math.atan2(dy, dx)
        psi = _yaw_from_quaternion(
            state.pose.orientation.w,
            state.pose.orientation.x,
            state.pose.orientation.y,
            state.pose.orientation.z,
        )
        err = _wrap_pi(bearing - psi)
        out = Twist()
        # Slow forward when off-axis so the rover spins in place
        # rather than tracing big arcs.
        out.linear.x = _ROVER_V_FWD * max(0.0, math.cos(err))
        out.angular.z = max(-2.0, min(2.0, _ROVER_K_PSI * err))
        return out

    @staticmethod
    def _plane_pursuit_cmd(state: AgentState,
                           target: tuple[float, float, float],
                           capability: Optional[Capability]):
        # Fixed-wing cmd_vel is world-ENU velocity (per
        # docs/control.md). Drive a unit vector toward the target
        # at a safe cruise speed; fixed_wing_adapter projects
        # this onto airspeed + heading + climb via the Dubins-
        # airplane mapping. Once we stop publishing (i.e., once
        # ready_when fires), the adapter's cruise fallback takes
        # over and the plane loiters near the waypoint.
        dx = target[0] - state.pose.position.x
        dy = target[1] - state.pose.position.y
        dz = target[2] - state.pose.position.z
        # Pick a target speed inside the airspeed envelope. The
        # captured Capability gives us [v_min, v_max]; pursue at
        # 70% of v_max so the adapter still has headroom to
        # slow/turn. Without a captured Capability (sub timing,
        # missing publisher), fall back to the documented zephyr
        # cruise (12 m/s) — that's what fixed_wing_adapter would
        # use for its own cruise fallback anyway.
        if capability is not None and capability.v_max > 0.0:
            v_target = max(capability.v_min, 0.7 * capability.v_max)
        else:
            v_target = 12.0
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        out = Twist()
        if norm < 1e-6:
            # At target: stop driving; adapter will cruise-fallback.
            return out
        out.linear.x = v_target * dx / norm
        out.linear.y = v_target * dy / norm
        # Cap climb component independently so the plane doesn't
        # try to point straight up if the target is well above.
        max_climb = 4.0
        out.linear.z = max(-max_climb, min(max_climb,
                                           v_target * dz / norm))
        return out

    @staticmethod
    def _quad_pursuit_cmd(state: AgentState,
                          target: tuple[float, float, float]):
        # Body-frame velocity command; see AgentState.msg twist
        # contract. Compute world-frame error then rotate into
        # body frame using current yaw.
        dx = target[0] - state.pose.position.x
        dy = target[1] - state.pose.position.y
        dz = target[2] - state.pose.position.z
        psi = _yaw_from_quaternion(
            state.pose.orientation.w,
            state.pose.orientation.x,
            state.pose.orientation.y,
            state.pose.orientation.z,
        )
        # World-frame velocity: proportional to position error,
        # capped at the warmup speed. Z is independent of yaw.
        vx_w = _QUAD_K_POS * dx
        vy_w = _QUAD_K_POS * dy
        norm = math.hypot(vx_w, vy_w)
        if norm > _QUAD_V_MAX:
            vx_w *= _QUAD_V_MAX / norm
            vy_w *= _QUAD_V_MAX / norm
        # World -> body rotation about z.
        c = math.cos(-psi)
        s = math.sin(-psi)
        out = Twist()
        out.linear.x = c * vx_w - s * vy_w
        out.linear.y = s * vx_w + c * vy_w
        out.linear.z = max(-_QUAD_V_MAX,
                           min(_QUAD_V_MAX, _QUAD_K_POS * dz))
        return out

    def _publish_role_assignments(self) -> None:
        if self._roles_published:
            return
        for aid, cfg in self._configs.items():
            ra = RoleAssignment()
            ra.agent_id = aid
            ra.role = cfg.role
            ra.team_id = 0
            self._pub_roles.publish(ra)
        self._roles_published = True
        self.get_logger().info(
            'all agents ready; published role_assignments and '
            'starting episode clock'
        )

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._phase == 'terminated':
            return
        if any(self._latest[aid] is None
               for aid in self._agent_ids):
            return
        if self._world_pose_required and any(
                aid not in self._world_pose
                for aid in self._agent_ids):
            return

        now = self.get_clock().now()

        if self._phase == 'param_setup':
            self._try_dispatch_param_setup()
            # Still publish a pre-warmup GameState so consumers
            # know the runner is alive.
            resolved = {aid: self._resolved_state(aid)
                        for aid in self._agent_ids}
            self._publish_game_state(now, 0.0, [], resolved)
            return

        # Capability resync may need a few ticks to fire (the
        # adapter's TRANSIENT_LOCAL message may not be in our
        # subscription queue at the moment the param_setup phase
        # ends). Idempotent; cheap to call every tick until done.
        self._try_publish_capability_overrides()

        resolved = {aid: self._resolved_state(aid)
                    for aid in self._agent_ids}
        values = {aid: _agent_values(state)
                  for aid, state in resolved.items()}

        if self._phase == 'warmup':
            ready = self._warmup_tick(now, resolved, values)
            self._publish_game_state(now, 0.0, [], resolved)
            if ready:
                self._publish_role_assignments()
                self._start_time_s = now.nanoseconds * 1e-9
                self._phase = 'running'
            return

        # phase == 'running'
        if self._start_time_s is None:
            # No warmup work, no params; this is the first running tick.
            self._publish_role_assignments()
            self._start_time_s = now.nanoseconds * 1e-9
        elapsed_s = now.nanoseconds * 1e-9 - self._start_time_s

        active: list[str] = []
        fired: CompiledPredicate | None = None
        for predicate in self._predicates:
            try:
                satisfied = predicate.evaluate(values)
            except Exception as e:
                self.get_logger().warning(
                    f"predicate {predicate.id!r} eval failed: {e}"
                )
                satisfied = False
            now_s = now.nanoseconds * 1e-9
            held, just_fired = self._dwells[predicate.id].update(
                now_s, satisfied)
            if held:
                active.append(predicate.id)
            if just_fired and fired is None:
                fired = predicate

        self._publish_game_state(now, elapsed_s, active, resolved)

        if fired is not None:
            self._terminate(now, elapsed_s, fired.id, fired.outcome,
                            resolved)
        elif elapsed_s >= self._duration_s:
            self._terminate(now, elapsed_s, '',
                            TerminationEvent.OUTCOME_TIMEOUT,
                            resolved)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_game_state(self, now, elapsed_s: float,
                            active: list[str],
                            resolved: dict[str, AgentState]) -> None:
        gs = GameState()
        gs.header.stamp = now.to_msg()
        gs.header.frame_id = 'map'
        gs.episode_id = self._episode_id
        gs.sim_time = self._duration_msg(elapsed_s)
        gs.agents = [resolved[aid] for aid in self._agent_ids]
        gs.active_predicates = active
        self._pub_state.publish(gs)

    def _terminate(self, now, elapsed_s: float,
                   predicate_id: str, outcome: str,
                   resolved: dict[str, AgentState]) -> None:
        ev = TerminationEvent()
        ev.header.stamp = now.to_msg()
        ev.header.frame_id = 'map'
        ev.episode_id = self._episode_id
        ev.sim_time = self._duration_msg(elapsed_s)
        ev.predicate_id = predicate_id
        ev.outcome = outcome
        ev.terminal_state.header.stamp = now.to_msg()
        ev.terminal_state.header.frame_id = 'map'
        ev.terminal_state.episode_id = self._episode_id
        ev.terminal_state.sim_time = self._duration_msg(elapsed_s)
        ev.terminal_state.agents = [
            resolved[aid] for aid in self._agent_ids]
        ev.terminal_state.active_predicates = (
            [predicate_id] if predicate_id else [])
        self._pub_event.publish(ev)
        self._phase = 'terminated'
        self.get_logger().info(
            f"episode {self._episode_id!r} terminated at "
            f"sim_t={elapsed_s:.3f} s: "
            f"predicate_id={predicate_id!r}, outcome={outcome!r}"
        )

    def _resolved_state(self, agent_id: str) -> AgentState:
        latest = self._latest[agent_id]
        if not self._world_pose_required:
            return latest
        position, rotation = self._world_pose[agent_id]
        merged = AgentState()
        merged.header = latest.header
        merged.agent_id = latest.agent_id or agent_id
        merged.role = latest.role
        merged.platform = latest.platform
        merged.pose.position = position
        merged.pose.orientation = rotation
        merged.twist = latest.twist
        merged.saturation = latest.saturation
        return merged

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_msg(elapsed_s: float) -> DurationMsg:
        secs = int(elapsed_s)
        nsecs = int(round((elapsed_s - secs) * 1e9))
        if nsecs >= 1_000_000_000:
            secs += 1
            nsecs -= 1_000_000_000
        out = DurationMsg()
        out.sec = secs
        out.nanosec = nsecs
        return out

    @staticmethod
    def _validate_schema(episode: dict, source: Path) -> None:
        if not isinstance(episode, dict):
            raise EpisodeLoadError(
                f"{source}: top-level YAML must be a mapping")
        version = episode.get('schema_version')
        if version != _SCHEMA_VERSION_SUPPORTED:
            raise EpisodeLoadError(
                f"{source}: schema_version must be "
                f"{_SCHEMA_VERSION_SUPPORTED} "
                f"(got {version!r})"
            )
        for key in ('name', 'manifest', 'duration_s'):
            if key not in episode:
                raise EpisodeLoadError(
                    f"{source}: missing required key {key!r}"
                )
        # Reject the legacy top-level keys so a half-migrated YAML
        # fails fast rather than silently ignoring intent.
        for legacy in ('roles', 'parameters'):
            if legacy in episode:
                raise EpisodeLoadError(
                    f"{source}: top-level {legacy!r} is no longer "
                    f"supported; move into the per-agent "
                    f"`agents:` block"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = EpisodeRunner()
    except (EpisodeLoadError, PredicateError) as e:
        rclpy.logging.get_logger('episode_runner').error(str(e))
        rclpy.shutdown()
        raise SystemExit(2) from e
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
