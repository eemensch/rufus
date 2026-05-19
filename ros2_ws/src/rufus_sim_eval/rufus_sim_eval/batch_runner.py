"""Batch runner — drives one sweep YAML through the chain.

Per-run lifecycle (sequential):

  1. Materialise the per-run episode YAML to disk
     (under <output_dir>/runs/run_NNNN/episode.yaml).
  2. Bring up the chain (gz + SITL × N + multi_agent_sim) using
     `chain.bring_up`.
  3. Wait until every agent reaches its platform-specific ready
     state via `chain.wait_ready`.
  4. Spawn `episode_with_strategies` against the per-run YAML
     via `chain.run_episode`.
  5. Subscribe to `/game/termination_event` (TRANSIENT_LOCAL)
     and `/game/state` (volatile) with this process's own rclpy
     node; spin until the TerminationEvent arrives or a wallclock
     deadline (`duration_s + grace`) is reached.
  6. Tear the whole chain down via `chain.tear_down`.
  7. Append a CSV row with axis values, outcome, sim_time, and
     terminal positions per agent.

Shared `summary.csv` lives at `<output_dir>/summary.csv`. The CSV
is flushed after every row so you can monitor progress live.

The runner sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for the
chain processes to keep DDS discovery on shm transport — the
multi-host case uses SUBNET, which has been spam-prone in
sessions with active VPNs/multicast filtering.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
    QoSReliabilityPolicy,
)

from ament_index_python.packages import get_package_share_directory
from rufus_sim_msgs.msg import GameState, TerminationEvent

from .chain import bring_up, run_episode, tear_down, wait_ready
from .sweep import (
    SweepLoadError, SweepRun, SweepSpec, enumerate_runs, load_sweep,
)


READY_TIMEOUT_S = 240.0    # generous; quad/plane takeoff is slow
TERMINATION_GRACE_S = 30.0


def _resolve_manifest(episode_yaml: dict, episode_dir: Path) -> Path:
    ref = episode_yaml['manifest']
    if ref.startswith('package://'):
        rest = ref[len('package://'):]
        pkg, rel = rest.split('/', 1)
        share = get_package_share_directory(pkg)
        return Path(share) / rel
    p = Path(ref)
    return p if p.is_absolute() else (episode_dir / p).resolve()


class _RunCollector(Node):
    """Subscribes to /game/termination_event and /game/state for
    one episode; exposes the captured fields after spin."""

    def __init__(self):
        super().__init__('batch_collector')
        self._termination: TerminationEvent | None = None
        self._latest_state: GameState | None = None

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            TerminationEvent, '/game/termination_event',
            self._on_termination, latched,
        )
        self.create_subscription(
            GameState, '/game/state', self._on_state, 10,
        )

    def _on_termination(self, msg: TerminationEvent) -> None:
        if self._termination is None:
            self._termination = msg

    def _on_state(self, msg: GameState) -> None:
        self._latest_state = msg

    @property
    def termination(self) -> TerminationEvent | None:
        return self._termination

    @property
    def latest_state(self) -> GameState | None:
        return self._latest_state


def _spin_until_termination(
        timeout_s: float) -> tuple[TerminationEvent | None,
                                   GameState | None]:
    """Spin a fresh rclpy context for one run; returns the
    TerminationEvent (or None on timeout) plus the most recent
    GameState seen.
    """
    rclpy.init()
    node = _RunCollector()
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.termination is not None:
                # Drain a few more cycles so the latest GameState
                # represents the moment of termination, not one
                # tick before.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.1)
                break
        return node.termination, node.latest_state
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _agent_position(state: GameState | None,
                    agent_id: str) -> tuple[float, float, float]:
    if state is None:
        return float('nan'), float('nan'), float('nan')
    for a in state.agents:
        if a.agent_id == agent_id:
            p = a.pose.position
            return p.x, p.y, p.z
    return float('nan'), float('nan'), float('nan')


def _csv_header(spec: SweepSpec) -> list[str]:
    cols = ['run_id', 'seed', 'wallclock_s']
    cols += [a.parameter for a in spec.axes]
    cols += ['outcome', 'predicate_id', 'sim_time_s']
    for agent in _manifest_agents(spec):
        cols += [f'{agent}_x', f'{agent}_y', f'{agent}_z']
    return cols


def _manifest_agents(spec: SweepSpec) -> list[str]:
    """Read the manifest off the base episode and return agent
    ids in declared order."""
    ref = spec.base_episode['manifest']
    if ref.startswith('package://'):
        pkg, rel = ref[len('package://'):].split('/', 1)
        share = get_package_share_directory(pkg)
        manifest_path = Path(share) / rel
    else:
        p = Path(ref)
        manifest_path = (p if p.is_absolute()
                         else (spec.episode_path.parent / p))
    manifest = yaml.safe_load(Path(manifest_path).read_text())
    return [a['id'] for a in manifest['agents']]


def _row_for_run(spec: SweepSpec, run: SweepRun,
                 wallclock_s: float,
                 termination: TerminationEvent | None,
                 final_state: GameState | None) -> list:
    cols = [run.run_id, run.seed, f'{wallclock_s:.2f}']
    for axis in spec.axes:
        cols.append(run.axis_values.get(axis.parameter, ''))
    if termination is not None:
        sim_time = (termination.sim_time.sec
                    + termination.sim_time.nanosec * 1e-9)
        cols += [termination.outcome,
                 termination.predicate_id,
                 f'{sim_time:.3f}']
    else:
        cols += ['__no_termination__', '', '']
    for aid in _manifest_agents(spec):
        x, y, z = _agent_position(final_state, aid)
        cols += [f'{x:.4f}', f'{y:.4f}', f'{z:.4f}']
    return cols


def _run_one(spec: SweepSpec, run: SweepRun, env: dict) -> dict:
    run_dir = spec.output_dir / 'runs' / f'run_{run.run_id:04d}'
    run_dir.mkdir(parents=True, exist_ok=True)
    episode_path = run_dir / 'episode.yaml'
    episode_path.write_text(yaml.safe_dump(run.episode_yaml,
                                           default_flow_style=False))
    manifest_path = _resolve_manifest(run.episode_yaml,
                                      episode_path.parent)
    manifest = yaml.safe_load(manifest_path.read_text())
    world_name = manifest['world_name']

    print(f"[run {run.run_id:04d}] axis={run.axis_values} "
          f"seed={run.seed} world={world_name} "
          f"speedup={spec.speedup}", flush=True)

    t0 = time.monotonic()
    handle = bring_up(world_name=world_name, manifest=manifest,
                      manifest_path=manifest_path,
                      work_dir=run_dir, speedup=spec.speedup,
                      env=env)
    try:
        ok = wait_ready(handle, timeout_s=READY_TIMEOUT_S)
        if not ok:
            print(f"[run {run.run_id:04d}] READY timeout; "
                  f"recording as no_termination", flush=True)
            wallclock = time.monotonic() - t0
            return {'wallclock': wallclock,
                    'termination': None,
                    'final_state': None}
        run_episode(handle, episode_path,
                    bag_dir=run_dir / 'bag', env=env)
        duration_s = float(run.episode_yaml.get('duration_s', 60.0))
        # The world SDF's <real_time_factor> is fixed at 1.0 by
        # the generator, so gz steps at wallclock pace and the
        # SITL --speedup flag has no effect under lock-step
        # (SITL waits for gz). The wallclock budget therefore
        # tracks duration_s 1:1 plus warmup + grace. Honouring
        # `--speedup > 1` is a Stage 8 follow-up that needs the
        # world template parameterised by RTF.
        wallclock_budget = duration_s + TERMINATION_GRACE_S
        termination, final_state = _spin_until_termination(
            wallclock_budget)
        wallclock = time.monotonic() - t0
        return {'wallclock': wallclock,
                'termination': termination,
                'final_state': final_state}
    finally:
        tear_down(handle)


def _build_env(spec: SweepSpec) -> dict:
    """Environment for chain processes. Inherits the parent
    process env and adds the gz model paths and the
    LOCALHOST DDS discovery range."""
    project_root = Path(get_package_share_directory(
        'rufus_sim_eval')).parents[4]
    env = dict(os.environ)
    env.setdefault('GZ_SIM_SYSTEM_PLUGIN_PATH',
                   str(project_root / 'external/ardupilot_gazebo/build'))
    extra = ':'.join([
        str(project_root / 'external/ardupilot_gazebo/models'),
        str(project_root / 'external/ardupilot_gazebo/worlds'),
        str(project_root / 'external/SITL_Models/Gazebo/models'),
        str(project_root / 'external/SITL_Models/Gazebo/worlds'),
    ])
    env['GZ_SIM_RESOURCE_PATH'] = (
        f"{env.get('GZ_SIM_RESOURCE_PATH', '')}:{extra}".strip(':'))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return env


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sweep_path', type=Path,
                        help='Sweep YAML file.')
    parser.add_argument('--limit', type=int, default=None,
                        help='Run at most this many runs '
                             '(useful for smoke).')
    cli = parser.parse_args(args)

    try:
        spec = load_sweep(cli.sweep_path)
    except SweepLoadError as e:
        print(f'sweep load error: {e}', file=sys.stderr)
        return 2

    spec.output_dir.mkdir(parents=True, exist_ok=True)
    runs = enumerate_runs(spec)
    if cli.limit is not None:
        runs = runs[:cli.limit]
    print(f"sweep {spec.name!r}: {len(runs)} runs into "
          f"{spec.output_dir}", flush=True)

    env = _build_env(spec)
    summary_path = spec.output_dir / 'summary.csv'
    new_file = not summary_path.exists()
    with open(summary_path, 'a', newline='') as csv_f:
        writer = csv.writer(csv_f)
        if new_file:
            writer.writerow(_csv_header(spec))
            csv_f.flush()
        for run in runs:
            result = _run_one(spec, run, env=env)
            row = _row_for_run(
                spec, run,
                wallclock_s=result['wallclock'],
                termination=result['termination'],
                final_state=result['final_state'])
            writer.writerow(row)
            csv_f.flush()
            term = result['termination']
            if term is None:
                outcome = '__no_termination__'
                sim_t_str = '-'
            else:
                outcome = term.outcome
                sim_t = (term.sim_time.sec
                         + term.sim_time.nanosec * 1e-9)
                sim_t_str = f'{sim_t:.2f}'
            print(f"[run {run.run_id:04d}] done in "
                  f"{result['wallclock']:.1f}s wallclock; "
                  f"outcome={outcome}, sim_t={sim_t_str}s",
                  flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
