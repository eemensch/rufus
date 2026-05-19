"""Chain-bringup helper for the batch runner.

Encapsulates the ugly subprocess plumbing of one full
gz + SITL + MAVROS + adapter + episode chain. The batch runner
calls `bring_up`, `wait_ready`, `run_episode`, `tear_down` per
run so the main loop stays readable.

The same recipe lives in operations.md and the per-stage smoke
scripts; this module is just the Python wrapper around those
exact invocations.
"""

from __future__ import annotations

import os
import re
import shlex
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory


# Per-platform SITL binary + default-params file. Keep aligned
# with operations.md. The sysid_<n>.parm file is appended on
# top via `--defaults`.
# The bringup launch log looks like
#   `[rover_adapter-3] [INFO] [...] [rover_adapter_R0]: armed; READY`
# so we anchor the agent id between `_` and `]:` to avoid a
# greedy `\S+` swallowing the closing bracket.
_PLATFORM_SITL = {
    'rover': {
        'binary': 'ardurover',
        'defaults': (
            'Tools/autotest/default_params/rover.parm',
            'Tools/autotest/default_params/rover-skid.parm',
        ),
        'extra_defaults_pkg': (
            'SITL_Models', 'Gazebo/config/r1_rover.param',
        ),
        # Project-owned override applied after r1_rover.param.
        # See ros2_ws/src/rufus_sim_bringup/config/r1_rover_tune.parm.
        'extra_defaults_local': (
            'rufus_sim_bringup', 'config/r1_rover_tune.parm',
        ),
        'ready_pattern': (
            r'\[rover_adapter_(\S+?)\]:.*armed; READY'
        ),
    },
    'quad': {
        'binary': 'arducopter',
        'defaults': (
            'Tools/autotest/default_params/copter.parm',
        ),
        'extra_defaults_pkg': None,
        'extra_defaults_local': None,
        'ready_pattern': (
            r'\[quad_adapter_(\S+?)\]:.*altitude reached'
        ),
    },
    'plane': {
        'binary': 'arduplane',
        'defaults': (
            'Tools/autotest/default_params/gazebo-zephyr.parm',
        ),
        'extra_defaults_pkg': None,
        'extra_defaults_local': None,
        'ready_pattern': (
            r'\[fixed_wing_adapter_(\S+?)\]:.*altitude reached'
        ),
    },
}


@dataclass
class ChainHandle:
    work_dir: Path
    world_name: str
    manifest: dict
    project_root: Path
    bringup_log: Path
    speedup: float = 1.0
    procs: list = field(default_factory=list)


def _project_root() -> Path:
    # External SITL trees and SITL_Models live at the project
    # root, not inside the ROS workspace.
    pkg_share = Path(get_package_share_directory('rufus_sim_eval'))
    # pkg_share = <project>/ros2_ws/install/rufus_sim_eval/share/rufus_sim_eval
    # parents[0] = .../share
    # parents[1] = .../rufus_sim_eval (install dir)
    # parents[2] = .../install
    # parents[3] = .../ros2_ws
    # parents[4] = <project>
    return pkg_share.parents[4]


def _spawn(cmd, log_path: Path, env=None) -> subprocess.Popen:
    """Spawn a process with stdout+stderr redirected to log_path,
    in its own process group so we can SIGKILL the group cleanly
    later."""
    log_f = open(log_path, 'a')
    return subprocess.Popen(
        cmd,
        stdout=log_f, stderr=subprocess.STDOUT,
        env=env, preexec_fn=os.setsid,
    )


def bring_up(world_name: str, manifest: dict,
             manifest_path: Path, work_dir: Path,
             speedup: float = 1.0,
             env: Optional[dict] = None) -> ChainHandle:
    """Bring up gz + SITL_per_agent + multi_agent_sim. Does not
    block on readiness — call `wait_ready` next.
    """
    work_dir.mkdir(parents=True, exist_ok=True)
    project_root = _project_root()
    bringup_log = work_dir / 'bringup.log'
    bringup_log.write_text('')   # truncate

    procs = []

    # gz sim
    world_path = (Path(get_package_share_directory('rufus_sim_worlds'))
                  / 'worlds' / f'{world_name}.sdf')
    gz_cmd = [
        'gz', 'sim', '-v', '2', '-r', '-s',
        '--headless-rendering', str(world_path),
    ]
    procs.append(_spawn(gz_cmd, work_dir / 'gz.log', env=env))
    time.sleep(5)   # gz needs a moment before SITL can connect

    # SITL per agent
    for agent in manifest['agents']:
        platform = agent['type']
        instance = int(agent['instance'])
        cfg = _PLATFORM_SITL[platform]
        binary = (project_root
                  / 'external/ardupilot/build/sitl/bin'
                  / cfg['binary'])
        defaults = [str(project_root / 'external/ardupilot' / d)
                    for d in cfg['defaults']]
        if cfg['extra_defaults_pkg']:
            pkg, rel = cfg['extra_defaults_pkg']
            defaults.append(str(project_root / 'external' / pkg / rel))
        if cfg['extra_defaults_local']:
            pkg, rel = cfg['extra_defaults_local']
            local_parm = (
                Path(get_package_share_directory(pkg)) / rel
            )
            defaults.append(str(local_parm))
        sysid_parm = (Path(get_package_share_directory(
            'rufus_sim_bringup'))
            / 'config' / 'sysid_overrides'
            / f'sysid_{instance + 1}.parm')
        defaults.append(str(sysid_parm))
        sitl_cwd = work_dir / f"sitl_{agent['id']}"
        sitl_cwd.mkdir(exist_ok=True)
        sitl_cmd = [
            str(binary), '-w', '--model', 'JSON',
            '--speedup', str(speedup),
            '--slave', '0',
            '--sim-address=127.0.0.1',
            f'-I{instance}',
            '--defaults', ','.join(defaults),
        ]
        procs.append(subprocess.Popen(
            sitl_cmd, cwd=sitl_cwd,
            stdout=open(work_dir / f"sitl_{agent['id']}.log", 'w'),
            stderr=subprocess.STDOUT,
            env=env, preexec_fn=os.setsid,
        ))
    time.sleep(8)

    # multi_agent_sim launch — uses the actual manifest path
    # threaded in by the caller. The manifest filename is not
    # always `<world_name>.yaml` (e.g. two_rovers.yaml carries
    # world_name: two_rovers_minimal), so reconstructing the
    # path from world_name alone would miss many manifests.
    bringup_cmd = [
        'ros2', 'launch', 'rufus_sim_bringup',
        'multi_agent_sim.launch.py',
        f'manifest:={manifest_path}',
    ]
    procs.append(_spawn(bringup_cmd, bringup_log, env=env))

    return ChainHandle(
        work_dir=work_dir,
        world_name=world_name,
        manifest=manifest,
        project_root=project_root,
        bringup_log=bringup_log,
        speedup=speedup,
        procs=procs,
    )


def wait_ready(handle: ChainHandle, timeout_s: float) -> bool:
    """Block until every agent in the manifest reports its
    platform-specific ready pattern in the bringup log, or
    timeout. Returns True iff all agents reached ready."""
    needed = {a['id']: _PLATFORM_SITL[a['type']]['ready_pattern']
              for a in handle.manifest['agents']}
    seen: set[str] = set()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            text = handle.bringup_log.read_text()
        except FileNotFoundError:
            text = ''
        for aid, pat in needed.items():
            if aid in seen:
                continue
            for m in re.finditer(pat, text):
                if m.group(1) == aid:
                    seen.add(aid)
                    break
        if len(seen) == len(needed):
            return True
        time.sleep(2.0)
    return False


def run_episode(handle: ChainHandle, episode_path: Path,
                bag_dir: Optional[Path] = None,
                env: Optional[dict] = None
                ) -> subprocess.Popen:
    """Spawn `episode_with_strategies` for one run. Returns the
    Popen so the caller can wait on /game/termination_event and
    then tear it down."""
    cmd = [
        'ros2', 'launch', 'rufus_sim_strategies',
        'episode_with_strategies.launch.py',
        f'episode_path:={episode_path}',
    ]
    if bag_dir is not None:
        cmd += [f'bag_dir:={bag_dir}', 'record_bag:=true']
    else:
        cmd += ['record_bag:=false']
    log = handle.work_dir / 'episode.log'
    proc = _spawn(cmd, log, env=env)
    handle.procs.append(proc)
    return proc


def tear_down(handle: ChainHandle) -> None:
    # Kill the per-process group so launch's children die too.
    for p in handle.procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    # Also nuke any straggling chain processes by name in case
    # the launch trees daemonised something.
    subprocess.run(
        ['pkill', '-9', '-f',
         'gz sim|ardurover|arducopter|arduplane|mavros_node|'
         'rover_adapter|quad_adapter|fixed_wing_adapter|'
         'parameter_bridge|episode_runner|strategy_runner|'
         'world_pose_bridge|ros2 launch'],
        check=False,
    )
    # FastDDS shm cleanup so the next run discovers fresh.
    for shm in Path('/dev/shm').glob('fastrtps_*'):
        try:
            shm.unlink()
        except OSError:
            pass
    # Daemon stop so a fresh `ros2 topic list` after the next run
    # doesn't see stale state.
    subprocess.run(['ros2', 'daemon', 'stop'],
                   capture_output=True, check=False)
    time.sleep(1.0)
