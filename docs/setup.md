# System setup and extension

This document covers how to set up the simulation environment from a
fresh machine, run the existing rover demo, and add new elements
(agents, vehicle types, worlds).

The current state of the project is **Stage 1 complete**: a single
ground rover (Aion R1, skid-steer) is brought up under ArduPilot SITL
with full gz Harmonic physics, and a ROS 2 adapter forwards
`TwistStamped` body-frame velocity commands to ArduRover via MAVROS.
Stage 2 (quadrotor) and Stage 3 (fixed-wing) extend the same pattern.

## Prerequisites

- **Ubuntu 24.04 (Noble)**.
- **ROS 2 Jazzy** installed at `/opt/ros/jazzy/`. Includes
  `ros-jazzy-gz-sim-vendor` and friends, which transitively install
  Gazebo Harmonic (gz Sim 8.x). `which gz` should return
  `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`.
- **Build tools**: `git`, `cmake >= 3.16`, `ccache`, `gcc/g++` (13.x),
  `colcon`. All present on a standard Jazzy install.
- **System libraries already installed by the ROS 2 Jazzy gz vendor
  packages**: `libgz-sim8-dev`, `libgz-cmake3-dev`, `libgz-msgs10-dev`,
  etc. ArduPilot SITL configure also passes without additional apt
  packages on this baseline.
- **MAVROS** (apt installable):

  ```bash
  sudo apt install ros-jazzy-mavros ros-jazzy-mavros-msgs \
      ros-jazzy-mavros-extras
  sudo bash /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
  ```

  The second command installs the EGM96 geoid data MAVROS needs at
  startup (`/usr/share/GeographicLib/geoids/egm96-5.pgm`). MAVROS
  crashes on launch without it.

## Initial setup

### Clone the project

```bash
cd ~/gitRepos/iman/   # or your preferred location
# project repo lives here as `rufus/`; not yet pushed to a remote.
```

### Vendor ArduPilot and SITL-side assets

Three repos go under `external/`:

```bash
cd <project>/
git clone --recurse-submodules --jobs 8 \
    https://github.com/ArduPilot/ardupilot.git external/ardupilot
git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
    external/ardupilot_gazebo
git clone --depth 1 https://github.com/ArduPilot/SITL_Models.git \
    external/SITL_Models
```

Pinned to whatever is on the upstream main/master at clone time.
Working configurations as of 2026-04-30:

- ArduPilot HEAD `3d313de` (Plane-4.6, Copter-4.6, Rover-4.6 series)
- ardupilot_gazebo HEAD `082a0fe`
- SITL_Models HEAD `047d601`

If you want stricter reproducibility, record commit hashes after
clone and check them out by SHA. The build below builds against
whatever HEAD is.

### Python virtualenv

ArduPilot's `waf` build needs a few Python packages. Per the project
convention, never install Python packages with
`--break-system-packages`: use a project-root `.venv/` instead.

```bash
cd <project>/
python3 -m venv .venv
.venv/bin/pip install --upgrade pip setuptools wheel
.venv/bin/pip install 'empy<4' pexpect lxml future pyserial \
    pymavlink MAVProxy
```

Whenever you build or run ArduPilot tooling, prepend `.venv/bin` to
`PATH`. Equivalently, `source .venv/bin/activate` first.

### Build ArduPilot SITL

```bash
cd <project>/external/ardupilot
PATH=<project>/.venv/bin:$PATH ./waf configure --board sitl
PATH=<project>/.venv/bin:$PATH ./waf copter plane rover
```

Outputs `build/sitl/bin/{arducopter,arduplane,ardurover}`. About 5 MB
each. ~12 minutes on a modern desktop.

### Build the ardupilot_gazebo plugin

```bash
cd <project>/external/ardupilot_gazebo
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

Outputs `libArduPilotPlugin.so` (plus camera/parachute/gst plugins).

### Build the ROS 2 workspace

```bash
cd <project>/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Builds `rufus_sim_msgs` (interfaces) and `rufus_sim_adapters` (the rover
adapter).

## Running and stopping the simulation

Day-to-day bring-up and tear-down sequences for the SITL chain are in
[`operations.md`](operations.md). Treat that as the operational
cheatsheet; this file is for one-time install and extension.

## Running evaluations

Batch sweeps over an episode parameter are documented in
[`evaluation.md`](evaluation.md). Common entry point:

```bash
source ros2_ws/install/setup.bash
ros2 run rufus_sim_eval batch_runner \
    ros2_ws/src/rufus_sim_eval/config/sweeps/rover_v_max_smoke.yaml
```

The plotting helper requires the project venv:

```bash
source .venv/bin/activate
python3 -m pip install matplotlib numpy        # one-time
python3 scripts/plot_sweep.py /tmp/pe_eval_smoke/summary.csv
```

## Repository layout

```
<project>/
  .venv/                       # Python venv for ArduPilot tooling
  config/                      # episode YAMLs, world defs (planned)
  docs/                        # this directory
  external/
    ardupilot/                 # ArduPilot SITL source (gitignored)
    ardupilot_gazebo/          # gz plugin source (gitignored)
    SITL_Models/               # extra vehicle SDFs (gitignored)
  ros2_ws/
    src/
      rufus_sim_msgs/             # AgentState, Capability, GameState,
                               # TerminationEvent, RoleAssignment,
                               # SaturationFlags
      rufus_sim_adapters/         # rover_adapter, quad_adapter,
                               # fixed_wing_adapter
      rufus_sim_worlds/
        worlds/                # single-agent SDFs:
                               #   iris_minimal.sdf,
                               #   zephyr_minimal.sdf
        templates/             # multi-agent generator inputs:
                               #   r1_rover.{sdf,config}.in,
                               #   world.sdf.in
        config/agents/         # multi-agent manifests:
                               #   two_rovers.yaml
        scripts/
          generate_world.py    # build-time SDF generator
      rufus_sim_bringup/
        launch/                # iris_sim.launch.py,
                               # zephyr_sim.launch.py,
                               # multi_agent_sim.launch.py
        config/sysid_overrides/
                               # sysid_1.parm, sysid_2.parm:
                               # per-instance MAV_SYSID overrides
                               # chained on ardurover --defaults
      rufus_sim_game/
        rufus_sim_game/           # predicate_engine, episode_runner,
                               # agent_params (per-platform
                               # high-level → FCU translation)
        config/episodes/       # rover_capture_1v1.yaml, ...
        launch/                # episode.launch.py
        test/                  # predicate-engine test suite
      rufus_sim_strategies/
        rufus_sim_strategies/     # strategy ABC, registry,
                               # reference strategies
                               # (PurePursuit / ConstantBearing
                               # / LeadPursuer), strategy_runner
        config/strategies/     # rover_capture_pp_vs_cb.yaml,
                               # ... (episode YAMLs that include
                               # `strategy:` blocks)
        launch/                # episode_with_strategies.launch.py
        test/                  # strategy unit tests
  scripts/
    rover_bench.py             # rover tracking-error benchmark
    quad_bench.py              # quad tracking-error benchmark
    fixed_wing_bench.py        # fixed-wing tracking-error benchmark
    dummy_pursuer.py           # 1v1 pure-pursuit demo strategy
    sitl_run/                  # SITL log directory (gitignored)
```

Planned packages (Stages 7–8): `rufus_sim_eval`, `rufus_sim_models`.
(`rufus_sim_worlds` and `rufus_sim_bringup` were stood up early during
Stage 2; `rufus_sim_game` landed in Stage 5; `rufus_sim_strategies`
landed in Stage 6.)

## Adding a new agent

A multi-agent scenario is described by a YAML manifest in
`rufus_sim_worlds/config/agents/`. The manifest is the single source
of truth: at colcon-build time `generate_world.py` reads it and
emits per-instance gz model dirs and the world SDF; at launch
time `multi_agent_sim.launch.py` reads the same file and spawns
one MAVROS + one type-specific adapter per agent.

To add a third rover to the existing two-rover scenario:

1. Edit `rufus_sim_worlds/config/agents/two_rovers.yaml` (or copy it
   to a new file under the same directory):

   ```yaml
   agents:
     - id: R0
       type: rover
       role: pursuer
       instance: 0          # SITL `-I 0`; MAVLink TCP 5760, FDM 9002
       spawn:
         xyz: [0.0, 0.0, 0.15]
         rpy_degrees: [0.0, 0.0, 90.0]
     # ... R1 ...
     - id: R2
       type: rover
       role: pursuer
       instance: 2          # SITL `-I 2`; MAVLink TCP 5780, FDM 9022
       spawn:
         xyz: [10.0, 0.0, 0.15]
         rpy_degrees: [0.0, 0.0, 90.0]
   ```

   Each `instance` value picks a disjoint port slot (MAVLink TCP =
   `5760 + 10·instance`, FDM UDP = `9002 + 10·instance`). Use
   `instance = 0, 1, 2, ...` and do not skip values within a
   scenario.

2. Add a `sysid_<n>.parm` file under
   `rufus_sim_bringup/config/sysid_overrides/` for the new instance,
   where `n = instance + 1`:

   ```
   MAV_SYSID 3
   ```

   This is the AP 4.7+ name for what older docs call
   `SYSID_THISMAV`. The legacy name is silently ignored. Without
   the per-instance override, every SITL emits heartbeats with
   sysid=1, MAVROS targets resolve to the same FCU, and driving
   `/<id_a>/cmd_vel` moves *every* rover.

3. `colcon build --symlink-install --packages-select rufus_sim_worlds
   rufus_sim_bringup`. The generator emits
   `r1_rover_inst2/{model.sdf,model.config}` into the install tree
   and rewrites the world SDF with the new `<include>`.

4. Bring up: start one extra `ardurover -I 2 ... --defaults
   <base>,sysid_3.parm` per the recipe in
   [`operations.md`](operations.md). The launch file picks up the
   new agent automatically because it reads the same manifest.

To run a *different* scenario without editing the default
manifest, write a new file (e.g. `four_agents.yaml`) and pass it
to the launch via `manifest:=<abs path>`. Stage 4 task #33 will
extend the templates to support `quad` and `plane` agent types.

## Adding a new vehicle type

Each platform needs:

1. **An SDF model with the ArduPilot plugin element**. Look at
   `external/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf`
   for a quad and
   `external/SITL_Models/Gazebo/models/r1_rover/model.sdf` for the
   skid-steer rover. The plugin block names channel-to-joint
   mappings and PID gains for each control output.
2. **An ArduPilot SITL frame** that maps the vehicle's parameter
   defaults. Frames are catalogued in
   `external/ardupilot/Tools/autotest/pysim/vehicleinfo.json`. For
   custom defaults, add a `.parm` file to
   `external/ardupilot/Tools/autotest/default_params/` and reference
   it in `vehicleinfo.json`, or pass it via `--defaults` on the SITL
   command line.
3. **A Python adapter** that mirrors `rover_adapter.py`:
   - Subscribes to `<ns>/cmd_vel`.
   - Pulls platform-specific FCU params and computes a `Capability`.
   - Drives the appropriate MAVROS setpoint topic. For quad: same
     `setpoint_velocity/cmd_vel_unstamped` (in body frame). For
     fixed-wing: a different setpoint surface depending on what
     ArduPlane GUIDED accepts.
   - Publishes `<ns>/state`.
4. **A new entry in `rufus_sim_adapters/setup.py`**:

   ```python
   entry_points={
       'console_scripts': [
           'rover_adapter = rufus_sim_adapters.rover_adapter:main',
           'quad_adapter  = rufus_sim_adapters.quad_adapter:main',
       ],
   }
   ```

The platform-specific work is in step (3): mapping a `TwistStamped`
to a MAVROS setpoint that the autopilot accepts in GUIDED. The rover
adapter is the simplest case; quad is similar; fixed-wing requires
projecting the Twist onto airspeed + heading + altitude rate (Dubins
airplane), since fixed-wing cannot hover.

## Adding a new world

Drop an `.sdf` file into a directory on `GZ_SIM_RESOURCE_PATH` (or
add the directory to `GZ_SIM_RESOURCE_PATH`). Reference an existing
ardupilot-aware world (e.g. `r1_rover_runway.sdf`) for the boilerplate
plugin set: `gz-sim-physics-system`, `gz-sim-user-commands-system`,
`gz-sim-scene-broadcaster-system`, `gz-sim-imu-system`, plus an
`<include>` of the desired vehicle model.

The `rufus_sim_worlds` package now exists; new SDFs go in
`ros2_ws/src/rufus_sim_worlds/worlds/` and become resolvable to gz
automatically — the package's ament environment hook prepends
`share/rufus_sim_worlds/worlds` to `GZ_SIM_RESOURCE_PATH` on
workspace source.

A world plus a manifest is what the SITL chain runs; an episode
on top of those is what defines a *game*. Episode YAMLs live
under `rufus_sim_game/config/episodes/`, are loaded by
`rufus_sim_game/episode_runner`, and bind a manifest to a list of
named termination predicates with dwell timers, optional
per-agent FCU parameter overrides, and optional warmup
waypoints. See [`episodes.md`](episodes.md) for the schema and
the per-platform parameter reference.

## Build-time troubleshooting

For run-time issues (pre-arm, MAVROS streams, FastDDS discovery, gz
plugin protocol mismatch), see [`operations.md`](operations.md).

**`./waf configure` cannot find `python`.** The `.venv` is not on
`PATH`. Prepend it: `PATH=$PROJ/.venv/bin:$PATH ./waf configure
--board sitl`.

**`pip install` fails with `error: externally-managed-environment`.**
Ubuntu 24.04 blocks pip into the system Python. Use the project venv;
do not pass `--break-system-packages`.

**`cmake` build of `ardupilot_gazebo` reports `gz-sim8 not found`.**
Install or re-source the ROS 2 Jazzy environment, which provides the
gz Harmonic dev libs:

```bash
sudo apt install --reinstall ros-jazzy-gz-sim-vendor
source /opt/ros/jazzy/setup.bash
```

**MAVROS install fails with missing `geographiclib-tools`.** Install
the geoid dataset script (Prerequisites). MAVROS itself installs
fine without the dataset, but its `mavros_node` crashes at start.
