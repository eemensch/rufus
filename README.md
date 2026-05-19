# Rufus

A pursuit-evasion differential-game simulation environment built on
ROS 2 Jazzy, ArduPilot SITL, and Gazebo Harmonic. Strategies command
the exact kinematic-model control inputs of each platform over a
uniform ROS 2 topic contract; the per-platform adapter clamps those
commands to a declared admissible control set.

The name is an oblique homage to Rufus Isaacs, founder of
differential game theory (*Games of Pursuit and Evasion*).

## Overview

An *agent* is a simulated vehicle (skid-steer rover, quadrotor, or
fixed-wing) flown under ArduPilot SITL with full Gazebo physics. A
*strategy* is a Python object that maps one tick's world view to a
control command. An *episode* is a YAML game over a fixed agent
manifest with termination predicates. The evaluation harness runs
sweeps of episodes headlessly and writes one CSV row plus a rosbag
per run.

Every agent exposes the same contract (see `docs/control.md`), for
an agent in namespace `<ns>`:

| Direction | Topic              | Type                            |
|-----------|--------------------|---------------------------------|
| in        | `<ns>/cmd_vel`     | `geometry_msgs/TwistStamped`    |
| out       | `<ns>/state`       | `rufus_sim_msgs/AgentState`     |
| out       | `<ns>/capability`  | `rufus_sim_msgs/Capability`     |

`capability` is latched `TRANSIENT_LOCAL`. The command is
interpreted as the platform's true kinematic inputs (for a
fixed-wing: airspeed, turn rate, climb rate), directly constrained
by the platform's admissible set rather than unified into a generic
velocity vector.

## Repository layout

```
ros2_ws/src/
  rufus_sim_msgs        ROS 2 interfaces (AgentState, Capability,
                        GameState, RoleAssignment, TerminationEvent)
  rufus_sim_adapters    per-platform cmd_vel -> MAVROS adapters
                        (rover, quad, fixed-wing); admissible-set clamp
  rufus_sim_bringup     launch files, agent manifests, sysid params
  rufus_sim_worlds      SDF worlds + per-instance model generation
                        (+ GZ_SIM_RESOURCE_PATH env-hook)
  rufus_sim_game        episode_runner: episode YAML, FCU-param
                        overrides, predicate engine, role hand-off
  rufus_sim_strategies  Strategy ABC, registry, strategy_runner node,
                        reference strategies (pure-pursuit, lead,
                        constant-bearing)
  rufus_sim_eval        headless batch sweep runner -> CSV + rosbag
docs/                   architecture and operating documentation
scripts/                interactive benches and plotting helpers
CLAIMS.md               claim ledger (reproducible-check status)
```

## Prerequisites

- Ubuntu 24.04 (Noble).
- ROS 2 Jazzy at `/opt/ros/jazzy/`, including the `gz-sim-vendor`
  packages (Gazebo Harmonic / gz Sim 8.x).
- Build tools: `git`, `cmake >= 3.16`, `ccache`, `gcc/g++` 13.x,
  `colcon`.
- ArduPilot SITL and the SITL-side Gazebo assets, vendored under
  `external/` (cloned, not tracked here). See `docs/setup.md`.

The unit-test suite needs only the colcon build. A live simulation
additionally needs the vendored `external/` trees; follow
`docs/setup.md` then `docs/operations.md`.

## Build

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Test

The package unit suites are pure Python plus the generated
`rufus_sim_msgs` bindings; no SITL or `external/` trees required:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
pytest -q ros2_ws/src/rufus_sim_game/test \
          ros2_ws/src/rufus_sim_strategies/test \
          ros2_ws/src/rufus_sim_adapters/test \
          ros2_ws/src/rufus_sim_eval/test
```

This currently reports 123 passing.

## Running a live simulation

A live run is a four-process chain, started in order: `gz sim`,
the ArduPilot SITL binary, `mavros_node`, and the platform adapter.
`docs/operations.md` gives the exact bring-up and tear-down
sequences, `docs/episodes.md` the episode format, and
`docs/evaluation.md` the headless sweep harness.

## Documentation

- `docs/setup.md` — fresh-machine setup, vendoring, extension.
- `docs/operations.md` — SITL chain bring-up and tear-down.
- `docs/control.md` — the cmd_vel / state / capability contract
  and per-platform admissible control sets.
- `docs/strategies.md` — the strategy interface and reference
  strategies.
- `docs/episodes.md` — episode YAML and the role hand-off contract.
- `docs/evaluation.md` — sweep YAML and the batch runner.
- `docs/plan.md` — eight-stage architecture and current status.

## Project status

Status is tracked, not asserted here. `docs/plan.md` holds the
per-stage status table; `CLAIMS.md` is the claim ledger, where the
analogue of a proof is a reproducible check, not a theorem. A claim
is closed only when no gaps remain and the author has agreed; no
claim is closed yet. Trust the code, `git log`, and the operational
docs for current ground truth.

## Name

Rufus is an oblique homage to Rufus Isaacs. "Isaacs" collides with
NVIDIA Isaac Sim/Lab and "Barrier" with Control/Lyapunov barrier
functions; the given name keeps the lineage with no term-of-art
clash and is a clean ROS package prefix (`rufus_sim_*`).

## Licence

Apache License 2.0. Copyright 2026 Iman Shames. See `LICENSE`; all
package manifests declare `Apache-2.0`.
