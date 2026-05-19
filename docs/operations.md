# Bring-up and tear-down

Repeated operational sequences for the SITL chain. Assumes the
project has been built per [`setup.md`](setup.md).

The SITL chain comprises **four processes**, started in this order:

1. `gz sim`               — physics simulator with the vehicle world
2. `ardurover` (or other) — ArduPilot SITL flight stack
3. `mavros_node`          — MAVLink ↔ ROS 2 bridge
4. `rover_adapter`        — pursuit-evasion adapter (cmd_vel → MAVROS)

Each runs in its own terminal. They communicate over UDP/TCP:

```
strategy ─cmd_vel→ rover_adapter ──→ /mavros/setpoint_velocity/...
                                       └→ mavros_node ─MAVLink TCP 5760→ ardurover
                                                                    └─UDP 9002↔gz sim plugin
```

## Environment

Set these once in each shell that runs gz, the SITL binary, or the
ROS 2 workspace. Adjust `PROJ` for your install location.

```bash
export PROJ=$HOME/gitRepos/iman/rufus

# gz: where to find ardupilot_gazebo plugin and model trees
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PROJ/external/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=\
$PROJ/external/ardupilot_gazebo/models:\
$PROJ/external/ardupilot_gazebo/worlds:\
$PROJ/external/SITL_Models/Gazebo/models:\
$PROJ/external/SITL_Models/Gazebo/worlds

# ArduPilot Python deps (waf, sim_vehicle.py)
export PATH=$PROJ/.venv/bin:$PATH

# ROS 2 + workspace
source /opt/ros/jazzy/setup.bash
source $PROJ/ros2_ws/install/setup.bash
```

Consider keeping these in a `~/.config/rufus_sim/env.sh` and sourcing it
per terminal.

## Bring-up: rover

### 1. gz sim (terminal 1)

```bash
gz sim -s -r -v 2 r1_rover_runway.sdf
```

Flags:

- `-s` server-only (headless). Drop `-s` to open the GUI.
- `-r` run on start (don't start paused).
- `-v 2` warning verbosity. `-v 4` for debug if diagnosing the
  ardupilot_gazebo plugin.

**Verify:** the gz log prints SDF-parse warnings, then nothing more
unless errors. Confirm the simulation is advancing:

```bash
gz topic -e -t /world/runway/stats --duration 1 | head -8
```

`real_time_factor` should be near 1.0; `sim_time` should advance.

### 2. ArduRover SITL (terminal 2)

```bash
mkdir -p $PROJ/scripts/sitl_run/rover && cd $_
$PROJ/external/ardupilot/build/sitl/bin/ardurover \
    -w --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I0 \
    --defaults \
$PROJ/external/ardupilot/Tools/autotest/default_params/rover.parm,\
$PROJ/external/ardupilot/Tools/autotest/default_params/rover-skid.parm,\
$PROJ/external/SITL_Models/Gazebo/config/r1_rover.param
```

Why these arguments:

- `-w` wipes the EEPROM image so we start from defaults.
- `--model JSON` selects the new gz JSON protocol. **Do not use
  `--model gazebo-iris` style strings**; that selects the legacy
  binary protocol and the gz plugin rejects packets with
  `Incorrect protocol magic 0`.
- `--speedup 1` runs at real time. Increase for batch evaluation.
- `--slave 0`, `-I0` instance 0 (default MAVLink ports).
- `--defaults` chains rover defaults, skid-steer mapping, and the
  Aion R1 PID tunes from SITL_Models. Comma-separated, no spaces.
- `cwd = scripts/sitl_run/rover/` keeps SITL log files out of the
  repo root.

**Verify:** the SITL log prints

```
JSON received:
    timestamp
    imu: gyro
    imu: accel_body
    position
    quaternion
    velocity
    no_time_sync
    no_lockstep
```

within a few seconds. That confirms the gz plugin → ArduRover sensor
link.

### 3. MAVROS (terminal 3)

```bash
ros2 launch mavros apm.launch \
    fcu_url:='tcp://localhost:5760' \
    tgt_system:=1 tgt_component:=1
```

Plugin loading takes 30–60 s on first start (the apm pluginlist is
~100 plugins). The relevant log lines to wait for:

```
[mavros.mavros]: MAVROS UAS via /uas1 started.
[mavros.mavros_router]: link[1001] detected remote address 1.191
```

### 4. rover adapter (terminal 4)

```bash
ros2 run rufus_sim_adapters rover_adapter \
    --ros-args -p agent_id:=R0 -p role:=2
```

The adapter walks a state machine: WAIT_CONNECT → REQUESTING_STREAMS
→ PULLING_PARAMS → DISABLING_CHECKS → SETTING_MODE → ARMING → READY.
Expect ~10–25 s end-to-end, dominated by EKF convergence (the arm
step retries until pre-arm passes).

A successful bring-up:

```
[INFO] rover_adapter started for agent_id=R0, mavros_ns=/mavros, agent_ns=''
[INFO] FCU connected; requesting streams
[INFO] streams requested; pulling params
[INFO] pulled 1275 FCU params
[INFO] Capability: v_max=1.00 m/s, yaw_rate_max=2.09 rad/s
[INFO] ARMING_CHECK set request issued
[INFO] GUIDED mode set
[WARN] arm acknowledged but FCU not armed; retrying    # while EKF settles
...
[INFO] armed; READY
```

`READY` means `/cmd_vel` is now wired through to ArduRover's GUIDED
mode velocity setpoint. Drive it per [`control.md`](control.md).

## Bring-up: quadrotor

The chain has the same roles as for the rover with two
substitutions: `arducopter` replaces `ardurover`, and a
`rufus_sim_bringup` launch starts both a gz/ROS clock bridge and
MAVROS with `use_sim_time:=true`. Five processes:

1. `gz sim`           — physics with `iris_minimal.sdf`
2. `arducopter`       — ArduPilot SITL flight stack
3. `parameter_bridge` — gz `/world/iris_minimal/clock` → ROS `/clock`
4. `mavros_node`      — MAVLink ↔ ROS 2 with sim time
5. `quad_adapter`     — pursuit-evasion adapter

Processes 3 and 4 are launched together by
`rufus_sim_bringup/iris_sim.launch.py`.

### 1. gz sim (terminal 1)

```bash
gz sim -s -r -v 2 iris_minimal.sdf
```

`iris_minimal.sdf` lives in `rufus_sim_worlds/worlds/` and is
resolved via the package's ament environment hook (sourcing the
workspace prepends `share/rufus_sim_worlds/worlds` to
`GZ_SIM_RESOURCE_PATH`). It is a stripped-down version of the
upstream `iris_runway.sdf` — same physics and plugins, but
spawning `iris_with_ardupilot` (no gimbal, no camera) instead of
`iris_with_gimbal`. The lighter model removes the per-tick
variance in physics step time that triggered `Time jump detected`
warnings under the upstream world.

### 2. ArduCopter SITL (terminal 2)

```bash
mkdir -p $PROJ/scripts/sitl_run/quad && cd $_
$PROJ/external/ardupilot/build/sitl/bin/arducopter \
    -w --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I0 \
    --defaults \
$PROJ/external/ardupilot/Tools/autotest/default_params/copter.parm
```

Same flags as `ardurover`; only the binary and the default-params
file differ. See the rover bring-up for the rationale on each.

**Verify:** within a few seconds of MAVROS connecting on TCP
5760, the SITL log prints the same `JSON received: ...` block as
the rover, with `Forcing use_time_sync=0` (matches the world's
plugin configuration; expected).

### 3 + 4. Clock bridge + MAVROS (terminal 3)

```bash
ros2 launch rufus_sim_bringup iris_sim.launch.py
```

This single launch:

- Starts `ros_gz_bridge parameter_bridge` mapping gz's
  `/world/iris_minimal/clock` to ROS `/clock`.
- Starts `mavros_node` directly (neither `apm.launch` nor
  `node.launch` exposes `use_sim_time`) with the apm
  pluginlists/config plus `use_sim_time:=true`.

The clock bridge plus `use_sim_time` makes MAVROS read time from
the gz physics clock — the same source the FCU reports against —
which eliminates the recurring
`Time jump detected. Resetting time synchroniser.` warnings
otherwise observed under default wall-clock bring-up of MAVROS
against ArduCopter.

Optional argument:

```bash
ros2 launch rufus_sim_bringup iris_sim.launch.py \
    fcu_url:=tcp://localhost:5760
```

Plugin loading takes 30–60 s on first start, same as the rover.

### 5. quad adapter (terminal 4)

```bash
ros2 run rufus_sim_adapters quad_adapter \
    --ros-args -p agent_id:=Q0 -p role:=2 -p use_sim_time:=true
```

The bring-up state machine adds a `TAKING_OFF` step compared to
the rover:

```
WAIT_CONNECT → REQUESTING_STREAMS → PULLING_PARAMS →
DISABLING_CHECKS → SETTING_MODE → ARMING → TAKING_OFF → READY
```

After arming, the adapter issues `MAV_CMD_NAV_TAKEOFF` with the
configured altitude (default `takeoff_altitude_m` 5.0 m) and
gates `READY` on local-position altitude exceeding the threshold
(default `takeoff_alt_threshold_m` 4.0 m). This is required:
ArduCopter in GUIDED on the ground ignores velocity setpoints,
so the adapter must lift off before strategies can drive it.

Successful bring-up:

```
[INFO] quad_adapter started for agent_id=Q0, ...
[INFO] FCU connected; requesting streams
[INFO] streams requested; pulling params
[INFO] pulled 1379 FCU params
[INFO] Capability: v_max=10.00 m/s, vz_up=2.50 m/s,
       vz_dn=1.50 m/s, yaw_rate_max=1.57 rad/s,
       bank_max=30.0 deg
[INFO] ARMING_CHECK set request issued
[INFO] GUIDED mode set
[INFO] armed; commanding takeoff
[INFO] takeoff acknowledged; climbing
[INFO] altitude reached (4.58 m); READY
```

End-to-end ~15–20 s wall time on a fresh chain.

`READY` means `/cmd_vel` is wired through to ArduCopter's
GUIDED-mode body-frame velocity setpoint and the iris is hovering
at takeoff altitude. Drive it per [`control.md`](control.md).

## Bring-up: fixed-wing (zephyr)

The chain has the same five roles as for the iris quadrotor,
with `arduplane` replacing `arducopter` and `zephyr_minimal.sdf`
replacing `iris_minimal.sdf`. The adapter's bring-up sequence
differs at takeoff because ArduPlane in GUIDED with the gz
JSON setup will not auto-launch via `MAV_CMD_NAV_TAKEOFF`; the
adapter instead uses the canonical zephyr recipe documented in
`external/ardupilot_gazebo/README.md` (`mode fbwa` → `arm` →
`rc 3 1800`).

Five processes:

1. `gz sim`             — physics with `zephyr_minimal.sdf`
2. `arduplane`          — ArduPilot SITL flight stack
3. `parameter_bridge`   — `/world/zephyr_minimal/clock` →
   ROS `/clock`
4. `mavros_node`        — MAVLink ↔ ROS 2 with sim time
5. `fixed_wing_adapter` — pursuit-evasion adapter

Processes 3 and 4 are launched together by
`rufus_sim_bringup/zephyr_sim.launch.py`.

### 1. gz sim (terminal 1)

```bash
gz sim -s -r -v 2 zephyr_minimal.sdf
```

`zephyr_minimal.sdf` lives in `rufus_sim_worlds/worlds/`. It is a
copy of upstream's `zephyr_runway.sdf` with `real_time_factor`
pinned to 1.0 (the upstream world uses -1.0 for unthrottled
operation, which composes badly with our `use_sim_time` +
lockstep chain). The zephyr is spawned at `roll=-90 deg` (on
its side) — a deliberate SITL hack from upstream so that
throttle thrust along body-X becomes vertical thrust in the
world frame. The plane lifts off straight up under the
FBWA + throttle takeoff sequence.

### 2. ArduPlane SITL (terminal 2)

```bash
mkdir -p $PROJ/scripts/sitl_run/zephyr && cd $_
$PROJ/external/ardupilot/build/sitl/bin/arduplane \
    -w --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I0 \
    --defaults \
$PROJ/external/ardupilot/Tools/autotest/default_params/gazebo-zephyr.parm
```

Same flags as `ardurover` and `arducopter`; only the binary and
the default-params file differ. `gazebo-zephyr.parm` configures
elevon servo mappings and INS calibration offsets for the
zephyr.

### 3 + 4. Clock bridge + MAVROS (terminal 3)

```bash
ros2 launch rufus_sim_bringup zephyr_sim.launch.py
```

Identical structure to `iris_sim.launch.py`; only the bridged
clock topic differs (`/world/zephyr_minimal/clock`).

### 5. fixed-wing adapter (terminal 4)

```bash
ros2 run rufus_sim_adapters fixed_wing_adapter \
    --ros-args -p agent_id:=F0 -p role:=2 -p use_sim_time:=true
```

The bring-up state machine adds two states relative to the
quad: `SETTING_GCS_SYSID`, and a `TAKING_OFF` state that
publishes RC throttle override every tick:

```
WAIT_CONNECT → REQUESTING_STREAMS → PULLING_PARAMS →
DISABLING_CHECKS → SETTING_GCS_SYSID →
SETTING_MODE (FBWA) → ARMING →
TAKING_OFF (RC throttle override + altitude poll) →
SETTING_GUIDED → READY
```

The non-obvious step is `SETTING_GCS_SYSID`. ArduPlane defaults
`MAV_GCS_SYSID` to 255 but MAVROS sends with sysid 1; without
the match, AP silently drops every `rc/override` message. The
adapter sets it to 1. With that done, the FBWA + RC throttle
high recipe lifts the airframe vertically off the spawn pose
(throttle = body-X = vertical because of the -90 deg roll). At
the altitude threshold (default 40 m) the adapter releases the
override and switches mode to GUIDED.

Successful bring-up:

```
[INFO] fixed_wing_adapter started for agent_id=F0, ...
[INFO] FCU connected; requesting streams
[INFO] streams requested; pulling params
[INFO] pulled 1366 FCU params
[INFO] Capability: V=[9.0, 22.0] m/s, vz_up=7.52 m/s,
       vz_dn=9.30 m/s, bank_max=45.0 deg,
       climb_angle_max=20.0 deg,
       yaw_rate_at_cruise=0.82 rad/s
[INFO] ARMING_CHECK set request issued
[INFO] MAV_GCS_SYSID=1 set request issued
       (needed for RC override pass-through)
[INFO] FBWA mode set
[INFO] armed in FBWA; pushing RC throttle override
[INFO] altitude reached (40.35 m); releasing throttle
       override and switching to GUIDED
[INFO] GUIDED mode set; READY
```

End-to-end ~50 s wall time on a fresh chain.

`READY` means `/cmd_vel` is wired through to ArduPlane GUIDED
as a world-frame velocity setpoint and the plane is cruising
at takeoff altitude. Drive it per [`control.md`](control.md).

## Why not `sim_vehicle.py`

ArduPilot ships a convenience wrapper, `sim_vehicle.py`, that spawns
the SITL binary plus a MAVProxy console. We deliberately bypass it
and call `ardurover` (and later `arducopter`, `arduplane`) directly.
Reasons:

- **Process model.** The wrapper launches SITL inside an `xterm` by
  default; if the terminal dies, SITL dies with it. Direct invocation
  makes SITL a child of your shell, so signals, PIDs, and exit codes
  behave predictably.
- **Headless / CI.** The wrapper's defaults require `DISPLAY` and
  `xterm`. SSH sessions, CI runners, and Docker containers fail
  immediately. `-N` / `--no-mavproxy` work in practice, but the flag
  matrix has shifted across ArduPilot releases.
- **Reproducibility.** The exact argv is recorded in this document.
  `sim_vehicle.py -v ArduRover -f rover-skid` translates to a binary
  argv via opaque, version-dependent code; if upstream changes the
  frame map, our recipe silently breaks.
- **Multi-agent scaling (Stage 4).** N parallel SITL instances under
  `ros2 launch` is N direct binary calls with `-I 0..N-1`. Wrapper
  mode is N xterms + N MAVProxy processes + N Python parents, and is
  much harder to template in a launch file.
- **Failure surface.** Direct mode: SITL faults only. Wrapper mode:
  SITL + wrapper Python + MAVProxy + xterm + `DISPLAY` + the
  wrapper's argv translation. More layers, more places to look.

The wrapper is the right tool for one-off interactive debugging with
MAVProxy's console and map. It is not the right tool for anything
that needs to survive batch evaluation, multi-agent scaling, or CI.

## Tear-down

Stop processes, clear FastDDS shared-memory locks, stop the ROS 2
discovery daemon. Skipping the shm cleanup is the most common cause
of `ros2 topic list` returning only `/parameter_events` and
`/rosout` on the next session.

```bash
pkill -9 -f \
    'rover_adapter|quad_adapter|fixed_wing_adapter|mavros_node|parameter_bridge|ardurover|arducopter|arduplane|gz sim'
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 daemon stop
```

Verify nothing is left behind:

```bash
pgrep -af \
    'rover_adapter|quad_adapter|fixed_wing_adapter|mavros_node|parameter_bridge|ardurover|arducopter|arduplane|gz sim' \
    | grep -v pgrep
ss -ltn | grep -E ':57(60|62|63)'   # MAVLink TCP ports
ls /dev/shm/ | grep fastrtps
```

All three should be empty.

## One-shot scripts

For convenience, the four bring-up commands and the tear-down can
live in `scripts/`. None are checked in yet; the canonical recipe is
this document. If you do write wrappers, keep `gz sim` in a
foreground terminal for visibility — backgrounded gz often emits
errors that go unread.

## Common bring-up failures

**Pre-arm `Accels inconsistent`.** EKF has not converged. The adapter
retries arming every 0.5 s; allow ~20 s.

**Pre-arm `AHRS: not using configured AHRS type`.** Same root cause
(EKF still settling). Resolves on its own; appears in
`/mavros/state`'s STATUSTEXT and in MAVROS log.

**MAVROS crash on launch with `GeographicLib exception`.** EGM96
geoid data missing. See [`setup.md`](setup.md) prerequisites.

**`/mavros/state` shows `connected: true` but other MAVROS topics
silent.** Stream rate is zero. The adapter requests streams during
bring-up; if you bypass the adapter, request manually:

```bash
ros2 service call /mavros/set_stream_rate \
    mavros_msgs/srv/StreamRate \
    '{stream_id: 0, message_rate: 10, on_off: true}'
```

**ArduRover armed but disarms after a few seconds.** No setpoint is
flowing. With the adapter in `READY`, it republishes the latest
`cmd_vel` at 20 Hz, which keeps the FCU active. If you bypass the
adapter, publish a setpoint at >2 Hz to suppress the FCU's
inactivity timeout.

**`Incorrect protocol magic 0 should be 18458` in gz log.** The
SITL binary is using the legacy protocol. Restart with
`--model JSON`.

**`ros2 topic list` returns only `/parameter_events`, `/rosout`
despite the chain being up.** FastDDS shm is stale. Tear down,
clear shm, restart:

```bash
ros2 daemon stop
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

**gz log shows `Error Code 14: Unable to find
uri[model://zephyr_inst0]` and the gz process goes `<defunct>`;
SITL spams `No JSON sensor message received`; `wait_ready`
then silently burns its full timeout.** A standalone driver
(anything driving `chain.py` outside `batch_runner`) built its
subprocess env by *overwriting* `GZ_SIM_RESOURCE_PATH` with only
the external ardupilot_gazebo/SITL_Models trees. The generated
per-instance models (`zephyr_instN`, `iris_instN`,
`r1_rover_instN`) live under
`install/rufus_sim_worlds/share/rufus_sim_worlds/models` and are put
on `GZ_SIM_RESOURCE_PATH` by the colcon `rufus_sim_worlds` env hook
when you `source install/setup.bash`. Mirror
`batch_runner._build_env` exactly: `env = dict(os.environ)` then
**append** the external trees
(`f"{env.get('GZ_SIM_RESOURCE_PATH','')}:{extra}"`); never
assign a fresh value. Related: a pre-cleanup `pkill -9 -f
'gz sim|arduplane|...'` must sit *inside* the wrapper script,
not in the outer launch command line — otherwise `pkill -f`
matches the launcher's own cmdline (which contains those tokens)
and SIGKILLs itself, giving an instant exit 1 with empty output.

**A single-agent bench (`fixed_wing_bench.py`, `rover_bench.py`,
`quad_bench.py`) hangs forever at `waiting for /state` even
though `wait_ready` reported the adapter took off.** The bench
uses **absolute** `/state` and `/cmd_vel`, but
`multi_agent_sim.launch.py` runs the adapter with
`agent_ns='/<id>'`, so it publishes `/<id>/state` and subscribes
`/<id>/cmd_vel`. Absolute (leading-slash) topic names are *not*
rewritten by node-namespace remap, so the two never connect.
Run the bench with explicit topic remaps to the manifest's
agent id, e.g. for `P0`:
`--ros-args -r /cmd_vel:=/P0/cmd_vel -r /state:=/P0/state`.
(The manual operations.md recipe launches the adapter via
`ros2 run` with `agent_ns` default-empty, so the topics are
already `/state`,`/cmd_vel` — the mismatch only bites the
chain.py / multi_agent_sim path.)

**Driving `/<id_a>/cmd_vel` moves more than one agent in a
multi-agent run.** The per-instance `MAV_SYSID` parm did not
take effect. Most common cause: the parm file has the legacy
`SYSID_THISMAV` name, which AP 4.7+ silently ignores — the SITL
keeps emitting heartbeats with sysid=1, both `mavros_*` targets
resolve to the same FCU, and a setpoint to one is forwarded to
both. Confirm by reading the heartbeat sysid in the MAVROS
log: each `mavros_X.sys: VER: <SYS>.<COMP>` line should report
`<SYS>` matching that instance's `tgt_system`. If they all show
`1.1`, fix the parm file to use `MAV_SYSID <n>`, restart the
chain.

## Bring-up: multi-agent (two rovers)

The chain has the same logical roles as the single-agent rover
recipe, repeated per agent: one `ardurover` SITL + one
`mavros_node` + one `rover_adapter` per agent, plus a single
shared `gz sim` and one `parameter_bridge`. Five-or-more
processes for two rovers:

1. `gz sim`           — physics with `two_rovers_minimal.sdf`
2. `ardurover` × 2    — `-I 0` and `-I 1` (port slots `5760/9002`
                        and `5770/9012`), each chained with a
                        per-instance `MAV_SYSID` parm file
3. `parameter_bridge` — gz `/world/two_rovers_minimal/clock` →
                        ROS `/clock`
4. `mavros_node` × 2  — namespaces `/mavros_0` and `/mavros_1`
5. `rover_adapter` × 2 — namespaces `/R0` and `/R1`

Processes 3–5 are launched together by
`rufus_sim_bringup/multi_agent_sim.launch.py`, which reads the same
YAML manifest (`rufus_sim_worlds/config/agents/two_rovers.yaml`)
that produced the world SDF. Adding or removing an agent is a
manifest edit + rebuild; see [`setup.md`](setup.md).

### 1. gz sim (terminal 1)

```bash
gz sim -s -r -v 2 two_rovers_minimal.sdf
```

The world SDF is generated at colcon-build time from
`rufus_sim_worlds/templates/world.sdf.in` and the agent manifest.
Each agent appears as a `<include>` of its per-instance model
`r1_rover_inst<N>`, also generated, where the `<fdm_port_in>` is
shifted to `9002 + 10·N` so each ardurover talks to its own gz
plugin instance.

**Verify:** before starting any SITL, `ss -lun | grep -E
'9002|9012'` shows two distinct UDP listeners on `127.0.0.1`.

### 2. ArduRover SITL (terminals 2 and 3)

```bash
PROJ=$HOME/gitRepos/iman/rufus
DEFAULTS_BASE=\
$PROJ/external/ardupilot/Tools/autotest/default_params/rover.parm,\
$PROJ/external/ardupilot/Tools/autotest/default_params/rover-skid.parm,\
$PROJ/external/SITL_Models/Gazebo/config/r1_rover.param
SYSID_DIR=\
$PROJ/ros2_ws/install/rufus_sim_bringup/share/rufus_sim_bringup/config/sysid_overrides

# Terminal 2: rover 0
mkdir -p $PROJ/scripts/sitl_run/rover0 && cd $_
$PROJ/external/ardupilot/build/sitl/bin/ardurover \
    -w --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I0 \
    --defaults ${DEFAULTS_BASE},${SYSID_DIR}/sysid_1.parm

# Terminal 3: rover 1
mkdir -p $PROJ/scripts/sitl_run/rover1 && cd $_
$PROJ/external/ardupilot/build/sitl/bin/ardurover \
    -w --model JSON --speedup 1 --slave 0 \
    --sim-address=127.0.0.1 -I1 \
    --defaults ${DEFAULTS_BASE},${SYSID_DIR}/sysid_2.parm
```

`-I 0` and `-I 1` shift the MAVLink TCP and FDM UDP ports by
10·instance; the gz plugin's `<fdm_port_in>` is set accordingly
in the per-instance models. The `sysid_<n>.parm` chain (n =
instance + 1) sets `MAV_SYSID` so each SITL emits heartbeats with
a distinct MAVLink system id; without it both rovers come up with
sysid=1 and any MAVROS write to one ardurover routes to *both*.
`MAV_SYSID` is the AP 4.7+ name; the legacy `SYSID_THISMAV` is
silently ignored (see `Common bring-up failures`).

**Verify:** rover 0's first heartbeat-derived `VER` line in the
MAVROS log reads `VER: 1.1`, rover 1's reads `VER: 2.1`. Once
both SITLs see their gz plugins, both log `JSON received: ...`
just like the single-rover bring-up.

### 3 + 4 + 5. Clock bridge + MAVROS + adapters (terminal 4)

```bash
ros2 launch rufus_sim_bringup multi_agent_sim.launch.py
```

Default manifest is `two_rovers.yaml`. Override:

```bash
ros2 launch rufus_sim_bringup multi_agent_sim.launch.py \
    manifest:=$PROJ/path/to/other.yaml
```

Per agent, the launch sets `tgt_system = instance + 1` (matching
the per-instance `MAV_SYSID`), `system_id = 240 + instance` (the
MAVROS sender's own sysid; arbitrary but distinct between
processes), and `use_sim_time:=true`. Each adapter starts in its
own DDS namespace (`/<id>`), publishes `<id>/state` and
`<id>/capability`, subscribes to `<id>/cmd_vel`, and reaches
`READY` independently.

Successful bring-up logs (excerpt):

```
[mavros_0.sys]: VER: 1.1: Capabilities ...
[mavros_1.sys]: VER: 2.1: Capabilities ...
[rover_adapter_R0]: armed; READY
[rover_adapter_R1]: armed; READY
```

End-to-end ~30–45 s wall time. Drive each agent independently
via `/R0/cmd_vel` and `/R1/cmd_vel` per [`control.md`](control.md).

**Verify isolation:**

```bash
gz model -m R0 --pose
gz model -m R1 --pose
ros2 topic pub -r 10 -t 50 /R0/cmd_vel geometry_msgs/msg/TwistStamped \
    '{header: {frame_id: base_link}, twist: {linear: {x: 0.5}}}'
gz model -m R0 --pose
gz model -m R1 --pose
```

R0 should advance by ~2.5 m along world `+y` (its body `+x` after
yaw=90°); R1 should be unchanged within a millimetre. Cross-talk
of any larger magnitude on R1 implies the `MAV_SYSID` override
did not take effect — confirm via `ros2 topic echo
/mavros_1/state` that `system_status` and the heartbeat trace
report sysid=2.
