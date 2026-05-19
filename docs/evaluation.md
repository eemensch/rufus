# Evaluation harness

Stage 8 wraps the SITL chain in a headless batch runner so a single
sweep YAML produces a Cartesian product of episodes, runs them
sequentially, and writes one CSV row per run plus a per-run rosbag.
The same harness drives smoke runs (4 runs, ~6 min) and full
Monte Carlo sweeps (100+ runs).

The package is `rufus_sim_eval`. It depends on `rufus_sim_strategies`
(episode authoring), `rufus_sim_bringup` (manifests, sysid params), and
`rufus_sim_msgs` (`TerminationEvent`, `GameState`).

## Sweep YAML

Schema v1, validated by `rufus_sim_eval.sweep.load_sweep`:

```yaml
schema_version: 1
name: rover_v_max_smoke           # used as a sub-dir under output_dir
episode: package://rufus_sim_strategies/config/strategies/rover_capture_smoke.yaml
axes:
  - parameter: agents.R1.parameters.high_level.v_max
    values: [0.6, 0.9]
seeds: 2                          # repeats per axis combination
speedup: 1.0                      # see "Speedup caveat" below
output_dir: /tmp/pe_eval_smoke    # absolute path
```

Required: `schema_version`, `name`, `episode`, `seeds`, `output_dir`.
`axes` may be empty or absent, in which case the harness produces
`seeds` runs of the unmodified base episode.

Each axis entry is a dotted path into the base episode YAML and a
list of values to substitute. Paths resolve against the loaded
episode dict; intermediate keys may be missing (the loader will
create them at write time), but if any intermediate key exists and
is not a mapping, the loader fails fast with the offending prefix.

Run order is row-major: outermost axis varies slowest, innermost
fastest, with seeds as the innermost loop. A 3-value × 4-seed sweep
produces 12 runs in order
`[(v0, s0), (v0, s1), …, (v0, s3), (v1, s0), …]`.

Every run carries an episode YAML named
`<base_name>__r<run_id:04d>`. The runner materialises that YAML to
`<output_dir>/runs/run_NNNN/episode.yaml` before bringing up the
chain, so each run is reproducible from disk alone.

## Running a sweep

```bash
source ros2_ws/install/setup.bash
ros2 run rufus_sim_eval batch_runner \
    ros2_ws/src/rufus_sim_eval/config/sweeps/rover_v_max_smoke.yaml
```

The single optional flag is `--limit N`, which runs at most N
runs from the enumerated product. Useful when a sweep YAML
specifies 100 runs but you only want to confirm the chain comes
up.

The runner is foreground: it prints one line per run start, one per
run end (with wallclock and outcome), and flushes
`<output_dir>/summary.csv` after every row so progress is visible
live. Tail the CSV from another shell if you want a live capture
rate.

Per-run lifecycle (sequential, one run at a time):

1. Materialise the per-run episode YAML.
2. Bring up gz, SITL × N, and `multi_agent_sim`.
3. Wait until every agent reports its platform-specific READY
   pattern in the bringup log.
4. Spawn `episode_with_strategies` against the per-run YAML.
5. Subscribe to `/game/termination_event`
   (`TRANSIENT_LOCAL`, latched) and `/game/state` (volatile) and
   spin until the event arrives or `duration_s + 30 s` of
   wallclock pass.
6. Tear the chain down: `SIGKILL` the per-process group, then
   `pkill` any straggler by name, then unlink `/dev/shm/fastrtps_*`
   and stop the ROS 2 daemon. Wait 1 s before the next run.
7. Append a CSV row.

The bringup uses `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` to keep
DDS on shm transport, since the multi-host case (SUBNET) has been
spam-prone with active VPNs and multicast filtering on the
development host.

## Output layout

```
<output_dir>/
├── summary.csv
└── runs/
    ├── run_0000/
    │   ├── episode.yaml          # the materialised per-run episode
    │   ├── bringup.log           # multi_agent_sim launch stdout
    │   ├── episode.log           # episode_with_strategies stdout
    │   ├── gz.log                # gz sim stdout
    │   ├── sitl_R0.log           # one per agent
    │   ├── sitl_R1.log
    │   ├── sitl_R0/              # per-agent SITL working dir
    │   ├── sitl_R1/
    │   └── bag/                  # rosbag2, default storage
    └── run_0001/
        └── ...
```

`summary.csv` columns, in order:

| Column          | Source                                            |
|-----------------|---------------------------------------------------|
| `run_id`        | Index in row-major enumeration                    |
| `seed`          | `0..seeds-1`                                      |
| `wallclock_s`   | Bringup + episode + teardown, measured by runner  |
| `<axis path>`   | One column per axis, in axis-declaration order    |
| `outcome`       | `pursuer_win` / `evader_win` / `timeout` / `__no_termination__` / custom string |
| `predicate_id`  | The predicate that fired, empty on timeout/no-event |
| `sim_time_s`    | `TerminationEvent.sim_time` in seconds            |
| `<agent>_x/y/z` | Last `GameState` pose per agent at termination    |

`__no_termination__` means the runner timed out before any
predicate fired. The CSV row is still written so the run is visible
in aggregates; it does not count as a capture or escape.

## Metrics

```python
from rufus_sim_eval.metrics import aggregate, format_summary, load_summary
groups = aggregate(load_summary('summary.csv'))
print(format_summary(groups))
```

`aggregate` groups rows by axis combination and returns a list of
`GroupResult` records:

| Field                                    | Meaning                          |
|------------------------------------------|----------------------------------|
| `axis_values`                            | dotted-path → value mapping      |
| `n`, `n_pursuer_win`, `n_evader_win`, `n_timeout`, `n_other` | counts |
| `capture_rate`                           | `n_pursuer_win / n`              |
| `capture_time_mean / median / p95`       | over `pursuer_win` runs only; NaN if none |
| `terminal_positions[<agent>]`            | NaN-aware mean / std / count for each of x, y, z |

Outcomes other than `pursuer_win` contribute to the rate denominator
but not to the timing quantiles; this is the convention used by the
predicate-design literature and matches the smoke baseline.

`format_summary` produces a tab-separated text table suitable for
piping into a markdown cell or a notebook print.

## Plotting

```bash
source .venv/bin/activate            # matplotlib + numpy
python3 scripts/plot_sweep.py /tmp/pe_eval_smoke/summary.csv
```

Writes two PNGs alongside the CSV: `capture_rate.png` (bar over the
first axis) and `capture_time_box.png` (box-and-whisker, restricted
to capture runs). Multiple-axis sweeps emit a stderr warning and
plot only the first axis; an axis-pair heatmap is in the Stage 8
follow-ups.

## Speedup caveat

The sweep YAML accepts a `speedup` field that the runner forwards
to `ardurover --speedup`, but the world SDFs generated by
`rufus_sim_worlds` hard-code `<real_time_factor>1.0</real_time_factor>`
in `<physics>`. Under SITL lock-step the SITL binary blocks until gz
advances, so gz steps at wallclock pace and `--speedup > 1` has no
practical effect. The wallclock budget for `/game/termination_event`
is therefore `duration_s + 30 s` regardless of `speedup`.

Honouring `speedup` end-to-end requires parameterising the world
template by RTF. This is open as Stage 8 follow-up #1 (`plan.md`).

## Cross-run drift and the first-run penalty

Cross-run determinism at `speedup=1.0` is bounded but not
bit-exact. Measured on the rover smoke against
`config/sweeps/rover_determinism_check.yaml` (5 identical 30 s
episodes):

- **Run 0 trails runs 1–4 by ~2.0 m on each rover's x-coordinate.**
  Both rovers' average speed during run 0 is ~6 % below their
  speed during runs 1–4. The likely cause is host-side cold-cache
  effects (gz, SITL, FastDDS) shifting the effective real-time
  factor of the first few seconds of the first episode.
- **Runs 1–4 are stable.** Terminal-position σ(R{0,1}_x) ≈ 0.10 m,
  σ(R{0,1}_y) ≤ 0.03 m, σ(sim_time) = 11 ms. This is well below
  the typical capture radius (0.5 m), so capture-rate ensemble
  statistics are not perturbed.

**Operational rule:** discard the first run, or run one disposable
"warmup" sweep before the real one. A `--warmup-runs N` flag is a
Stage 8 follow-up.

## Episode-side requirements

The episode referenced from the sweep:

- Must declare `duration_s`. The runner uses it to size the
  `/game/termination_event` wait. Choose a value tight enough that
  `duration_s + 30 s` covers the slowest expected capture, and
  loose enough that an interesting episode does not get cut.
- Must declare a `manifest` (typically `package://rufus_sim_worlds/...`).
  The runner reads the manifest to enumerate agents (one SITL per
  agent) and to pick a world.
- Must declare termination predicates. An episode without
  predicates is technically legal, but the runner then records
  `__no_termination__` for every run, which is rarely useful.
- Should keep `tick_rate_hz` near its default (50 Hz). Lower rates
  delay the first `GameState` past the warmup window; higher rates
  mostly cost CPU. See `episodes.md` for bounds.

The two reference episodes Stage 8 ships with:

- `rufus_sim_strategies/config/strategies/rover_capture_pp_vs_cb.yaml`:
  90 s, full episode for end-to-end demos.
- `rufus_sim_strategies/config/strategies/rover_capture_smoke.yaml`:
  30 s, used by the Stage 8 smoke sweep so a 4-run pass stays
  under 10 min wallclock.

## Smoke sweep

`config/sweeps/rover_v_max_smoke.yaml` is the reference smoke. Two
v_max values × 2 seeds = 4 runs. From a clean tree, expected:

```
[run 0000] axis=v_max=0.6 seed=0 → pursuer_win  ~78 s wallclock
[run 0001] axis=v_max=0.6 seed=1 → pursuer_win  ~78 s
[run 0002] axis=v_max=0.9 seed=0 → timeout      ~91 s
[run 0003] axis=v_max=0.9 seed=1 → timeout      ~91 s
```

The slow evader gets caught around `sim_t≈15.4 s`; the fast evader
stays just out of the 0.5 m capture radius for the full 30 s.
`format_summary` on the resulting CSV reports a 100% / 0% capture
rate split, and `plot_sweep.py` writes the two PNGs.

## Common failures

- **Every run records `__no_termination__`.** Either `duration_s`
  underestimates the slowest expected capture, or the predicates
  do not actually fire under the agents' kinematics. Re-run one
  episode by hand and watch `/game/termination_event` echo.
- **Run records READY timeout, no rows past run 0.** One adapter
  never produced its READY line. Inspect
  `runs/run_0000/bringup.log` for the per-platform pattern
  (`armed; READY`, `altitude reached`). Most often the platform
  default-params file is missing, or `MAV_GCS_SYSID` did not
  propagate (see `MEMORY.md` on the AP 4.7+ rename).
- **Capture rates drift between re-runs of the same sweep.**
  Strategy or platform state crossed a run boundary. Teardown
  unlinks `/dev/shm/fastrtps_*` and stops the ROS 2 daemon for
  exactly this reason; if you see drift anyway, check that no
  SITL or gz process survived `tear_down`. `pgrep -af 'ardurover|
  gz sim'` after a run should return empty.
- **Plotting script crashes on `import matplotlib`.** Activate
  `.venv` first: `source .venv/bin/activate`. The harness does
  not depend on matplotlib; only the plotting script does.
- **`summary.csv` has `outcome=timeout` rows you did not
  declare.** `timeout` is the built-in outcome the episode runner
  emits when `duration_s` elapses without any predicate firing. It
  is not a custom outcome. To suppress it, add an explicit
  predicate that captures the desired terminal state, or accept
  it as the natural "evader survived" label.
