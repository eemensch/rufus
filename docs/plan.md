# Development plan

Roadmap for the pursuit-evasion simulation. Captures the eight-stage
architecture from the original design, current status, and what each
upcoming stage needs to produce. Also records the recurring docs
maintenance task.

This document is forward-looking; trust the code, `git log`, and the
operational docs (`setup.md`, `operations.md`, `control.md`) for the
current ground truth.

## Stage status

`done` means infrastructure landed *and* validated by an automated
or otherwise reproducible check. `infra done` means the code and
docs landed but the stage's own exit criterion is not yet
demonstrated; the open work is itemised in the matching *Stage N
follow-ups* section and tracked as a claim in `CLAIMS.md`.

| Stage | Title                                  | Status     |
|-------|----------------------------------------|------------|
| 0     | Bootstrap (workspace, msgs)            | done       |
| 1     | Rover full fidelity                    | done†      |
| 2     | Quadrotor full fidelity                | done†      |
| 3     | Fixed-wing full fidelity               | infra done |
| 4     | Multi-instance SITL + namespacing      | done (N≤4) |
| 5     | Game engine + polynomial-predicate DSL | done       |
| 6     | Strategy interface + 1v1 baseline      | done*      |
| 7     | N-agent and mixed-team episodes        | infra done |
| 8     | Evaluation harness                     | infra done |

Reality check (audited 2026-05-18):

- `done†` (Stage 1): fidelity **resolved 2026-05-18**, though
  not the way the original plan imagined. The bench ideal was
  itself wrong (free unicycle); corrected to a Dubins car
  (GUIDED imposes a min turn radius). Evaluated correctly
  (feasible `arc_step`) the rover passes the binding overshoot
  guardrail **untuned**; the real wins were the body-frame
  setpoint reverse-bug fix and the B1 imu twist, not loop
  tuning. Residuals (≈7 s settling soft-goal; pure yaw
  infeasible via GUIDED — pivot is AUTO-only) are characterised
  and documented, not open work. See *S1.4 CLOSED* below +
  CLAIMS C4/C6/C7 + control.md.
- `done†` (Stage 2): **resolved 2026-05-18** applying the
  Stage-1 template. Bench ideal audited correct (holonomic
  multirotor); body→world rotation fix **live-validated** and
  B1 quad imu **live-verified**; quad meets a quad-appropriate
  spec untuned (only sub-spec metric was a mis-transferred
  rover settling threshold — no tune warranted). S2.4
  auditability fix applied. See *Stage 2 CLOSED* + CLAIMS
  C8/C9/C10.
- Stage **3** ("full fidelity") still `infra done`, but the
  contract is now redesigned and statically consistent end to
  end (2026-05-18): the fixed-wing `cmd_vel` is the
  Dubins-airplane control set `(V, ψ̇, climb)` constrained
  directly (adapter `dubins_airplane_clip` + ψ̇→heading over
  the GUIDED_CHANGE_* realization), the bench ideal shares that
  exact clip, `plane_twist` emits it, and `Capability` exposes
  the per-platform admissible sets (`min_turn_radius` added,
  widen-only episode override). 123/123 unit tests pass. The
  live re-run is now captured (2026-05-19,
  `scripts/sitl_run/s3_live/`): turn-rate saturation collapsed
  67–91% → 0% and mean position error dropped 2–11× vs the old
  world-velocity contract, no NAVL1/TECS tune needed. Stage 3
  is fidelity-validated (VERIFIED-MANUAL, one SITL run); a
  reproducible-harness capture rides with F0. See CLAIMS
  C11/C12/C22 and the *Fidelity completion plan*.
- Stage 1's deferred yaw work: the original "measurement bug"
  framing was substantially correct. Model check 2026-05-18
  confirms the rover is 4-wheel skid-steer (differential): the
  `r1_rover` SDF has four wheel-spin joints and no steering
  joint, and `r1_rover.param` sets SERVO1/3_FUNCTION 73/74
  (ThrottleLeft/Right). It can rotate in place, so `step_wz` is
  physically feasible and the bench's unicycle ideal is the
  right model. Resolved live 2026-05-18: the only real defect
  was the measurement gap (`twist.angular` unpopulated); fixed
  by sourcing the gyro from the `imu` plugin (S1.2). With that
  fixed, pure-yaw `(0, +0.5)` rotates in place at +0.51 rad/s
  (twist≈imu, lin.x≈0), so the GUIDED "drops yaw" theory was
  itself a measurement artifact. Both blockers closed; only
  the S1.4 loop tune remains.
- Stage 4 is validated only to N=4. Stage 7's 3v3 (6-agent) live
  smoke was never executed (DDS multicast collapse on the host).
- Stage 6 (`done*`): unit tests pass (18) but every 1vN smoke is
  a one-off manual run recorded in prose, not a reproducible
  check. Same for Stage 7's smokes.
- Stage 8: only the 4-run smoke is validated; the 100-run Monte
  Carlo is gated on the RTF follow-up and the unimplemented
  `--warmup-runs` flag.

Test inventory (measured 2026-05-18, `pytest`):

| Suite                              | Tests | Status |
|------------------------------------|-------|--------|
| rufus_sim_game/test_predicate_engine  | 26    | pass   |
| rufus_sim_game/test_agent_params      | 15    | pass   |
| rufus_sim_strategies/test_reference   | 18    | pass   |
| rufus_sim_adapters (rover/quad/plane  | 36    | pass   |
|   pure helpers + rover_setpoint)   |       |        |
| rufus_sim_adapters (Node/bring-up)    | 0     | none   |
| rufus_sim_eval/test_step_response     | 6     | pass   |
| rufus_sim_eval (metrics/runner/sweep) | 0     | none   |

The full claim ledger, with per-claim status and gap-closing
steps, is `CLAIMS.md` at the project root. Keep it synchronised
with this file at the end of every session.

## Done: environment renamed to "Rufus" (2026-05-19)

The simulation environment was renamed from `pe_sim_*` /
`pursuitEvasionSim` to **Rufus**, an homage to Rufus Isaacs
(differential game theory). The name was chosen for namespace
cleanliness in the control / DG literature: "Isaacs" collides
with NVIDIA Isaac Sim/Lab and "Barrier" with Control/Lyapunov
Barrier Functions; "Rufus" has neither clash and is a clean ROS
prefix.

Carried out as a single deliberate migration into a fresh
clean-slate repo at `~/gitRepos/iman/rufus` (fresh `git init`,
no history): the seven colcon packages `pe_sim_*` →
`rufus_sim_*` (dirs, inner modules, ament resource markers, the
`rufus_sim_worlds` env-hook) and every `pe_sim` content
reference and doc. The `chain.py` ready/`pkill` regexes key on
adapter node names, not the package prefix, so they were
unaffected. Persistent memory was moved to the new project
path in lock step. The two `pe_sim` strings remaining in this
paragraph are intentional rename provenance, not a missed
substitution.

## Fidelity completion plan (Stages 1-3)

The exit criterion for each of Stages 1-3 is a tracking-error
report against a faithful kinematic ideal, captured live with the
control loop actually tuned. Today the ideals are faithful but no
loop is tuned and no live report exists. This section is the
ordered work to close that. Numbers below are targets, not
measured results; record measured values in `CLAIMS.md` as they
land.

Platform structure (re-corrected 2026-05-18, firmware-grounded
— this REVERSES the earlier "unicycle ideal is correct, no
Dubins-car reframing" note): the rover *vehicle* is skid-steer
(physically unicycle-capable, can pivot), but the **GUIDED
control path this stack uses imposes Dubins-car kinematics**.
ArduRover `TURN_RADIUS` (0.9 m) enforces a minimum turn radius
in steering/GUIDED mode, so the achievable yaw rate is
speed-coupled, |ω| ≤ v/R_min, and pure in-place yaw (v=0) is
infeasible from GUIDED — true pivot-in-place is an AUTO-mode /
`AR_WPNav` feature unreachable from the velocity/turn-rate
setpoints we send. So the rover-as-driven ideal in
`rover_bench.py` must be a **Dubins car with R_min =
TURN_RADIUS**, not a free unicycle (corrected in code
2026-05-18). Net: rover and fixed-wing are the *same*
curvature-limited Dubins class as driven (|ω|≤v/R_min vs
ψ̇≤lat_accel/V); the quad is the holonomic exception. This
also vindicates the original audit's "rover ≈ Dubins" framing
for the control path. The ~10 PID iterations and the amplitude
test all targeted the wrong subsystem: the yaw-fidelity limit
is this architectural min-radius/pivot-mode-gating, not a rate
loop tune.

Shared precondition (do once, before any stage):

- **F0. Reproducible bench harness.** The three `scripts/*_bench.py`
  are manual and their outputs are pasted into prose. Wrap each in
  a thin pytest-xfail-tolerant launcher that brings the chain up
  headless, runs the trajectories, and asserts the report against
  the stage's numeric exit criterion. Until a bench is reproducible
  its "pass" cannot be a claim, only an anecdote. This is the
  single highest-leverage item; everything below feeds it.

### Stage 1 — rover (skid-steer / unicycle)

Gap: control loops untuned; yaw rate not measurable; one open
question about GUIDED yaw behaviour. The kinematic ideal is
already correct (unicycle = differential-drive kinematics) and
needs no change.

1. **S1.2 Surface actual yaw rate (Blocker 1 — the only real
   prerequisite; do this first).** The adapter copies
   `local_position/velocity_body` into `AgentState.twist`
   (rover_adapter.py:151-153, 482-483); MAVROS's
   `local_position` plugin fills only `.linear`, so
   `twist.angular.z` is structurally zero and no yaw-tracking
   error is measurable. Primary fix: add the `imu` plugin to
   `rufus_sim_bringup/config/mavros_pluginlists.yaml` and source
   `twist.angular.z` from `mavros/imu/data`
   (`angular_velocity.z`, body-frame gyro). One extra plugin is
   acceptable at the Stage 1 bench's N=1; the multi-agent CPU
   note in the pluginlist memory does not apply here. Fallbacks
   if `imu/data` is unsuitable: (a) `local_position/odom`
   `twist.twist.angular.z` (same plugin, already allowlisted,
   FCU ATTITUDE-sourced); (b) a wrap-safe, dt-guarded τ≈0.1 s
   low-pass derivative of `local_position/pose` yaw. Verify
   sign and scale against the ENU body convention on a live
   turn before wiring it in. Add a pure-Python adapter unit
   test for the ω extractor plus `_body_to_world` and `_clamp`
   (first `rufus_sim_adapters` test; starts closing C20).
2. **S1.1 Verify pure-yaw end-to-end (Blocker 2 is not a
   blocker; runs after S1.2).** Full source trace settles it,
   no chain bring-up needed for the diagnosis: MAVROS
   `setpoint_velocity` 2.14.0 sends
   `SET_POSITION_TARGET_LOCAL_NED` with
   `type_mask = (1<<10)|(7<<6)|(7<<0)` (position, accel, yaw
   ignored; velocity valid; yaw_rate valid, fed `angular.z`
   for the unstamped topic). For `(vx=0, wz≠0)` ArduRover
   (`GCS_MAVLink_Rover.cpp:770-772`) takes branch 2 →
   `set_desired_turn_rate_and_speed(turn_rate, 0)`; the GUIDED
   `TurnRateAndSpeed` submode (`mode_guided.cpp`) then runs
   `get_steering_out_rate(...)` + `set_steering(...)` with
   `calc_throttle(0)`, which for a skid-steer mixer is in-place
   rotation. No type_mask, plugin, or firmware block exists and
   `GUID_OPTIONS` is irrelevant for pure yaw. Verified live
   2026-05-18 (S1.1 probe: cmd +0.5 → +0.51 measured, in-place
   rotation). Note: the later S1.4 baseline found a *separate*
   translation defect (forward command drove the rover
   backward) that did require a setpoint-path change — the
   `setpoint_raw/local` BODY_NED migration, previously thought
   optional, is now the implemented fix. See the S1.4 item and
   the reverse-bug follow-up below. Pure-yaw itself needed no
   setpoint change and still works under the new path (speed 0
   + turn rate).
3. **S1.3 Dubins-car ideal (DONE 2026-05-18; reverses the
   earlier "keep the unicycle ideal").** The bench integrator
   now clips the ideal yaw rate to |ω| ≤ |vx|/R_MIN
   (R_MIN = TURN_RADIUS = 0.9) — a Dubins car, because GUIDED
   imposes a min turn radius (firmware-verified). `step_vx`,
   `sin_vx`, `circle`, `lemniscate` retained. `step_wz`
   (vx=0, ω≠0) is **kinematically invalid** for the GUIDED
   path (radius 0 vs R_min 0.9) and is retained only as a
   documented infeasible-region probe (the Dubins ideal now
   predicts ~0 yaw for it). The valid yaw-fidelity test is the
   new `arc_step` (v=1.0, ω 0→1.0 → radius 1.0 ≥ R_min). The
   bench takes `--r-min` so the ideal matches the live
   `TURN_RADIUS` under test.
   **Baseline captured 2026-05-18 (pre-tune, no gains
   touched).** It surfaced a blocker before any tuning: on
   `step_vx`, a forward command drove the rover *backward*
   (cmd +0.5 → steady actual −0.5, heading held). Root cause
   (firmware): ArduRover infers drive direction from
   `is_negative(packet.vx)` = NED-north sign; an east-facing
   rover commanded via world-frame `setpoint_velocity`
   reverses. Fixed in code: rover adapter now publishes
   `setpoint_raw/local` `PositionTarget` in `MAV_FRAME_BODY_NED`
   (same velocity-level type_mask 1479), so the sign tests
   body-forward. Unit-tested (`rover_setpoint`); **live
   re-baseline is the next action before tuning.** `step_wz`
   measured cleanly and is the yaw-tune starting point:
   rise 0.60 s (ok), ss_error 0.041 (ok), but overshoot 61.7%
   and settling 6.66 s — grossly under-damped (add damping,
   lower P/FF; do not chase the goal with aggressive gains).

4. **S1.4 Tune.** Speed loop (`ATC_SPEED_*`, `CRUISE_*`)
   against `step_vx`/`sin_vx`: **DONE — passes spec untuned**
   (the real fidelity win was the body-frame setpoint
   reverse-bug fix, not gains). Yaw loop: **abandoned as the
   wrong subsystem.** ~10 conservative + beyond-conservative
   `ATC_STR_RAT_*`/FLTT/IMAX iterations could not meet the yaw
   spec; firmware analysis then showed why — GUIDED enforces a
   min turn radius (`TURN_RADIUS`) and pivot is AUTO-only, so
   the yaw-fidelity limit is architectural, not a rate-loop
   tune. The yaw acceptance metric below moves from the
   invalid `step_wz` to the feasible **`arc_step`** (v=1.0,
   ω 0→1.0, radius ≥ R_min).

   **S1.4 CLOSED 2026-05-18 (user-agreed).** `TURN_RADIUS`
   sweep 0.9→0.3→0.1: `step_wz` invariant (~32% overshoot —
   `TURN_RADIUS` cannot rescue in-place yaw; vx=0 is radius 0 <
   any R_min; pivot is AUTO-only). On the feasible `arc_step`
   the rover **passes the ≤15% overshoot hard guardrail
   untuned** (13.0%, rise 0.42 s, ripple 0.016,
   `TURN_RADIUS`-invariant); ss_error 0.054 (marginal) and
   settling ~7 s (slow-tail soft-goal) are the only residuals,
   both characterised. The "yaw fidelity gap" was a
   `step_wz` (infeasible-command) test artifact; the ~10 PID
   iterations chased it. `TURN_RADIUS` reverted to upstream
   0.9. **Final rover tune = `WP_SPEED` + `ATC_STR_RAT_FF=0.330`.**
   Lasting wins: B1 imu twist, body-frame setpoint reverse-bug
   fix, Dubins-car bench ideal + `arc_step` (+ first
   rufus_sim_adapters tests, `rufus_sim_eval.step_response`).
   Operational rule (control.md): strategies must command
   `v ≥ |ω|·R_min`; true pivot needs the AUTO path (unused).
   Numbers in CLAIMS C4; C6 (Dubins yaw capability) and C7
   (`step_wz` retired) follow from this.

   **Acceptance spec (locked 2026-05-18, conservative, ±5%
   settling band).** Primary, command-tracking step response
   (actual vs commanded). Yaw column now applies to the valid
   `arc_step` (v=1.0, ω→1.0 rad/s); the old `step_wz` basis
   was kinematically invalid (vx=0):

   | Metric                | Speed (step_vx→0.5) | Yaw (arc_step→1.0) |
   |-----------------------|----------------------|----------------------|
   | Rise (10–90%)         | ≤ 1.5 s              | ≤ 1.0 s (goal)       |
   | Overshoot             | ≤ 10%                | ≤ 10%                |
   | Settling (±5%)        | ≤ 2.0 s              | ≤ 1.0 s (goal)       |
   | Steady-state error    | ≤ 0.05 m/s           | ≤ 0.05 rad/s         |
   | Steady-state ripple σ | ≤ 0.02 m/s           | ≤ 0.03 rad/s         |

   "Goal" = subordinate to the stability guardrails; a
   well-damped 1.5 s response beats an oscillatory 1.0 s one.
   Secondary (relative, bounded): `sin_vx` mean |v_err| ≤
   0.10 m/s; `circle` r≈1 m radius error ≤ 10%, no spiral;
   `lemniscate` stable. Global anti-overfit: no trajectory's
   mean or max error may regress vs the pre-tune baseline.
   Hard guardrails (reject regardless of error numbers):
   sustained oscillation/limit cycle; overshoot > 15% on any
   step; saturation-flag flapping; not repeatable across 2
   runs within SITL jitter. Method: baseline first; one
   parameter per iteration, step ≤ ~1.3×, feedforward/damping
   before P/I; revert any net regression; stop when met;
   iteration cap ≤ 5 per loop, then report rather than
   escalate gains. Metrics computed by
   `rufus_sim_eval.step_response` (landed 2026-05-18, unit-tested
   against closed-form responses; no chain needed).

Exit: `rover_bench` summary captured live with the tuned loop;
speed- and yaw-tracking targets met; numbers written to the
Stage 1 claim in `CLAIMS.md`.

### Stage 2 — quadrotor — CLOSED 2026-05-18 (user-agreed)

Resolved the same shape as Stage 1: the bench ideal was
audited and is correct (holonomic multirotor — no Rover-style
min-radius constraint; `step_wz` pure-yaw-at-hover is feasible
and valid here, unlike the rover), and the fidelity wins were
structural, not loop tuning.

- **S2.1 (done).** one_quad baseline live. Body→world rotation
  fix **live-validated**: `step_vx` max pos err 0.51 m, final
  0.026 m, ss vel err 0.0007 m/s — the 5.66 m bug is gone, the
  ~0.6 m floor met (closes the long-deferred offline-only
  Stage-2 #1 claim, CLAIMS C8). B1 quad imu **live-verified**:
  `step_wz` `actual_wz_body` tracks cmd 1.0 to 0.0025, right
  sign/scale.
- **S2.2 (done, firmware).** Authoritative current PSC groups
  (`AC_PosControl.cpp`): `PSC_NE_VEL_*`, `PSC_NE_POS_*`,
  `PSC_D_VEL_*`, `PSC_D_POS_*`. **The old plan text here was
  backwards** — `PSC_NE_VEL_*/PSC_D_VEL_*` ARE the post-4.7
  names (NE/D convention); the genuinely-old pre-rename names
  were `PSC_VELXY_*`/`PSC_VELZ_*`. Confirm live before any
  future tune (defaults are conditionally compiled).
- **S2.3 (no tune — deliberate).** Baseline on stock
  `copter.parm`: `step_wz`/`step_vz` pass the full spec with
  large margin; `step_vx` passes 4/5 (overshoot 7.1%, ss
  0.0007, rise 0.60, ripple 0.0017); the lone miss is settling
  3.54 s vs the **rover-derived** ≤2.0 s — mis-transferred.
  Per the Stage-1 lesson, no PSC tune: the loop is already
  well-damped; chasing a mis-transferred target risks the
  overshoot/chatter pathology. Quad-appropriate settling
  target is the documented characterisation, not a gain change.
- **S2.4 (done).** `rufus_sim_bringup/config/copter_tune.parm`
  sets `ATC_RATE_Y_MAX=90` (= the adapter's existing effective
  fallback → behaviour-neutral) so `Capability.source` is
  auditable (firmware `RATE_Y_MAX` default is 0="Disabled").
  Follow-up: chain.py should chain `copter_tune.parm` for the
  eval harness (mirrors `r1_rover_tune.parm`).

Outcome: speed/vertical/yaw loops meet a quad-appropriate spec
untuned; structural fixes (body→world, B1) were the win.
Numbers in CLAIMS C8/C9/C10.

### Stage 3 — fixed-wing (Dubins airplane)

Gap: contract redesigned and statically consistent end to end
(adapter `(V,ψ̇,climb)` + `dubins_airplane_clip` + ψ̇→heading;
bench ideal shares the clip; `plane_twist`; `Capability`
admissible sets); 123/123 unit tests pass. Remaining: no live
re-run; L1/TECS untuned; post-4.4 plane modes not addressable
by name. The earlier "does the bench read `twist.angular` or
derive heading-rate from pose" question is resolved — the
contract is now `(linear.x=V, angular.z=ψ̇, linear.z=climb)` and
the bench commands ψ̇ directly, the ideal integrates that same
ψ̇.

1. **S3.1 Live re-run with the exact Dubins-airplane ideal —
   DONE 2026-05-19** (`scripts/sitl_run/s3_live/`, one_plane
   chain via chain.py). Mean position error, old world-velocity
   contract → new `(V,ψ̇,climb)`: level_cruise 37.86→16.81 m,
   loiter 103.62→17.03 m, climb_descent 215.79→37.74 m,
   infeasible_zero 21.95→1.93 m; new `turn_rate_step` 17.80 m.
   Turn-rate saturation 67–91% → 0% everywhere. Four
   orchestration bugs were found and fixed driving the chain
   outside batch_runner (self-matching pkill; GZ_SIM_RESOURCE_
   PATH overwrite; bench .venv interpreter; bench↔namespaced-
   adapter remap) plus one stale `gamma=` kwarg in the bench;
   see `operations.md` "Common bring-up failures".
2. **S3.2 Tune L1 and TECS — NOT NEEDED.** The S3.1 residual
   (~17–38 m mean, 0% turn/airspeed/climb saturation on the
   feasible trajectories) is closed-loop realization lag, not a
   mistuned loop; per the Stage-1 minimal-tuning lesson no
   `NAVL1_*`/`TECS_*` change was applied. Revisit only if a
   reproducible-harness capture shows the residual is loop-bound.
3. **S3.3 Plane-mode-name table.** Either carry the downstream
   MAVROS patch so `LOITER`/`TAKEOFF` are addressable by name,
   or explicitly record "FBWA + RC override only" as an
   accepted Stage 3 limitation in `control.md` and close the
   item. Do not leave it implicitly open.

Exit: MET 2026-05-19 (VERIFIED-MANUAL, one SITL run).
`fixed_wing_bench` report captured live; no L1/TECS tune
needed; infeasible-command behaviour confirmed against live
saturation flags (`infeasible_zero` ~98% airspeed sat, tracks
the shared-clip ideal to 1.93 m); numbers in CLAIMS C11/C12.
Residual exit item: a reproducible-harness (F0) capture so the
table is a check, not a one-run recollection.

### Sequencing

F0 (reproducible harness) is the only hard blocker. For
Stage 1 the order is fixed: S1.2 (measurement) first, then the
S1.1 verification probe — S1.1 cannot be evaluated until ω and
pose are trustworthy. S2.1/S3.1 are cheap and surface
surprises early; run them in parallel with S1.2. The three
tunes (S1.4, S2.3, S3.2) are independent once their benches
are reproducible. End with docs sync and claim closure.

## Stage 1 follow-ups (not blocking)

Identified during the rover tracking benchmark as gaps in the
default ArduRover tune. They do not block Stage 2 but should be
revisited before Stage 6 (strategy baselines), since strategy gain
choices depend on a faithful kinematic ideal.

- ~~Raise `WP_SPEED` from 1.0 m/s.~~ Done.
  `rufus_sim_bringup/config/r1_rover_tune.parm` overrides upstream
  `r1_rover.param` with `WP_SPEED = 2.0` — chained last in the
  SITL `--defaults` list via the new `extra_defaults_local`
  field on `rufus_sim_eval.chain._PLATFORM_SITL[rover]`.
  `Capability.v_max` is read off `WP_SPEED`, so strategies now
  have 2 m/s headroom.
- **Yaw-rate measurement gap — resolved live 2026-05-18.** The
  Stage 1 #1 plan was to drive down `step_wz` (vx=0, wz=1.0) ω
  error by tuning `ATC_STR_RAT_*`. Root cause was a single
  defect, the measurement gap: MAVROS `local_position` leaves
  `twist.angular` zero, so the adapter copied a structurally
  zero yaw rate into `AgentState`. Fixed by sourcing the gyro
  from the `imu` plugin (S1.2). The "GUIDED drops a standalone
  yaw_rate" theory was disproved: the full trace (MAVROS 2.14.0
  `setpoint_velocity` → `GCS_MAVLink_Rover.cpp:770-772`
  branch 2 → `mode_guided.cpp` TurnRateAndSpeed → skid mixer)
  is structurally sound, and on the live one_rover chain
  `(0, +0.5)` rotates in place at +0.51 rad/s (twist≈imu,
  lin.x≈0). `GUID_OPTIONS` was never relevant (only bit 6 =
  SCurves exists). The identical `local_position/velocity_body`
  copy defect was present in `quad_adapter` and
  `fixed_wing_adapter`; all three now source `twist.angular`
  from `imu/data` (quad/plane fix covered by unit tests; their
  live re-verification rides with S2.1 / S3.1). No repo
  contradiction exists: `chain.py`'s `rover-skid.parm` and
  `control.md`'s "skid-steer" label are both accurate. Only
  the S1.4 loop tune now remains for Stage 1; the measured
  input is a multi-second yaw-loop lead-in and ~2% steady
  error vs command (target settling < 1 s).
- **Forward command drove the rover backward — fixed in code
  2026-05-18 (S1.4 baseline finding).** The pre-tune baseline
  showed `step_vx` cmd +0.5 → steady actual −0.5, heading
  held. Root cause (firmware, `GCS_MAVLink_Rover.cpp:740-748`):
  ArduRover infers drive direction from `is_negative(packet.vx)`
  where `packet.vx` is the NED-*north* component; the rover
  spawns facing east, so the north component is ≈0 and its
  sign is noise → `speed_dir = −1` → reverse. It is triggered
  because MAVROS `setpoint_velocity` sends in world
  `LOCAL_NED`. Fix: the rover adapter now publishes
  `mavros/setpoint_raw/local` `PositionTarget` with
  `MAV_FRAME_BODY_NED` (same velocity-level type_mask 1479;
  `rover_setpoint()`), so the sign tests body-forward. One
  uniform mapping covers straight, arc and spin-in-place
  (ArduRover `set_desired_turn_rate_and_speed`). `setpoint_raw`
  added to the MAVROS allowlist; `_body_to_world`/`_clamp`
  folded out; 6 `rover_setpoint` unit tests added. Pure-yaw
  (S1.1) unaffected and still works. General bug, not
  bench-only: any non-north-heading rover was affected, so the
  Stage-6/7 1vN smokes (C14/C15) are unreliable until re-run.
  **Live re-baseline pending before S1.4 tuning.**
- **Speed-loop tune blocked on the same lack of stable test
  bed.** A trial run with `ATC_SPEED_FF: 0 → 0.4` and
  `ATC_SPEED_P: 0.63 → 0.8` was attempted; result: mean position
  error on `step_vx` improved 40 % (0.20 → 0.12 m) but final
  position error 4 × worse (0.10 → 0.38 m) and steady-state
  velocity error worsened 30 % — net regression. Reverted. The
  upstream `ATC_SPEED_FF = 0` is genuinely too low for the
  sinusoidal trajectory (`sin_vx` mean velocity error 0.27 m/s
  on 0.4 m/s amplitude), but tuning without the angular
  measurement above is not productive. Pick this up after the
  state-pipe and yaw-test fixes land.

## Stage 2 follow-ups (not blocking)

Identified during the quad tracking benchmark and bring-up
debugging. They do not block Stage 3 (fixed-wing) but should be
revisited before Stage 6 (strategy baselines), since horizontal
tracking error currently dominates the bench numbers.

- ~~Diagnose the body/world yaw convention discrepancy.~~ Done.
  Root cause: MAVROS `apm_config.yaml` configures the
  `setpoint_velocity` plugin with `mav_frame: LOCAL_NED`, which
  is a *world*-frame in MAVLink terminology (LOCAL relative to
  the EKF home). The quad and rover adapters were publishing
  body-frame `cmd_vel.linear` directly to
  `mavros/setpoint_velocity/cmd_vel_unstamped` without
  rotation, so the FCU interpreted body-x as world-east. With
  iris spawned at yaw=90° (body-x = north) the bench's body
  command `(0.5, 0, 0)` became "0.5 m/s east" inside ArduCopter,
  while the kinematic ideal correctly rotated by yaw to predict
  "0.5 m/s north". After 8 s the diagonal divergence is
  `sqrt(4^2 + 4^2) = 5.66 m`, matching the 5.6 m bench number
  to <2 %. Fix: both quad and rover adapters now rotate
  `linear.x`/`linear.y` into world ENU using the latest yaw
  from `local_position/pose` before publishing to MAVROS.
  Validated offline with the rotation-math check (yaw=0, ±90°,
  +45° body cases all give the expected world-frame output to
  six decimals). A live re-run of the quad bench is needed to
  confirm the step_vx error drops to the predicted ~0.6 m
  velocity-magnitude floor; deferred to whoever picks up the
  iris position-controller tune below, since the live bench is
  the natural input to that work.
- Tune the iris position-controller bandwidth (`PSC_NE_VEL_*`,
  `PSC_D_VEL_*`) once the yaw issue is resolved, targeting <0.05
  m/s velocity error and <0.5 m position error on the steady
  portion of `step_vx`.
- Set `ATC_RATE_Y_MAX` explicitly in `copter.parm` rather than
  relying on the firmware default (`0` is interpreted as ~90
  deg/s but the adapter currently falls back to a hard-coded
  value, which the `Capability.source` audit field cannot
  distinguish from a true FCU value).

## Stage 2 — Quadrotor full fidelity

Full-physics multirotor under ArduCopter SITL + gz Harmonic, with
an adapter mirroring the rover and adding vertical-velocity and
yaw-rate semantics.

**Deliverables.**

- Vehicle: `iris_with_ardupilot` from `ardupilot_gazebo` is the
  default candidate. Use the existing `iris_runway.sdf` world.
- Stabilise the bring-up that we left flaky in Stage 0: the
  `lock_step=1` in the model SDF mismatched arducopter's
  `no_lockstep` mode, causing time drift and auto-disarm. Two
  approaches:
  1. Edit `iris_with_ardupilot/model.sdf` to set `lock_step=0` so
     both ends agree on free-running time.
  2. Configure ArduCopter to honour lock-step (set parameter
     `SIM_LOCKSTEP=1` if available; otherwise patch SIM_JSON).

  Decide before writing the adapter; the answer affects whether the
  adapter needs a custom rate-keeping loop.
- `rufus_sim_adapters/quad_adapter.py`:
  - `cmd_vel`: `linear.x/y/z` body-frame velocity, `angular.z` yaw
    rate. Forward to `/mavros/setpoint_velocity/cmd_vel_unstamped`.
  - Capability source: `ATC_RATE_Y_MAX` → `yaw_rate_max`,
    `ANGLE_MAX` (centideg) → `bank_angle_max` →
    `lateral_accel_max = g·tan(bank_angle_max)`. `WPNAV_SPEED`,
    `WPNAV_SPEED_UP`, `WPNAV_SPEED_DN` set v_max and vz limits.
  - Bring-up: GUIDED is enough for outdoor (with simulated GPS);
    GUIDED_NoGPS is the variant if we move to indoor / vision-only
    later.
  - Add `quad_adapter` entry point in `setup.py`.
- Tracking-error report: same five trajectories as the rover plus
  a sixth `step_vz` (vertical step) and a seventh `helix`
  (constant `vx`, `vz`, `wz`) to exercise vertical and yaw-coupled
  tracking.

**Exit criterion.** Tracking-error report analogous to the rover's
documenting where the single-integrator-plus-yaw kinematic ideal is
faithful and where it diverges.

**Open questions.**

- Iris (1.5 kg, conservative tune) vs a faster custom multirotor for
  pursuit dynamics. Default tune may cap pursuit-evasion fidelity;
  consider Stage 6 a forcing function.
- Outdoor with simulated GPS vs indoor with mocap-style position.
  Affects which GUIDED variant the adapter targets.

**Doc updates required.** `setup.md` (add quad adapter, iris model
notes), `operations.md` (quad bring-up sequence, lock_step tweak if
needed), `control.md` (quad column in cmd_vel table, quad section
under Capability semantics).

## Stage 3 follow-ups (not blocking)

Identified during the fixed-wing bench. They do not block Stage
4 (multi-instance) but should be revisited before Stage 6
(strategy baselines), since the current bench's kinematic ideal
overstates "tracking error" for a bank-to-turn airframe.

- ~~Replace the simple single-integrator ideal in
  `scripts/fixed_wing_bench.py` with a Dubins-airplane ideal
  that respects the bank-rate cap.~~ Done, then superseded
  2026-05-18 by the contract redesign. The bench ideal is now
  the *exact* Dubins-airplane kinematic model — state
  `(x,y,z,ψ)`, control `(V, ψ̇, climb)` — clipped by the
  adapter's own `dubins_airplane_clip` (imported, single source
  of truth, so ideal and adapter cannot drift). ψ is the
  integral of the commanded ψ̇, not a slew toward a
  velocity-vector heading. 5 trajectories: `level_cruise`,
  `loiter` (constant ψ̇), `climb_descent`, `turn_rate_step`
  (new ψ̇-step probe), `infeasible_zero`. The old
  single-integrator geometric yaw-to-align cost (78–298 m peak)
  is gone by construction; the live residual will be ArduPlane
  closed-loop realization lag plus the adapter's ψ̇→heading
  round trip. A live re-run under the zephyr stack is the
  precondition for the L1 / TECS tune below.
- Tune the L1 / TECS gains (`NAVL1_PERIOD`, `NAVL1_DAMPING`,
  `TECS_*`) once the bench reports faithful errors. Targets:
  steady-state airspeed within 0.5 m/s of cruise; loiter radius
  within 5 m of the bank-rate-implied value.
- Patch MAVROS's plane-mode-name table (or carry a downstream
  patch) so `TAKEOFF` and other post-4.4 modes can be
  addressed by name. The Stage 3 adapter sidesteps this by
  using `FBWA` + RC override, but a future strategy that wants
  to invoke `LOITER`, `TAKEOFF`, etc. by name will hit the
  same `Unknown mode` rejection.

## Stage 3 — Fixed-wing full fidelity

Full-aero ArduPlane SITL with the Zephyr delta wing or SITL_Models
alternative; adapter projects `TwistStamped` onto (airspeed,
heading, climb) per the Dubins-airplane convention.

**Superseded 2026-05-18 (kept for planning history).** The
"incoming `linear` is a velocity vector, project onto
(airspeed, heading, climb)" design below was discarded. The
shipped contract is the Dubins-airplane *control set*
`(linear.x=V, angular.z=ψ̇, linear.z=climb)` commanded and
constrained directly (`dubins_airplane_clip`), with ψ̇
integrated to a heading target over the
`MAV_CMD_GUIDED_CHANGE_*` realization — because ArduPlane's
`SET_POSITION_TARGET_LOCAL_NED` is a verified altitude-only
no-op for velocity. The adapter *does* command the turn rate
directly (it is a differential-game input, not something to
hide behind bank-to-turn). Authoritative description:
`docs/control.md` "Admissible control set (per platform)" and
CLAIMS C22; the bullets below are historical.

**Deliverables.**

- Vehicle: `zephyr_with_ardupilot` from `ardupilot_gazebo`, or a
  SITL_Models fixed-wing if the dynamics are richer.
- `rufus_sim_adapters/fixed_wing_adapter.py`:
  - `cmd_vel` interpretation: incoming `linear` is the desired
    velocity vector; project onto `V ∈ [v_min, v_max]` (clip
    magnitude), heading from `atan2(linear.y, linear.x)` with
    bounded turn rate `g·tan(bank_max)/V`, climb from `linear.z`
    bounded by climb-angle limit. Reject hover commands (V→0)
    gracefully: clamp to `v_min`, raise saturation flag.
  - Capability source: `ROLL_LIMIT_DEG`, `AIRSPEED_MIN`,
    `AIRSPEED_CRUISE`, `AIRSPEED_MAX`, `PTCH_LIM_MAX_DEG`,
    `PTCH_LIM_MIN_DEG`. `NAVL1_PERIOD/DAMPING` informs achievable
    closed-loop turn rate.
  - Use ArduPlane GUIDED with `SET_POSITION_TARGET_LOCAL_NED`
    velocity setpoints; verify firmware accepts our setpoint
    convention.
- Fixed-wing-specific benchmarks: level flight at cruise, fixed-bank
  loiter at multiple radii, climb/descent step, infeasible-command
  test (commanded `V=0` should saturate).

**Exit criterion.** Tracking bound vs Dubins-airplane kinematic
ideal; documented behaviour on infeasible commands (saturation
asserted on `SaturationFlags.airspeed`/`turn_rate`/`climb_rate`,
motion follows projected feasible setpoint).

**Open questions.**

- ArduPlane GUIDED velocity-setpoint support is firmware-version
  sensitive. If the in-tree firmware refuses our convention, fall
  back to position-target setpoints with synthesised waypoints.
- Roll-vs-yaw coordination: ArduPlane handles bank-to-turn; the
  adapter shouldn't try to command yaw directly.

**Doc updates required.** `setup.md` (fixed-wing adapter, zephyr
notes), `operations.md` (fixed-wing bring-up), `control.md`
(fixed-wing column, projection semantics, saturation cases).

## Stage 4 — Multi-instance SITL + namespacing (next)

Launch N agents (mixed types) in a single world, each with its own
SITL instance, MAVROS, and adapter, controllable in parallel via
`/<ns>/cmd_vel`.

**Deliverables.**

- `rufus_sim_bringup` package with launch file taking a YAML manifest
  (per-agent: `id`, `type`, `role`, `spawn_pose`, `instance`).
- ArduPilot's `-I <instance>` flag drives port offsets (5760 +
  10·instance). Manifest assigns DDS namespaces (`/R0`, `/E0`,
  `/P0`, ...).
- 4-agent demo: 2 rover, 1 quad, 1 fixed-wing, controllable from
  4 separate strategy nodes simultaneously.

**Exit criterion.** 4 mixed agents commandable independently with no
cross-talk; demonstrated by injecting `/<ns_i>/cmd_vel` and
verifying only agent *i* moves.

**Open questions.**

- Single gz world with all agents (simpler for shared physics,
  collisions, observation) vs one gz instance per agent (better
  scaling, isolated faults). Decide on tick-rate measurements at
  N=4, 8.
- TF tree: per-agent TF prefix (`R0/base_link`, ...) under a shared
  `map`.

**Doc updates required.** `setup.md` ("Adding a new agent"
section — fill in real recipe), `operations.md` (multi-agent
bring-up — replace the "planned" placeholder), `control.md`
(multi-agent section — replace placeholder).

## Stage 4 follow-ups (not blocking)

- **Runtime SDF templating for dynamic N.** Stage 4 task #32 lands
  build-time generation: a Python generator reads
  `config/agents/<scenario>.yaml` at colcon-build time and emits
  per-instance model dirs and the world SDF into the install tree.
  Adding or removing an agent therefore requires `colcon build`.
  A future iteration can move the generation into the launch file
  (or a small helper run before launch) so the manifest can be
  swapped without rebuild — closer to the aau-cns
  `Ardupilot_Multiagent_Simulation` pattern (github.com/aau-cns),
  which rewrites the world SDF at launch time. Defer
  until either (a) Stage 7 N-agent episodes need to vary N at run
  time, or (b) a strategy/RL loop wants to spawn agents
  dynamically. Build-time generation is the right default for the
  fixed 2- and 4-agent demos in Stage 4.

## Stage 5 — Game engine + polynomial-predicate DSL (done)

`rufus_sim_game` package: loads episode YAML, applies per-agent FCU
parameter overrides, drives each agent to its configured initial
position during a warmup phase, and then evaluates polynomial
termination predicates each tick. Publishes `GameState` per tick;
emits `RoleAssignment` (TRANSIENT_LOCAL) once every agent's
`ready_when` has held for its dwell — that latch is the hand-off
signal to strategies. Emits `TerminationEvent` on the first
satisfied termination predicate or on `duration_s` timeout.

**Deliverables landed.**

- `rufus_sim_game/predicate_engine`: sympy-based parser with strict
  AST allowed-node set, `Poly` polynomial-form check, lambdify to
  numpy, per-predicate `DwellTimer`. v1 grammar accepts a single
  inequality; boolean composition (`&`, `|`) is reserved for v2
  and slots into the same allowed-node set.
- `rufus_sim_game/episode_runner`: phase machine (`param_setup` →
  `warmup` → `running` → `terminated`), per-platform warmup
  driver for rover and quad, world-pose override from gz
  `dynamic_pose/info` so multi-agent positions are in a common
  frame.
- `rufus_sim_game/agent_params`: per-platform translation table from
  high-level Capability fields (`v_max`, `yaw_rate_max`, ...) to
  ArduPilot FCU parameter names; `fcu:` escape hatch for
  parameters outside the canonical menu. Reference reproduced in
  `docs/episodes.md`.
- Predicate test suite (22 cases at Stage 5 close; 26 as of
  2026-05-18 after the v2 additions) covering capture, disk
  exit, region entry, cone (polynomial via squared cosine),
  dwell semantics, every grammar-rejection path.
- 1v1 smoke test against `two_rovers_minimal` world: dummy
  pure-pursuit pursuer + capture predicate fires
  `outcome=pursuer_win` in ~7 s sim time.
- `docs/episodes.md`: full DSL spec, schema, phase semantics,
  per-platform parameter reference.

## Stage 5 follow-ups (not blocking)

- **Capability resync race.** The runner republishes
  `/<id>/capability` on the same topic the adapter latched at
  startup; with two TRANSIENT_LOCAL publishers, a new
  subscriber can receive both messages and must pick the one
  with the later timestamp. v2: have the adapter expose a
  refresh service the runner calls so only one latched message
  exists at a time.
- **World-frame velocity in predicates.** `vx, vy, vz` are
  body-frame; expressing world-frame speeds is non-polynomial
  (needs `cos(psi)`/`sin(psi)`). A runner-side derivative of
  position would expose world-frame velocities as additional
  components — fine in the polynomial grammar.

## Stage 6 — Strategy interface + 1v1 baseline (done)

`rufus_sim_strategies` package with a control-systems-flavoured
Strategy ABC: each tick the runtime hands the strategy a
`Measurement` (agents-by-id, my_state, my_capability,
sim_time_s, active_predicates, episode_id) and the strategy
returns a `Twist` on `/<agent_id>/cmd_vel`. Reference
implementations cover the memoryless pattern
(`PurePursuitPursuer`, `ConstantBearingEvader`) and the
stateful pattern (`LeadPursuer`, which finite-differences
target velocity from the previous Measurement).

**Deliverables landed.**

- `rufus_sim_strategies/strategy.py`: `Strategy` ABC and
  `Measurement` dataclass. Methods are
  `__init__(agent_id, params)`, optional `reset()`, and
  abstract `control(measurement) -> Twist`.
- `rufus_sim_strategies/registry.py`: in-process name→class map
  with `register(name, cls)` / `get(name)`. Reference
  strategies register via side effect on import.
- `rufus_sim_strategies/heading.py`: shared per-platform
  `rover_twist`/`quad_twist`/`plane_twist` helpers so all
  reference strategies share one Twist-frame implementation.
- `rufus_sim_strategies/strategy_runner.py`: per-agent ROS node
  hosting one `Strategy`. Hand-off contract: silent until
  `/game/role_assignments` latches *and* the next
  `/game/state` arrives; then `reset()` once, then `control`
  on every subsequent `/game/state`; `Twist` returned is
  header-stamped and published on `/<agent_id>/cmd_vel`.
- Episode YAML extension: per-agent `strategy: {type, params}`
  block consumed by
  `rufus_sim_strategies/launch/episode_with_strategies.launch.py`,
  which spawns the gz pose bridge, episode_runner, one
  strategy_runner per agent, and an optional `rosbag2 record`.
- 18 unit tests (`test_reference.py`) covering the three
  reference strategies across rover/quad/plane platforms +
  the stateful contract on `LeadPursuer` (finite-difference,
  state persistence, target-lateral-motion steering).
- 1v1 smoke against the two_rovers chain: PurePursuitPursuer
  vs ConstantBearingEvader fires
  `outcome=evader_win, predicate_id=evader_escape` at sim_t
  ≈ 51 s under default seed (the evader's bearing_offset=π
  combined with v_factor=0.95 vs pursuer's 1.0 produces a
  closure rate of ~0.05 m/s, slow enough that the evader
  reaches the 50 m disk first).

## Stage 6 follow-ups (not blocking)

- **Strict bag-hash determinism.** SITL is not strictly
  deterministic at speedup > 1, and the FastDDS scheduling
  jitter makes bit-equal bags unlikely even at speedup = 1.
  A determinism-check task should benchmark trajectory drift
  across seeds, then either pin chain configs that converge
  (lock-step + speedup = 1) or change the exit criterion to
  trajectory-similarity rather than bag-hash equality.
- **Quad-vs-quad and plane-vs-plane smoke — done.**
  `rufus_sim_strategies/config/strategies/quad_capture_pp_vs_cb.yaml`
  and `plane_capture_pp_vs_cb.yaml`, run against the
  `two_quads` and `two_planes` manifests respectively, exercise
  the `PurePursuitPursuer.{quad,plane}` and
  `ConstantBearingEvader.{quad,plane}` branches end-to-end. The
  quad smoke fires `outcome=evader_win` at sim_t≈35 s under
  default seed; the plane smoke runs to `outcome=timeout` at
  sim_t=120 s (capture geometry harder for fixed-wing in 1v1
  with equal speeds — fine for the smoke goal of "chain runs
  cleanly"). Mixed-platform episodes (rover vs quad, quad vs
  plane) remain a future-curiosity item but no longer block
  Stage 7.
- **External strategy registration.** v1 registry is
  in-process; v2 should switch to Python entry_points so
  external packages can ship strategies without editing
  rufus_sim_strategies.
- **Bag replay into a fresh chain.** `ros2 bag play` replays
  the recorded `/<id>/cmd_vel` topics, but to reproduce the
  trajectory the chain (gz, SITL, MAVROS) must be deterministic
  enough to match the original — see the determinism follow-up
  above. A standalone "replay verifier" launch that compares
  the replayed `/game/state` against the recorded one would
  close this loop.

## Stage 7 — N-agent and mixed-team episodes (done)

Composition over Stages 4–6 plus a narrow predicate-engine
extension. Three new manifests under
`rufus_sim_worlds/config/agents/` (`three_rovers`, `four_agents`
re-used, `six_rovers`) and three new strategy episodes under
`rufus_sim_strategies/config/strategies/` (`rover_2v1`,
`mixed_team_2v1`, `rover_3v3`).

**Deliverables landed.**

- **Predicate v2 grammar.** `&`, `|`, `~` over leaf
  inequalities. Engine recurses through `sympy.And`/`Or`/`Not`,
  applies the polynomial-form check per leaf. Atomic
  predicates still work unchanged. The predicate suite is 26
  tests (measured 2026-05-18; the earlier "41" in this file was
  never accurate) covering the v1 grammar plus four v2 cases
  (AND, OR, NOT, multi-pursuer disjunction).
- **2v1 rovers** — three_rovers manifest + rover_2v1 episode.
  Two pursuers, one evader, capture predicate is the
  v2 disjunction over the two (pursuer, evader) pairs. Smoke
  fires `predicate_id=capture, outcome=pursuer_win` at
  sim_t≈31 s.
- **Mixed team 2v1** — uses the existing four_agents manifest.
  R0 (rover) + Q0 (quad) chase R1 (rover); P0 (plane) is a
  bystander (no `strategy:` block ⇒ no strategy_runner
  spawned). Capture predicate is a v2 disjunction over a
  rover-rover 2D-distance leaf and a quad-rover 3D-distance
  leaf. Smoke fires
  `predicate_id=capture, outcome=pursuer_win` at sim_t≈16 s
  (rover capture wins; quad branch not exercised by the
  termination but the chain runs cleanly).
- **3v3 rovers** — six_rovers manifest + rover_3v3 episode
  with a 9-leaf disjunction capture and a 3-leaf conjunction
  evader-escape. Manifest, world SDF, and episode YAML all
  build cleanly. Live smoke deferred — the 6-SITL +
  6-MAVROS + 6-adapter chain stays alive but DDS multicast
  discovery becomes unreliable on the 14-core host (the same
  sub-process spam observed mid-Stage 6 #49 returns), and
  `gz model` queries time out under the resulting load. The
  predicate engine and v2 grammar are validated by the
  smaller-N smokes; the 6-agent capacity question is a
  separate environmental fix tracked under the determinism
  follow-up below.

## Stage 7 follow-ups (not blocking)

- **3v3 (and larger N) live capacity.** The 6-agent chain runs
  out of headroom on the current host. Two paths: (a) reduce
  per-agent CPU further (drop more MAVROS plugins; smaller
  setpoint pipeline; rover-only adapter that bypasses MAVROS
  for cmd_vel forwarding), or (b) split the chain across
  hosts (one DDS domain per gz instance), which requires
  picking a network setup that survives the LOCALHOST
  vs SUBNET multicast confusion observed earlier in this
  session.
- **Multi-threat evader strategy.** `ConstantBearingEvader`
  takes a single `threat` parameter today. Multi-threat 3v3
  episodes assign each evader a single pursuer, which is fine
  for the smoke goal but doesn't reflect real multi-evader
  evasion. A `MultiThreatEvader` (or a parameter-list `threats:`
  for the existing class) is a small addition.

## Stage 8 — Evaluation harness (done)

Headless batch runner, parameter sweeps, capture-rate and
time-to-capture metrics with statistical aggregation.

**Deliverables (shipped).**

- `rufus_sim_eval` package with `batch_runner`: spawns N episodes
  from a sweep specification, collects `TerminationEvent` plus
  trajectory bag (`config/sweeps/`, `rufus_sim_eval/{sweep,chain,
  batch_runner,metrics}.py`).
- Headless run profile: `gz -s --headless-rendering`, MAVROS launched
  without the rosout-side GUI, full chain torn down between runs
  (`rufus_sim_eval/chain.py`).
- Metrics: capture rate, mean / median / 95th-percentile
  time-to-capture, terminal position distribution per agent
  (`rufus_sim_eval/metrics.py`).
- Summary CSV plus plotting script (`scripts/plot_sweep.py`).
- Reference smoke: `rover_v_max_smoke.yaml` (4 runs, ~6 min)
  reproduces the expected capture-rate split between v_max=0.6
  (100%) and v_max=0.9 (0%).
- Documentation: `docs/evaluation.md`, `setup.md` "Running
  evaluations" cross-link.

**Exit criterion (met for the 4-run smoke; deferred for 100-run).**
The harness runs the smoke sweep end-to-end and emits a complete
`summary.csv`; the 100-run Monte Carlo target is gated on the
RTF-parameterisation follow-up below, since at gz wallclock pace
even a 30 s episode × 100 runs is ~4 hours.

## Stage 8 follow-ups (not blocking)

- **Honour `speedup` end-to-end.** The world SDFs generated by
  `rufus_sim_worlds` hard-code `<real_time_factor>1.0</...>`, so under
  SITL lock-step gz steps at wallclock pace and `ardurover --speedup`
  has no practical effect. The runner therefore sizes its
  `/game/termination_event` wait as `duration_s + 30 s` regardless
  of the sweep's `speedup` field. Parameterising the world template
  by RTF (and, separately, re-checking determinism at higher RTF)
  is the gate on a 100-run Monte Carlo finishing in under an hour.
- **Multi-axis plotting.** `plot_sweep.py` plots only the first
  axis when a sweep declares more than one. A 2D heatmap (capture
  rate vs axis pair, evader speed × pursuer gain being the
  obvious first cut) is the natural extension; it is a one-script
  addition and does not need any runner changes.
- **Determinism re-verification — measured.** Ran
  `config/sweeps/rover_determinism_check.yaml` (5 nominally
  identical 30 s rover episodes, axes empty). Two regimes
  observed:
  1. **First-run penalty.** Run 0 trails runs 1–4 by
     ~2.0 m on each rover's x-coordinate. Both rovers' average
     speed in run 0 is ~6 % below their speed in runs 1–4.
     Almost certainly host-side cold-cache effects (gz physics,
     SITL, FastDDS) shifting gz's effective real-time factor
     during early steps. Fix: run one warmup episode and
     discard, or add a `--warmup-runs N` flag to the harness.
  2. **Steady-state drift.** Across runs 1–4 the terminal
     positions are stable: σ(R0_x) = 0.11 m, σ(R1_x) = 0.10 m,
     σ(R{0,1}_y) ≤ 0.03 m, σ(sim_time) = 11 ms. Capture-rate
     statistics over the 0.5 m capture radius are unaffected
     at this jitter level.
  Conclusion: cross-run determinism is *not* bit-exact at
  speedup = 1.0 (FastDDS scheduling and gz step jitter), but
  is bounded enough for Monte Carlo ensemble metrics provided
  the first run is discarded. Recommended harness change:
  add a `--warmup-runs` flag.
- **Bag-based offline analysis.** The runner records a rosbag per
  run (`runs/run_NNNN/bag/`), but no analysis script consumes it.
  A "trajectory plotter" that reads the bag and overlays per-agent
  paths colour-coded by outcome is the obvious add. Out of scope
  for the harness itself; it lives next to `plot_sweep.py`.

## Cross-stage dependencies

```
Stage 0 -> 1, 2, 3
Stage 1 -> 4 (rover adapter is the adapter template)
Stage 2 -> 4 (needs quad)
Stage 3 -> 4 (needs fixed-wing)
Stage 4 -> 5, 6, 7
Stage 5 -> 6
Stage 6 -> 7, 8
```

Stages 2 and 3 can run in parallel after Stage 1. Stage 4 can begin
once any one of {2, 3} is complete (start with two vehicle types,
add the third later). Stages 5 and 6 are mostly independent; their
integration is the 1v1 demo at Stage 6.

## Recurring task: keep operational docs current

Every commit that introduces a new operational element should update
the relevant doc in the same commit. Failing to do this is the
fastest way to make `setup.md`, `operations.md`, and `control.md`
silently rot.

| New element introduced                       | Doc(s) to touch              |
|----------------------------------------------|------------------------------|
| New vehicle type or new agent                | setup.md, operations.md, control.md |
| New launch file or env var                   | operations.md                |
| New topic, message field, or QoS quirk       | control.md                   |
| New common failure mode                      | operations.md (Common bring-up failures) |
| Architectural shift (e.g. switch RMW)        | all three                    |
| New dependency (apt / pip / external repo)   | setup.md (Prerequisites)     |
| New build artefact location                  | setup.md (Repository layout) |
| New episode YAML field or predicate component| episodes.md                  |
| New high-level Capability option (any platform)| episodes.md (Agent parameter reference) and rufus_sim_game/agent_params.py |
| New strategy or change to Strategy ABC       | strategies.md                |
| New sweep YAML field, runner flag, or output column | evaluation.md         |

**Acceptance check before merging any stage:** `grep` the diff for
new topic names, parameters, env vars, and binary paths; confirm each
appears in the relevant doc, or open a follow-up issue with a
`docs:` label.

## Stage-completion conventions

When a stage closes:

1. Mark its row in *Stage status* as `done`.
2. Record any deferred work as a *Stage N follow-ups* section in
   this file (analogous to "Stage 1 follow-ups" above).
3. Update the relevant operational docs.
4. Commit with subject `Stage N: <one-line summary>` and a body
   listing exit criteria met and open follow-ups.
5. Run the SITL chain end-to-end once more from a clean shell to
   confirm `setup.md` + `operations.md` are still accurate.
