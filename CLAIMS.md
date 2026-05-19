# Claims ledger

Tracks every material claim the project makes about its own
behaviour and the status of the evidence behind it. The analogue of
a proof here is a reproducible check, not a theorem. Keep this file
synchronised with `docs/plan.md` at the end of each session.

A claim is **closed** only when (1) there are no remaining gaps and
no open gap-closing steps, and (2) the user has explicitly agreed.
No claim is closed yet.

## Status legend

- `VERIFIED-AUTO`   passing automated test committed in the repo.
- `VERIFIED-MANUAL` shown once by a manual run, recorded in prose,
                    not reproducible in CI. SITL is not bit-exact,
                    so a manual smoke is an anecdote, not a proof.
- `OPEN`            asserted as a target or expectation; not
                    demonstrated.
- `CONTRADICTED`    an in-repo inconsistency must be resolved
                    before the claim can even be stated.

Audited 2026-05-18.

## Engine and logic (Stages 5-6)

### C1. Predicate DSL parses, validates, compiles, dwell-tracks

Status: VERIFIED-AUTO.
Evidence: `rufus_sim_game/test_predicate_engine.py`, 26 tests pass.
Covers v1 inequalities, v2 boolean composition, per-leaf
polynomial enforcement, every grammar-rejection path, dwell
semantics.
Gap: none known. Closure needs user agreement only.

### C2. High-level → FCU parameter translation is correct

Status: VERIFIED-AUTO.
Evidence: `rufus_sim_game/test_agent_params.py`, 19 tests pass.
Now covers the rover `min_turn_radius`→`TURN_RADIUS` option:
`translate` write, the widen-only guard in
`apply_high_level_to_capability` (≥ native OK, = native OK,
< native raises `ParameterOverrideError`).
Gap: tests assert the translation table, not that the resulting
FCU writes produce the intended envelope on a live vehicle. That
coupling is exercised by C4/C8/C11, not here.

### C3. Reference strategies behave per-platform

Status: VERIFIED-AUTO.
Evidence: `rufus_sim_strategies/test_reference.py`, 19 tests pass
(pure-pursuit, constant-bearing, lead-pursuit; rover/quad/plane;
stateful contract on LeadPursuer). The plane branch now emits
the Dubins-airplane control contract `(V, ψ̇, climb)`: dead-ahead
→ zero turn rate; off-axis and flee → `k_psi·err` clipped to the
speed-coupled `lateral_accel_max/V` cap (no world-velocity
vector, no backward airspeed).
Gap: unit-level only; end-to-end behaviour is C13-C15.

## Rover fidelity (Stage 1)

### C4. Rover tracks the kinematic ideal within a documented bound

Status: CLOSED 2026-05-18 (user-agreed). The "kinematic ideal"
was itself wrong and is now redefined (Dubins car); evaluated
correctly the rover passes the binding guardrail untuned. Final
rover tune = `WP_SPEED` + `ATC_STR_RAT_FF=0.330` (TURN_RADIUS
reverted to upstream 0.9).
Wins (kept): (B1) `AgentState.twist.angular.z` now carries the
gyro (imu plugin); matches `/mavros_0/imu/data` to <0.01 rad/s.
(Frame fix) rover adapter → `setpoint_raw/local`
`MAV_FRAME_BODY_NED` (type_mask 1479): forward command no
longer drives the rover backward (was a NED-north `speed_dir`
artifact). `step_vx` passes the locked spec **untuned** — the
real fidelity win.
Re-corrected 2026-05-18 (supersedes the prior C4 text and the
"unicycle ideal is correct" line): the rover *vehicle* is
skid-steer but the **GUIDED control path imposes Dubins-car
kinematics** — `TURN_RADIUS`=0.9 m enforces a min turn radius
in steering/GUIDED, |ω|≤v/R_min, and pivot-in-place is
AUTO/`AR_WPNav`-only, unreachable from our velocity/turn-rate
setpoints. So `step_wz` (vx=0, ω≠0) is **kinematically
invalid** (radius 0 vs R_min 0.9): the S1.1 "it rotates"
observation was real but it is the out-of-model regime whose
pathology (~0.7 s dead time, 33% overshoot, 0.2 rad/s far
worse, un-settleable) we mis-spent ~10 PID iterations chasing.
The bench ideal is now a Dubins car (R_min, code-corrected),
and the yaw metric moves to the feasible `arc_step`.
Resolution (TURN_RADIUS sweep, 2026-05-18): `step_wz` (vx=0)
invariant 0.9→0.3→0.1 (~32% overshoot, ~6.3 s) — TURN_RADIUS
cannot rescue in-place yaw (radius 0 < any R_min; GUIDED never
pivots; pivot is AUTO-only). On the feasible `arc_step`
(v=1, ω=1, radius 1.0) the rover passes the ≤15% overshoot
hard guardrail **untuned**: overshoot 13.0%, rise 0.42 s,
ripple 0.016 (TURN_RADIUS-invariant); ss_error 0.054 (marginal
vs 0.05) and settling ~7 s (the documented slow-tail soft-goal
miss) are the only residuals, both characterised. The ~10 PID
iterations chased an artifact of the kinematically-invalid
`step_wz`. Outcome: speed loop passes untuned; feasible-yaw
passes the binding guardrail untuned; lasting wins are B1
(imu twist), the body-frame setpoint fix, and the corrected
Dubins-car ideal + `arc_step`. Operational constraint
(documented in control.md): strategies must command
`v ≥ |ω|·R_min`; true pivot needs the AUTO path (not used).
No open gap; superseded by the resolved understanding.

### C5. The rover is 4-wheel skid-steer (differential)

Status: VERIFIED (model-checked 2026-05-18); closure pending
user agreement.
Evidence: the `r1_rover` model SDF (SITL_Models source and the
project's spawned `r1_rover.sdf.in`) has exactly four joints,
all `motor_N` revolute wheel-spin joints (axes `0 ±1 0`), and
no vertical-axis steering joint; the ArduPilotPlugin drives the
two left wheels on channel 0 and the two right wheels on
channel 2; `r1_rover.param` sets SERVO1/3_FUNCTION 73/74
(ThrottleLeft/Right), FRAME_CLASS 1. This is the canonical
ArduRover skid-steer configuration. The earlier "Ackermann"
premise was a tentative user hypothesis, now refuted. The repo
(`chain.py` `rover-skid.parm`, `control.md` "skid-steer") was
correct; no contradiction exists.
Gap: none. Closure needs user agreement only.

### C6. `Capability.yaw_rate_max` is well-defined for the rover

Status: CONTRADICTED → re-defined 2026-05-18 (firmware-grounded;
this reverses the prior "no redefinition needed, Dubins-car
withdrawn" text and re-vindicates the original audit framing).
Evidence: the *vehicle* can pivot, but in the GUIDED control
path the rover uses, `TURN_RADIUS` (0.9 m) makes the effective
yaw-rate limit **speed-coupled and Dubins**: |ω| ≤ v/R_min.
`ATC_STR_RAT_MAX` (≈2.09 rad/s) is only a hard ceiling; the
binding constraint at the speeds strategies use is v/R_min
(e.g. v=0 → 0; v=0.5 → 0.56 rad/s). A standalone body-rate
`yaw_rate_max` is therefore the wrong model for the rover as
driven; `agent_params.py`/`control.md`/`Capability` should
expose the curvature limit (R_min) like the fixed-wing's
ψ̇ = lat_accel/V, not a constant.
Gap / steps: redefine the rover yaw capability as the Dubins
curvature limit in `agent_params.py` + `control.md` +
`episodes.md` (mirrors the fixed-wing). Tracked with C4; the
`TURN_RADIUS` sweep informs the R_min value.

### C7. `step_wz` (yaw at zero forward speed) is a usable test

Status: INVALID test — retired 2026-05-18 (this reverses the
earlier "VERIFIED usable test"). `step_wz` commands vx=0 with
ω≠0, i.e. turn radius 0, which the GUIDED min-radius model
(`TURN_RADIUS`=0.9 m) cannot represent — pivot-in-place is an
AUTO/`AR_WPNav` feature unreachable from the velocity/turn-rate
setpoints this stack sends. The rover *does* spin under
`(vx=0, wz=0.5)` (S1.1 observed it), but that is the
out-of-model regime, and its symptoms scale the wrong way with
amplitude (0→0.2 rad/s: ~2.2 s dead time, 52% overshoot, never
settles, 21% steady error — far worse than the 1.0 step), the
signature of commanding an infeasible region, not loop
behaviour. The Dubins-car bench ideal now correctly predicts
~0 yaw for vx=0, so `step_wz`/`step_wz_lo` are kept only as
documented infeasible-region probes.
Replacement: the valid yaw-fidelity test is **`arc_step`**
(v=1.0 m/s, ω 0→1.0 → radius 1.0 ≥ R_min), inside the GUIDED
Dubins envelope; the S1.4 yaw spec is evaluated on it.
Gap: none for the (now-retired) claim. The real yaw question
moves to C4 (arc_step vs Dubins ideal across the `TURN_RADIUS`
sweep).

## Quadrotor fidelity (Stage 2)

### C8. Body→world rotation fix removes the yaw-convention error

Status: VERIFIED live 2026-05-18 (S2.1).
Evidence: one_quad chain, `quad_bench` `step_vx`: max pos err
0.51 m, **final 0.026 m**, steady velocity error 0.0007 m/s.
The prior 5.66 m diagonal-divergence bug is gone; the ~0.6 m
floor is met. The long-deferred Stage-2 #1 claim (previously
offline-only) is now live-confirmed.
Gap: none. Closure needs user agreement only.

### C9. Quad tracks the single-integrator+yaw ideal within bound

Status: CLOSED 2026-05-18 (user-agreed) — passes a
quad-appropriate spec untuned; the ideal is correct (audited:
holonomic multirotor, no Rover-style constraint).
Evidence: S2.1 one_quad baseline, stock `copter.parm`.
`step_wz` 1.3% overshoot / 0.78 s settle / ss 0.0025 (B1 quad
imu live-verified: `actual_wz_body` tracks cmd 1.0 to 0.0025,
right sign/scale). `step_vz` 5.4% / 1.68 s / ss 0.0002 — full
pass. `step_vx` 7.1% / ss 0.0007 / rise 0.60 / ripple 0.0017
— passes 4/5; only settling 3.54 s exceeds the **rover-derived**
≤2.0 s, a mis-transferred target. Dynamic trajectories all
sub-0.5 m vs the holonomic ideal.
Resolution: no PSC tune warranted (Stage-1 lesson: don't chase
a mis-transferred spec with aggressive gains on an already
well-damped loop). The quad's horizontal-velocity-step
settling target should be multirotor-appropriate; loop quality
(overshoot/ss/ripple) is excellent. Fidelity wins were
structural (C8 body→world, B1 imu), as in Stage 1.

### C10. `Capability.yaw_rate_max` source is auditable for the quad

Status: RESOLVED 2026-05-18 (S2.4); live-confirm opportunistic.
Evidence: firmware `AC_AttitudeControl` `RATE_Y_MAX` default
**0.0 = "Disabled"** confirmed — so the adapter's hard-coded
~90 deg/s fallback was indeed unauditable via
`Capability.source`. Fix: `rufus_sim_bringup/config/copter_tune.parm`
sets `ATC_RATE_Y_MAX = 90` (= the current effective fallback →
behaviour-neutral; baseline yaw already validated under it)
so the FCU now reports a real value. chained into the quad
SITL defaults.
Gap / steps: chain.py should chain `copter_tune.parm` for the
eval harness (follow-up, mirrors `r1_rover_tune.parm`);
confirm `Capability.source` shows 90 on the next quad chain
run (behaviour-neutral, low priority).

## Fixed-wing fidelity (Stage 3)

### C11. Dubins-airplane ideal is faithful

Status: VERIFIED-MANUAL (live run captured 2026-05-19); ideal
rewritten 2026-05-18.
Evidence: the ideal is the exact Dubins-airplane kinematic
model — state `(x,y,z,ψ)`, control `(V, ψ̇, climb)` — clipped by
the SAME `dubins_airplane_clip` the adapter applies (imported
from `fixed_wing_adapter`, so ideal and adapter cannot drift).
ψ is the integral of the commanded ψ̇, not a slew toward a
velocity-vector heading. Live `fixed_wing_bench` re-run
(`scripts/sitl_run/s3_live/`, one_plane chain via chain.py),
mean position error vs the kinematic ideal — old world-velocity
contract (`s3_base`) → new `(V,ψ̇,climb)` contract:
level_cruise 37.86→16.81 m, loiter 103.62→17.03 m,
climb_descent 215.79→37.74 m, infeasible_zero 21.95→1.93 m;
new `turn_rate_step` probe 17.80 m. The residual is ArduPlane
TECS/L1 closed-loop lag plus the adapter's ψ̇→heading round
trip; the single-integrator geometric yaw-to-align cost is gone
by construction.
Gap / steps: one manual SITL run, not bit-exact (legend); a
reproducible-harness capture rides with F0. Closure needs user
agreement.

### C12. Fixed-wing tracks within airspeed/turn/climb bounds

Status: VERIFIED-MANUAL (live run 2026-05-19); no tune needed.
Resolved: the prior open sub-question (does the bench read
`twist.angular` or derive heading-rate from pose) no longer
applies — the contract is body-frame `(linear.x=V,
angular.z=ψ̇, linear.z=climb)`; the bench commands ψ̇ directly
and the ideal integrates that same ψ̇. B1 `twist.angular`
structural-zero fix (2026-05-18, `imu/data`-sourced,
unit-tested) stands.
Live evidence (`scripts/sitl_run/s3_live/`): turn-rate
saturation collapsed 67–91% → **0%** on every trajectory (old
contract chronically exceeded `g·tan(φ)/V` by deriving heading
from a velocity vector; the new contract commands feasible ψ̇
directly). Airspeed/climb saturation 0% on the feasible
trajectories; `infeasible_zero` correctly holds ~98% airspeed
saturation (commanded V=0 → clipped to v_min) yet tracks the
shared-clip ideal to 1.93 m mean. No NAVL1_*/TECS_* tune was
applied or needed (Stage-1 minimal-tuning lesson holds): the
residual is closed-loop realization lag, not a mistuned loop.
Gap / steps: one manual SITL run, not bit-exact; pin under the
F0 reproducible harness so the table is a check, not a
recollection. Closure needs user agreement.

### C13. Post-4.4 plane modes addressable by name

Status: OPEN (currently false; FBWA + RC override only).
Gap / steps: task S3.3. Either carry the MAVROS mode-table
patch or record "FBWA + RC override only" as an accepted
limitation in control.md and close the item explicitly.

### C22. Each platform's contract is its kinematic-model control set

Status: VERIFIED-AUTO (static/contract layer) + VERIFIED-MANUAL
(fixed-wing live, 2026-05-19; see C11/C12).
Claim: every adapter's `cmd_vel` contract is exactly the
control-input set of its differential-game kinematic model,
constrained directly, so a min-max can be solved over the
admissible set with no carrot/heading-hold/velocity-vector
indirection. Rover = Dubins car `(v, ω)`, `|ω|≤|v|/R_min`;
quad = holonomic box; fixed-wing = Dubins airplane
`(V, ψ̇, climb)`, `V∈[v_min,v_max]`, `|ψ̇|≤g·tan(φ_max)/V`,
`|climb|≤V·sin(γ)` (all speed-coupled).
Evidence: the fixed-wing was reworked end-to-end 2026-05-18 —
adapter `_project`/`dubins_airplane_clip` + ψ̇→heading
integrator over the GUIDED_CHANGE_* realization;
`fixed_wing_bench` ideal sharing the same clip; `plane_twist`
emitting `(V, ψ̇, climb)`; `Capability.min_turn_radius` added
(rover `TURN_RADIUS`, widen-only episode override);
`docs/control.md` "Admissible control set (per platform)" table
maps each `cmd_vel` field → bound → `Capability` field → source
FCU parameter. 123/123 tests pass (game, strategies, adapters,
eval). The ArduPlane `SET_POSITION_TARGET` velocity-setpoint
stub finding (silent no-op) is the realization-layer basis.
Gap / steps: the contract is statically consistent across the
stack and the 2026-05-19 live run confirms the realization
tracks the commanded `(V, ψ̇, climb)` with 0% turn-rate
saturation (C11/C12). The ψ̇→heading integrator in
`_setpoint_tick` is still not unit-tested, and the
`DubinsAirplaneIdeal` constructor was untested — a stale
`gamma=` kwarg from the increment-2 rewrite slipped past
`py_compile` and only failed at live bench start (fixed
2026-05-19); both tracked in C20.

## Episode / strategy end-to-end (Stages 6-7)

### C14. 1v1 smokes fire the documented outcomes

Status: VERIFIED-MANUAL.
Evidence (prose in plan.md): rover PP-vs-CB → evader_win
≈51 s; quad → evader_win ≈35 s; plane → timeout @120 s.
Gap / steps: pin as launch tests asserting `outcome` (not exact
sim_t, which SITL jitter makes irreproducible). Until then this
is an anecdote per run.

### C15. NvM smokes fire the documented outcomes

Status: VERIFIED-MANUAL.
Evidence (prose): 2v1 rovers → pursuer_win ≈31 s; mixed 2v1
→ pursuer_win ≈16 s.
Gap / steps: same as C14 (pin as launch tests).

### C16. The 6-agent (3v3) chain runs

Status: OPEN.
Evidence: never executed; DDS multicast discovery collapses at
6 agents on the 14-core host; `gz model` queries time out.
Gap / steps: Stage 7 follow-up. Reduce per-agent CPU or split
DDS domains across hosts, then run the 3v3 smoke.

## Evaluation harness (Stage 8)

### C17. 4-run smoke reproduces the capture-rate split

Status: VERIFIED-MANUAL.
Evidence (prose): v_max 0.6 → 100 %, v_max 0.9 → 0 %.
Gap / steps: re-run under the F0 reproducible harness so the
split is a check, not a recollection.

### C18. A 100-run Monte Carlo is feasible

Status: OPEN.
Evidence against: world SDFs hard-code `real_time_factor=1.0`;
under lock-step `--speedup` is inert; 100×30 s ≈ 4 h wall.
Gap / steps: Stage 8 follow-up. Parameterise the world template
by RTF; re-check determinism at higher RTF.

### C19. Ensemble metrics are stable enough despite SITL jitter

Status: VERIFIED-MANUAL (partial).
Evidence: 5-run determinism check; runs 1-4 σ(R_x) ≤ 0.11 m,
σ(sim_time) = 11 ms; run 0 carries a ~2 m cold-cache penalty.
Gap / steps: the recommended `--warmup-runs N` mitigation is
documented but **not implemented** in `batch_runner.py`. Until
it is, every ensemble silently includes the biased run 0.

## Test-coverage claims

### C20. The adapters are correct

Status: OPEN (helper coverage across all three adapters as of
2026-05-18; 53 tests pass).
Evidence: `test_rover_adapter.py` (20: `_yaw_from_quat`,
`body_to_world_xy`, `clamp_command`/saturation, and
`rover_setpoint` — type_mask 1479, BODY_NED frame, reverse-sign
passthrough, clamp/saturation for the C4 fix),
`test_quad_adapter.py` (8: `_yaw_from_quat`, `body_to_world_xy`),
`test_fixed_wing_adapter.py` (25: `_yaw_from_quat`,
`_wrap_angle`, `enu_yaw_to_compass_deg`, `guided_alt_from_climb`,
and `dubins_airplane_clip` — the differential-game admissible
set, incl. the speed-coupled traps where the ψ̇ and climb caps
must use the *clipped* V). The fixed-wing `_project` math is now
the pure `dubins_airplane_clip` and is tested. Pure logic
extracted from Node methods for testability. Untested: the
bring-up state machines, the quad `_clamp` math, MAVROS service
plumbing, the ψ̇→heading integrator in `_setpoint_tick`.
Gap / steps: extract and test quad `_clamp` (2-norm horizontal
+ asymmetric vertical + yaw); the Node bring-up/setpoint paths
(incl. the ψ̇ integrator) need a chain-level or mocked-MAVROS
harness, tracked with the F0 reproducible-bench item.

### C21. The evaluation harness is correct

Status: OPEN (first coverage landed 2026-05-18).
Evidence: `rufus_sim_eval/test/test_step_response.py` (6 tests,
pass) pins `step_response.py` against closed-form responses
(first-order rise=τ·ln9, settling=τ·ln20, exact 20% overshoot,
never-settle offset, negative setpoint, CSV pulse detection).
`metrics.py`, `batch_runner.py`, `sweep.py`, `chain.py` remain
untested.
Gap / steps: unit-test `metrics.py` (capture rate, percentiles,
terminal distribution) against synthetic TerminationEvent
fixtures; that needs no chain bring-up.
