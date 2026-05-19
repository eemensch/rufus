"""Step-response metrics for the rover S1.4 loop tune.

`rover_bench.py` records per-sample (t, cmd, actual) but only
summarises mean/max/final tracking error. The S1.4 acceptance
spec (plan.md / CLAIMS C4) is stated in classic step-response
terms — rise time, overshoot, settling time, steady-state error
and ripple — so this module turns a step trace into exactly
those numbers.

Pure and dependency-light (stdlib only) so it is unit-testable
without a ROS chain. Thresholds live in the criteria, not here:
this module measures, the caller judges.

Conventions (locked 2026-05-18):
  - rise time is 10%->90% of the commanded step, referenced to
    the pre-step baseline;
  - settling band is +/-5% of the commanded setpoint
    (`band_frac=0.05`);
  - steady state is the mean/stdev over the last `ss_window_s`
    of the step.
"""

from __future__ import annotations

import csv
import math
import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class StepMetrics:
    rise_time: float | None        # s, None if 10/90 not reached
    overshoot_pct: float           # % beyond setpoint, >=0
    settling_time: float | None    # s from onset, None if never
    settled: bool
    ss_value: float                # mean over the last ss window
    ss_error: float                # |ss_value - setpoint|
    ss_ripple: float               # stdev over the last ss window
    peak: float


def analyze_step(
    times: list[float],
    values: list[float],
    setpoint: float,
    t_onset: float,
    t_end: float,
    *,
    baseline: float | None = None,
    band_frac: float = 0.05,
    ss_window_s: float = 2.0,
) -> StepMetrics:
    """Compute step-response metrics over the window
    [`t_onset`, `t_end`].

    `setpoint` is the commanded level during the step. `baseline`
    defaults to the first sample at or after `t_onset` (the
    bench steps up from rest, so this is the pre-step value).
    Works for negative steps too (the 10/90 references follow the
    signed span).
    """
    win = [(t, v) for t, v in zip(times, values)
           if t_onset <= t <= t_end]
    if len(win) < 2:
        raise ValueError("step window has < 2 samples")
    win.sort(key=lambda p: p[0])
    wt = [p[0] for p in win]
    wv = [p[1] for p in win]

    if baseline is None:
        baseline = wv[0]
    span = setpoint - baseline
    eps = 1e-9

    # --- rise time: signed 10% -> 90% of the commanded span
    if abs(span) < eps:
        rise_time: float | None = 0.0
    else:
        sgn = 1.0 if span > 0 else -1.0
        lo = baseline + 0.1 * span
        hi = baseline + 0.9 * span
        t_lo = next((t for t, v in win
                     if (v - lo) * sgn >= 0), None)
        t_hi = (None if t_lo is None else
                next((t for t, v in win
                      if t >= t_lo and (v - hi) * sgn >= 0), None))
        rise_time = (None if (t_lo is None or t_hi is None)
                     else t_hi - t_lo)

    # --- overshoot: furthest excursion past setpoint, in the
    # direction of the step (0 if it never exceeds setpoint)
    sgn = 1.0 if span >= 0 else -1.0
    peak = max(wv) if span >= 0 else min(wv)
    denom = abs(setpoint) if abs(setpoint) > eps else abs(span)
    overshoot_pct = max(0.0, (peak - setpoint) * sgn / denom * 100.0) \
        if denom > eps else 0.0

    # --- settling: last exit from the +/-band, then stays in
    band = band_frac * (abs(setpoint) if abs(setpoint) > eps
                        else abs(span))
    last_out = None
    for i, (t, v) in enumerate(win):
        if abs(v - setpoint) > band:
            last_out = i
    if last_out is None:
        settling_time: float | None = 0.0
        settled = True
    elif last_out == len(win) - 1:
        settling_time = None
        settled = False
    else:
        settling_time = wt[last_out + 1] - t_onset
        settled = True

    # --- steady state: mean/stdev over the last ss_window_s
    ss = [v for t, v in win if t >= t_end - ss_window_s]
    if len(ss) < 2:
        ss = wv[-2:]
    ss_value = statistics.fmean(ss)
    ss_ripple = statistics.pstdev(ss)
    ss_error = abs(ss_value - setpoint)

    return StepMetrics(
        rise_time=rise_time,
        overshoot_pct=overshoot_pct,
        settling_time=settling_time,
        settled=settled,
        ss_value=ss_value,
        ss_error=ss_error,
        ss_ripple=ss_ripple,
        peak=peak,
    )


def _detect_step(times: list[float], cmd: list[float],
                 eps: float = 1e-6) -> tuple[float, float, float]:
    """Infer (setpoint, t_onset, t_end) from a pulse command
    column: first sample where |cmd| rises above `eps`, the level
    it holds, and the first later sample where it returns to ~0.
    """
    on = next((i for i, c in enumerate(cmd) if abs(c) > eps), None)
    if on is None:
        raise ValueError("command never leaves zero; not a step")
    setpoint = cmd[on]
    off = next((i for i in range(on + 1, len(cmd))
                if abs(cmd[i]) <= eps), len(cmd) - 1)
    return setpoint, times[on], times[off]


def analyze_step_csv(path: str, signal_col: str, cmd_col: str,
                     *, band_frac: float = 0.05,
                     ss_window_s: float = 2.0) -> StepMetrics:
    """Read a rover_bench per-trajectory CSV and analyse the
    rising edge of `cmd_col` against `signal_col` (e.g.
    cmd_vx/actual_vx_body or cmd_wz/actual_wz_body)."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    t = [float(r['t']) for r in rows]
    cmd = [float(r[cmd_col]) for r in rows]
    sig = [float(r[signal_col]) for r in rows]
    setpoint, t0, t1 = _detect_step(t, cmd)
    return analyze_step(t, sig, setpoint, t0, t1,
                        band_frac=band_frac,
                        ss_window_s=ss_window_s)
