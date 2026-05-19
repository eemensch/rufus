"""Unit tests for rufus_sim_eval.step_response (first rufus_sim_eval
coverage; CLAIMS C21).

Each case is a closed-form synthetic signal whose rise time,
overshoot, settling time and steady-state stats are known
analytically, so the assertions pin the metric definitions
(10-90% rise, +/-5% settling band, overshoot past setpoint,
steady-state mean/stdev).
"""

import csv
import math

from rufus_sim_eval.step_response import (
    analyze_step,
    analyze_step_csv,
)

DT = 0.02
T0, T1 = 2.0, 10.0


def _grid():
    n = int(round(12.0 / DT)) + 1
    return [round(i * DT, 6) for i in range(n)]


def _firstorder(sp, tau):
    t = _grid()
    y = [0.0 if ti < T0 else sp * (1.0 - math.exp(-(ti - T0) / tau))
         for ti in t]
    return t, y


def test_ideal_step_is_instant_and_exact():
    t = _grid()
    y = [0.0 if ti < T0 else 0.5 for ti in t]
    m = analyze_step(t, y, 0.5, T0, T1)
    assert m.overshoot_pct == 0.0
    assert m.settled and m.settling_time == 0.0
    assert math.isclose(m.ss_value, 0.5, abs_tol=1e-9)
    assert math.isclose(m.ss_error, 0.0, abs_tol=1e-9)
    assert math.isclose(m.ss_ripple, 0.0, abs_tol=1e-9)


def test_first_order_rise_and_settling_match_theory():
    tau = 0.5
    t, y = _firstorder(1.0, tau)
    m = analyze_step(t, y, 1.0, T0, T1)
    # 10->90% of a first-order system is tau*ln(9)
    assert m.rise_time is not None
    assert math.isclose(m.rise_time, tau * math.log(9.0),
                         abs_tol=3 * DT)
    # +/-5% band reached at tau*ln(20)
    assert m.settled
    assert math.isclose(m.settling_time, tau * math.log(20.0),
                        abs_tol=3 * DT)
    assert m.overshoot_pct == 0.0
    assert math.isclose(m.ss_error, 0.0, abs_tol=1e-3)
    assert m.ss_ripple < 1e-3


def test_overshoot_piecewise_exact_20pct():
    t = _grid()
    y = []
    for ti in t:
        if ti < T0:
            y.append(0.0)
        elif ti < 3.0:
            y.append(1.2 * (ti - T0))          # 0 -> 1.2 peak
        elif ti < 5.0:
            y.append(1.2 - 0.1 * (ti - 3.0))   # 1.2 -> 1.0
        else:
            y.append(1.0)
    m = analyze_step(t, y, 1.0, T0, T1)
    assert math.isclose(m.peak, 1.2, abs_tol=1e-9)
    assert math.isclose(m.overshoot_pct, 20.0, abs_tol=0.5)
    assert math.isclose(m.rise_time, 0.75 - 0.5 / 6.0,
                        abs_tol=3 * DT)        # 0.1->0.9 of 1.2/s ramp
    assert m.settled
    assert math.isclose(m.settling_time, 2.5, abs_tol=3 * DT)
    assert math.isclose(m.ss_value, 1.0, abs_tol=1e-9)


def test_steady_state_offset_and_ripple_never_settles():
    t = _grid()
    y = []
    for ti in t:
        if ti < T0:
            y.append(0.0)
        elif ti < 2.5:
            y.append(1.8 * (ti - T0))                  # ramp to 0.9
        else:
            y.append(0.9 + 0.02 * math.sin(
                2 * math.pi * (ti - 2.5) / 0.5))
    m = analyze_step(t, y, 1.0, T0, T1)
    # asymptote 0.9 vs setpoint 1.0 -> 0.1 offset, never in band
    assert m.settled is False and m.settling_time is None
    assert math.isclose(m.ss_error, 0.1, abs_tol=0.01)
    # stdev of a 0.02-amplitude sine is 0.02/sqrt(2)
    assert math.isclose(m.ss_ripple, 0.02 / math.sqrt(2),
                        abs_tol=3e-3)


def test_negative_setpoint_uses_signed_span():
    tau = 0.4
    t, y = _firstorder(-0.5, tau)
    m = analyze_step(t, y, -0.5, T0, T1)
    assert math.isclose(m.rise_time, tau * math.log(9.0),
                        abs_tol=3 * DT)
    assert m.overshoot_pct == 0.0
    assert math.isclose(m.settling_time, tau * math.log(20.0),
                        abs_tol=3 * DT)
    assert math.isclose(m.ss_value, -0.5, abs_tol=1e-3)


def test_analyze_step_csv_detects_pulse(tmp_path):
    tau = 0.5
    t, y = _firstorder(1.0, tau)
    cmd = [0.0 if (ti < T0 or ti >= T1) else 1.0 for ti in t]
    p = tmp_path / "step_wz.csv"
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cmd_wz", "actual_wz_body"])
        for ti, ci, yi in zip(t, cmd, y):
            w.writerow([ti, ci, yi])
    m = analyze_step_csv(str(p), "actual_wz_body", "cmd_wz")
    assert math.isclose(m.rise_time, tau * math.log(9.0),
                        abs_tol=3 * DT)
    assert m.settled
