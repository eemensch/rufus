"""Metrics aggregation over a sweep's `summary.csv`.

The CSV is the source of truth — one row per run. This module
groups rows by axis combination and computes:

  - capture_rate  : fraction of runs ending in `pursuer_win`.
  - mean / median / p95 time-to-capture, computed only over
    runs that actually captured (`pursuer_win`); other outcomes
    contribute to the rate denominator but not the timing
    quantiles.
  - terminal position distribution per agent (NaN-aware mean +
    std), useful for spotting "evader always escapes east"
    biases.

The aggregator is plain Python + numpy; no ROS dependency.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


PURSUER_WIN = 'pursuer_win'


@dataclass(frozen=True)
class GroupResult:
    axis_values: dict           # parameter dotted path -> value
    n: int                      # total runs in this group
    n_pursuer_win: int
    n_evader_win: int
    n_timeout: int
    n_other: int
    capture_rate: float
    capture_time_mean: float    # NaN if n_pursuer_win == 0
    capture_time_median: float
    capture_time_p95: float
    terminal_positions: dict    # agent_id -> dict(x_mean, ..., z_std, n)


def load_summary(path: Path) -> list[dict]:
    rows: list[dict] = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def _axis_columns(rows: list[dict]) -> list[str]:
    if not rows:
        return []
    fixed = {'run_id', 'seed', 'wallclock_s',
             'outcome', 'predicate_id', 'sim_time_s'}
    keys = list(rows[0].keys())
    cols: list[str] = []
    for k in keys:
        if k in fixed:
            continue
        if k.endswith('_x') or k.endswith('_y') or k.endswith('_z'):
            continue
        cols.append(k)
    return cols


def _agent_ids(rows: list[dict]) -> list[str]:
    if not rows:
        return []
    out: list[str] = []
    for k in rows[0].keys():
        if k.endswith('_x'):
            out.append(k[:-2])
    return out


def _group_key(row: dict, axis_cols: list[str]) -> tuple:
    return tuple((c, row[c]) for c in axis_cols)


def _quantile(values: list[float], q: float) -> float:
    if not values:
        return float('nan')
    return float(np.quantile(np.array(values), q))


def aggregate(rows: list[dict]) -> list[GroupResult]:
    axis_cols = _axis_columns(rows)
    agents = _agent_ids(rows)

    groups: dict[tuple, list[dict]] = {}
    for row in rows:
        groups.setdefault(_group_key(row, axis_cols), []).append(row)

    results: list[GroupResult] = []
    for key, group_rows in groups.items():
        axis_values = dict(key)
        n = len(group_rows)
        outcomes = [r['outcome'] for r in group_rows]
        n_pw = sum(1 for o in outcomes if o == 'pursuer_win')
        n_ew = sum(1 for o in outcomes if o == 'evader_win')
        n_to = sum(1 for o in outcomes if o == 'timeout')
        n_other = n - n_pw - n_ew - n_to

        capture_times: list[float] = []
        for r in group_rows:
            if r['outcome'] == PURSUER_WIN:
                try:
                    capture_times.append(float(r['sim_time_s']))
                except ValueError:
                    pass

        terminal: dict[str, dict] = {}
        for aid in agents:
            xs = []
            ys = []
            zs = []
            for r in group_rows:
                try:
                    xs.append(float(r[f'{aid}_x']))
                    ys.append(float(r[f'{aid}_y']))
                    zs.append(float(r[f'{aid}_z']))
                except (KeyError, ValueError):
                    pass
            xa, ya, za = np.array(xs), np.array(ys), np.array(zs)
            terminal[aid] = {
                'n': len(xs),
                'x_mean': float(np.nanmean(xa)) if len(xs) else float('nan'),
                'x_std': float(np.nanstd(xa)) if len(xs) else float('nan'),
                'y_mean': float(np.nanmean(ya)) if len(ys) else float('nan'),
                'y_std': float(np.nanstd(ya)) if len(ys) else float('nan'),
                'z_mean': float(np.nanmean(za)) if len(zs) else float('nan'),
                'z_std': float(np.nanstd(za)) if len(zs) else float('nan'),
            }

        results.append(GroupResult(
            axis_values=axis_values,
            n=n,
            n_pursuer_win=n_pw,
            n_evader_win=n_ew,
            n_timeout=n_to,
            n_other=n_other,
            capture_rate=n_pw / n if n else float('nan'),
            capture_time_mean=(
                float(np.mean(capture_times)) if capture_times
                else float('nan')),
            capture_time_median=(
                float(np.median(capture_times)) if capture_times
                else float('nan')),
            capture_time_p95=_quantile(capture_times, 0.95),
            terminal_positions=terminal,
        ))
    return results


def format_summary(results: list[GroupResult]) -> str:
    """Human-readable rendering — one line per axis group, plus
    a header. Used by the CLI smoke and the docs."""
    lines = []
    if not results:
        return 'no runs.'
    axes = list(results[0].axis_values.keys())
    header = (axes
              + ['n', 'capture_rate',
                 't_mean', 't_med', 't_p95'])
    lines.append('\t'.join(header))
    for r in results:
        cols = [str(r.axis_values[a]) for a in axes]
        cols += [str(r.n), f'{r.capture_rate:.2f}']
        for v in (r.capture_time_mean, r.capture_time_median,
                  r.capture_time_p95):
            cols.append(f'{v:.2f}' if not _isnan(v) else '-')
        lines.append('\t'.join(cols))
    return '\n'.join(lines)


def _isnan(x: float) -> bool:
    return x != x   # NaN-safe
