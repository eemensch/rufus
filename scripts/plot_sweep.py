#!/usr/bin/env python3
"""Plot a sweep's summary.csv.

Generates two figures next to the CSV:

  capture_rate.png     bar plot of capture rate per axis-value
                       group (one line per axis when there are
                       multiple, but Stage 8 #58 only exercises
                       a single-axis sweep).
  capture_time_box.png box-and-whisker of time-to-capture per
                       axis-value group, restricted to runs
                       that actually captured.

Run as `python3 scripts/plot_sweep.py <summary.csv>`. Requires
the project venv (matplotlib, numpy) — `source .venv/bin/activate`.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


PURSUER_WIN = 'pursuer_win'


def _read_rows(path: Path) -> list:
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        return list(reader)


def _axis_columns(rows):
    fixed = {'run_id', 'seed', 'wallclock_s',
             'outcome', 'predicate_id', 'sim_time_s'}
    return [k for k in rows[0].keys()
            if k not in fixed
            and not k.endswith('_x')
            and not k.endswith('_y')
            and not k.endswith('_z')]


def _group_by_axis(rows, axis_col):
    groups = {}
    for r in rows:
        key = r[axis_col]
        groups.setdefault(key, []).append(r)
    # Sort by the numeric value of the axis when possible.
    try:
        order = sorted(groups, key=lambda k: float(k))
    except ValueError:
        order = sorted(groups)
    return [(k, groups[k]) for k in order]


def plot_capture_rate(rows, axis_col, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(6, 4))
    keys, rates = [], []
    for k, group in _group_by_axis(rows, axis_col):
        n = len(group)
        n_win = sum(1 for r in group if r['outcome'] == PURSUER_WIN)
        keys.append(k)
        rates.append(n_win / n if n else 0.0)
    ax.bar(range(len(keys)), rates, color='#4477aa')
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels(keys)
    ax.set_xlabel(axis_col)
    ax.set_ylabel('capture rate')
    ax.set_ylim(0, 1.0)
    ax.set_title(f'Capture rate vs {axis_col}')
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_capture_time_box(rows, axis_col,
                          out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(6, 4))
    keys, times_per_key = [], []
    for k, group in _group_by_axis(rows, axis_col):
        ts = []
        for r in group:
            if r['outcome'] != PURSUER_WIN:
                continue
            try:
                ts.append(float(r['sim_time_s']))
            except ValueError:
                pass
        keys.append(k)
        times_per_key.append(ts if ts else [np.nan])
    ax.boxplot(times_per_key, tick_labels=keys, showmeans=True)
    ax.set_xlabel(axis_col)
    ax.set_ylabel('sim time to capture (s)')
    ax.set_title(f'Time-to-capture vs {axis_col} '
                 f'(only pursuer_win runs)')
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path,
                        help='Path to the sweep summary.csv.')
    args = parser.parse_args()

    rows = _read_rows(args.summary)
    if not rows:
        print('summary.csv is empty', file=sys.stderr)
        return 1
    axis_cols = _axis_columns(rows)
    if not axis_cols:
        print('summary.csv has no axis columns; nothing to plot',
              file=sys.stderr)
        return 1
    if len(axis_cols) > 1:
        print(f'multiple axes ({axis_cols}); only the first '
              f'({axis_cols[0]!r}) will be plotted',
              file=sys.stderr)
    axis_col = axis_cols[0]

    out_dir = args.summary.parent
    plot_capture_rate(rows, axis_col,
                      out_dir / 'capture_rate.png')
    plot_capture_time_box(rows, axis_col,
                          out_dir / 'capture_time_box.png')
    print(f"wrote {out_dir / 'capture_rate.png'}")
    print(f"wrote {out_dir / 'capture_time_box.png'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
