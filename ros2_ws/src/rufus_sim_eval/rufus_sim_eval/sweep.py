"""Sweep YAML schema and loader for the evaluation harness.

A sweep specifies a base episode YAML plus a Cartesian product
of axis values, replicated across `seeds` independent runs per
combination. The batch runner enumerates the product, applies
each combination as a per-run override on top of the base
episode, executes the chain, and records one CSV row per run.

Sweep YAML schema (v1)
----------------------

    schema_version: 1
    name: <string>                  # used as a sub-dir under output_dir
    episode: <package://...|path>   # base episode YAML
    axes:                            # zero or more axes
      - parameter: <dotted path>     # e.g. agents.R1.parameters.high_level.v_max
        values: [<float>, <float>, ...]
    seeds: <int>                    # repeats per axis combination
    speedup: <float>                # SITL --speedup; default 1.0
    output_dir: <path>              # absolute output directory

Required fields: schema_version, name, episode, seeds,
output_dir. axes may be empty or absent (then there is one
combination — just the base episode — replicated `seeds` times).

The dotted path for `parameter` must reference an existing key
in the base episode YAML; the loader resolves it eagerly so a
typo fails fast.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import yaml

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory,
)


_SCHEMA_VERSION_SUPPORTED = 1


class SweepLoadError(RuntimeError):
    pass


@dataclass(frozen=True)
class Axis:
    parameter: str          # dotted path into the episode YAML
    values: tuple


@dataclass(frozen=True)
class SweepSpec:
    name: str
    episode_path: Path
    base_episode: dict
    axes: tuple
    seeds: int
    speedup: float
    output_dir: Path


@dataclass(frozen=True)
class SweepRun:
    """One concrete run produced by enumerating the Cartesian
    product of axis values × seeds."""
    run_id: int
    seed: int               # 0..seeds-1, identifies the repetition
    axis_values: dict       # parameter dotted path -> value
    episode_yaml: dict      # base episode with axis_values applied


def _resolve_path(reference: str, base: Path) -> Path:
    if reference.startswith('package://'):
        rest = reference[len('package://'):]
        if '/' not in rest:
            raise SweepLoadError(
                f"reference {reference!r}: package:// URI must "
                f"include a path"
            )
        pkg, rel = rest.split('/', 1)
        try:
            share = get_package_share_directory(pkg)
        except PackageNotFoundError as e:
            raise SweepLoadError(
                f"reference {reference!r}: package {pkg!r} not "
                f"on the ament index"
            ) from e
        return (Path(share) / rel).resolve()
    p = Path(reference)
    return p if p.is_absolute() else (base / p).resolve()


def _validate_dotted(obj, path: Sequence[str]) -> None:
    """Validate a dotted path: each intermediate key, if present
    in `obj`, must be a dict. The final key may be missing — the
    sweep can introduce new leaves on top of the base episode
    (e.g., `agents.R1.parameters.high_level.v_max` when R1's
    `parameters` block is omitted in the base YAML).

    Raises KeyError when an intermediate key exists but isn't a
    dict (e.g., the user pointed at a path that goes through a
    string), with the offending prefix so the error message
    points at the typo.
    """
    cur = obj
    for i, key in enumerate(path[:-1]):
        if not isinstance(cur, dict):
            raise KeyError('.'.join(path[:i + 1]))
        if key not in cur:
            return  # rest of path will be created at write time
        cur = cur[key]
    if not isinstance(cur, dict):
        raise KeyError('.'.join(path[:-1]))


def _set_dotted(obj: dict, path: Sequence[str], value: Any) -> None:
    """Set a dotted key in a nested dict, creating intermediate
    dicts if missing. Mutates `obj`."""
    cur = obj
    for key in path[:-1]:
        if key not in cur or not isinstance(cur[key], dict):
            cur[key] = {}
        cur = cur[key]
    cur[path[-1]] = value


def load_sweep(sweep_path: Path) -> SweepSpec:
    sweep_path = sweep_path.resolve()
    if not sweep_path.is_file():
        raise SweepLoadError(f"sweep YAML not found: {sweep_path}")
    sweep = yaml.safe_load(sweep_path.read_text())
    if not isinstance(sweep, dict):
        raise SweepLoadError(
            f"{sweep_path}: top-level YAML must be a mapping"
        )
    version = sweep.get('schema_version')
    if version != _SCHEMA_VERSION_SUPPORTED:
        raise SweepLoadError(
            f"{sweep_path}: schema_version must be "
            f"{_SCHEMA_VERSION_SUPPORTED} (got {version!r})"
        )
    for key in ('name', 'episode', 'seeds', 'output_dir'):
        if key not in sweep:
            raise SweepLoadError(
                f"{sweep_path}: missing required key {key!r}"
            )

    name = str(sweep['name'])
    episode_path = _resolve_path(sweep['episode'],
                                 sweep_path.parent)
    if not episode_path.is_file():
        raise SweepLoadError(
            f"{sweep_path}: episode YAML not found: {episode_path}"
        )
    base_episode = yaml.safe_load(episode_path.read_text())

    seeds = int(sweep['seeds'])
    if seeds < 1:
        raise SweepLoadError(
            f"{sweep_path}: seeds must be >= 1, got {seeds}"
        )
    speedup = float(sweep.get('speedup', 1.0))
    if not 0.1 <= speedup <= 50.0:
        raise SweepLoadError(
            f"{sweep_path}: speedup must be in [0.1, 50], got "
            f"{speedup}"
        )
    output_dir = Path(sweep['output_dir']).expanduser()
    if not output_dir.is_absolute():
        raise SweepLoadError(
            f"{sweep_path}: output_dir must be an absolute path, "
            f"got {output_dir}"
        )

    raw_axes = sweep.get('axes') or []
    axes: list[Axis] = []
    for entry in raw_axes:
        if 'parameter' not in entry or 'values' not in entry:
            raise SweepLoadError(
                f"{sweep_path}: each axis needs `parameter` and "
                f"`values` (got {entry!r})"
            )
        param = str(entry['parameter'])
        try:
            _validate_dotted(base_episode, param.split('.'))
        except KeyError as e:
            raise SweepLoadError(
                f"{sweep_path}: axis parameter {param!r} resolves "
                f"into a non-mapping at {e}"
            ) from e
        values = tuple(entry['values'])
        if not values:
            raise SweepLoadError(
                f"{sweep_path}: axis {param!r} has empty values"
            )
        axes.append(Axis(parameter=param, values=values))

    return SweepSpec(
        name=name,
        episode_path=episode_path,
        base_episode=base_episode,
        axes=tuple(axes),
        seeds=seeds,
        speedup=speedup,
        output_dir=output_dir,
    )


def enumerate_runs(spec: SweepSpec) -> list:
    """Cartesian product of axes × seeds, in row-major order.

    Run order is: outermost axis varies slowest, innermost
    fastest, with seeds as the innermost loop. A 3-value × 4-seed
    sweep produces 12 runs in order
    `[(v0, s0), (v0, s1), (v0, s2), (v0, s3), (v1, s0), ...]`.
    """
    if not spec.axes:
        combos: list[dict] = [{}]
    else:
        combos = [{}]
        for axis in spec.axes:
            new_combos = []
            for c in combos:
                for v in axis.values:
                    cc = dict(c)
                    cc[axis.parameter] = v
                    new_combos.append(cc)
            combos = new_combos

    runs: list[SweepRun] = []
    run_id = 0
    for combo in combos:
        for seed in range(spec.seeds):
            episode_yaml = copy.deepcopy(spec.base_episode)
            for path, value in combo.items():
                _set_dotted(episode_yaml, path.split('.'), value)
            # Stamp the run identity into the episode name so
            # logs across runs don't collide.
            episode_yaml['name'] = (
                f"{episode_yaml.get('name', spec.name)}"
                f"__r{run_id:04d}"
            )
            runs.append(SweepRun(
                run_id=run_id,
                seed=seed,
                axis_values=combo,
                episode_yaml=episode_yaml,
            ))
            run_id += 1
    return runs
