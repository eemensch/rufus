"""Per-platform parameter override mapping for the episode runner.

An episode YAML can list per-agent envelope overrides under
`parameters: <agent_id>: {high_level: {...}, fcu: {...}}`. The
`high_level` block uses platform-portable names that map onto
fields of `rufus_sim_msgs/Capability`; this module translates them
into the underlying FCU parameter names that ArduPilot reads. The
`fcu` block bypasses the translation and writes the named FCU
parameter directly.

The translation tables here are the canonical reference for what
high-level options each platform supports — `docs/episodes.md`
keeps a human-readable mirror; this file is the source of truth
for the runtime check.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable


GRAVITY = 9.80665


@dataclass(frozen=True)
class FCUWrite:
    """One FCU-side parameter write derived from a high-level option."""
    param: str
    transform: Callable[[float], float]
    unit: str


@dataclass(frozen=True)
class HighLevelOption:
    """A user-facing capability knob for one platform."""
    name: str                       # canonical name (Capability field)
    description: str
    unit: str                       # high-level (input) unit
    writes: tuple[FCUWrite, ...]    # FCU params it expands to


def _identity(x: float) -> float:
    return float(x)


def _rad_to_deg(x: float) -> float:
    return math.degrees(float(x))


def _mps2_to_g(x: float) -> float:
    return float(x) / GRAVITY


# --- Rover (PLATFORM_ROVER) --------------------------------------
#
# Capability:
#   v_max        <- max(WP_SPEED, CRUISE_SPEED, GUID_SPEED_MAX)
#   yaw_rate_max <- ATC_STR_RAT_MAX (deg/s)
#   lat_accel_max<- ATC_TURN_MAX_G (g)
#
# The adapter takes the maximum of the three speed params, so a
# single-param write would not actually constrain v_max in cases
# where another of the three is higher; we write all three at once
# to make the override deterministic.

ROVER_OPTIONS: tuple[HighLevelOption, ...] = (
    HighLevelOption(
        name='v_max',
        description='top forward/back speed in body frame',
        unit='m/s',
        # GUID_SPEED_MAX is queried by rover_adapter when present
        # but is not reliably available across ArduRover builds —
        # MAVROS reports `Failed to get parameter type` on the
        # vendored 4.8-dev. We deliberately don't try to set it
        # here so the override doesn't fail-loudly on missing
        # params; WP_SPEED and CRUISE_SPEED cover the primary
        # GUIDED-mode speed cap on every build.
        writes=(
            FCUWrite('WP_SPEED', _identity, 'm/s'),
            FCUWrite('CRUISE_SPEED', _identity, 'm/s'),
        ),
    ),
    HighLevelOption(
        name='yaw_rate_max',
        description='top yaw rate; ArduRover steering rate cap',
        unit='rad/s',
        writes=(FCUWrite('ATC_STR_RAT_MAX', _rad_to_deg, 'deg/s'),),
    ),
    HighLevelOption(
        name='lateral_accel_max',
        description='top lateral acceleration; ArduRover turn cap',
        unit='m/s^2',
        writes=(FCUWrite('ATC_TURN_MAX_G', _mps2_to_g, 'g'),),
    ),
    HighLevelOption(
        name='min_turn_radius',
        description='Dubins-car R_min; ArduRover TURN_RADIUS. '
                    'Sets the |omega| <= |v| / R_min coupling. '
                    'Widen-only: must be >= the controller-'
                    'deliverable native TURN_RADIUS (a smaller '
                    'value is rejected)',
        unit='m',
        writes=(FCUWrite('TURN_RADIUS', _identity, 'm'),),
    ),
)


# --- Quadrotor (PLATFORM_QUADROTOR) ------------------------------
#
# Capability:
#   v_max          <- WP_SPD (m/s)
#   vz_max_up      <- WP_SPD_UP (m/s)
#   vz_max_down    <- WP_SPD_DN (m/s)
#   yaw_rate_max   <- ATC_RATE_Y_MAX (deg/s)
#   bank_angle_max <- ATC_ANGLE_MAX (deg)

QUAD_OPTIONS: tuple[HighLevelOption, ...] = (
    HighLevelOption(
        name='v_max',
        description='top horizontal speed (body frame)',
        unit='m/s',
        writes=(FCUWrite('WP_SPD', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='vz_max_up',
        description='top climb rate',
        unit='m/s',
        writes=(FCUWrite('WP_SPD_UP', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='vz_max_down',
        description='top descent rate',
        unit='m/s',
        writes=(FCUWrite('WP_SPD_DN', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='yaw_rate_max',
        description='top body yaw rate',
        unit='rad/s',
        writes=(FCUWrite('ATC_RATE_Y_MAX', _rad_to_deg, 'deg/s'),),
    ),
    HighLevelOption(
        name='bank_angle_max',
        description='top roll/pitch tilt',
        unit='rad',
        writes=(FCUWrite('ATC_ANGLE_MAX', _rad_to_deg, 'deg'),),
    ),
)


# --- Fixed-wing (PLATFORM_FIXED_WING) ----------------------------
#
# Capability:
#   v_min          <- AIRSPEED_MIN (m/s)
#   v_max          <- AIRSPEED_MAX (m/s)
#   bank_angle_max <- ROLL_LIMIT_DEG
#   climb_angle_max<- PTCH_LIM_MAX_DEG (deg)
#
# Note: AIRSPEED_CRUISE is also exposed because it sets the
# default cruise that ArduPlane targets between waypoints; it
# does not bound the envelope, but episode authors often want
# to set it alongside v_min / v_max.

PLANE_OPTIONS: tuple[HighLevelOption, ...] = (
    HighLevelOption(
        name='v_min',
        description='stall airspeed; below this AP refuses to fly',
        unit='m/s',
        writes=(FCUWrite('AIRSPEED_MIN', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='v_cruise',
        description='target cruise airspeed (informational, not a bound)',
        unit='m/s',
        writes=(FCUWrite('AIRSPEED_CRUISE', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='v_max',
        description='top airspeed; AP throttles back above this',
        unit='m/s',
        writes=(FCUWrite('AIRSPEED_MAX', _identity, 'm/s'),),
    ),
    HighLevelOption(
        name='bank_angle_max',
        description='top roll angle in FBWA/GUIDED',
        unit='rad',
        writes=(FCUWrite('ROLL_LIMIT_DEG', _rad_to_deg, 'deg'),),
    ),
    HighLevelOption(
        name='climb_angle_max',
        description='top climb pitch in FBWA/GUIDED',
        unit='rad',
        writes=(FCUWrite('PTCH_LIM_MAX_DEG', _rad_to_deg, 'deg'),),
    ),
    HighLevelOption(
        name='climb_angle_min',
        description='top descent pitch (negative)',
        unit='rad',
        writes=(FCUWrite('PTCH_LIM_MIN_DEG', _rad_to_deg, 'deg'),),
    ),
)


PLATFORM_OPTIONS: dict[str, tuple[HighLevelOption, ...]] = {
    'rover': ROVER_OPTIONS,
    'quad': QUAD_OPTIONS,
    'plane': PLANE_OPTIONS,
}


class ParameterOverrideError(ValueError):
    """Raised when a `parameters:` block in the episode YAML
    references an unknown platform, agent_id, or high-level
    option, or carries a non-numeric value."""


def translate(platform: str,
              high_level: dict | None,
              fcu: dict | None) -> dict[str, float]:
    """Translate one agent's `parameters` block into a flat dict of
    FCU param name -> double-typed value.

    Order of precedence: high_level expansions first, then `fcu`
    overrides on top, so that an episode can pin one component
    via `high_level` and a niche related parameter via `fcu` in
    the same block.
    """

    if platform not in PLATFORM_OPTIONS:
        raise ParameterOverrideError(
            f"unknown platform {platform!r}; valid: "
            f"{sorted(PLATFORM_OPTIONS)}"
        )
    options_by_name = {o.name: o for o in PLATFORM_OPTIONS[platform]}

    flat: dict[str, float] = {}
    for name, value in (high_level or {}).items():
        if name not in options_by_name:
            raise ParameterOverrideError(
                f"unknown high-level option {name!r} for platform "
                f"{platform!r}; valid: "
                f"{sorted(options_by_name)}"
            )
        try:
            v = float(value)
        except (TypeError, ValueError) as e:
            raise ParameterOverrideError(
                f"high-level option {name!r}: value {value!r} is "
                f"not numeric"
            ) from e
        for write in options_by_name[name].writes:
            flat[write.param] = write.transform(v)

    for fcu_name, value in (fcu or {}).items():
        if not isinstance(fcu_name, str):
            raise ParameterOverrideError(
                f"fcu parameter name must be a string, got "
                f"{fcu_name!r}"
            )
        try:
            flat[fcu_name] = float(value)
        except (TypeError, ValueError) as e:
            raise ParameterOverrideError(
                f"fcu parameter {fcu_name!r}: value {value!r} is "
                f"not numeric"
            ) from e

    return flat


def options_for(platform: str) -> Iterable[HighLevelOption]:
    return PLATFORM_OPTIONS[platform]


def apply_high_level_to_capability(
        platform: str,
        high_level: dict | None,
        cap) -> None:
    """Patch a `rufus_sim_msgs/Capability` message in place with the
    high-level overrides.

    Each high-level option lands on the field of the same name in
    `Capability` (the option names were chosen to mirror the
    Capability schema for exactly this reason). A handful of
    coupled fields are re-derived to keep the message
    self-consistent:

      rover.v_max          -> also v_min = -v_max
      rover.yaw_rate_max   -> also lateral_accel_max =
                              yaw_rate_max * v_max
      quad.v_max           -> also v_min = -v_max
      quad.bank_angle_max  -> also lateral_accel_max =
                              g tan(bank_angle_max)
      plane.bank_angle_max -> also lateral_accel_max =
                              g tan(bank_angle_max)
      plane.climb_angle_max-> also vz_max_up =
                              v_max sin(climb_angle_max)
      plane.climb_angle_min-> also vz_max_down =
                              v_max sin(|climb_angle_min|)

    rover.min_turn_radius is monotone-up only: it may be set
    larger than the vehicle controller's native TURN_RADIUS (a
    more restrictive, always-realizable admissible set) but a
    value below it is rejected — the simulated dynamics cannot
    honour a tighter turn than the vehicle executes.

    Couplings that need a parameter not in the high-level menu
    (e.g. rover.lateral_accel_max relies on the underlying
    ATC_TURN_MAX_G read at adapter startup) are left as-is on
    the input Capability — overriding only `v_max` will leave
    the adapter's stored `lateral_accel_max` untouched.

    `high_level` may be None or empty; in that case this is a
    no-op. Unknown option names raise the same
    `ParameterOverrideError` `translate()` raises.
    """

    if not high_level:
        return
    options_by_name = {o.name: o for o in PLATFORM_OPTIONS[platform]}
    for name, value in high_level.items():
        if name not in options_by_name:
            raise ParameterOverrideError(
                f"unknown high-level option {name!r} for platform "
                f"{platform!r}; valid: "
                f"{sorted(options_by_name)}"
            )
        v = float(value)

        if platform == 'rover':
            if name == 'v_max':
                cap.v_max = v
                cap.v_min = -v
                cap.lateral_accel_max = max(
                    cap.lateral_accel_max,
                    cap.yaw_rate_max * v,
                )
            elif name == 'yaw_rate_max':
                cap.yaw_rate_max = v
                cap.lateral_accel_max = max(
                    cap.lateral_accel_max,
                    v * cap.v_max,
                )
            elif name == 'lateral_accel_max':
                cap.lateral_accel_max = v
            elif name == 'min_turn_radius':
                # Monotone-up only. `cap.min_turn_radius` here is
                # the vehicle controller's native TURN_RADIUS (read
                # by the adapter, not yet overridden). An episode
                # may only *widen* R_min — a strictly more
                # restrictive admissible set, always realizable by
                # simply not turning as tight. A smaller R_min
                # would claim a tighter turn than the vehicle can
                # execute (an admissible set the dynamics do not
                # honour), so it is rejected, not silently clamped.
                native = cap.min_turn_radius
                if v < native:
                    raise ParameterOverrideError(
                        f"rover.min_turn_radius={v} m is below the "
                        f"vehicle controller's TURN_RADIUS="
                        f"{native} m; an episode may only widen "
                        f"R_min (a more restrictive admissible "
                        f"set), never claim a tighter turn than "
                        f"the vehicle can execute"
                    )
                cap.min_turn_radius = v

        elif platform == 'quad':
            if name == 'v_max':
                cap.v_max = v
                cap.v_min = -v
            elif name == 'vz_max_up':
                cap.vz_max_up = v
            elif name == 'vz_max_down':
                cap.vz_max_down = v
            elif name == 'yaw_rate_max':
                cap.yaw_rate_max = v
            elif name == 'bank_angle_max':
                cap.bank_angle_max = v
                cap.lateral_accel_max = GRAVITY * math.tan(v)

        elif platform == 'plane':
            if name == 'v_min':
                cap.v_min = v
            elif name == 'v_cruise':
                # Not a Capability field; informational only.
                pass
            elif name == 'v_max':
                cap.v_max = v
                # Re-scale climb-rate ceiling. vz_max_down is left
                # alone — the descent angle isn't stored on
                # Capability, so we can't re-derive it from v_max
                # alone; set climb_angle_min explicitly if you need
                # the descent rate to track v_max.
                if cap.climb_angle_max:
                    cap.vz_max_up = v * math.sin(cap.climb_angle_max)
            elif name == 'bank_angle_max':
                cap.bank_angle_max = v
                cap.lateral_accel_max = GRAVITY * math.tan(v)
            elif name == 'climb_angle_max':
                cap.climb_angle_max = v
                cap.vz_max_up = cap.v_max * math.sin(v)
            elif name == 'climb_angle_min':
                # Stored as a positive descent rate.
                cap.vz_max_down = cap.v_max * math.sin(abs(v))
