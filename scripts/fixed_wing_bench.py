#!/usr/bin/env python3
"""Fixed-wing tracking-error benchmark.

Drives the live zephyr (via the fixed_wing_adapter) through a
sequence of prescribed Dubins-airplane control-input
trajectories `(V, psidot, climb)` and in parallel integrates
the exact Dubins-airplane kinematic model seeded with the
plane's pose at trajectory start. Records actual and ideal
state per sample, writes a per-trajectory CSV plus a summary
report, and prints a tracking-error table.

Run after the zephyr stack is up and the adapter has reached
READY (i.e. plane is airborne in GUIDED at takeoff altitude):

    gz sim ... + arduplane ... +
    `ros2 launch rufus_sim_bringup zephyr_sim.launch.py` +
    `ros2 run rufus_sim_adapters fixed_wing_adapter ...`

Then:

    python3 scripts/fixed_wing_bench.py --output /tmp/fw_bench \\
        --ros-args -p use_sim_time:=true

The ideal is the exact Dubins-airplane kinematic model the
differential game integrates: state `(x, y, z, psi)`, control
`(V, psidot, climb)` clipped by the SAME admissible set the
adapter enforces — `dubins_airplane_clip` is imported from the
adapter so the ideal and the adapter cannot drift apart. psi is
the integral of the commanded `psidot` (a control input,
directly constrained), NOT a slew toward a heading derived from
a velocity vector. Differences between this ideal and the
actual trajectory therefore isolate ArduPlane's L1 / TECS
*closed-loop* realization lag plus the adapter's
integrate-psidot-to-heading round trip, with the kinematic
abstraction (no bank-to-bank dynamics) the only modelled gap.
Saturation flags (`airspeed`, `turn_rate`, `climb_rate`) from
`AgentState.saturation` are recorded per sample so the summary
can show what fraction of the trajectory hit each limit.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

from rufus_sim_msgs.msg import AgentState

# Single source of truth for the admissible set: the ideal is
# clipped by exactly the function the adapter applies, so the
# kinematic truth and the realized command share one definition.
from rufus_sim_adapters.fixed_wing_adapter import dubins_airplane_clip


# ---------------------------------------------------------------------------
# trajectory functions: t (s since traj start) -> (V, psidot, climb)
#   V      airspeed command            (m/s)
#   psidot turn-rate command           (rad/s, ENU/CCW)
#   climb  climb-rate command          (m/s, +up)
# ---------------------------------------------------------------------------

V_CRUISE = 12.0   # m/s, matches AIRSPEED_CRUISE on the zephyr
GRAVITY = 9.81

# Defaults mirror the fixed_wing_adapter capability fields, which
# are themselves read from AIRSPEED_*, ROLL_LIMIT_DEG, and
# PTCH_LIM_*_DEG on the zephyr. Override at the CLI if a different
# zephyr tune is in use.
V_MIN = 9.0
V_MAX = 22.0
BANK_MAX_DEG = 45.0
CLIMB_ANGLE_MAX_DEG = 20.0
DESCENT_ANGLE_MAX_DEG = 25.0


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class DubinsAirplaneIdeal:
    """Exact Dubins-airplane kinematic model.

    State `(x, y, z, psi)` in world ENU; control `(V, psidot,
    climb)`. Each step the commanded control is clipped by
    `dubins_airplane_clip` — the very function the adapter
    applies — so the ideal integrates exactly the admissible
    control the strategy is allowed to use:

        psi  <- psi + psidot * dt
        vh    = sqrt(V^2 - climb^2)        (cos-gamma horizontal)
        x    <- x + vh * cos(psi) * dt
        y    <- y + vh * sin(psi) * dt
        z    <- z + climb * dt

    No flight-path-angle or bank dynamics: the kinematic
    abstraction is the only modelled gap, so the actual-vs-ideal
    residual is ArduPlane's closed-loop realization lag plus the
    adapter's psidot->heading round trip. psi is the integral of
    the *commanded* turn rate, not a slew toward a heading.
    """

    x: float
    y: float
    z: float
    psi: float

    v_min: float = V_MIN
    v_max: float = V_MAX
    bank_max: float = math.radians(BANK_MAX_DEG)
    climb_max: float = math.radians(CLIMB_ANGLE_MAX_DEG)
    descent_max: float = math.radians(DESCENT_ANGLE_MAX_DEG)

    def step(self, dt: float, v_cmd: float, psidot_cmd: float,
             climb_cmd: float) -> None:
        V, psidot, climb, _, _, _ = dubins_airplane_clip(
            v_cmd, psidot_cmd, climb_cmd,
            self.v_min, self.v_max,
            GRAVITY * math.tan(self.bank_max),
            self.climb_max, self.descent_max,
        )
        self.psi = _wrap_pi(self.psi + psidot * dt)
        vh = math.sqrt(max(V * V - climb * climb, 0.0))
        self.x += vh * math.cos(self.psi) * dt
        self.y += vh * math.sin(self.psi) * dt
        self.z += climb * dt


def _traj_level_cruise(t: float) -> tuple[float, float, float]:
    return V_CRUISE, 0.0, 0.0


def _traj_loiter(t: float) -> tuple[float, float, float]:
    # Constant coordinated turn: a steady turn-rate command equal
    # to the 30 deg-bank rate omega = g*tan(bank)/V (inside the
    # 45 deg cap, so no turn_rate saturation). period 2*pi/omega.
    omega = GRAVITY * math.tan(math.radians(30.0)) / V_CRUISE
    return V_CRUISE, omega, 0.0


def _traj_climb_descent(t: float) -> tuple[float, float, float]:
    if t < 10.0:
        return V_CRUISE, 0.0, 1.5  # climb
    if t < 20.0:
        return V_CRUISE, 0.0, 0.0  # level
    return V_CRUISE, 0.0, -1.5     # descend


def _traj_turn_rate_step(t: float) -> tuple[float, float, float]:
    # Canonical Dubins control-input step: hold straight, then
    # step the turn-rate command to a feasible 0.3 rad/s (the cap
    # at V_CRUISE is g*tan(45 deg)/12 ~= 0.82 rad/s, so 0.3 is
    # well inside). Exercises the psidot contract end to end and
    # lets step_response analyse heading-rate tracking.
    return (V_CRUISE, 0.0, 0.0) if t < 4.0 else (V_CRUISE, 0.3, 0.0)


def _traj_infeasible_zero(t: float) -> tuple[float, float, float]:
    # Commanded airspeed 0; the adapter must clip to v_min and
    # raise the airspeed saturation flag. Plane keeps flying at
    # v_min along its heading; position diverges from the ideal,
    # which is also clipped to v_min, only through realization
    # lag — the clip is shared, so both fly v_min straight.
    return 0.0, 0.0, 0.0


@dataclass
class TrajSpec:
    name: str
    fn: Callable[[float], tuple[float, float, float]]
    duration: float


TRAJECTORIES: list[TrajSpec] = [
    TrajSpec('level_cruise', _traj_level_cruise, 12.0),
    TrajSpec('loiter', _traj_loiter, 28.0),
    TrajSpec('climb_descent', _traj_climb_descent, 30.0),
    TrajSpec('turn_rate_step', _traj_turn_rate_step, 20.0),
    TrajSpec('infeasible_zero', _traj_infeasible_zero, 10.0),
]


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


# ---------------------------------------------------------------------------
# benchmark runner
# ---------------------------------------------------------------------------


class FixedWingBench(Node):

    def __init__(self, output_dir: Path) -> None:
        super().__init__('fixed_wing_bench')
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._latest_state: AgentState | None = None
        self._cur: TrajSpec | None = None
        self._t0_clock = None
        self._last_tick_clock = None
        self._samples: list[dict] = []
        self._ideal: DubinsAirplaneIdeal | None = None
        self._reports: list[dict] = []

        self._pub_cmd = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )
        self.create_subscription(
            AgentState, '/state', self._state_cb, 10
        )

    def _state_cb(self, msg: AgentState) -> None:
        self._latest_state = msg

    def run_all(self, trajectories: list[TrajSpec]) -> None:
        self.get_logger().info(
            'waiting for /state with z>1 m (airborne)'
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            s = self._latest_state
            if s is not None and s.pose.position.z > 1.0:
                break
        s = self._latest_state
        assert s is not None
        self.get_logger().info(
            f'plane at altitude {s.pose.position.z:.2f} m; '
            f'starting trajectories'
        )

        for spec in trajectories:
            if not rclpy.ok():
                break
            self._run_one(spec)
            self._stop_for(2.0)

        self._print_summary()

    def _run_one(self, spec: TrajSpec) -> None:
        self.get_logger().info(
            f'--- {spec.name} ({spec.duration:.1f}s) ---'
        )

        s = self._latest_state
        assert s is not None
        ao = s.pose.orientation
        self._ideal = DubinsAirplaneIdeal(
            x=s.pose.position.x,
            y=s.pose.position.y,
            z=s.pose.position.z,
            psi=yaw_from_quat(ao.x, ao.y, ao.z, ao.w),
        )
        self._cur = spec
        self._samples = []
        self._t0_clock = self.get_clock().now()
        self._last_tick_clock = self._t0_clock

        period = 0.02  # 50 Hz tick
        next_t = time.monotonic()
        while rclpy.ok():
            now_clock = self.get_clock().now()
            t = (now_clock - self._t0_clock).nanoseconds * 1e-9
            if t > spec.duration:
                break
            self._tick(spec, t, now_clock)
            rclpy.spin_once(self, timeout_sec=0.0)
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

        self._write_csv(spec)

    def _tick(
        self, spec: TrajSpec, t: float, now_clock,
    ) -> None:
        dt = (now_clock - self._last_tick_clock).nanoseconds * 1e-9
        self._last_tick_clock = now_clock

        v_cmd, psidot_cmd, climb_cmd = spec.fn(t)

        cmd = TwistStamped()
        cmd.header.stamp = now_clock.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v_cmd        # airspeed V
        cmd.twist.angular.z = psidot_cmd  # turn rate (rad/s)
        cmd.twist.linear.z = climb_cmd    # climb rate (m/s, +up)
        self._pub_cmd.publish(cmd)

        # Step the exact Dubins-airplane model forward by dt with
        # the same control the strategy commanded.
        assert self._ideal is not None
        self._ideal.step(dt, v_cmd, psidot_cmd, climb_cmd)

        s = self._latest_state
        if s is None:
            return
        ap = s.pose.position
        ao = s.pose.orientation
        actual_yaw = yaw_from_quat(ao.x, ao.y, ao.z, ao.w)
        sat = s.saturation

        self._samples.append({
            't': t,
            'cmd_V': v_cmd, 'cmd_psidot': psidot_cmd,
            'cmd_climb': climb_cmd,
            'actual_x': ap.x, 'actual_y': ap.y, 'actual_z': ap.z,
            'actual_yaw': actual_yaw,
            'ideal_x': self._ideal.x,
            'ideal_y': self._ideal.y,
            'ideal_z': self._ideal.z,
            'ideal_psi': self._ideal.psi,
            'sat_airspeed': int(sat.airspeed),
            'sat_turn_rate': int(sat.turn_rate),
            'sat_climb_rate': int(sat.climb_rate),
        })

    def _stop_for(self, secs: float) -> None:
        # Between trajectories we publish nothing; the adapter's
        # cmd_vel_timeout falls back to its `_cruise_setpoint`,
        # which holds heading at cruise — appropriate for a
        # fixed-wing that cannot hover.
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---- output ----

    def _write_csv(self, spec: TrajSpec) -> None:
        if not self._samples:
            self.get_logger().warning(f'no samples for {spec.name}')
            return
        path = self.output_dir / f'{spec.name}.csv'
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(
                f, fieldnames=list(self._samples[0].keys())
            )
            w.writeheader()
            w.writerows(self._samples)

        n = len(self._samples)
        sum_pos_err = max_pos_err = 0.0
        sat_airspeed = sat_turn = sat_climb = 0
        for r in self._samples:
            dx = r['actual_x'] - r['ideal_x']
            dy = r['actual_y'] - r['ideal_y']
            dz = r['actual_z'] - r['ideal_z']
            pe = math.sqrt(dx * dx + dy * dy + dz * dz)
            sum_pos_err += pe
            max_pos_err = max(max_pos_err, pe)
            sat_airspeed += r['sat_airspeed']
            sat_turn += r['sat_turn_rate']
            sat_climb += r['sat_climb_rate']
        final = self._samples[-1]
        fdx = final['actual_x'] - final['ideal_x']
        fdy = final['actual_y'] - final['ideal_y']
        fdz = final['actual_z'] - final['ideal_z']
        final_pos_err = math.sqrt(fdx * fdx + fdy * fdy + fdz * fdz)

        report = {
            'name': spec.name,
            'samples': n,
            'mean_pos_err_m': sum_pos_err / n,
            'max_pos_err_m': max_pos_err,
            'final_pos_err_m': final_pos_err,
            'pct_sat_airspeed': 100.0 * sat_airspeed / n,
            'pct_sat_turn_rate': 100.0 * sat_turn / n,
            'pct_sat_climb_rate': 100.0 * sat_climb / n,
            'csv_path': str(path),
        }
        self._reports.append(report)
        self.get_logger().info(
            f'{spec.name}: '
            f'mean_pe={report["mean_pos_err_m"]:.2f} m, '
            f'max_pe={report["max_pos_err_m"]:.2f} m, '
            f'final_pe={report["final_pos_err_m"]:.2f} m, '
            f'sat_airspeed={report["pct_sat_airspeed"]:.0f}%, '
            f'sat_turn={report["pct_sat_turn_rate"]:.0f}%, '
            f'sat_climb={report["pct_sat_climb_rate"]:.0f}%'
        )

    def _print_summary(self) -> None:
        if not self._reports:
            return
        path = self.output_dir / 'summary.csv'
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(
                f, fieldnames=list(self._reports[0].keys())
            )
            w.writeheader()
            w.writerows(self._reports)
        self.get_logger().info(f'wrote summary to {path}')

        cols = [
            'name', 'samples',
            'mean_pos_err_m', 'max_pos_err_m', 'final_pos_err_m',
            'pct_sat_airspeed', 'pct_sat_turn_rate',
            'pct_sat_climb_rate',
        ]
        widths = [max(len(c), 14) for c in cols]
        print()
        print(' | '.join(c.ljust(w) for c, w in zip(cols, widths)))
        print('-' * (sum(widths) + 3 * (len(cols) - 1)))
        for r in self._reports:
            row = []
            for c, w in zip(cols, widths):
                v = r[c]
                row.append(
                    str(v).ljust(w) if isinstance(v, (int, str))
                    else f'{v:.2f}'.ljust(w)
                )
            print(' | '.join(row))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', '-o', default='/tmp/fw_bench',
        help='output directory for per-trajectory CSVs and summary',
    )
    parser.add_argument(
        '--only', default=None,
        help='comma-separated trajectory names to run; '
             'default runs all',
    )
    args, ros_args = parser.parse_known_args()

    if args.only:
        wanted = set(args.only.split(','))
        trajectories = [t for t in TRAJECTORIES if t.name in wanted]
        if not trajectories:
            raise SystemExit(
                f'no matching trajectories in --only={args.only}'
            )
    else:
        trajectories = TRAJECTORIES

    rclpy.init(args=ros_args)
    node = FixedWingBench(Path(args.output))
    try:
        node.run_all(trajectories)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
