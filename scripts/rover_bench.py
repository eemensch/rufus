#!/usr/bin/env python3
"""Rover tracking-error benchmark.

Drives the live rover (via the rufus_sim_adapters/rover node) through a
sequence of prescribed (vx, wz) trajectories. In parallel integrates a
**Dubins car** (minimum-turn-radius constraint, R_min = TURN_RADIUS)
using the SAME commanded inputs starting from the rover's pose at
trajectory start. The ideal is NOT a free unicycle: ArduRover GUIDED
imposes a minimum turn radius, so the achievable yaw rate is
speed-coupled, |omega| <= |vx|/R_min, and pure in-place yaw (vx=0)
is infeasible from the GUIDED velocity/turn-rate path this stack uses
(true pivot-in-place is an AUTO-mode-only feature). A free unicycle
ideal mis-scores every yaw command; see docs/plan.md S1.4 and
CLAIMS C4/C6/C7. Records actual and ideal state, writes a
per-trajectory CSV, and prints position-, velocity-, and
yaw-rate-tracking errors.

Run after the rover stack is up:
    gz sim ... + ardurover ... + mavros ... + rover_adapter

Then:
    python3 scripts/rover_bench.py --output /tmp/rover_bench
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


# ---------------------------------------------------------------------------
# Minimum turn radius the ArduRover GUIDED steering path enforces
# at low speed (Rover/Parameters.cpp TURN_RADIUS default 0.9 m;
# r1_rover.param sets 0.9). The ideal Dubins car clips yaw rate to
# |omega| <= |vx| / R_MIN. Keep in sync with the live TURN_RADIUS
# (override via --r-min for the TURN_RADIUS sweep).
R_MIN = 0.9

# trajectory functions: t (seconds since traj start) -> (vx, wz)
# ---------------------------------------------------------------------------

def _traj_step_vx(t: float) -> tuple[float, float]:
    if 2.0 <= t < 10.0:
        return 0.5, 0.0
    return 0.0, 0.0


def _traj_step_wz(t: float) -> tuple[float, float]:
    # INFEASIBLE-REGION PROBE, not a fidelity test. vx=0 with
    # wz!=0 demands turn radius 0, which the GUIDED min-radius
    # model (R_MIN) cannot do (pivot is AUTO-only). The Dubins
    # ideal correctly predicts ~0 yaw here; the metric vs that
    # ideal quantifies how badly GUIDED flails on an infeasible
    # command. Use arc_step for actual yaw fidelity.
    if 2.0 <= t < 10.0:
        return 0.0, 1.0
    return 0.0, 0.0


def _traj_step_wz_lo(t: float) -> tuple[float, float]:
    # Infeasible-region probe at small amplitude (companion to
    # step_wz). The amplitude-scaling result (0.2 worse than 1.0)
    # is itself a symptom of commanding inside the infeasible
    # vx=0 region, not a loop property.
    if 2.0 <= t < 10.0:
        return 0.0, 0.2
    return 0.0, 0.0


def _traj_arc_step(t: float) -> tuple[float, float]:
    # VALID yaw-rate step: a step in wz at a feasible speed.
    # v=1.0 m/s, wz 0->1.0 => radius v/wz = 1.0 m >= R_MIN 0.9,
    # inside the GUIDED Dubins envelope. This is the
    # kinematically-valid replacement for step_wz and the basis
    # for the S1.4 yaw acceptance spec.
    if 2.0 <= t < 10.0:
        return 1.0, 1.0
    return 1.0, 0.0


def _traj_sin_vx(t: float) -> tuple[float, float]:
    return 0.4 * math.sin(2.0 * math.pi * t / 8.0), 0.0


def _traj_circle(t: float) -> tuple[float, float]:
    if t >= 1.0:
        return 0.5, 0.5  # radius v/w = 1.0 m
    return 0.0, 0.0


def _traj_lemniscate(t: float) -> tuple[float, float]:
    return 0.5, 0.6 * math.sin(2.0 * math.pi * t / 8.0)


@dataclass
class TrajSpec:
    name: str
    fn: Callable[[float], tuple[float, float]]
    duration: float


TRAJECTORIES: list[TrajSpec] = [
    TrajSpec('step_vx', _traj_step_vx, 12.0),
    TrajSpec('step_wz', _traj_step_wz, 12.0),
    TrajSpec('step_wz_lo', _traj_step_wz_lo, 12.0),
    TrajSpec('arc_step', _traj_arc_step, 12.0),
    TrajSpec('sin_vx', _traj_sin_vx, 24.0),
    TrajSpec('circle', _traj_circle, 16.0),
    TrajSpec('lemniscate', _traj_lemniscate, 24.0),
]


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


# ---------------------------------------------------------------------------
# benchmark runner
# ---------------------------------------------------------------------------

class RoverBench(Node):

    def __init__(self, output_dir: Path) -> None:
        super().__init__('rover_bench')
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._latest_state: AgentState | None = None
        self._cur: TrajSpec | None = None
        self._t0_clock = None
        self._last_tick_clock = None
        self._samples: list[dict] = []
        self._ideal_x = 0.0
        self._ideal_y = 0.0
        self._ideal_yaw = 0.0
        self._reports: list[dict] = []

        self._pub_cmd = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )
        self.create_subscription(
            AgentState, '/state', self._state_cb, 10
        )

    # ---- state callback ----

    def _state_cb(self, msg: AgentState) -> None:
        self._latest_state = msg

    # ---- top-level driver ----

    def run_all(self, trajectories: list[TrajSpec]) -> None:
        # wait for first state message
        self.get_logger().info('waiting for /state ...')
        while rclpy.ok() and self._latest_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('got /state; starting trajectories')

        for spec in trajectories:
            if not rclpy.ok():
                break
            self._run_one(spec)
            # 2-second cool-down with zero command
            self._stop_for(2.0)

        self._print_summary()

    def _run_one(self, spec: TrajSpec) -> None:
        self.get_logger().info(f'--- {spec.name} ({spec.duration:.1f}s) ---')

        # capture start pose for ideal integrator
        s = self._latest_state
        assert s is not None
        self._ideal_x = s.pose.position.x
        self._ideal_y = s.pose.position.y
        self._ideal_yaw = yaw_from_quat(
            s.pose.orientation.x, s.pose.orientation.y,
            s.pose.orientation.z, s.pose.orientation.w,
        )
        self._cur = spec
        self._samples = []
        self._t0_clock = self.get_clock().now()
        self._last_tick_clock = self._t0_clock

        # tick at 50 Hz inside this trajectory
        period = 0.02
        next_t = time.monotonic()
        while rclpy.ok():
            now_clock = self.get_clock().now()
            t = (now_clock - self._t0_clock).nanoseconds * 1e-9
            if t > spec.duration:
                break
            self._tick(spec, t, now_clock)
            # spin a bit, then sleep until next tick
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

        vx, wz = spec.fn(t)

        # publish commanded twist
        cmd = TwistStamped()
        cmd.header.stamp = now_clock.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = vx
        cmd.twist.angular.z = wz
        self._pub_cmd.publish(cmd)

        # advance the ideal: a DUBINS CAR, not a free unicycle.
        # ArduRover GUIDED enforces a minimum turn radius R_MIN, so
        # the achievable yaw rate is speed-coupled: |omega| <=
        # |vx|/R_MIN. At vx=0 the feasible yaw rate is 0 (pivot in
        # place is AUTO-mode-only, unreachable from the GUIDED
        # velocity/turn-rate path). Clipping here makes "tracking
        # error" measure deviation from the *feasible* command, and
        # correctly flags vx=0 yaw steps as commanding nothing.
        omega_max = abs(vx) / R_MIN
        omega_eff = max(-omega_max, min(omega_max, wz))
        self._ideal_yaw += omega_eff * dt
        self._ideal_x += vx * math.cos(self._ideal_yaw) * dt
        self._ideal_y += vx * math.sin(self._ideal_yaw) * dt

        s = self._latest_state
        if s is None:
            return
        ap = s.pose.position
        ao = s.pose.orientation
        actual_yaw = yaw_from_quat(ao.x, ao.y, ao.z, ao.w)

        self._samples.append({
            't': t,
            'cmd_vx': vx,
            'cmd_wz': wz,
            'actual_x': ap.x,
            'actual_y': ap.y,
            'actual_yaw': actual_yaw,
            'actual_vx_body': s.twist.linear.x,
            'actual_wz_body': s.twist.angular.z,
            'ideal_x': self._ideal_x,
            'ideal_y': self._ideal_y,
            'ideal_yaw': self._ideal_yaw,
        })

    def _stop_for(self, secs: float) -> None:
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            self._pub_cmd.publish(cmd)
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

        # summary
        n = len(self._samples)
        sum_pos_err = max_pos_err = 0.0
        sum_v_err = max_v_err = 0.0
        sum_w_err = max_w_err = 0.0
        for r in self._samples:
            dx = r['actual_x'] - r['ideal_x']
            dy = r['actual_y'] - r['ideal_y']
            pe = math.hypot(dx, dy)
            ve = abs(r['actual_vx_body'] - r['cmd_vx'])
            we = abs(r['actual_wz_body'] - r['cmd_wz'])
            sum_pos_err += pe
            max_pos_err = max(max_pos_err, pe)
            sum_v_err += ve
            max_v_err = max(max_v_err, ve)
            sum_w_err += we
            max_w_err = max(max_w_err, we)
        final = self._samples[-1]
        fdx = final['actual_x'] - final['ideal_x']
        fdy = final['actual_y'] - final['ideal_y']
        final_pos_err = math.hypot(fdx, fdy)

        report = {
            'name': spec.name,
            'samples': n,
            'mean_pos_err_m': sum_pos_err / n,
            'max_pos_err_m': max_pos_err,
            'final_pos_err_m': final_pos_err,
            'mean_v_err_mps': sum_v_err / n,
            'max_v_err_mps': max_v_err,
            'mean_w_err_radps': sum_w_err / n,
            'max_w_err_radps': max_w_err,
            'csv_path': str(path),
        }
        self._reports.append(report)
        self.get_logger().info(
            f'{spec.name}: '
            f'mean_pe={report["mean_pos_err_m"]:.3f} m, '
            f'max_pe={report["max_pos_err_m"]:.3f} m, '
            f'final_pe={report["final_pos_err_m"]:.3f} m, '
            f'mean_ve={report["mean_v_err_mps"]:.3f} m/s, '
            f'mean_we={report["mean_w_err_radps"]:.3f} rad/s'
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

        # pretty print
        cols = [
            'name', 'samples',
            'mean_pos_err_m', 'max_pos_err_m', 'final_pos_err_m',
            'mean_v_err_mps', 'mean_w_err_radps',
        ]
        widths = [max(len(c), 12) for c in cols]
        print()
        print(' | '.join(c.ljust(w) for c, w in zip(cols, widths)))
        print('-' * (sum(widths) + 3 * (len(cols) - 1)))
        for r in self._reports:
            row = []
            for c, w in zip(cols, widths):
                v = r[c]
                row.append(
                    str(v).ljust(w) if isinstance(v, (int, str))
                    else f'{v:.3f}'.ljust(w)
                )
            print(' | '.join(row))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', '-o', default='/tmp/rover_bench',
        help='output directory for per-trajectory CSVs and summary',
    )
    parser.add_argument(
        '--only', default=None,
        help='comma-separated trajectory names to run; '
             'default runs all',
    )
    parser.add_argument(
        '--r-min', type=float, default=None,
        help='Dubins-car minimum turn radius (m) for the ideal; '
             'set to the live TURN_RADIUS under test so the ideal '
             'reflects the same constraint. Default: R_MIN (0.9).',
    )
    args, ros_args = parser.parse_known_args()

    if args.r_min is not None:
        global R_MIN
        R_MIN = args.r_min

    if args.only:
        wanted = set(args.only.split(','))
        trajectories = [t for t in TRAJECTORIES if t.name in wanted]
        if not trajectories:
            raise SystemExit(f'no matching trajectories in --only={args.only}')
    else:
        trajectories = TRAJECTORIES

    rclpy.init(args=ros_args)
    node = RoverBench(Path(args.output))
    try:
        node.run_all(trajectories)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
