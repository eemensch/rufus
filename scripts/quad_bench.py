#!/usr/bin/env python3
"""Quadrotor tracking-error benchmark.

Drives the live iris (via the rufus_sim_adapters/quad_adapter node)
through a sequence of prescribed (vx, vy, vz, wz) trajectories and
in parallel integrates a single-integrator-plus-yaw kinematic ideal
seeded with the quad's pose at trajectory start. Records both the
actual world-frame state and the ideal kinematic state, writes a
per-trajectory CSV, and prints a summary report of position-,
velocity-, and yaw-rate-tracking errors.

Run after the iris stack is up and the adapter has reached READY
(i.e. iris is hovering at takeoff altitude):

    gz sim ... + arducopter ... +
    `ros2 launch rufus_sim_bringup iris_sim.launch.py` +
    `ros2 run rufus_sim_adapters quad_adapter ...`

Then:

    python3 scripts/quad_bench.py --output /tmp/quad_bench \\
        --ros-args -p use_sim_time:=true

The kinematic ideal is

    psi_dot = wz
    x_world_dot = vx cos(psi) - vy sin(psi)
    y_world_dot = vx sin(psi) + vy cos(psi)
    z_world_dot = vz

i.e. body-frame velocity rotated to world frame about the yaw axis.
Roll and pitch contributions are ignored by design — the ideal is
the simplest hover-capable holonomic-with-yaw model. Differences
between the ideal and the actual quad reveal where ArduCopter's
attitude-velocity dynamics diverge from a pure kinematic model.
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
# trajectory functions: t (seconds since traj start) -> (vx, vy, vz, wz)
# ---------------------------------------------------------------------------

def _traj_step_vx(t: float) -> tuple[float, float, float, float]:
    if 2.0 <= t < 10.0:
        return 0.5, 0.0, 0.0, 0.0
    return 0.0, 0.0, 0.0, 0.0


def _traj_step_wz(t: float) -> tuple[float, float, float, float]:
    if 2.0 <= t < 10.0:
        return 0.0, 0.0, 0.0, 1.0
    return 0.0, 0.0, 0.0, 0.0


def _traj_step_vz(t: float) -> tuple[float, float, float, float]:
    if 2.0 <= t < 10.0:
        return 0.0, 0.0, 0.5, 0.0
    return 0.0, 0.0, 0.0, 0.0


def _traj_sin_vx(t: float) -> tuple[float, float, float, float]:
    return 0.4 * math.sin(2.0 * math.pi * t / 8.0), 0.0, 0.0, 0.0


def _traj_circle(t: float) -> tuple[float, float, float, float]:
    if t >= 1.0:
        return 0.5, 0.0, 0.0, 0.5  # radius v/w = 1.0 m
    return 0.0, 0.0, 0.0, 0.0


def _traj_lemniscate(t: float) -> tuple[float, float, float, float]:
    return 0.5, 0.0, 0.0, 0.6 * math.sin(2.0 * math.pi * t / 8.0)


def _traj_helix(t: float) -> tuple[float, float, float, float]:
    if t >= 1.0:
        return 0.5, 0.0, 0.3, 0.5
    return 0.0, 0.0, 0.0, 0.0


@dataclass
class TrajSpec:
    name: str
    fn: Callable[[float], tuple[float, float, float, float]]
    duration: float


TRAJECTORIES: list[TrajSpec] = [
    TrajSpec('step_vx', _traj_step_vx, 12.0),
    TrajSpec('step_wz', _traj_step_wz, 12.0),
    TrajSpec('step_vz', _traj_step_vz, 12.0),
    TrajSpec('sin_vx', _traj_sin_vx, 24.0),
    TrajSpec('circle', _traj_circle, 16.0),
    TrajSpec('lemniscate', _traj_lemniscate, 24.0),
    TrajSpec('helix', _traj_helix, 16.0),
]


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


# ---------------------------------------------------------------------------
# benchmark runner
# ---------------------------------------------------------------------------

class QuadBench(Node):

    def __init__(self, output_dir: Path) -> None:
        super().__init__('quad_bench')
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self._latest_state: AgentState | None = None
        self._cur: TrajSpec | None = None
        self._t0_clock = None
        self._last_tick_clock = None
        self._samples: list[dict] = []
        self._ideal_x = 0.0
        self._ideal_y = 0.0
        self._ideal_z = 0.0
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
        self.get_logger().info('waiting for /state ...')
        while rclpy.ok() and self._latest_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        s = self._latest_state
        assert s is not None
        if s.pose.position.z < 1.0:
            self.get_logger().warning(
                f'quad altitude is {s.pose.position.z:.2f} m; '
                f'expected >1 m. Adapter may not be in READY '
                f'(post-takeoff) state. Continuing anyway.'
            )
        else:
            self.get_logger().info(
                f'got /state at altitude {s.pose.position.z:.2f} m; '
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
        self._ideal_x = s.pose.position.x
        self._ideal_y = s.pose.position.y
        self._ideal_z = s.pose.position.z
        self._ideal_yaw = yaw_from_quat(
            s.pose.orientation.x, s.pose.orientation.y,
            s.pose.orientation.z, s.pose.orientation.w,
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

        vx, vy, vz, wz = spec.fn(t)

        cmd = TwistStamped()
        cmd.header.stamp = now_clock.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = wz
        self._pub_cmd.publish(cmd)

        # advance ideal: integrate yaw, then rotate body velocity to
        # world and integrate position. Vertical body velocity maps
        # directly to world z (no roll/pitch contribution by design).
        self._ideal_yaw += wz * dt
        cy = math.cos(self._ideal_yaw)
        sy = math.sin(self._ideal_yaw)
        self._ideal_x += (vx * cy - vy * sy) * dt
        self._ideal_y += (vx * sy + vy * cy) * dt
        self._ideal_z += vz * dt

        s = self._latest_state
        if s is None:
            return
        ap = s.pose.position
        ao = s.pose.orientation
        actual_yaw = yaw_from_quat(ao.x, ao.y, ao.z, ao.w)

        self._samples.append({
            't': t,
            'cmd_vx': vx,
            'cmd_vy': vy,
            'cmd_vz': vz,
            'cmd_wz': wz,
            'actual_x': ap.x,
            'actual_y': ap.y,
            'actual_z': ap.z,
            'actual_yaw': actual_yaw,
            'actual_vx_body': s.twist.linear.x,
            'actual_vy_body': s.twist.linear.y,
            'actual_vz_body': s.twist.linear.z,
            'actual_wz_body': s.twist.angular.z,
            'ideal_x': self._ideal_x,
            'ideal_y': self._ideal_y,
            'ideal_z': self._ideal_z,
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

        n = len(self._samples)
        sum_pos_err = max_pos_err = 0.0
        sum_v_err = max_v_err = 0.0
        sum_w_err = max_w_err = 0.0
        for r in self._samples:
            dx = r['actual_x'] - r['ideal_x']
            dy = r['actual_y'] - r['ideal_y']
            dz = r['actual_z'] - r['ideal_z']
            pe = math.sqrt(dx * dx + dy * dy + dz * dz)
            cmd_v_norm = math.sqrt(
                r['cmd_vx'] ** 2 + r['cmd_vy'] ** 2 + r['cmd_vz'] ** 2
            )
            actual_v_norm = math.sqrt(
                r['actual_vx_body'] ** 2 + r['actual_vy_body'] ** 2
                + r['actual_vz_body'] ** 2
            )
            ve = abs(actual_v_norm - cmd_v_norm)
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
        fdz = final['actual_z'] - final['ideal_z']
        final_pos_err = math.sqrt(fdx * fdx + fdy * fdy + fdz * fdz)

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
        '--output', '-o', default='/tmp/quad_bench',
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
    node = QuadBench(Path(args.output))
    try:
        node.run_all(trajectories)
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
