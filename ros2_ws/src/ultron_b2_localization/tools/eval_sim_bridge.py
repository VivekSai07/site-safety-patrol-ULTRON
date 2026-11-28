#!/usr/bin/env python3
"""Eval-only wrapper around the validated simulation bridge (src/ultron/autonomy/sim_bridge.py).

Rewires its topics to the B2 stack contract under /ultron/sim (sensors/lidar/points_filtered,
sensors/scan) and, for odom_source mola/kiss, moves the bridge's simulated leg odometry and its
odom->base_link TF off the stack topics so the lidar odometry under test is the only TF authority.
Ground truth is only logged to <out>/ground_truth.csv (evaluation), never fed to the estimator.
"""
import argparse
import dataclasses
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / 'src'))

from ultron.autonomy import sim_bridge  # noqa: E402
from ultron.autonomy.config import load_route, load_sim_slam_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom-source', choices=['sim', 'mola', 'kiss'], required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--route', type=Path, default=REPO / 'config/autonomy/routes/hall_survey_loop.yaml')
    a = ap.parse_args()
    config = load_sim_slam_config()
    ns = '/ultron/sim'
    stack_odom = a.odom_source == 'sim'
    topics = dict(config.topics, points=f'{ns}/sensors/lidar/points_filtered', scan=f'{ns}/sensors/scan',
                  odom=f'{ns}/state/odom' if stack_odom else f'{ns}/eval/sim_leg_odom')
    config = dataclasses.replace(config, topics=topics, odometry=dict(config.odometry, seed=a.seed))
    sim_bridge.check_isolation(config)
    import rclpy
    rclpy.init(args=None if stack_odom else ['--ros-args', '-r', f'/tf:={ns}/eval/sim_leg_odom_tf'])
    try:
        bridge = sim_bridge.SimBridge(config, load_route(a.route), a.rate, a.out)
        bridge.run()
        bridge.node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
