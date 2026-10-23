#!/usr/bin/env python3
"""Measure ROS /clock elapsed seconds against monotonic wall elapsed seconds.

Usage after Gazebo starts:
    ros2 run gz_sim check_real_time_factor.py --duration 15 --tolerance 0.05

A target real_time_factor of 1.0 in SDF is aspirational; this tool measures
actual time progression received by ROS.  It does not rewrite /clock or alter
physics settings.  It intentionally uses a monotonic wall clock, not ROS time.
"""

import argparse
import math
import sys
import time


def real_time_factor(sim_start, sim_end, wall_start, wall_end):
    """Return actual RTF, or None for insufficient / invalid measurements."""
    wall_elapsed = wall_end - wall_start
    sim_elapsed = sim_end - sim_start
    if wall_elapsed <= 0 or sim_elapsed < 0:
        return None
    return sim_elapsed / wall_elapsed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15.0,
                        help='Measurement duration in wall-clock seconds (default: 15).')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='Allowed absolute RTF deviation from 1.0 (default: 0.05).')
    parser.add_argument('--clock-topic', default='/clock',
                        help='ROS clock topic (default: /clock).')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be a finite positive number')
    if not math.isfinite(args.tolerance) or not 0 <= args.tolerance < 1:
        parser.error('--tolerance must be finite and between 0 (inclusive) and 1 (exclusive)')

    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
    except ImportError as exc:
        print(f'ROS 2 dependencies unavailable: {exc}', file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = rclpy.create_node('gazebo_realtime_factor_check')
    measurements = []

    def on_clock(msg):
        sim_seconds = msg.clock.sec + msg.clock.nanosec * 1e-9
        measurements.append((time.monotonic(), sim_seconds))

    # A BEST_EFFORT reader accepts both reliable and best-effort clock writers.
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    sub = node.create_subscription(Clock, args.clock_topic, on_clock, qos)
    del sub  # the node retains ownership of the subscription

    started = time.monotonic()
    deadline = started + args.duration
    print(f'Checking {args.clock_topic} for {args.duration:g} wall-clock seconds; '
          f'target RTF=1.0 ± {args.tolerance:.3f} ...', flush=True)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.25, max(0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        print('Interrupted.', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(measurements) < 2:
        print('FAIL: no usable /clock samples; check bridge and that Gazebo is running/unpaused.')
        return 2

    # If simulation time was reset mid-measurement, only use readings since the
    # last reset.  This avoids reporting nonsensical negative RTF values.
    segment_start = 0
    for i in range(1, len(measurements)):
        if measurements[i][1] < measurements[i - 1][1]:
            segment_start = i
    valid = measurements[segment_start:]
    if len(valid) < 2:
        print('FAIL: /clock reset during the measurement; retry after restart.')
        return 2
    t0_wall, t0_sim = valid[0]
    t1_wall, t1_sim = valid[-1]
    ratio = real_time_factor(t0_sim, t1_sim, t0_wall, t1_wall)
    if ratio is None or (t1_wall - t0_wall) < args.duration * 0.5:
        print('FAIL: insufficient continuous /clock data; check pause or reset.')
        return 2
    deviation = abs(ratio - 1.0)
    result = 'PASS' if deviation <= args.tolerance else 'FAIL'
    print(f'{result}: simulated {t1_sim - t0_sim:.3f}s over '
          f'{t1_wall - t0_wall:.3f}s real -> RTF={ratio:.4f} '
          f'(deviation {deviation * 100:.2f}%).')
    if result != 'PASS':
        print('Target remains 1.0 in SDF; hardware/rendering/physics may be '
              'unable to sustain it. Compare headless and camera-disabled runs.')
    return 0 if result == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
