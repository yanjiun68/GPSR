#!/usr/bin/env python3
"""Manual integration test for the default RoboCup 2026 arena.

Run: ros2 run gz_sim validate_robocup2026_arena.py
Requires a built/sourced workspace and working headless Gazebo rendering.
Uses ROS domain 184 and a unique Gazebo partition; do not share domain 184
with hardware or another validation run. Logs are retained under /tmp.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import qos_profile_sensor_data


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    run_dir = Path(tempfile.mkdtemp(prefix='putrabot-arena2026-test-'))
    os.environ['ROS_DOMAIN_ID'] = '184'
    os.environ['IGN_PARTITION'] = 'putrabot_arena2026_validation_' + str(os.getpid())
    os.environ['GZ_PARTITION'] = os.environ['IGN_PARTITION']
    os.environ['ROS_LOG_DIR'] = str(run_dir / 'ros')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    print('Test log:', run_dir / 'simulation.log', flush=True)

    process = None
    node = None
    publisher = None
    with (run_dir / 'simulation.log').open('w') as log:
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'gz_sim', 'robocup_arena.launch.py',
                 'headless:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            rclpy.init()
            node = rclpy.create_node('arena_validation')
            samples = {}
            subscriptions = []
            for name, msgtype, topic in [
                ('odom', Odometry, '/odometry'),
                ('scan', LaserScan, '/scan'),
                ('rgb', Image, '/camera/color/image_raw'),
            ]:
                subscriptions.append(node.create_subscription(
                    msgtype, topic,
                    lambda msg, key=name: samples.__setitem__(key, msg),
                    qos_profile_sensor_data,
                ))
            publisher = node.create_publisher(Twist, '/cmd_vel', 10)

            deadline = time.monotonic() + 45
            while time.monotonic() < deadline and len(samples) < 3:
                rclpy.spin_once(node, timeout_sec=0.2)
                require(process.poll() is None, 'Launch exited early')
            print('Received:', sorted(samples), flush=True)
            require('odom' in samples, 'No odometry')
            p = samples['odom'].pose.pose.position
            print('Start world odometry:', p.x, p.y, p.z, flush=True)
            require(abs(p.x + 0.4) < 0.04 and abs(p.y - 3.5119048) < 0.04,
                    'Start position mismatch')
            require('scan' in samples and any(math.isfinite(v) for v in samples['scan'].ranges),
                    'No finite LiDAR returns')
            require('rgb' in samples and samples['rgb'].width > 0, 'No RGB frames')

            speed = Twist()
            speed.linear.x = 0.15
            # Render-heavy headless runs can be slower than real time.
            # Measure the drive interval using odometry's simulation stamp.
            def odom_time():
                stamp = samples['odom'].header.stamp
                return stamp.sec + stamp.nanosec * 1e-9

            start_time = odom_time()
            deadline = time.monotonic() + 180
            while odom_time() - start_time < 8 and time.monotonic() < deadline:
                require(process.poll() is None, 'Launch exited during traversal')
                publisher.publish(speed)
                rclpy.spin_once(node, timeout_sec=0.1)
            publisher.publish(Twist())
            require(odom_time() - start_time >= 8, 'Simulation stalled during traversal')
            p = samples['odom'].pose.pose.position
            print('After driving toward entrance:', p.x, p.y, flush=True)
            require(p.x > 0.35 and abs(p.y - 3.5119048) < 0.15,
                    'Robot did not pass entrance')
            print('PASS: spawn pose, LiDAR, camera, command bridge, and entrance traversal.',
                  flush=True)
        finally:
            # Always stop this launch group, including failed ROS initialization.
            try:
                if publisher is not None and rclpy.ok():
                    publisher.publish(Twist())
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)


if __name__ == '__main__':
    main()
