#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, PoseStamped


class PeopleCandidatesBridge(Node):
    def __init__(self):
        super().__init__('people_candidates_bridge')

        self.declare_parameter('candidates_topic', '/people_candidates')
        self.declare_parameter('target_topic', '/person_target_locked')
        self.declare_parameter('max_tracking_jump_m', 0.8)

        self.candidates_topic = str(self.get_parameter('candidates_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.max_tracking_jump_m = float(self.get_parameter('max_tracking_jump_m').value)

        self._last_target_xy: Optional[Tuple[float, float]] = None

        self.sub_candidates = self.create_subscription(
            PoseArray,
            self.candidates_topic,
            self.candidates_cb,
            10,
        )
        self.pub_target = self.create_publisher(PoseStamped, self.target_topic, 10)

        self.get_logger().info('[PeopleCandidatesBridge] Started')
        self.get_logger().info(f'  candidates_topic   : {self.candidates_topic}')
        self.get_logger().info(f'  target_topic       : {self.target_topic}')
        self.get_logger().info(f'  max_tracking_jump  : {self.max_tracking_jump_m:.2f} m')

    def candidates_cb(self, msg: PoseArray):
        candidates = self._valid_candidates(msg.poses)
        if not candidates:
            self._last_target_xy = None
            return

        target = self._select_target(candidates)
        if target is None:
            self._last_target_xy = None
            return

        out = PoseStamped()
        out.header = msg.header
        out.pose = target
        if out.pose.orientation.w == 0.0:
            out.pose.orientation.w = 1.0

        self.pub_target.publish(out)
        self._last_target_xy = (float(target.position.x), float(target.position.y))

    def _valid_candidates(self, poses: List[Pose]) -> List[Pose]:
        candidates = []
        for pose in poses:
            x = float(pose.position.x)
            y = float(pose.position.y)
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            if x <= 0.05:
                continue
            candidates.append(pose)
        return candidates

    def _select_target(self, candidates: List[Pose]) -> Optional[Pose]:
        if self._last_target_xy is not None:
            px, py = self._last_target_xy
            tracked = min(
                candidates,
                key=lambda pose: math.hypot(float(pose.position.x) - px, float(pose.position.y) - py),
            )
            tracked_dist = math.hypot(float(tracked.position.x) - px, float(tracked.position.y) - py)
            if tracked_dist <= self.max_tracking_jump_m:
                return tracked

        return min(
            candidates,
            key=lambda pose: math.hypot(float(pose.position.x), float(pose.position.y)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = PeopleCandidatesBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
