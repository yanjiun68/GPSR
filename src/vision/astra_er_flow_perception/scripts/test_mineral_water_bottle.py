#!/usr/bin/env python3
"""
Live integration test for mineral-water-bottle tracking.

Prerequisites:
- Astra camera driver is running.
- astra_registration_node is running.
- er_flow_tracking_node is running.
- UnifoLM-ER-Flow model is available and loaded by the tracking node.
- A mineral water bottle is visible in the camera image.

The script:
1. Publishes a target command for "mineral water bottle".
2. Subscribes to /astra_ai/result and /astra_ai/status.
3. Verifies:
   - object is found,
   - ROI is valid,
   - depth is valid and physically plausible,
   - tracking continues over multiple result frames.
4. Prints PASS/FAIL and exits with code 0/1.
"""

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MineralWaterBottleIntegrationTest(Node):
    def __init__(self):
        super().__init__("mineral_water_bottle_integration_test")

        self.declare_parameter("command_topic", "/astra_ai/command")
        self.declare_parameter("result_topic", "/astra_ai/result")
        self.declare_parameter("status_topic", "/astra_ai/status")

        self.declare_parameter("target", "mineral water bottle")
        self.declare_parameter(
            "prompt",
            (
                "Locate and track the visible mineral water bottle. "
                "Prefer a transparent or clear plastic drinking-water bottle "
                "with a bottle cap. Ignore cups, cans, thermos flasks, and other containers."
            ),
        )

        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("min_tracking_frames", 10)
        self.declare_parameter("min_valid_depth_frames", 5)
        self.declare_parameter("min_depth_m", 0.20)
        self.declare_parameter("max_depth_m", 6.00)
        self.declare_parameter("min_roi_width", 10)
        self.declare_parameter("min_roi_height", 20)

        self.command_topic = self.get_parameter("command_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.target = self.get_parameter("target").value
        self.prompt = self.get_parameter("prompt").value

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.min_tracking_frames = int(
            self.get_parameter("min_tracking_frames").value
        )
        self.min_valid_depth_frames = int(
            self.get_parameter("min_valid_depth_frames").value
        )
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_roi_width = int(self.get_parameter("min_roi_width").value)
        self.min_roi_height = int(self.get_parameter("min_roi_height").value)

        self.command_pub = self.create_publisher(
            String, self.command_topic, 10
        )
        self.create_subscription(
            String, self.result_topic, self.result_cb, 20
        )
        self.create_subscription(
            String, self.status_topic, self.status_cb, 20
        )

        self.started_at = time.monotonic()
        self.last_command_at = 0.0

        self.total_result_frames = 0
        self.found_frames = 0
        self.tracking_frames = 0
        self.valid_roi_frames = 0
        self.valid_depth_frames = 0
        self.er_flow_frames = 0
        self.tracker_frames = 0

        self.depth_values = []
        self.roi_values = []
        self.status_messages = []
        self.latest_result = None
        self.finished = False
        self.exit_code = 1

        # Send repeatedly for the first few seconds in case the tracking node
        # starts a fraction later than this test.
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info(
            "Mineral water bottle integration test started."
        )
        self.get_logger().info(
            f"Target: {self.target!r}"
        )
        self.get_logger().info(
            "Place one mineral water bottle clearly inside the Astra camera view."
        )

    def publish_track_command(self):
        payload = {
            "action": "track",
            "target": self.target,
            "prompt": self.prompt,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.command_pub.publish(msg)
        self.last_command_at = time.monotonic()
        self.get_logger().info(
            f"Published test command to {self.command_topic}: {msg.data}"
        )

    def status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}

        self.status_messages.append(payload)
        if len(self.status_messages) > 50:
            self.status_messages.pop(0)

        state = payload.get("state")
        if state in ("inference_error", "image_decode_error", "command_error"):
            self.get_logger().error(
                f"Perception status error: {payload}"
            )

    def result_cb(self, msg):
        if self.finished:
            return

        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Invalid JSON received on result topic: {msg.data}"
            )
            return

        self.latest_result = result
        self.total_result_frames += 1

        found = bool(result.get("found", False))
        tracking = bool(result.get("tracking", False))
        source = result.get("source", "")

        if found:
            self.found_frames += 1
        if found and tracking:
            self.tracking_frames += 1

        if source == "er_flow":
            self.er_flow_frames += 1
        elif source == "tracker":
            self.tracker_frames += 1

        roi = result.get("roi_xywh")
        roi_valid = False
        if (
            isinstance(roi, list)
            and len(roi) == 4
            and all(isinstance(v, (int, float)) for v in roi)
        ):
            x, y, w, h = roi
            roi_valid = (
                x >= 0
                and y >= 0
                and w >= self.min_roi_width
                and h >= self.min_roi_height
            )
            if roi_valid:
                self.valid_roi_frames += 1
                self.roi_values.append([float(x), float(y), float(w), float(h)])

        depth = result.get("depth_m")
        depth_valid = False
        if isinstance(depth, (int, float)):
            depth = float(depth)
            depth_valid = (
                math.isfinite(depth)
                and self.min_depth_m <= depth <= self.max_depth_m
            )
            if depth_valid:
                self.valid_depth_frames += 1
                self.depth_values.append(depth)

        if self.total_result_frames % 5 == 0:
            self.get_logger().info(
                "progress: "
                f"results={self.total_result_frames}, "
                f"found={self.found_frames}, "
                f"tracking={self.tracking_frames}, "
                f"valid_roi={self.valid_roi_frames}, "
                f"valid_depth={self.valid_depth_frames}, "
                f"last_source={source}, "
                f"last_depth={result.get('depth_m')}"
            )

        if self._pass_conditions_met():
            self.finish(True, "All mineral-water-bottle test criteria passed.")

    def _pass_conditions_met(self):
        return (
            self.found_frames >= 1
            and self.valid_roi_frames >= self.min_tracking_frames
            and self.tracking_frames >= self.min_tracking_frames
            and self.valid_depth_frames >= self.min_valid_depth_frames
        )

    def tick(self):
        if self.finished:
            return

        elapsed = time.monotonic() - self.started_at

        # Publish at t ~= 0, 0.5, 1.0, ... for first 3 seconds.
        if elapsed <= 3.0:
            if time.monotonic() - self.last_command_at >= 0.45:
                self.publish_track_command()

        if elapsed >= self.timeout_sec:
            self.finish(
                self._pass_conditions_met(),
                "Test timeout reached.",
            )

    def finish(self, passed, reason):
        if self.finished:
            return

        self.finished = True
        self.exit_code = 0 if passed else 1

        mean_depth = None
        if self.depth_values:
            mean_depth = sum(self.depth_values) / len(self.depth_values)

        report = {
            "test": "mineral_water_bottle_live_tracking",
            "passed": bool(passed),
            "reason": reason,
            "target": self.target,
            "criteria": {
                "min_tracking_frames": self.min_tracking_frames,
                "min_valid_depth_frames": self.min_valid_depth_frames,
                "depth_range_m": [self.min_depth_m, self.max_depth_m],
                "minimum_roi_size_px": [
                    self.min_roi_width,
                    self.min_roi_height,
                ],
            },
            "observed": {
                "total_result_frames": self.total_result_frames,
                "found_frames": self.found_frames,
                "tracking_frames": self.tracking_frames,
                "valid_roi_frames": self.valid_roi_frames,
                "valid_depth_frames": self.valid_depth_frames,
                "er_flow_frames": self.er_flow_frames,
                "tracker_frames": self.tracker_frames,
                "mean_valid_depth_m": mean_depth,
                "last_result": self.latest_result,
            },
        }

        print("\n" + "=" * 72)
        print("MINERAL WATER BOTTLE TEST:", "PASS" if passed else "FAIL")
        print("=" * 72)
        print(json.dumps(report, indent=2))
        print("=" * 72 + "\n")

        # Stop tracking after test.
        stop = String()
        stop.data = json.dumps({"action": "stop"})
        self.command_pub.publish(stop)

        # Allow output/stop message to flush, then shutdown.
        self.create_timer(0.15, self._shutdown_once)

    def _shutdown_once(self):
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MineralWaterBottleIntegrationTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finished:
            node.finish(False, "Interrupted by user.")
    finally:
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(code)


if __name__ == "__main__":
    main()
