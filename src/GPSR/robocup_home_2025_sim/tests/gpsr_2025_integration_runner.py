#!/usr/bin/env python3
"""Focused, evidence-producing GPSR 2025 simulation integration runner.

The runner never launches hardware, teleports the robot, or injects detections.
Start the simulation/Nav2/GPSR graph first, then run this process once.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = ROOT / "test-results"
STATUSES = ("PASS", "FAIL", "INCONCLUSIVE", "NOT_EXECUTED")


def record(results: list[dict[str, Any]], test_id: str, status: str,
           reason: str, **evidence: Any) -> None:
    assert status in STATUSES
    item = {"test_id": test_id, "status": status, "reason": reason,
            "evidence": evidence}
    results.append(item)
    print(f"{status:12s} {test_id}: {reason}", flush=True)


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as stream:
        tokens: list[bytes] = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise ValueError("truncated PGM header")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        if tokens[0] != b"P5":
            raise ValueError("map must be binary P5 PGM")
        width, height, maximum = map(int, tokens[1:4])
        data = stream.read(width * height)
        if len(data) != width * height:
            raise ValueError("truncated PGM pixels")
    return width, height, maximum, data


def static_checks(results: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    import xml.etree.ElementTree as ET
    world_path = ROOT / "worlds" / "robocup_home_2025_vision.sdf"
    map_yaml_path = ROOT / "maps" / "vision_geometric_reference.yaml"
    waypoint_path = ROOT / "config" / "gpsr_2025_waypoints.yaml"
    knowledge_path = ROOT / "config" / "object_knowledge_2025.yaml"
    try:
        world = ET.parse(world_path).getroot().find("world")
        assert world is not None and world.get("name")
        frames = {f.get("name"): f.findtext("pose") for f in world.findall("frame")}
        assert "instruction_point" in frames
        map_doc = yaml.safe_load(map_yaml_path.read_text())
        wp_doc = yaml.safe_load(waypoint_path.read_text())
        knowledge = yaml.safe_load(knowledge_path.read_text())
        width, height, maximum, pixels = read_pgm(map_yaml_path.parent / map_doc["image"])
        assert maximum == 255
        assert len(knowledge["objects"]) == 21
        assert {"drinks", "food", "snacks", "tableware"}.issubset(knowledge["categories"])
    except Exception as exc:
        record(results, "static_parse", "FAIL", str(exc))
        return {}

    resolution = float(map_doc["resolution"])
    origin_x, origin_y = map(float, map_doc["origin"][:2])
    waypoints: dict[str, dict[str, Any]] = {}
    occupied = [(index % width, index // width) for index, value in enumerate(pixels)
                if value < 250]
    errors: list[str] = []
    cells: dict[str, tuple[int, int]] = {}
    clearances: dict[str, float] = {}
    for entry in wp_doc.get("waypoints", []):
        name = entry["name"]
        position = entry["pose"]["position"]
        x, y = float(position["x"]), float(position["y"])
        col = round((x - origin_x) / resolution)
        row = height - 1 - round((y - origin_y) / resolution)
        if not (0 <= col < width and 0 <= row < height):
            errors.append(f"{name} is outside map")
            continue
        value = pixels[row * width + col]
        clearance = min(math.hypot(col - ox, row - oy) for ox, oy in occupied) * resolution
        clearances[name] = round(clearance, 3)
        cells[name] = (col, row)
        if value < 250 or clearance < 0.30:
            errors.append(f"{name} value={value} clearance={clearance:.3f}m")
        q = entry["pose"]["orientation"]
        norm = math.sqrt(sum(float(q[k]) ** 2 for k in ("x", "y", "z", "w")))
        if abs(norm - 1.0) > 0.01:
            errors.append(f"{name} quaternion norm={norm:.4f}")
        waypoints[name] = entry

    required = {"instruction_point", "living_room", "kitchen", "dining_room", "bedroom"}
    if set(waypoints) != required:
        errors.append(f"waypoint names={sorted(waypoints)}")
    # Four-neighbour connectivity over known free cells; this guards disconnected islands.
    if cells:
        start = next(iter(cells.values()))
        seen = {start}
        queue = deque([start])
        while queue:
            col, row = queue.popleft()
            for nxt in ((col + 1, row), (col - 1, row), (col, row + 1), (col, row - 1)):
                nc, nr = nxt
                if (0 <= nc < width and 0 <= nr < height and nxt not in seen
                        and pixels[nr * width + nc] >= 250):
                    seen.add(nxt)
                    queue.append(nxt)
        disconnected = [name for name, cell in cells.items() if cell not in seen]
        if disconnected:
            errors.append(f"disconnected waypoints={disconnected}")
    record(results, "static_parse", "FAIL" if errors else "PASS",
           "; ".join(errors) if errors else "world/map/waypoints/2025 knowledge parse and validate",
           map_size=[width, height], resolution=resolution, clearances_m=clearances,
           world=str(world_path), map=str(map_yaml_path))
    return waypoints


def runtime_checks(results: list[dict[str, Any]], waypoints: dict[str, dict[str, Any]],
                   timeout: float) -> None:
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image, LaserScan
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool
        from putrabot_interfaces.msg import ObjectDetections
        from robot_ai_interfaces.action import ExecuteActionQueue, NavigateNamedPose
        from robot_ai_interfaces.msg import ActionQueue, ActionStep
    except Exception as exc:
        record(results, "runtime_imports", "NOT_EXECUTED", f"ROS interfaces unavailable: {exc}")
        return

    rclpy.init()

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("gpsr_2025_integration_runner")
            self.samples: dict[str, Any] = {}
            self.cmd_samples: list[dict[str, float]] = []
            self.detection_frames: deque[dict[str, Any]] = deque(maxlen=20)
            self.create_subscription(Clock, "/clock", lambda m: self.samples.__setitem__("clock", m), 10)
            self.create_subscription(LaserScan, "/scan", lambda m: self.samples.__setitem__("scan", m), 10)
            self.create_subscription(Odometry, "/odometry", lambda m: self.samples.__setitem__("odom", m), 20)
            self.create_subscription(Image, "/camera/color/image_raw", lambda m: self.samples.__setitem__("rgb", m), 10)
            self.create_subscription(Image, "/camera/depth/image_raw", lambda m: self.samples.__setitem__("depth", m), 10)
            self.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: self.samples.__setitem__("info", m), 10)
            self.create_subscription(Bool, "/perception/object_detector_ready", lambda m: self.samples.__setitem__("ready", m), 10)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
            self.create_subscription(ObjectDetections, "/perception/raw_object_detections", self.on_detections, 10)
            self.queue_client = ActionClient(self, ExecuteActionQueue, "/execute_action_queue")
            self.named_client = ActionClient(self, NavigateNamedPose, "/nav/navigate_named_pose")

        def on_cmd(self, msg: Twist) -> None:
            self.cmd_samples.append({"wall_time": time.time(), "linear_x": msg.linear.x,
                                     "linear_y": msg.linear.y, "angular_z": msg.angular.z})

        def on_detections(self, msg: ObjectDetections) -> None:
            stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            self.detection_frames.append({"stamp": stamp, "detections": [
                {"label": d.name, "confidence": round(float(d.confidence), 4),
                 "bbox": [d.x1, d.y1, d.x2, d.y2]} for d in msg.detections]})

    node = Probe()

    def spin_until(predicate, seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    spin_until(lambda: all(k in node.samples for k in ("clock", "scan", "odom")), 15.0)
    base_ok = all(k in node.samples for k in ("clock", "scan", "odom"))
    scan_ok = base_ok and any(math.isfinite(v) for v in node.samples["scan"].ranges)
    record(results, "gazebo_bridge", "PASS" if base_ok and scan_ok else "FAIL",
           "clock, finite LiDAR, and odometry observed" if base_ok and scan_ok else "missing /clock, /scan, or /odometry",
           topics=sorted(node.samples), finite_scan=bool(scan_ok))

    spin_until(lambda: all(k in node.samples for k in ("rgb", "depth", "info")), 15.0)
    camera_ok = all(k in node.samples for k in ("rgb", "depth", "info"))
    camera_evidence: dict[str, Any] = {}
    if camera_ok:
        rgb, depth, info = (node.samples[k] for k in ("rgb", "depth", "info"))
        camera_evidence = {"rgb": [rgb.width, rgb.height, rgb.encoding, rgb.header.frame_id],
                           "depth": [depth.width, depth.height, depth.encoding, depth.header.frame_id],
                           "camera_info": [info.width, info.height, info.header.frame_id],
                           "fx_fy": [info.k[0], info.k[4]],
                           "nonzero_rgb_bytes": sum(value != 0 for value in rgb.data[:20000]),
                           "nonzero_depth_bytes": sum(value != 0 for value in depth.data[:20000])}
    record(results, "rgbd_stream", "PASS" if camera_ok and camera_evidence.get("fx_fy", [0])[0] > 0 else "FAIL",
           "live RGB, depth, and calibrated CameraInfo observed" if camera_ok else "camera topics absent",
           **camera_evidence)

    server_ok = node.queue_client.wait_for_server(timeout_sec=10.0)
    named_ok = node.named_client.wait_for_server(timeout_sec=10.0)
    record(results, "action_servers", "PASS" if server_ok and named_ok else "FAIL",
           "executor and named navigation actions available" if server_ok and named_ok else "required action server unavailable",
           execute_action_queue=server_ok, navigate_named_pose=named_ok)

    def pose() -> list[float] | None:
        msg = node.samples.get("odom")
        return None if msg is None else [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def execute_steps(test_id: str, specs: list[dict[str, Any]], action_timeout: float) -> dict[str, Any] | None:
        if not server_ok:
            record(results, test_id, "NOT_EXECUTED", "/execute_action_queue unavailable")
            return None
        goal = ExecuteActionQueue.Goal()
        queue = ActionQueue()
        queue.task_id = test_id
        for index, spec in enumerate(specs):
            step = ActionStep()
            step.step_id = f"{test_id}_{index}"
            step.task_id = test_id
            step.task_family = "GPSR_2025_SIM_VALIDATION"
            step.action_type = spec["action_type"]
            step.target_location = spec.get("target_location", "")
            step.target_object = spec.get("target_object", "")
            step.target_category = spec.get("target_category", "")
            step.parameters_json = json.dumps(spec.get("parameters", {}), sort_keys=True)
            step.timeout_sec = float(spec.get("timeout_sec", action_timeout))
            step.max_retries = 0
            step.requires_success = [f"{test_id}_{i}" for i in spec.get("requires_success_indices", [])]
            step.requires_data = list(spec.get("requires_data", []))
            step.result_variable = spec.get("result_variable", "")
            queue.steps.append(step)
        goal.queue = queue
        start_pose, cmd_start, wall_start = pose(), len(node.cmd_samples), time.monotonic()
        future = node.queue_client.send_goal_async(goal)
        if not spin_until(future.done, 10.0) or not future.result() or not future.result().accepted:
            record(results, test_id, "FAIL", "executor rejected or did not accept goal")
            return None
        result_future = future.result().get_result_async()
        if not spin_until(result_future.done, action_timeout + 10.0):
            future.result().cancel_goal_async()
            record(results, test_id, "FAIL", "bounded executor goal timeout")
            return None
        wrapped = result_future.result()
        end_pose = pose()
        displacement = (math.dist(start_pose, end_pose)
                        if start_pose is not None and end_pose is not None else None)
        moving = [sample for sample in node.cmd_samples[cmd_start:]
                  if abs(sample["linear_x"]) + abs(sample["linear_y"]) + abs(sample["angular_z"]) > 1e-4]
        payload = {"ros_status": wrapped.status, "success": wrapped.result.success,
                   "outcome": wrapped.result.outcome,
                   "skill_results": [{"action_type": r.action_type, "outcome": r.outcome,
                                      "message": r.message, "result_json": r.result_json}
                                     for r in wrapped.result.results],
                   "initial_odom_xy": start_pose, "final_odom_xy": end_pose,
                   "odometry_displacement_m": displacement, "cmd_vel_nonzero_samples": len(moving),
                   "wall_elapsed_sec": round(time.monotonic() - wall_start, 3)}
        expected_not_found = bool(specs[-1].get("expected_not_found", False))
        outcomes = [r.outcome.casefold() for r in wrapped.result.results]
        success = ((wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success)
                   or (expected_not_found and outcomes[-1:] == ["not_found"]
                       and outcomes[:1] == ["success"]))
        record(results, test_id, "PASS" if success else "FAIL",
               wrapped.result.outcome or ("executor queue succeeded" if success else "executor queue failed"), **payload)
        return payload

    # One direct server rejection proves an invalid name is bounded and motion-free.
    if named_ok:
        start_pose, cmd_start = pose(), len(node.cmd_samples)
        goal = NavigateNamedPose.Goal(); goal.target_name = "nonexistent_room"
        future = node.named_client.send_goal_async(goal)
        completed = spin_until(future.done, 5.0)
        accepted = bool(completed and future.result() and future.result().accepted)
        spin_until(lambda: True, 0.2)
        moved = math.dist(start_pose, pose()) if start_pose and pose() else None
        commands = len(node.cmd_samples) - cmd_start
        record(results, "invalid_waypoint", "PASS" if completed and not accepted and (moved or 0) < 0.02 else "FAIL",
               "unknown named waypoint rejected without motion" if completed and not accepted else "invalid waypoint was not cleanly rejected",
               accepted=accepted, odometry_displacement_m=moved, cmd_vel_samples=commands)
    else:
        record(results, "invalid_waypoint", "NOT_EXECUTED", "named navigation server unavailable")

    # Perception evidence must pre-exist these action results; the runner never injects it.
    spin_until(lambda: len(node.detection_frames) >= 5, 10.0)
    fresh_frames = list(node.detection_frames)[-5:]
    detector_ready = bool(getattr(node.samples.get("ready"), "data", False))
    labels = Counter(d["label"] for frame in fresh_frames for d in frame["detections"])
    record(results, "object_detector_frames", "PASS" if detector_ready and len(fresh_frames) >= 5 else "FAIL",
           "five live detector frames and ready signal observed" if detector_ready and len(fresh_frames) >= 5 else "detector not ready or fewer than five frames",
           detector_ready=detector_ready, frames=fresh_frames, label_counts=dict(labels))
    def pattern(test_id: str, room: str, target: str, report_relation: bool = False,
                expected_not_found: bool = False) -> None:
        execute_steps(f"reset_{test_id}", [{"action_type": "NAVIGATE",
                      "target_location": "instruction_point"}], timeout)
        steps = [
            {"action_type": "NAVIGATE", "target_location": room},
            {"action_type": "FIND_OBJECT", "target_object": target,
             "target_location": room, "requires_success_indices": [0],
             "result_variable": "found_object" if report_relation else "",
             "expected_not_found": expected_not_found,
             "parameters": {"search_scope": "STRICT_LOCATION", "approach_target": False,
                            "preferred_viewpoint": room,
                            "query": {"requested_fields": ["image_position",
                                      "furniture_relative_location"]} if report_relation else {}}},
        ]
        if report_relation:
            article = "I found the" if target == "cola" else "The"
            steps.append({"action_type": "SAY", "requires_success_indices": [1],
                          "requires_data": ["found_object"],
                          "parameters": {"text_template":
                              f"{article} {{selection.name}} {{selection.furniture_relative_phrase}}.",
                              "substitution": "found_object"}})
        execute_steps(test_id, steps, timeout * (2 if report_relation else 1))

    pattern("A1", "living_room", "cola")
    pattern("A2", "dining_room", "bowl")
    pattern("B1", "living_room", "cola", report_relation=True)
    pattern("B2", "dining_room", "bowl", report_relation=True)
    pattern("N1", "bedroom", "cola", expected_not_found=True)
    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--static-only", action="store_true")
    parser.add_argument("--timeout-sec", type=float, default=150.0)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS)
    args = parser.parse_args()
    results: list[dict[str, Any]] = []
    waypoints = static_checks(results)
    if not args.static_only and waypoints:
        runtime_checks(results, waypoints, max(10.0, args.timeout_sec))
    args.results_dir.mkdir(parents=True, exist_ok=True)
    counts = Counter(item["status"] for item in results)
    report = {"generated_unix_time": time.time(), "results": results,
              "totals": {status: counts[status] for status in STATUSES}}
    output = args.results_dir / "gpsr_2025_results.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print("TOTALS " + " ".join(f"{s}={counts[s]}" for s in STATUSES), flush=True)
    print(f"REPORT {output}", flush=True)
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
