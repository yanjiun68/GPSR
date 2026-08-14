import sys
import types

try:
    from geometry_msgs.msg import PointStamped, Pose, PoseArray
except ModuleNotFoundError:
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")

    class Header:
        def __init__(self):
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)
            self.frame_id = ""

    class Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Orientation:
        def __init__(self):
            self.w = 0.0

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Orientation()

    class PointStamped:
        def __init__(self):
            self.header = Header()
            self.point = Point()

    class PoseArray:
        def __init__(self):
            self.header = Header()
            self.poses = []

    geometry_msgs_msg.PointStamped = PointStamped
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.PoseArray = PoseArray
    geometry_msgs.msg = geometry_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

try:
    from std_msgs.msg import Float32, String
except ModuleNotFoundError:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class Float32:
        def __init__(self, data=0.0):
            self.data = data

    class String:
        def __init__(self, data=""):
            self.data = data

    std_msgs_msg.Float32 = Float32
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

try:
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_time = types.ModuleType("rclpy.time")

    class Node:
        pass

    class Time:
        @classmethod
        def from_msg(cls, msg):
            return msg

    rclpy_node.Node = Node
    rclpy_time.Time = Time
    rclpy.node = rclpy_node
    rclpy.time = rclpy_time
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.time"] = rclpy_time

from bag_point_selection.target_fusion_node import TargetFusionNode


def make_point(x: float, y: float, z: float) -> PointStamped:
    msg = PointStamped()
    msg.point.x = x
    msg.point.y = y
    msg.point.z = z
    return msg


def make_pose(x: float, y: float, z: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def make_node_stub(max_dist: float = 0.30, lidar_age_sec: float = 0.0):
    node = object.__new__(TargetFusionNode)
    node.latest_lidar_clusters = PoseArray()
    node.latest_lidar_clusters.header.stamp.sec = 123
    node.latest_lidar_clusters.header.stamp.nanosec = 0

    params = {
        "base_frame": "base_link",
        "lidar_match_max_dist_m": max_dist,
        "max_lidar_age_sec": 0.75,
    }

    class Param:
        def __init__(self, value):
            self.value = value

    node.get_parameter = lambda name: Param(params[name])
    node._make_point = TargetFusionNode._make_point.__get__(node, TargetFusionNode)
    node._point_distance_xy = TargetFusionNode._point_distance_xy.__get__(node, TargetFusionNode)
    node._closest_lidar_cluster = TargetFusionNode._closest_lidar_cluster.__get__(node, TargetFusionNode)
    node._stamp_age_sec = lambda stamp: lidar_age_sec
    return node


def test_point_distance_xy_ignores_z_difference():
    node = object.__new__(TargetFusionNode)

    point_a = make_point(1.0, 2.0, 0.6)
    point_b = make_point(1.0, 2.0, 0.0)

    assert TargetFusionNode._point_distance_xy(node, point_a, point_b) == 0.0


def test_closest_lidar_cluster_matches_when_xy_is_close_even_if_camera_z_is_raised():
    node = make_node_stub(max_dist=0.30)
    node.latest_lidar_clusters.poses = [make_pose(1.05, 0.02, 99.0)]

    support, dist = node._closest_lidar_cluster(make_point(1.0, 0.0, 0.55))

    assert support is not None
    assert dist is not None
    assert dist < 0.30
    assert support.point.z == 0.0


def test_closest_lidar_cluster_rejects_when_xy_exceeds_match_radius():
    node = make_node_stub(max_dist=0.30)
    node.latest_lidar_clusters.poses = [make_pose(1.31, 0.0, 0.0)]

    support, dist = node._closest_lidar_cluster(make_point(1.0, 0.0, 0.01))

    assert support is None
    assert dist is None


def test_closest_lidar_cluster_rejects_stale_lidar_data():
    node = make_node_stub(max_dist=0.30, lidar_age_sec=1.0)
    node.latest_lidar_clusters.poses = [make_pose(1.02, 0.01, 0.0)]

    support, dist = node._closest_lidar_cluster(make_point(1.0, 0.0, 0.4))

    assert support is None
    assert dist is None
