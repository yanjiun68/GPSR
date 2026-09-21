"""RViz polygon-mesh and grasp-geometry marker construction."""

from __future__ import annotations

# Section: ROS standard messages represent runtime mesh geometry without a static mesh resource.
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Section: Numeric conversion uses validated typed geometry records from the core pipeline.
import numpy as np

from target_geometry.core import (
    GraspCandidate,
    OrientedBoundingBox,
    TargetMesh,
    rotation_matrix_to_quaternion_xyzw,
)


def _point(values: np.ndarray) -> Point:
    """Convert one finite three-vector into a ROS Point."""
    # Section: Geometry validation occurs upstream; this boundary converts explicit scalars.
    message = Point()
    message.x, message.y, message.z = [float(value) for value in values]
    return message


def _quaternion(values: np.ndarray) -> Quaternion:
    """Convert an XYZW vector into a ROS Quaternion."""
    # Section: Preserve ROS quaternion field ordering explicitly.
    message = Quaternion()
    message.x, message.y, message.z, message.w = [float(value) for value in values]
    return message


def make_grasp_pose(mesh: TargetMesh, grasp: GraspCandidate) -> PoseStamped:
    """Publish the selected surface grasp in the target geometry frame."""
    # Section: This pose contains geometry only and is never sent to MoveIt by this package.
    pose = PoseStamped()
    pose.header.frame_id = mesh.frame_id
    pose.header.stamp = mesh.stamp
    pose.pose.position = _point(grasp.point)
    pose.pose.orientation = _quaternion(grasp.orientation_xyzw)
    return pose


def make_target_markers(
    mesh: TargetMesh,
    obb: OrientedBoundingBox,
    grasp: GraspCandidate,
    *,
    label: str,
    grasp_marker_size: float,
    approach_marker_length: float,
    closing_axis_length: float,
) -> MarkerArray:
    """Build the complete stable-ID RViz target and grasp visualization."""
    # Section: Every marker shares the measured optical frame and observation timestamp.
    markers = MarkerArray()

    # Section: TRIANGLE_LIST is the primary representation for reconstructed geometry.
    mesh_marker = Marker()
    mesh_marker.header.frame_id = mesh.frame_id
    mesh_marker.header.stamp = mesh.stamp
    mesh_marker.ns = "target_mesh"
    mesh_marker.id = 0
    mesh_marker.type = Marker.TRIANGLE_LIST
    mesh_marker.action = Marker.ADD
    mesh_marker.pose.orientation.w = 1.0
    mesh_marker.scale.x = 1.0
    mesh_marker.scale.y = 1.0
    mesh_marker.scale.z = 1.0
    mesh_marker.color = ColorRGBA(r=0.10, g=0.72, b=0.95, a=0.82)
    for triangle in mesh.triangles:
        for vertex_index in triangle:
            mesh_marker.points.append(_point(mesh.vertices[int(vertex_index)]))
    markers.markers.append(mesh_marker)

    # Section: The supplementary wireframe cube exposes PCA pose and metric dimensions.
    obb_marker = Marker()
    obb_marker.header = mesh_marker.header
    obb_marker.ns = "target_geometry"
    obb_marker.id = 0
    obb_marker.type = Marker.CUBE
    obb_marker.action = Marker.ADD
    obb_marker.pose.position = _point(obb.center)
    obb_marker.pose.orientation = _quaternion(rotation_matrix_to_quaternion_xyzw(obb.axes))
    obb_marker.scale.x, obb_marker.scale.y, obb_marker.scale.z = [
        max(0.001, float(value)) for value in obb.extents
    ]
    obb_marker.color = ColorRGBA(r=1.0, g=0.72, b=0.08, a=0.22)
    markers.markers.append(obb_marker)

    # Section: A sphere distinguishes the actual surface contact from the object centroid.
    grasp_marker = Marker()
    grasp_marker.header = mesh_marker.header
    grasp_marker.ns = "target_grasp"
    grasp_marker.id = 0
    grasp_marker.type = Marker.SPHERE
    grasp_marker.action = Marker.ADD
    grasp_marker.pose.position = _point(grasp.point)
    grasp_marker.pose.orientation.w = 1.0
    grasp_marker.scale.x = float(grasp_marker_size)
    grasp_marker.scale.y = float(grasp_marker_size)
    grasp_marker.scale.z = float(grasp_marker_size)
    grasp_marker.color = ColorRGBA(r=0.05, g=1.0, b=0.18, a=1.0)
    markers.markers.append(grasp_marker)

    # Section: The approach arrow points from free space into the selected contact surface.
    approach_marker = Marker()
    approach_marker.header = mesh_marker.header
    approach_marker.ns = "target_grasp"
    approach_marker.id = 1
    approach_marker.type = Marker.ARROW
    approach_marker.action = Marker.ADD
    approach_start = grasp.point - grasp.approach_vector * float(approach_marker_length)
    approach_marker.points = [_point(approach_start), _point(grasp.point)]
    approach_marker.scale.x = max(0.003, float(grasp_marker_size) * 0.20)
    approach_marker.scale.y = max(0.006, float(grasp_marker_size) * 0.45)
    approach_marker.scale.z = max(0.006, float(grasp_marker_size) * 0.45)
    approach_marker.color = ColorRGBA(r=1.0, g=0.12, b=0.08, a=1.0)
    markers.markers.append(approach_marker)

    # Section: LINE_LIST renders the independent parallel-jaw closing direction bidirectionally.
    closing_marker = Marker()
    closing_marker.header = mesh_marker.header
    closing_marker.ns = "target_grasp"
    closing_marker.id = 2
    closing_marker.type = Marker.LINE_LIST
    closing_marker.action = Marker.ADD
    half_closing = 0.5 * float(closing_axis_length) * grasp.closing_axis
    closing_marker.points = [
        _point(grasp.point - half_closing),
        _point(grasp.point + half_closing),
    ]
    closing_marker.scale.x = max(0.002, float(grasp_marker_size) * 0.14)
    closing_marker.color = ColorRGBA(r=0.92, g=0.05, b=1.0, a=1.0)
    markers.markers.append(closing_marker)

    # Section: Concise view-facing text reports semantic identity and geometry validity.
    text_marker = Marker()
    text_marker.header = mesh_marker.header
    text_marker.ns = "target_text"
    text_marker.id = 0
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position = _point(obb.center + np.asarray([0.0, -0.08, 0.0]))
    text_marker.pose.orientation.w = 1.0
    text_marker.scale.z = max(0.025, float(grasp_marker_size))
    text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    text_marker.text = (
        f"{label}\nmesh: valid\ntriangles: {len(mesh.triangles)}\n"
        f"grasp: valid ({grasp.quality:.2f})"
    )
    markers.markers.append(text_marker)
    return markers


def make_delete_all_markers(frame_id: str, stamp: object) -> MarkerArray:
    """Clear every stale target marker after loss, change, or failure."""
    # Section: DELETEALL prevents old geometry from appearing valid after a perception failure.
    marker = Marker()
    marker.header.frame_id = str(frame_id)
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])
