"""Hardware-independent tests for semantic target mesh and grasp geometry."""

from __future__ import annotations

# Section: ROS message constants verify RViz output without running a ROS graph.
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

# Section: NumPy creates deterministic synthetic masks and aligned depth surfaces.
import numpy as np
import pytest

from target_geometry.core import (
    _validated_triangle,
    build_organized_mesh,
    compute_oriented_bounding_box,
    depth_to_meters,
    project_masked_depth,
    scale_camera_matrix,
    select_surface_grasp,
    validate_binary_mask,
)
from target_geometry.grounding import GroundingError, parse_qwen_grounding
from target_geometry.markers import make_delete_all_markers, make_grasp_pose, make_target_markers


def synthetic_surface(
    *,
    height: int = 20,
    width: int = 24,
    depth_value: float = 1.0,
):
    """Build one planar segmented RGB-D target for repeated geometry tests."""
    # Section: A background border makes distance-to-boundary scoring meaningful.
    depth = np.full((height, width), 2.0, dtype=np.float32)
    mask = np.zeros((height, width), dtype=bool)
    mask[3:height - 3, 4:width - 4] = True
    depth[mask] = float(depth_value)
    camera_matrix = np.asarray(
        [[140.0, 0.0, width / 2.0], [0.0, 140.0, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    surface = project_masked_depth(
        depth,
        mask,
        camera_matrix,
        min_depth_m=0.15,
        max_depth_m=3.0,
        minimum_valid_depth_ratio=0.9,
    )
    return surface, mask


def synthetic_mesh():
    """Triangulate the shared planar target with camera-facing normals."""
    # Section: Conservative thresholds admit adjacent points but no distant surface connection.
    surface, mask = synthetic_surface()
    mesh = build_organized_mesh(
        surface,
        frame_id="camera_color_optical_frame",
        stamp=Time(sec=12, nanosec=34),
        pixel_stride=1,
        max_edge_length=0.05,
        max_depth_jump=0.02,
        min_triangle_area=1.0e-8,
        max_triangles=5000,
    )
    return mesh, mask


def test_qwen_json_parsing_and_coordinate_conversion() -> None:
    """Convert the existing normalized Qwen schema into bounded source pixels."""
    # Section: A red semantic descriptor still matches the returned physical class cup.
    response = """{
      "selected_entity": {
        "target_id": "red cup",
        "label": "cup",
        "bbox_2d": [250, 200, 750, 800],
        "point_2d": [500, 500],
        "coordinate_width": 1000,
        "coordinate_height": 1000,
        "confidence": 0.91
      }
    }"""
    grounded = parse_qwen_grounding(
        response,
        requested_target="red cup",
        image_width=640,
        image_height=480,
        minimum_confidence=0.35,
        minimum_bbox_fraction=0.001,
        maximum_bbox_fraction=0.8,
    )
    assert grounded.label == "cup"
    assert grounded.bbox_xyxy == (160, 96, 480, 384)
    assert grounded.point_uv == (320.0, 240.0)


def test_qwen_absent_target_is_never_fabricated() -> None:
    """Reject an explicit model absence result before segmentation."""
    with pytest.raises(GroundingError, match="not visible"):
        parse_qwen_grounding(
            '{"found": false, "instances": []}',
            requested_target="coke",
            image_width=640,
            image_height=480,
            minimum_confidence=0.35,
            minimum_bbox_fraction=0.001,
            maximum_bbox_fraction=0.8,
        )


def test_qwen_ambiguous_instances_are_rejected() -> None:
    """Reject multiple similarly likely instances without a spatial qualifier."""
    # Section: A semantic request without left/right evidence must fail closed.
    response = """{
      "found": true,
      "instances": [
        {"label": "cup", "confidence": 0.82, "bbox_2d": [10, 10, 80, 90]},
        {"label": "cup", "confidence": 0.80, "bbox_2d": [100, 10, 170, 90]}
      ]
    }"""
    with pytest.raises(GroundingError, match="ambiguous"):
        parse_qwen_grounding(
            response,
            requested_target="cup",
            image_width=200,
            image_height=100,
            minimum_confidence=0.35,
            minimum_bbox_fraction=0.001,
            maximum_bbox_fraction=0.8,
        )


def test_mask_validation_rejects_empty_and_oversized_masks() -> None:
    """Enforce target-support and scene-fraction mask limits."""
    with pytest.raises(ValueError, match="too few"):
        validate_binary_mask(
            np.zeros((20, 20), dtype=np.uint8),
            (20, 20),
            minimum_pixels=10,
            maximum_fraction=0.7,
        )
    with pytest.raises(ValueError, match="implausibly large"):
        validate_binary_mask(
            np.ones((20, 20), dtype=np.uint8),
            (20, 20),
            minimum_pixels=10,
            maximum_fraction=0.7,
        )


def test_depth_units_and_masked_extraction() -> None:
    """Convert millimetres and exclude every unsegmented background pixel."""
    depth_mm = np.full((10, 10), 2500, dtype=np.uint16)
    mask = np.zeros((10, 10), dtype=bool)
    mask[2:8, 3:7] = True
    depth_mm[mask] = 1000
    depth_m = depth_to_meters(depth_mm, "16UC1")
    surface = project_masked_depth(
        depth_m,
        mask,
        np.asarray([[100.0, 0.0, 5.0], [0.0, 100.0, 5.0], [0.0, 0.0, 1.0]]),
        min_depth_m=0.15,
        max_depth_m=2.0,
        minimum_valid_depth_ratio=1.0,
    )
    assert surface.valid_depth_pixels == 24
    assert np.allclose(surface.points[..., 2][surface.valid_mask], 1.0)
    assert np.all(np.isnan(surface.points[..., 2][~surface.valid_mask]))


def test_projection_uses_calibrated_intrinsics() -> None:
    """Validate metric 2D-to-3D pinhole projection and intrinsics scaling."""
    depth = np.asarray([[2.0]], dtype=np.float32)
    mask = np.asarray([[True]])
    matrix = scale_camera_matrix(
        [100.0, 0.0, 0.0, 0.0, 200.0, 0.0, 0.0, 0.0, 1.0],
        (1, 1),
        (1, 1),
    )
    surface = project_masked_depth(
        depth,
        mask,
        matrix,
        min_depth_m=0.1,
        max_depth_m=3.0,
        minimum_valid_depth_ratio=1.0,
    )
    assert np.allclose(surface.points[0, 0], [0.0, 0.0, 2.0])


def test_synthetic_depth_boundary_is_not_triangulated() -> None:
    """Never connect a foreground plane to greater-depth background."""
    # Section: A broad mask tests depth-jump rejection apart from segmentation.
    height, width = 8, 10
    depth = np.full((height, width), 1.30, dtype=np.float32)
    depth[:, :5] = 1.00
    mask = np.ones((height, width), dtype=bool)
    matrix = np.asarray([[100.0, 0.0, 5.0], [0.0, 100.0, 4.0], [0.0, 0.0, 1.0]])
    surface = project_masked_depth(
        depth,
        mask,
        matrix,
        min_depth_m=0.1,
        max_depth_m=2.0,
        minimum_valid_depth_ratio=1.0,
    )
    mesh = build_organized_mesh(
        surface,
        frame_id="camera",
        stamp=Time(),
        pixel_stride=1,
        max_edge_length=0.5,
        max_depth_jump=0.02,
        min_triangle_area=1.0e-8,
        max_triangles=1000,
    )
    for triangle in mesh.triangles:
        assert np.ptp(mesh.vertices[triangle, 2]) <= 0.02


def test_triangle_construction_and_normals_are_valid() -> None:
    """Build finite triangles with consistent camera-facing unit normals."""
    mesh, _mask = synthetic_mesh()
    assert len(mesh.triangles) > 2
    assert mesh.triangles.shape[1] == 3
    assert np.all(np.isfinite(mesh.triangle_normals))
    assert np.allclose(np.linalg.norm(mesh.triangle_normals, axis=1), 1.0)
    assert np.all(mesh.triangle_normals[:, 2] < 0.0)


def test_degenerate_triangle_is_rejected() -> None:
    """Reject collinear geometry before it can create NaN normals."""
    accepted, normal = _validated_triangle(
        np.asarray([0.0, 0.0, 1.0]),
        np.asarray([0.01, 0.0, 1.0]),
        np.asarray([0.02, 0.0, 1.0]),
        max_edge_length=0.1,
        max_depth_jump=0.02,
        min_triangle_area=1.0e-8,
        max_triangle_area=1.0,
    )
    assert not accepted
    assert np.allclose(normal, 0.0)


def test_obb_and_surface_grasp_are_distinct_from_centroid() -> None:
    """Select a real interior surface vertex with orthogonal grasp axes."""
    mesh, mask = synthetic_mesh()
    obb = compute_oriented_bounding_box(mesh.vertices)
    grasp = select_surface_grasp(
        mesh,
        obb,
        mask,
        boundary_margin_pixels=2.0,
    )
    assert np.all(obb.extents > 0.0)
    assert any(np.allclose(grasp.point, vertex) for vertex in mesh.vertices)
    assert not np.allclose(grasp.point, obb.center)
    assert np.isclose(np.linalg.norm(grasp.approach_vector), 1.0)
    assert np.isclose(np.linalg.norm(grasp.closing_axis), 1.0)
    assert abs(float(np.dot(grasp.approach_vector, grasp.closing_axis))) < 1.0e-6
    assert np.isclose(np.linalg.norm(grasp.orientation_xyzw), 1.0)


def test_marker_generation_contains_polygon_obb_and_grasp_geometry() -> None:
    """Build required RViz displays with stable identities."""
    mesh, mask = synthetic_mesh()
    obb = compute_oriented_bounding_box(mesh.vertices)
    grasp = select_surface_grasp(mesh, obb, mask, boundary_margin_pixels=2.0)
    markers = make_target_markers(
        mesh,
        obb,
        grasp,
        label="coke",
        grasp_marker_size=0.03,
        approach_marker_length=0.12,
        closing_axis_length=0.10,
    )
    by_identity = {(marker.ns, marker.id): marker for marker in markers.markers}
    mesh_marker = by_identity[("target_mesh", 0)]
    assert mesh_marker.type == Marker.TRIANGLE_LIST
    assert len(mesh_marker.points) == len(mesh.triangles) * 3
    assert by_identity[("target_geometry", 0)].type == Marker.CUBE
    assert by_identity[("target_grasp", 0)].type == Marker.SPHERE
    assert by_identity[("target_grasp", 1)].type == Marker.ARROW
    assert by_identity[("target_grasp", 2)].type == Marker.LINE_LIST
    assert by_identity[("target_text", 0)].type == Marker.TEXT_VIEW_FACING
    pose = make_grasp_pose(mesh, grasp)
    assert pose.header.frame_id == "camera_color_optical_frame"


def test_marker_cleanup_uses_delete_all() -> None:
    """Remove every stale namespace after loss or geometry failure."""
    cleanup = make_delete_all_markers("camera_color_optical_frame", Time(sec=1))
    assert len(cleanup.markers) == 1
    assert cleanup.markers[0].action == Marker.DELETEALL
