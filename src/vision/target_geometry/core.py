"""Pure numeric stages for target mask, RGB-D mesh, OBB, and grasp analysis."""

from __future__ import annotations

# Section: Typed immutable geometry records prevent arbitrary dictionaries between stages.
from dataclasses import dataclass
import math
from typing import Any

# Section: OpenCV and NumPy provide lightweight organized RGB-D processing without Open3D.
import cv2
import numpy as np


@dataclass(frozen=True)
class OrganizedSurface:
    """Preserve an image-organized target surface before triangulation."""

    points: np.ndarray
    valid_mask: np.ndarray
    source_mask_pixels: int
    valid_depth_pixels: int
    valid_depth_ratio: float


@dataclass(frozen=True)
class TargetMesh:
    """Carry validated dynamic target surface geometry with pixel provenance."""

    vertices: np.ndarray
    triangles: np.ndarray
    triangle_normals: np.ndarray
    vertex_normals: np.ndarray
    pixel_coordinates: np.ndarray
    frame_id: str
    stamp: Any


@dataclass(frozen=True)
class OrientedBoundingBox:
    """Store PCA-derived center, axes, extents, and eigenvalues."""

    center: np.ndarray
    axes: np.ndarray
    extents: np.ndarray
    eigenvalues: np.ndarray


@dataclass(frozen=True)
class GraspCandidate:
    """Store one surface-supported grasp without commanding a manipulator."""

    point: np.ndarray
    approach_vector: np.ndarray
    closing_axis: np.ndarray
    orientation_xyzw: np.ndarray
    quality: float
    status: str
    vertex_index: int


def depth_to_meters(depth_image: np.ndarray, encoding: str) -> np.ndarray:
    """Convert standard ROS depth encodings into explicit metric depth."""
    # Section: Copy input data so filtering cannot mutate a cached ROS image buffer.
    source = np.asarray(depth_image)
    if source.ndim != 2 or source.size == 0:
        raise ValueError("depth image must be a non-empty single-channel array")
    converted = source.astype(np.float32, copy=True)
    normalized = str(encoding or "").strip().upper()
    if normalized in {"16UC1", "MONO16"} or source.dtype == np.uint16:
        converted *= 0.001
    elif normalized == "32FC1" or np.issubdtype(source.dtype, np.floating):
        pass
    else:
        raise ValueError(f"unsupported depth encoding: {encoding}")
    return converted


def scale_camera_matrix(
    camera_matrix: list[float] | tuple[float, ...],
    calibration_size: tuple[int, int],
    image_size: tuple[int, int],
) -> np.ndarray:
    """Scale color intrinsics to an aligned-depth image resolution."""
    # Section: Validate the pinhole calibration and both rectangular image domains.
    if len(camera_matrix) != 9:
        raise ValueError("CameraInfo.k must contain nine values")
    calibration_width, calibration_height = [int(value) for value in calibration_size]
    image_width, image_height = [int(value) for value in image_size]
    if min(calibration_width, calibration_height, image_width, image_height) <= 0:
        raise ValueError("camera and image dimensions must be positive")
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3).copy()
    if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera intrinsics are invalid")
    scale_x = float(image_width) / float(calibration_width)
    scale_y = float(image_height) / float(calibration_height)
    matrix[0, 0] *= scale_x
    matrix[0, 2] *= scale_x
    matrix[1, 1] *= scale_y
    matrix[1, 2] *= scale_y
    return matrix


def validate_binary_mask(
    mask: np.ndarray,
    image_shape: tuple[int, int],
    *,
    minimum_pixels: int,
    maximum_fraction: float,
) -> np.ndarray:
    """Validate and normalize a genuine target segmentation mask."""
    # Section: A mask must cover the image and may not be a rectangle surrogate.
    array = np.asarray(mask)
    expected_height, expected_width = [int(value) for value in image_shape]
    if array.ndim == 3:
        array = np.any(array > 0, axis=2)
    if array.ndim != 2 or array.shape != (expected_height, expected_width):
        raise ValueError("segmentation mask dimensions do not match the RGB image")
    binary = array > 0
    count = int(np.count_nonzero(binary))
    total = max(1, expected_height * expected_width)
    if count < max(1, int(minimum_pixels)):
        raise ValueError(f"segmentation mask has too few pixels: {count}")
    fraction = float(count) / float(total)
    if fraction > float(maximum_fraction):
        raise ValueError(f"segmentation mask is implausibly large: {fraction:.3f}")
    return binary


def select_mask_component(
    mask: np.ndarray,
    *,
    seed_point: tuple[float, float] | None,
    minimum_pixels: int,
) -> np.ndarray:
    """Keep one connected target component selected by grounding evidence."""
    # Section: Eight-neighbour connectivity preserves diagonal object-mask boundaries.
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(binary, 8)
    if count <= 1:
        raise ValueError("segmentation mask contains no foreground component")

    # Section: Prefer the grounded-point component, then fall back to the largest.
    selected_label = 0
    if seed_point is not None:
        seed_u = int(round(float(seed_point[0])))
        seed_v = int(round(float(seed_point[1])))
        if 0 <= seed_v < labels.shape[0] and 0 <= seed_u < labels.shape[1]:
            selected_label = int(labels[seed_v, seed_u])
    if selected_label <= 0:
        component_areas = stats[1:, cv2.CC_STAT_AREA]
        selected_label = int(np.argmax(component_areas)) + 1
    selected = labels == selected_label
    pixels = int(np.count_nonzero(selected))
    if pixels < max(1, int(minimum_pixels)):
        raise ValueError(f"selected mask component has too few pixels: {pixels}")
    return selected


def resize_mask_nearest(mask: np.ndarray, image_shape: tuple[int, int]) -> np.ndarray:
    """Resample an RGB mask into aligned-depth pixel coordinates."""
    # Section: Nearest-neighbour interpolation is mandatory for categorical mask membership.
    target_height, target_width = [int(value) for value in image_shape]
    source = (np.asarray(mask) > 0).astype(np.uint8)
    if source.shape == (target_height, target_width):
        return source.astype(bool)
    resized = cv2.resize(source, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
    return resized > 0


def project_masked_depth(
    depth_m: np.ndarray,
    mask: np.ndarray,
    camera_matrix: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
    minimum_valid_depth_ratio: float,
) -> OrganizedSurface:
    """Back-project only valid segmented depth pixels into optical-frame XYZ."""
    # Section: Target membership comes exclusively from the segmentation mask.
    depth = np.asarray(depth_m, dtype=np.float32)
    binary = np.asarray(mask, dtype=bool)
    if depth.ndim != 2 or binary.shape != depth.shape:
        raise ValueError("depth and segmentation mask must have matching two-dimensional shapes")
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("camera matrix must be 3x3")
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0 or not all(math.isfinite(v) for v in (fx, fy, cx, cy)):
        raise ValueError("camera intrinsics are invalid")

    # Section: Reject zero, non-finite, and configured out-of-range sensor measurements.
    valid = (
        binary
        & np.isfinite(depth)
        & (depth != 0.0)
        & (depth >= float(min_depth_m))
        & (depth <= float(max_depth_m))
    )
    mask_pixels = int(np.count_nonzero(binary))
    valid_pixels = int(np.count_nonzero(valid))
    if mask_pixels <= 0:
        raise ValueError("segmentation mask contains no target pixels")
    valid_ratio = float(valid_pixels) / float(mask_pixels)
    if valid_ratio < float(minimum_valid_depth_ratio):
        raise ValueError(
            f"valid target depth ratio is too low: {valid_ratio:.3f}"
        )

    # Section: Vectorized pinhole projection retains image organization and invalid holes as NaN.
    rows, columns = np.indices(depth.shape, dtype=np.float32)
    points = np.full((*depth.shape, 3), np.nan, dtype=np.float32)
    z_values = depth[valid]
    points[..., 0][valid] = (columns[valid] - cx) * z_values / fx
    points[..., 1][valid] = (rows[valid] - cy) * z_values / fy
    points[..., 2][valid] = z_values
    return OrganizedSurface(points, valid, mask_pixels, valid_pixels, valid_ratio)


def reject_local_depth_outliers(
    surface: OrganizedSurface,
    *,
    kernel_size: int,
    maximum_deviation_m: float,
) -> OrganizedSurface:
    """Remove isolated organized-depth spikes without smoothing mesh geometry."""
    # Section: Median comparison changes membership only; original XYZ samples remain untouched.
    size = max(3, int(kernel_size) | 1)
    depth = np.asarray(surface.points[..., 2], dtype=np.float32)
    filled = np.where(surface.valid_mask, depth, 0.0).astype(np.float32)
    median = cv2.medianBlur(filled, size)
    keep = surface.valid_mask & (
        (median <= 0.0) | (np.abs(depth - median) <= float(maximum_deviation_m))
    )
    points = np.asarray(surface.points).copy()
    points[~keep] = np.nan
    valid_count = int(np.count_nonzero(keep))
    ratio = float(valid_count) / float(max(1, surface.source_mask_pixels))
    return OrganizedSurface(points, keep, surface.source_mask_pixels, valid_count, ratio)


def reject_support_plane(
    surface: OrganizedSurface,
    *,
    distance_threshold_m: float,
    minimum_samples: int = 30,
    minimum_inlier_fraction: float = 0.05,
    maximum_inlier_fraction: float = 0.45,
    iterations: int = 80,
) -> OrganizedSurface:
    """Conservatively remove a horizontal support plane."""
    # Section: Optional RANSAC is conservative so planar object faces survive.
    valid_pixels = np.argwhere(surface.valid_mask)
    points = surface.points[surface.valid_mask].astype(np.float64)
    if len(points) < max(3, int(minimum_samples)):
        return surface
    generator = np.random.default_rng(7)
    best_inliers: np.ndarray | None = None
    for _iteration in range(max(1, int(iterations))):
        sample_indices = generator.choice(len(points), size=3, replace=False)
        first, second, third = points[sample_indices]
        normal = np.cross(second - first, third - first)
        normal_length = float(np.linalg.norm(normal))
        if normal_length <= 1.0e-9:
            continue
        normal /= normal_length
        # Section: In an optical frame, a horizontal tabletop normal is primarily camera Y.
        if abs(float(normal[1])) < 0.65:
            continue
        offset = -float(np.dot(normal, first))
        distances = np.abs(points @ normal + offset)
        inliers = distances <= float(distance_threshold_m)
        new_count = int(np.count_nonzero(inliers))
        best_count = int(np.count_nonzero(best_inliers)) if best_inliers is not None else -1
        if best_inliers is None or new_count > best_count:
            best_inliers = inliers
    if best_inliers is None:
        return surface
    fraction = float(np.count_nonzero(best_inliers)) / float(len(points))
    if fraction < float(minimum_inlier_fraction) or fraction > float(maximum_inlier_fraction):
        return surface

    # Section: Remove plane inliers while retaining nearby lower object-boundary samples.
    keep = surface.valid_mask.copy()
    rejected_pixels = valid_pixels[best_inliers]
    keep[rejected_pixels[:, 0], rejected_pixels[:, 1]] = False
    output_points = surface.points.copy()
    output_points[~keep] = np.nan
    valid_count = int(np.count_nonzero(keep))
    ratio = float(valid_count) / float(max(1, surface.source_mask_pixels))
    return OrganizedSurface(output_points, keep, surface.source_mask_pixels, valid_count, ratio)


def voxel_downsample(points: np.ndarray, leaf_size: float) -> np.ndarray:
    """Reduce analysis density by retaining one centroid per metric voxel."""
    # Section: Voxel reduction applies to analysis, not image-topology mesh faces.
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    array = array[np.all(np.isfinite(array), axis=1)]
    if array.size == 0:
        raise ValueError("no finite points are available for voxel reduction")
    leaf = float(leaf_size)
    if leaf <= 0.0:
        return array.copy()
    keys = np.floor(array / leaf).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse).astype(np.float64)
    reduced = np.column_stack(
        [np.bincount(inverse, weights=array[:, axis]) / counts for axis in range(3)]
    )
    return reduced


def _validated_triangle(
    first: np.ndarray,
    second: np.ndarray,
    third: np.ndarray,
    *,
    max_edge_length: float,
    max_depth_jump: float,
    min_triangle_area: float,
    max_triangle_area: float,
) -> tuple[bool, np.ndarray]:
    """Validate one organized candidate triangle and return its unit normal."""
    # Section: Finite vertices and small local depth variation prevent cross-surface connections.
    vertices = np.asarray([first, second, third], dtype=np.float64)
    if not np.all(np.isfinite(vertices)):
        return False, np.zeros(3, dtype=np.float64)
    if float(np.ptp(vertices[:, 2])) > float(max_depth_jump):
        return False, np.zeros(3, dtype=np.float64)
    edges = (
        np.linalg.norm(vertices[1] - vertices[0]),
        np.linalg.norm(vertices[2] - vertices[1]),
        np.linalg.norm(vertices[0] - vertices[2]),
    )
    if max(edges) > float(max_edge_length):
        return False, np.zeros(3, dtype=np.float64)

    # Section: Cross-product magnitude rejects degeneracy and supplies a normal.
    cross = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
    cross_norm = float(np.linalg.norm(cross))
    area = 0.5 * cross_norm
    if cross_norm <= 1.0e-12 or area < float(min_triangle_area):
        return False, np.zeros(3, dtype=np.float64)
    if math.isfinite(float(max_triangle_area)) and area > float(max_triangle_area):
        return False, np.zeros(3, dtype=np.float64)
    normal = cross / cross_norm
    return True, normal


def build_organized_mesh(
    surface: OrganizedSurface,
    *,
    frame_id: str,
    stamp: Any,
    pixel_stride: int,
    max_edge_length: float,
    max_depth_jump: float,
    min_triangle_area: float,
    max_triangle_area: float = math.inf,
    max_triangles: int = 12000,
) -> TargetMesh:
    """Triangulate valid neighboring masked RGB-D pixels into a polygon mesh."""
    # Section: Direct image-neighborhood triangulation preserves measured surface adjacency.
    points = np.asarray(surface.points, dtype=np.float64)
    valid = np.asarray(surface.valid_mask, dtype=bool)
    if points.ndim != 3 or points.shape[2] != 3 or valid.shape != points.shape[:2]:
        raise ValueError("organized surface arrays are inconsistent")
    triangle_limit = max(1, int(max_triangles))
    # Section: Increase sampling uniformly when the requested limit would otherwise crop the mesh.
    # Two faces per sampled quad gives a conservative full-image complexity estimate.
    valid_pixel_count = int(np.count_nonzero(valid))
    adaptive_stride = int(
        math.ceil(math.sqrt(max(1.0, 2.0 * valid_pixel_count / triangle_limit)))
    )
    stride = max(1, int(pixel_stride), adaptive_stride)
    vertex_lookup: dict[tuple[int, int], int] = {}
    vertices: list[np.ndarray] = []
    pixels: list[tuple[int, int]] = []
    triangles: list[tuple[int, int, int]] = []
    triangle_normals: list[np.ndarray] = []

    def vertex_index(row: int, column: int) -> int:
        """Reuse one mesh vertex for every accepted neighboring face."""
        # Section: Pixel provenance is stored as (u, v) for later interior grasp selection.
        key = (row, column)
        if key not in vertex_lookup:
            vertex_lookup[key] = len(vertices)
            vertices.append(points[row, column].copy())
            pixels.append((column, row))
        return vertex_lookup[key]

    # Section: Each sampled image quad contributes two independently validated local triangles.
    height, width = valid.shape
    stop = False
    for row in range(0, max(0, height - stride), stride):
        if stop:
            break
        for column in range(0, max(0, width - stride), stride):
            corners = (
                (row, column),
                (row, column + stride),
                (row + stride, column),
                (row + stride, column + stride),
            )
            candidates = (
                (corners[0], corners[1], corners[2]),
                (corners[1], corners[3], corners[2]),
            )
            for candidate in candidates:
                if not all(valid[pixel] for pixel in candidate):
                    continue
                first, second, third = [points[pixel] for pixel in candidate]
                accepted, normal = _validated_triangle(
                    first,
                    second,
                    third,
                    max_edge_length=max_edge_length,
                    max_depth_jump=max_depth_jump,
                    min_triangle_area=min_triangle_area,
                    max_triangle_area=max_triangle_area,
                )
                if not accepted:
                    continue

                # Section: Orient surface normals toward the optical camera origin.
                center = (first + second + third) / 3.0
                ordered = candidate
                if float(np.dot(normal, -center)) < 0.0:
                    ordered = (candidate[0], candidate[2], candidate[1])
                    normal = -normal
                triangles.append(tuple(vertex_index(*pixel) for pixel in ordered))
                triangle_normals.append(normal)
                if len(triangles) >= triangle_limit:
                    stop = True
                    break
            if stop:
                break

    # Section: A usable polygon surface requires more than one non-degenerate face.
    if len(triangles) < 2 or len(vertices) < 3:
        raise ValueError("organized triangulation produced too few valid triangles")
    vertex_array = np.asarray(vertices, dtype=np.float64)
    triangle_array = np.asarray(triangles, dtype=np.int32)
    triangle_normal_array = np.asarray(triangle_normals, dtype=np.float64)

    # Section: Area-unweighted accumulation provides stable smoothed normals for grasp scoring.
    vertex_normals = np.zeros_like(vertex_array)
    for face_index, triangle in enumerate(triangle_array):
        vertex_normals[triangle] += triangle_normal_array[face_index]
    lengths = np.linalg.norm(vertex_normals, axis=1)
    good = np.isfinite(lengths) & (lengths > 1.0e-12)
    vertex_normals[good] /= lengths[good, None]
    vertex_normals[~good] = 0.0
    return TargetMesh(
        vertices=vertex_array,
        triangles=triangle_array,
        triangle_normals=triangle_normal_array,
        vertex_normals=vertex_normals,
        pixel_coordinates=np.asarray(pixels, dtype=np.int32),
        frame_id=str(frame_id),
        stamp=stamp,
    )


def compute_oriented_bounding_box(points: np.ndarray) -> OrientedBoundingBox:
    """Calculate a right-handed PCA oriented bounding box from target samples."""
    # Section: PCA operates on cleaned, optionally voxel-reduced target-only points.
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    array = array[np.all(np.isfinite(array), axis=1)]
    if len(array) < 4:
        raise ValueError("at least four finite points are required for an OBB")
    mean = np.mean(array, axis=0)
    centered = array - mean
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = np.maximum(eigenvalues[order], 0.0)
    axes = eigenvectors[:, order]
    if float(np.linalg.det(axes)) < 0.0:
        axes[:, 2] *= -1.0
    local = centered @ axes
    minimum = np.min(local, axis=0)
    maximum = np.max(local, axis=0)
    extents = maximum - minimum
    if not np.all(np.isfinite(extents)) or int(np.count_nonzero(extents > 1.0e-6)) < 2:
        raise ValueError("target geometry is degenerate for OBB calculation")
    # Section: A single-view surface may be exactly planar; retain it with a thin visible OBB axis.
    extents = np.maximum(extents, 0.001)
    local_center = 0.5 * (minimum + maximum)
    center = mean + axes @ local_center
    return OrientedBoundingBox(center, axes, extents, eigenvalues)


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to a normalized ROS quaternion."""
    # Section: Reject non-finite and reflected frames before computing orientation.
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation matrix must be finite and 3x3")
    if float(np.linalg.det(matrix)) < 0.0:
        raise ValueError("rotation matrix must be right-handed")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
        elif axis == 1:
            scale = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
        else:
            scale = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12 or not math.isfinite(norm):
        raise ValueError("rotation produced an invalid quaternion")
    return quaternion / norm


def select_surface_grasp(
    mesh: TargetMesh,
    obb: OrientedBoundingBox,
    mask: np.ndarray,
    *,
    boundary_margin_pixels: float,
    normal_search_radius: float = 0.02,
    lower_boundary_rejection_fraction: float = 0.12,
    minimum_axis_projection: float = 0.25,
) -> GraspCandidate:
    """Select an observed interior surface point and independent gripper axes."""
    # Section: Distance-to-boundary protects contacts from mask rims and missing-depth holes.
    binary = (np.asarray(mask) > 0).astype(np.uint8)
    boundary_distance = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
    pixels = np.asarray(mesh.pixel_coordinates, dtype=np.int32)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
    if len(vertices) == 0 or pixels.shape != (len(vertices), 2):
        raise ValueError("mesh does not contain pixel-correlated vertices")
    distances = boundary_distance[pixels[:, 1], pixels[:, 0]]
    normal_lengths = np.linalg.norm(normals, axis=1)
    candidates = np.isfinite(distances) & (distances >= float(boundary_margin_pixels))
    candidates &= np.isfinite(normal_lengths) & (normal_lengths > 0.5)

    # Section: Avoid the lowest strip where support-plane collision is most likely.
    rejection = max(0.0, min(0.4, lower_boundary_rejection_fraction))
    vertical_limit = float(np.quantile(pixels[:, 1], 1.0 - rejection))
    candidates &= pixels[:, 1] <= vertical_limit
    indices = np.flatnonzero(candidates)
    if indices.size == 0:
        raise ValueError("no mesh vertex satisfies grasp boundary and normal constraints")

    # Section: Score surface vertices; never substitute the centroid as contact.
    scale = max(float(np.max(obb.extents)), 1.0e-6)
    center_distance = np.linalg.norm(vertices[indices] - obb.center, axis=1) / scale
    interior_score = distances[indices] / max(float(np.max(distances[indices])), 1.0)
    view_vectors = -vertices[indices]
    view_lengths = np.linalg.norm(view_vectors, axis=1)
    view_vectors = view_vectors / np.maximum(view_lengths[:, None], 1.0e-9)
    facing_score = np.clip(np.sum(normals[indices] * view_vectors, axis=1), 0.0, 1.0)
    scores = 0.55 * interior_score + 0.30 * facing_score - 0.15 * center_distance
    selected_index = int(indices[int(np.argmax(scores))])
    point = vertices[selected_index].copy()

    # Section: Average camera-consistent normals within the configured metric neighborhood.
    radius = max(0.0, float(normal_search_radius))
    local_distances = np.linalg.norm(vertices - point, axis=1)
    local_mask = (local_distances <= radius) & (normal_lengths > 0.5)
    local_normals = normals[local_mask] if radius > 0.0 else normals[[selected_index]]
    outward_normal = np.sum(local_normals, axis=0)
    outward_length = float(np.linalg.norm(outward_normal))
    if not math.isfinite(outward_length) or outward_length <= 1.0e-9:
        raise ValueError("local surface normals cannot define a grasp approach")
    outward_normal /= outward_length

    # Section: Approach points from free camera-side space into the observed surface.
    approach = -outward_normal
    approach /= np.linalg.norm(approach)

    # Section: Choose the thinnest OBB axis orthogonal to the approach.
    axis_candidates: list[tuple[float, np.ndarray]] = []
    for axis_index in range(3):
        axis = obb.axes[:, axis_index].copy()
        projected = axis - float(np.dot(axis, approach)) * approach
        projected_norm = float(np.linalg.norm(projected))
        if projected_norm > float(minimum_axis_projection):
            axis_candidates.append((float(obb.extents[axis_index]), projected / projected_norm))
    if not axis_candidates:
        raise ValueError("OBB axes cannot define a closing direction orthogonal to approach")
    _extent, closing = min(axis_candidates, key=lambda item: item[0])

    # Section: Pose convention uses local +X as approach and local +Y as parallel-jaw closing axis.
    third_axis = np.cross(approach, closing)
    third_axis /= np.linalg.norm(third_axis)
    closing = np.cross(third_axis, approach)
    closing /= np.linalg.norm(closing)
    orientation = rotation_matrix_to_quaternion_xyzw(
        np.column_stack((approach, closing, third_axis))
    )
    quality = float(np.clip(scores[int(np.argmax(scores))], 0.0, 1.0))
    return GraspCandidate(
        point=point,
        approach_vector=approach,
        closing_axis=closing,
        orientation_xyzw=orientation,
        quality=quality,
        status="VALID_SURFACE_GRASP",
        vertex_index=selected_index,
    )
