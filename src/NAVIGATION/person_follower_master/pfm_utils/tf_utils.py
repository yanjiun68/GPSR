"""TF helpers for mapping between base_footprint and odom."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
import tf2_ros


def lookup_transform(
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    source_frame: str,
    stamp: Optional[Time] = None,
    timeout_sec: float = 0.1,
) -> Optional[TransformStamped]:
    """Lookup target<-source, returning None on failure (logs swallowed)."""
    try:
        t = stamp if stamp is not None else Time()
        return tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            t,
            timeout=Duration(seconds=float(timeout_sec)),
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        return None


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def apply_transform_2d(
    tf_msg: TransformStamped,
    x: float,
    y: float,
    z: float = 0.0,
) -> Tuple[float, float, float]:
    R = _quat_to_rot(
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z,
        tf_msg.transform.rotation.w,
    )
    p = np.array([x, y, z], dtype=np.float64)
    t = np.array([
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z,
    ], dtype=np.float64)
    out = R @ p + t
    return float(out[0]), float(out[1]), float(out[2])
