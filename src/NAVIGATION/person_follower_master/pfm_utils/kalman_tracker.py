"""Constant-velocity Kalman filter for 2D person tracking in the odom frame."""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional

import numpy as np

STATE_DIM = 4  # [x, y, vx, vy]
MEAS_DIM = 2   # [x, y]


def _F(dt: float) -> np.ndarray:
    return np.array(
        [[1.0, 0.0, dt, 0.0],
         [0.0, 1.0, 0.0, dt],
         [0.0, 0.0, 1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)


def _Q(dt: float, sigma_p: float, sigma_v: float) -> np.ndarray:
    sp2 = sigma_p * sigma_p
    sv2 = sigma_v * sigma_v
    return np.diag([
        sp2 * dt * dt * 0.5,
        sp2 * dt * dt * 0.5,
        sv2 * dt,
        sv2 * dt,
    ]).astype(np.float64)


_H = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)


class KalmanTrack:
    """A single pedestrian track with CV motion model + Re-ID gallery."""

    __slots__ = (
        "id", "x", "P", "hits", "lost_count", "age", "last_stamp",
        "status", "gallery", "last_embed", "last_bbox_area",
        "last_embed_ambig",
        "last_meas_stamp", "sigma_p", "sigma_v", "meas_std", "gallery_size",
    )

    def __init__(
        self,
        track_id: int,
        init_xy: np.ndarray,
        stamp: float,
        sigma_p: float = 0.30,
        sigma_v: float = 0.50,
        meas_std: float = 0.08,
        gallery_size: int = 10,
    ) -> None:
        self.id = int(track_id)
        self.x = np.array([float(init_xy[0]), float(init_xy[1]), 0.0, 0.0],
                          dtype=np.float64)
        self.P = np.diag([0.3, 0.3, 1.0, 1.0]).astype(np.float64)
        self.hits = 1
        self.lost_count = 0
        self.age = 1
        self.last_stamp = float(stamp)
        self.last_meas_stamp = float(stamp)
        self.status = "TENTATIVE"
        self.gallery: Deque[np.ndarray] = deque(maxlen=int(gallery_size))
        self.last_embed: Optional[np.ndarray] = None
        self.last_bbox_area: int = 0
        # True when the most recent detection bbox had significant overlap
        # with another person's bbox — the crop mixes pixels from both
        # identities and the embedding is unreliable for gallery updates
        # and identity decisions.
        self.last_embed_ambig: bool = False
        self.sigma_p = float(sigma_p)
        self.sigma_v = float(sigma_v)
        self.meas_std = float(meas_std)
        self.gallery_size = int(gallery_size)

    @property
    def position(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:].copy()

    def predict(self, stamp: float) -> None:
        dt = max(1e-3, min(1.0, stamp - self.last_stamp))
        F = _F(dt)
        Q = _Q(dt, self.sigma_p, self.sigma_v)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_stamp = float(stamp)
        self.age += 1

    def predicted_measurement(self) -> np.ndarray:
        return _H @ self.x  # (2,)

    def innovation_covariance(self) -> np.ndarray:
        R = np.diag([self.meas_std ** 2, self.meas_std ** 2]).astype(np.float64)
        return _H @ self.P @ _H.T + R

    def mahalanobis2(self, z: np.ndarray) -> float:
        S = self.innovation_covariance()
        y = z - self.predicted_measurement()
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return 1e6
        return float(y.T @ S_inv @ y)

    def update(self, z: np.ndarray, stamp: float) -> None:
        S = self.innovation_covariance()
        try:
            K = self.P @ _H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        y = z - self.predicted_measurement()
        self.x = self.x + K @ y
        I = np.eye(STATE_DIM)
        self.P = (I - K @ _H) @ self.P
        self.hits += 1
        self.lost_count = 0
        self.last_meas_stamp = float(stamp)

    def mark_missed(self) -> None:
        self.lost_count += 1

    def push_embedding(self, embed: np.ndarray, bbox_area: int) -> None:
        self.last_embed = embed.astype(np.float32, copy=True)
        self.last_bbox_area = int(bbox_area)
        self.gallery.append(self.last_embed)

    def best_gallery_similarity(self, query: np.ndarray) -> float:
        if query is None or not self.gallery:
            return 0.0
        q = query.astype(np.float32)
        qn = float(np.linalg.norm(q))
        if qn < 1e-8:
            return 0.0
        q = q / qn
        best = 0.0
        for g in self.gallery:
            gn = float(np.linalg.norm(g))
            if gn < 1e-8:
                continue
            sim = float(np.dot(q, g / gn))
            if sim > best:
                best = sim
        return best
