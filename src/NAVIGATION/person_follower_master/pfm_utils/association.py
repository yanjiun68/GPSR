"""Hungarian association with motion (Mahalanobis) + appearance (cosine) cost."""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError as exc:
    raise RuntimeError(
        "scipy is required. Install into the iaig_nav conda env: pip install scipy"
    ) from exc

LARGE_COST = 1e6


def build_cost_matrix(
    tracks: Sequence,
    detections: Sequence,
    alpha_motion: float = 1.0,
    beta_reid: float = 2.0,
    gate_mahalanobis: float = 9.21,
    gate_reid: float = 0.60,
) -> np.ndarray:
    """Construct a gated cost matrix of shape (n_tracks, n_detections).

    `tracks`: iterable of KalmanTrack-like (must expose mahalanobis2(z) and
              best_gallery_similarity(embed)).
    `detections`: iterable of (z_xy: np.ndarray[2], embed: Optional[np.ndarray]).

    Any entry above the gate is set to LARGE_COST so the Hungarian
    solver avoids it and the caller can filter invalid pairs afterwards.
    """
    n_t = len(tracks)
    n_d = len(detections)
    cost = np.full((n_t, n_d), LARGE_COST, dtype=np.float64)

    for i, trk in enumerate(tracks):
        for j, (z, embed) in enumerate(detections):
            m2 = trk.mahalanobis2(np.asarray(z, dtype=np.float64))
            if m2 > gate_mahalanobis:
                continue
            motion_cost = m2

            if embed is not None and trk.gallery:
                sim = trk.best_gallery_similarity(np.asarray(embed))
                reid_cost = 1.0 - sim
                if reid_cost > gate_reid:
                    continue
            else:
                # Motion-only fallback (no embed or empty gallery).
                reid_cost = 0.5  # neutral middle value

            cost[i, j] = alpha_motion * motion_cost + beta_reid * reid_cost

    return cost


def solve_assignment(
    cost: np.ndarray,
    invalid_threshold: float = LARGE_COST / 2.0,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Return (matches, unmatched_tracks, unmatched_detections)."""
    n_t, n_d = cost.shape
    matches: List[Tuple[int, int]] = []
    u_t: List[int] = list(range(n_t))
    u_d: List[int] = list(range(n_d))
    if n_t == 0 or n_d == 0:
        return matches, u_t, u_d

    row_ind, col_ind = linear_sum_assignment(cost)
    for r, c in zip(row_ind, col_ind):
        if cost[r, c] < invalid_threshold:
            matches.append((int(r), int(c)))

    matched_t = {r for r, _ in matches}
    matched_d = {c for _, c in matches}
    u_t = [i for i in range(n_t) if i not in matched_t]
    u_d = [j for j in range(n_d) if j not in matched_d]
    return matches, u_t, u_d
