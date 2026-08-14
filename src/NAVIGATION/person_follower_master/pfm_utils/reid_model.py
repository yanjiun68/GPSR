"""OSNet-x0_25 Re-ID feature extractor wrapper (CPU-only, industrial).

Single responsibility: given a BGR image and a list of pixel-space boxes,
return L2-normalised 512-dim embeddings. Designed so a failed/missing
torchreid install surfaces a clear error at construction, not at the first
frame.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

EMBED_DIM = 512


class OsNetExtractor:
    def __init__(
        self,
        model_name: str = "osnet_x0_25",
        device: str = "cpu",
        num_threads: int = 2,
        image_size: Tuple[int, int] = (256, 128),
    ) -> None:
        self.model_name = model_name
        self.device = device
        self.image_size = image_size

        import torch

        torch.set_num_threads(max(1, int(num_threads)))

        # The PyPI torchreid (0.2.5) exposes FeatureExtractor under
        # `torchreid.reid.utils`; the upstream GitHub master branch exposes
        # it under `torchreid.utils`. Try both so the same code runs on
        # either layout.
        FeatureExtractor = None
        _import_err = None
        for mod_path in ("torchreid.utils", "torchreid.reid.utils"):
            try:
                mod = __import__(mod_path, fromlist=["FeatureExtractor"])
                FeatureExtractor = getattr(mod, "FeatureExtractor")
                break
            except Exception as e:
                _import_err = e
                continue
        if FeatureExtractor is None:
            raise RuntimeError(
                "torchreid FeatureExtractor is not importable. Install with: "
                "pip install torchreid (or from git: "
                "pip install git+https://github.com/KaiyangZhou/deep-person-reid.git). "
                f"Last import error: {_import_err}"
            )

        self._torch = torch
        self._extractor = FeatureExtractor(
            model_name=model_name,
            device=device,
            image_size=image_size,
            verbose=False,
        )

    def warmup(self) -> None:
        dummy = np.zeros((self.image_size[0], self.image_size[1], 3), dtype=np.uint8)
        try:
            _ = self._extractor([dummy])
        except Exception:
            pass

    def extract(
        self,
        bgr_image: np.ndarray,
        boxes_xyxy: Sequence[Tuple[int, int, int, int]],
    ) -> Tuple[np.ndarray, List[int]]:
        """Extract L2-normalised features for the given boxes.

        Returns (embeddings[N, 512], kept_indices) where kept_indices maps
        each row to the caller's original box index (rejected boxes are
        skipped — caller must align with this index list).
        """
        if bgr_image is None or len(boxes_xyxy) == 0:
            return np.zeros((0, EMBED_DIM), dtype=np.float32), []

        h, w = bgr_image.shape[:2]
        crops: List[np.ndarray] = []
        kept: List[int] = []
        for i, (x1, y1, x2, y2) in enumerate(boxes_xyxy):
            x1 = int(max(0, min(w - 1, x1)))
            y1 = int(max(0, min(h - 1, y1)))
            x2 = int(max(0, min(w, x2)))
            y2 = int(max(0, min(h, y2)))
            if x2 - x1 < 8 or y2 - y1 < 16:
                continue
            crop_bgr = bgr_image[y1:y2, x1:x2]
            crop_rgb = crop_bgr[:, :, ::-1].copy()
            crops.append(crop_rgb)
            kept.append(i)

        if not crops:
            return np.zeros((0, EMBED_DIM), dtype=np.float32), []

        with self._torch.no_grad():
            feats = self._extractor(crops)  # (N, D) torch tensor on self.device

        feats = feats.detach().cpu().numpy().astype(np.float32)
        norm = np.linalg.norm(feats, axis=1, keepdims=True)
        norm = np.where(norm < 1e-8, 1.0, norm)
        feats = feats / norm

        if feats.shape[1] != EMBED_DIM:
            # OSNet-x0_25 default embedding is 512; guard anyway.
            return feats.astype(np.float32), kept
        return feats.astype(np.float32), kept


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    """Cosine similarity between two unit (or near-unit) vectors."""
    if a is None or b is None:
        return 0.0
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-8 or nb < 1e-8:
        return 0.0
    return float(np.dot(a, b) / (na * nb))
