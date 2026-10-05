import numpy as np


class DepthROI:
    def __init__(
        self,
        depth_scale=0.001,
        min_depth_m=0.15,
        max_depth_m=10.0,
        roi_inner_fraction=0.70,
        min_valid_pixels=20,
    ):
        self.depth_scale = float(depth_scale)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.roi_inner_fraction = float(roi_inner_fraction)
        self.min_valid_pixels = int(min_valid_pixels)

    @staticmethod
    def _clip_roi(roi, width, height):
        x, y, w, h = [int(round(v)) for v in roi]
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))
        return x, y, w, h

    def _inner_roi(self, roi, width, height):
        x, y, w, h = self._clip_roi(roi, width, height)
        f = min(max(self.roi_inner_fraction, 0.10), 1.0)
        iw = max(1, int(round(w * f)))
        ih = max(1, int(round(h * f)))
        ix = x + max(0, (w - iw) // 2)
        iy = y + max(0, (h - ih) // 2)
        return self._clip_roi((ix, iy, iw, ih), width, height)

    def estimate(self, depth_image, roi):
        height, width = depth_image.shape[:2]
        x, y, w, h = self._inner_roi(roi, width, height)

        crop = depth_image[y:y+h, x:x+w]
        if crop.size == 0:
            return None

        if np.issubdtype(crop.dtype, np.floating):
            depth_m = crop.astype(np.float32)
        else:
            depth_m = crop.astype(np.float32) * self.depth_scale

        valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.min_depth_m)
            & (depth_m <= self.max_depth_m)
        )
        values = depth_m[valid]

        if values.size < self.min_valid_pixels:
            return None

        return {
            "depth_m": float(np.median(values)),
            "mean_m": float(np.mean(values)),
            "min_m": float(np.min(values)),
            "max_m": float(np.max(values)),
            "valid_pixels": int(values.size),
            "sample_roi": [int(x), int(y), int(w), int(h)],
        }
