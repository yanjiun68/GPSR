import cv2
import numpy as np


class TemplateTracker:
    """
    Dependency-light real-time ROI tracker using local template matching.

    ER-Flow periodically reacquires the object. Between ER-Flow calls this
    tracker maintains the ROI at camera frame rate.
    """

    def __init__(
        self,
        search_expansion=2.0,
        match_threshold=0.35,
        template_update_rate=0.04,
    ):
        self.search_expansion = float(search_expansion)
        self.match_threshold = float(match_threshold)
        self.template_update_rate = float(template_update_rate)
        self.roi = None
        self.template = None
        self.score = 0.0

    def reset(self):
        self.roi = None
        self.template = None
        self.score = 0.0

    @staticmethod
    def _gray(image):
        if image.ndim == 2:
            return image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _clip(roi, width, height):
        x, y, w, h = [int(round(v)) for v in roi]
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(2, min(w, width - x))
        h = max(2, min(h, height - y))
        return x, y, w, h

    def initialize(self, image, roi):
        gray = self._gray(image)
        h_img, w_img = gray.shape[:2]
        x, y, w, h = self._clip(roi, w_img, h_img)
        crop = gray[y:y+h, x:x+w]
        if crop.size == 0 or crop.shape[0] < 2 or crop.shape[1] < 2:
            self.reset()
            return False

        self.roi = (x, y, w, h)
        self.template = crop.copy()
        self.score = 1.0
        return True

    def update(self, image):
        if self.roi is None or self.template is None:
            return None

        gray = self._gray(image)
        img_h, img_w = gray.shape[:2]
        x, y, w, h = self.roi

        sw = int(round(w * self.search_expansion))
        sh = int(round(h * self.search_expansion))
        sx = int(round(x + w / 2 - sw / 2))
        sy = int(round(y + h / 2 - sh / 2))
        sx, sy, sw, sh = self._clip((sx, sy, sw, sh), img_w, img_h)

        search = gray[sy:sy+sh, sx:sx+sw]
        th, tw = self.template.shape[:2]

        if search.shape[0] < th or search.shape[1] < tw:
            self.reset()
            return None

        response = cv2.matchTemplate(search, self.template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(response)
        self.score = float(max_val)

        if not np.isfinite(max_val) or max_val < self.match_threshold:
            self.reset()
            return None

        nx = sx + int(max_loc[0])
        ny = sy + int(max_loc[1])
        nx, ny, w, h = self._clip((nx, ny, w, h), img_w, img_h)
        self.roi = (nx, ny, w, h)

        # Slow template adaptation helps with modest appearance changes.
        new_patch = gray[ny:ny+h, nx:nx+w]
        if new_patch.shape == self.template.shape:
            alpha = min(max(self.template_update_rate, 0.0), 1.0)
            self.template = cv2.addWeighted(
                self.template, 1.0 - alpha, new_patch, alpha, 0.0
            )

        return {
            "roi": self.roi,
            "score": self.score,
        }
