"""Unit tests for the ROS-independent OpenCV YuNet/SFace backend."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from face_detection.face_backend import (
    DEFAULT_SFACE_MODEL,
    DEFAULT_YUNET_MODEL,
    FaceBackend,
    clip_roi_xyxy,
    face_row_to_global,
    normalize_feature,
    pad_image_to_stride,
)


# Section: Embedding validation prevents invalid SFace results from reaching ROS or storage.
def test_embedding_normalization():
    """Special function: Verify SFace output becomes a contiguous float32 unit vector."""
    embedding = normalize_feature(np.array([[3.0, 4.0]], dtype=np.float64))
    assert embedding.dtype == np.float32
    assert embedding.flags['C_CONTIGUOUS']
    assert np.allclose(embedding, [0.6, 0.8])


@pytest.mark.parametrize(
    'feature',
    [[], [0.0, 0.0], [np.nan, 1.0], [np.inf, 1.0]],
)
def test_invalid_embedding_rejection(feature):
    """Special function: Verify empty, zero, NaN, and infinite embeddings are rejected."""
    with pytest.raises(ValueError):
        normalize_feature(feature)


# Section: ROI tests guard the local-to-global coordinate boundary used with person tracking.
def test_roi_global_coordinate_conversion():
    """Special function: Verify bbox origin and five landmarks receive the ROI offset."""
    local_row = np.array(
        [10, 20, 30, 40, 11, 21, 29, 21, 20, 30, 13, 50, 27, 50, 0.95],
        dtype=np.float32,
    )
    global_row = face_row_to_global(local_row, 100, 200)
    assert np.allclose(global_row[:4], [110, 220, 30, 40])
    assert np.allclose(
        global_row[4:14],
        [111, 221, 129, 221, 120, 230, 113, 250, 127, 250],
    )
    assert global_row[14] == pytest.approx(0.95)


def test_roi_clipping():
    """Special function: Verify person ROI coordinates are clipped to the source frame."""
    assert clip_roi_xyxy([-10, 5, 120, 90], 100, 80) == (0, 5, 100, 80)
    with pytest.raises(ValueError):
        clip_roi_xyxy([10, 10, 10, 20], 100, 80)


# Section: Stride padding tests protect YuNet from incompatible dynamic input dimensions.
def test_stride_padding_preserves_pixels_and_coordinates():
    """Special function: Verify a 640x442 image becomes 640x448 without image scaling."""
    image = np.full((442, 640, 3), 127, dtype=np.uint8)
    padded, original_width, original_height = pad_image_to_stride(image)
    assert padded.shape == (448, 640, 3)
    assert (original_width, original_height) == (640, 442)
    assert np.array_equal(padded[:442, :640], image)
    assert np.count_nonzero(padded[442:, :]) == 0


def test_stride_padding_leaves_compatible_dimensions_unchanged():
    """Special function: Verify stride-compatible input retains its dimensions and pixels."""
    image = np.full((448, 640, 3), 63, dtype=np.uint8)
    padded, original_width, original_height = pad_image_to_stride(image)
    assert padded.shape == image.shape
    assert (original_width, original_height) == (640, 448)
    assert np.array_equal(padded, image)


# Section: Model-dependent checks skip clearly when operators have not placed ONNX assets.
def _model_paths():
    """Special function: Resolve source-tree model paths used only by optional load tests."""
    model_dir = Path(__file__).resolve().parents[1] / 'models'
    return model_dir / DEFAULT_YUNET_MODEL, model_dir / DEFAULT_SFACE_MODEL


def _require_models():
    """Special function: Skip model load tests rather than fabricating inference results."""
    yunet_path, sface_path = _model_paths()
    if not yunet_path.is_file() or not sface_path.is_file():
        pytest.skip('YuNet/SFace ONNX model files are not present in package models/.')
    return yunet_path, sface_path


def test_yunet_model_load():
    """Special function: Verify the supplied YuNet model constructs through OpenCV DNN."""
    yunet_path, _ = _require_models()
    detector = cv2.FaceDetectorYN.create(str(yunet_path), '', (320, 320), 0.8, 0.3, 5000)
    assert detector is not None


def test_sface_model_load():
    """Special function: Verify the supplied SFace model constructs through OpenCV DNN."""
    yunet_path, sface_path = _require_models()
    backend = FaceBackend(str(yunet_path), str(sface_path))
    assert backend.recognizer is not None


# Section: Detector integration test verifies padding and padded-region filtering without DNN coupling.
def test_detect_pads_non_aligned_frame_and_ignores_padding_detection():
    """Special function: Verify detect passes 640x448 to YuNet for a 640x442 source frame."""
    # Special test double: Capture the exact size and image passed into the YuNet detector.
    class RecordingDetector:
        """Record detector inputs and return one real-region and one padding-region face."""

        # Special function: Record the dimensions configured by FaceBackend.detect.
        def setInputSize(self, input_size):
            """Store the configured YuNet input size."""
            self.input_size = input_size

        # Special function: Return deterministic face rows on both sides of the padding boundary.
        def detect(self, image):
            """Store the inference image and return controlled fake detections."""
            self.image_shape = image.shape
            rows = np.array(
                [
                    [10, 20, 30, 40, 11, 21, 29, 21, 20, 30, 13, 50, 27, 50, 0.95],
                    [10, 443, 30, 4, 11, 443, 29, 443, 20, 444, 13, 445, 27, 445, 0.90],
                ],
                dtype=np.float32,
            )
            return 1, rows

    # Section: Construct only the state required by detect so this test remains model-independent.
    backend = FaceBackend.__new__(FaceBackend)
    backend.detector = RecordingDetector()
    backend.last_timings_ms = {'yunet_ms': 0.0, 'sface_ms': 0.0, 'total_face_pipeline_ms': 0.0}
    detections = backend.detect(np.zeros((442, 640, 3), dtype=np.uint8))
    assert backend.detector.input_size == (640, 448)
    assert backend.detector.image_shape == (448, 640, 3)
    assert len(detections) == 1
    assert detections[0].bbox == pytest.approx((10.0, 20.0, 30.0, 40.0))
