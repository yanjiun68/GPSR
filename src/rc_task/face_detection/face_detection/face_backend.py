"""ROS-independent OpenCV YuNet and SFace face inference backend."""

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Optional, Sequence

import cv2
import numpy as np


# Section: Model metadata identifies the embedding space persisted by identity_store.
EMBEDDING_BACKEND = 'opencv_sface'
DEFAULT_YUNET_MODEL = 'face_detection_yunet_2023mar.onnx'
DEFAULT_SFACE_MODEL = 'face_recognition_sface_2021dec.onnx'
MINIMUM_OPENCV_VERSION = '4.5.4'


@dataclass(frozen=True)
class FaceDetection:
    """Special data structure: Store one YuNet result in full-frame coordinates."""

    bbox: tuple[float, float, float, float]
    score: float
    landmarks: np.ndarray
    face_row: np.ndarray


@dataclass(frozen=True)
class FaceObservation:
    """Special data structure: Combine one detection with its normalized SFace feature."""

    bbox: tuple[float, float, float, float]
    score: float
    landmarks: np.ndarray
    embedding: np.ndarray


# Special function: Reject invalid model output instead of publishing a zero or NaN embedding.
def normalize_feature(feature: np.ndarray) -> np.ndarray:
    """Return a finite float32 unit vector or raise ValueError."""
    vector = np.asarray(feature, dtype=np.float32).reshape(-1)
    if vector.size == 0 or not np.all(np.isfinite(vector)):
        raise ValueError('SFace produced an empty or non-finite embedding.')
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 1e-6:
        raise ValueError('SFace produced an embedding with an invalid norm.')
    return np.ascontiguousarray(vector / norm, dtype=np.float32)


# Special function: Resolve a model without relying on a developer-specific home directory.
def resolve_model_path(
    explicit_path: str,
    model_filename: str,
    package_models_dir: str,
    external_model_dir: str = '',
) -> str:
    """Resolve explicit, package-share, then configured external model locations."""
    # Section: Preserve the required search order and report every attempted location on failure.
    candidates = []
    if str(explicit_path).strip():
        candidates.append(Path(str(explicit_path).strip()).expanduser())
    if str(package_models_dir).strip():
        candidates.append(Path(package_models_dir) / model_filename)
    if str(external_model_dir).strip():
        candidates.append(Path(str(external_model_dir).strip()).expanduser() / model_filename)
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    attempted = ', '.join(str(candidate) for candidate in candidates) or '<no paths configured>'
    raise FileNotFoundError(f"Missing face model '{model_filename}'. Checked: {attempted}")


# Special function: Clip an xyxy ROI and reject empty regions before OpenCV inference.
def clip_roi_xyxy(
    roi_xyxy: Optional[Sequence[float]],
    frame_width: int,
    frame_height: int,
) -> tuple[int, int, int, int]:
    """Return a valid integer full-frame ROI, defaulting to the whole frame."""
    if roi_xyxy is None:
        return 0, 0, int(frame_width), int(frame_height)
    if len(roi_xyxy) != 4:
        raise ValueError('ROI must contain exactly four xyxy coordinates.')
    x1, y1, x2, y2 = [float(value) for value in roi_xyxy]
    left = max(0, min(int(np.floor(x1)), int(frame_width)))
    top = max(0, min(int(np.floor(y1)), int(frame_height)))
    right = max(0, min(int(np.ceil(x2)), int(frame_width)))
    bottom = max(0, min(int(np.ceil(y2)), int(frame_height)))
    if right <= left or bottom <= top:
        raise ValueError(f'ROI is empty after clipping: {(left, top, right, bottom)}')
    return left, top, right, bottom


# Special function: Convert one 15-value YuNet face row from ROI-local to global coordinates.
def face_row_to_global(face_row: np.ndarray, offset_x: float, offset_y: float) -> np.ndarray:
    """Translate YuNet bbox origin and five landmarks into full-frame coordinates."""
    row = np.asarray(face_row, dtype=np.float32).reshape(-1).copy()
    if row.size < 15:
        raise ValueError(f'YuNet face row requires 15 values, received {row.size}.')
    # Section: Width, height, and score are invariant; only x/y coordinates are translated.
    row[[0, 4, 6, 8, 10, 12]] += float(offset_x)
    row[[1, 5, 7, 9, 11, 13]] += float(offset_y)
    return row


# Special function: Pad an image to dimensions compatible with YuNet's network stride.
def pad_image_to_stride(
    image: np.ndarray,
    stride: int = 32,
) -> tuple[np.ndarray, int, int]:
    """Pad the bottom and right edges without resizing or distorting the image."""
    # Section: Reject invalid inputs before calculating the padded dimensions.
    if image is None or not isinstance(image, np.ndarray) or image.ndim < 2:
        raise ValueError('Stride padding requires a valid image array.')
    if int(stride) <= 0:
        raise ValueError('Stride must be a positive integer.')
    # Section: Calculate the smallest width and height divisible by the requested stride.
    original_height, original_width = image.shape[:2]
    padded_width = ((original_width + int(stride) - 1) // int(stride)) * int(stride)
    padded_height = ((original_height + int(stride) - 1) // int(stride)) * int(stride)
    # Section: Add only bottom and right padding so all original coordinates remain unchanged.
    padded_image = cv2.copyMakeBorder(
        image,
        0,
        padded_height - original_height,
        0,
        padded_width - original_width,
        cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )
    return padded_image, original_width, original_height


# Special function: Map explicit configuration strings to CPU-first OpenCV DNN constants.
def resolve_dnn_policy(backend: str, target: str) -> tuple[int, int]:
    """Return supported OpenCV backend and target IDs without automatic CUDA selection."""
    normalized_backend = str(backend).strip().lower()
    normalized_target = str(target).strip().lower()
    backend_ids = {
        '': cv2.dnn.DNN_BACKEND_DEFAULT,
        'default': cv2.dnn.DNN_BACKEND_DEFAULT,
        'opencv': cv2.dnn.DNN_BACKEND_OPENCV,
    }
    target_ids = {
        '': cv2.dnn.DNN_TARGET_CPU,
        'cpu': cv2.dnn.DNN_TARGET_CPU,
    }
    if normalized_backend not in backend_ids:
        raise ValueError("opencv_dnn_backend must be 'default' or 'opencv'.")
    if normalized_target not in target_ids:
        raise ValueError("opencv_dnn_target must be 'cpu'; CUDA is never enabled automatically.")
    return backend_ids[normalized_backend], target_ids[normalized_target]


class FaceBackend:
    """Special backend: Run YuNet detection and SFace alignment/recognition through OpenCV DNN."""

    def __init__(
        self,
        yunet_model_path: str,
        sface_model_path: str,
        score_threshold: float = 0.8,
        nms_threshold: float = 0.3,
        top_k: int = 5000,
        dnn_backend: str = 'default',
        dnn_target: str = 'cpu',
    ):
        """Special function: Validate capabilities/models and allocate CPU-first OpenCV resources."""
        # Section: Fail at startup with the installed OpenCV version when required APIs are absent.
        missing_apis = [
            name for name in ('FaceDetectorYN', 'FaceRecognizerSF') if not hasattr(cv2, name)
        ]
        if missing_apis:
            raise RuntimeError(
                f"OpenCV {cv2.__version__} lacks {', '.join(missing_apis)}; "
                f'install OpenCV >= {MINIMUM_OPENCV_VERSION} with YuNet/SFace support.'
            )
        self.yunet_model_path = self._validate_model(yunet_model_path, 'YuNet')
        self.sface_model_path = self._validate_model(sface_model_path, 'SFace')
        backend_id, target_id = resolve_dnn_policy(dnn_backend, dnn_target)
        # Section: Input size is a construction placeholder and is replaced for every frame or ROI.
        self.detector = cv2.FaceDetectorYN.create(
            self.yunet_model_path,
            '',
            (320, 320),
            float(score_threshold),
            float(nms_threshold),
            max(1, int(top_k)),
            backend_id,
            target_id,
        )
        self.recognizer = cv2.FaceRecognizerSF.create(
            self.sface_model_path,
            '',
            backend_id,
            target_id,
        )
        self.embedding_model = Path(self.sface_model_path).name
        self.embedding_dimension: Optional[int] = None
        self.last_timings_ms = {'yunet_ms': 0.0, 'sface_ms': 0.0, 'total_face_pipeline_ms': 0.0}

    @staticmethod
    def _validate_model(model_path: str, label: str) -> str:
        """Special function: Resolve a model path and fail clearly when it is missing."""
        resolved = Path(str(model_path).strip()).expanduser()
        if not resolved.is_file():
            raise FileNotFoundError(f'{label} model file does not exist: {resolved}')
        return str(resolved.resolve())

    def detect(
        self,
        frame: np.ndarray,
        roi_xyxy: Optional[Sequence[float]] = None,
    ) -> list[FaceDetection]:
        """Special function: Detect faces in a frame or person ROI and return global coordinates."""
        if frame is None or not isinstance(frame, np.ndarray) or frame.ndim < 2:
            raise ValueError('Face detection requires a valid image array.')
        frame_height, frame_width = frame.shape[:2]
        left, top, right, bottom = clip_roi_xyxy(roi_xyxy, frame_width, frame_height)
        # Section: Pad the ROI to YuNet's stride without stretching faces or changing coordinates.
        inference_image = np.ascontiguousarray(frame[top:bottom, left:right])
        inference_image, original_width, original_height = pad_image_to_stride(
            inference_image,
            stride=32,
        )
        # Section: Configure YuNet with the padded dimensions before processing this frame.
        padded_height, padded_width = inference_image.shape[:2]
        self.detector.setInputSize((int(padded_width), int(padded_height)))
        started = time.perf_counter()
        _, face_rows = self.detector.detect(inference_image)
        self.last_timings_ms['yunet_ms'] = (time.perf_counter() - started) * 1000.0
        detections = []
        for local_row in [] if face_rows is None else face_rows:
            # Section: Ignore any detection whose center lies in the artificial padded region.
            local_row = np.asarray(local_row, dtype=np.float32).reshape(-1)
            center_x = float(local_row[0] + local_row[2] / 2.0)
            center_y = float(local_row[1] + local_row[3] / 2.0)
            if center_x >= original_width or center_y >= original_height:
                continue
            # Section: Padding preserves scale, so only the original ROI offset is required.
            global_row = face_row_to_global(local_row, left, top)
            landmarks = global_row[4:14].reshape(5, 2).copy()
            detections.append(
                FaceDetection(
                    bbox=tuple(float(value) for value in global_row[:4]),
                    score=float(global_row[14]),
                    landmarks=landmarks,
                    face_row=global_row,
                )
            )
        return detections

    def align(self, frame: np.ndarray, detection: FaceDetection) -> np.ndarray:
        """Special function: Align and crop a full-frame face using YuNet landmarks."""
        aligned = self.recognizer.alignCrop(frame, detection.face_row)
        if aligned is None or aligned.size == 0:
            raise ValueError('SFace alignCrop returned an empty image.')
        return aligned

    def embedding(self, frame: np.ndarray, detection: FaceDetection) -> np.ndarray:
        """Special function: Produce one normalized finite SFace embedding."""
        started = time.perf_counter()
        aligned = self.align(frame, detection)
        embedding = normalize_feature(self.recognizer.feature(aligned))
        self.last_timings_ms['sface_ms'] += (time.perf_counter() - started) * 1000.0
        if self.embedding_dimension is None:
            self.embedding_dimension = int(embedding.size)
        elif int(embedding.size) != self.embedding_dimension:
            raise ValueError(
                f'SFace embedding dimension changed from {self.embedding_dimension} to {embedding.size}.'
            )
        return embedding

    def detect_and_embed(
        self,
        frame: np.ndarray,
        roi_xyxy: Optional[Sequence[float]] = None,
    ) -> list[FaceObservation]:
        """Special function: Run the complete YuNet and SFace pipeline once."""
        pipeline_started = time.perf_counter()
        self.last_timings_ms['sface_ms'] = 0.0
        observations = []
        for detection in self.detect(frame, roi_xyxy=roi_xyxy):
            try:
                feature = self.embedding(frame, detection)
            except (cv2.error, ValueError):
                # Section: One bad alignment must not publish invalid data or discard other valid faces.
                continue
            observations.append(
                FaceObservation(
                    bbox=detection.bbox,
                    score=detection.score,
                    landmarks=detection.landmarks.copy(),
                    embedding=feature,
                )
            )
        self.last_timings_ms['total_face_pipeline_ms'] = (
            time.perf_counter() - pipeline_started
        ) * 1000.0
        return observations
