import importlib
from dataclasses import dataclass
from functools import lru_cache
from typing import Any


@dataclass(frozen=True)
class YoloImportResolution:
    available: bool
    torch_module: Any
    yolo_class: Any
    reason: str


def _format_import_error(prefix: str, exc: Exception) -> str:
    message = f'{prefix}: {type(exc).__name__}: {exc}'
    if 'torchvision::nms' in str(exc):
        message += ' (likely torch/torchvision binary mismatch)'
    return message


@lru_cache(maxsize=1)
def resolve_yolo_import() -> YoloImportResolution:
    try:
        torch_module = importlib.import_module('torch')
    except Exception as exc:
        return YoloImportResolution(
            available=False,
            torch_module=None,
            yolo_class=None,
            reason=_format_import_error('torch import failed', exc),
        )

    try:
        ultralytics = importlib.import_module('ultralytics')
        yolo_class = getattr(ultralytics, 'YOLO')
    except Exception as exc:
        return YoloImportResolution(
            available=False,
            torch_module=torch_module,
            yolo_class=None,
            reason=_format_import_error('ultralytics import failed', exc),
        )

    return YoloImportResolution(
        available=True,
        torch_module=torch_module,
        yolo_class=yolo_class,
        reason='ultralytics import succeeded',
    )
