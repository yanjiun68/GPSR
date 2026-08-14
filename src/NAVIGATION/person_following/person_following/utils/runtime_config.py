import re
from dataclasses import dataclass
from typing import Iterable, Sequence


DEFAULT_ONNX_PROVIDERS = ['CUDAExecutionProvider', 'CPUExecutionProvider']


@dataclass(frozen=True)
class OnnxRuntimeResolution:
    requested: list[str]
    available: list[str]
    resolved: list[str]
    enabled: bool
    reason: str


@dataclass(frozen=True)
class YoloRuntimeResolution:
    requested: str
    resolved: str
    reason: str
    cuda_available: bool
    cuda_device_count: int


def parse_provider_param(value, default: Sequence[str] | None = None) -> list[str]:
    if default is None:
        default = DEFAULT_ONNX_PROVIDERS

    if value is None:
        return list(default)
    if isinstance(value, str):
        raw = value.strip()
        if not raw:
            return list(default)
        if raw.startswith('[') and raw.endswith(']'):
            raw = raw[1:-1]
        items = [item.strip().strip('"').strip("'") for item in raw.split(',')]
        return [item for item in items if item]
    if isinstance(value, Iterable):
        out = []
        for item in value:
            text = str(item).strip()
            if text:
                out.append(text)
        return out if out else list(default)
    return list(default)


def resolve_onnx_providers(
    requested_value,
    ort_module,
    default: Sequence[str] | None = None,
) -> OnnxRuntimeResolution:
    requested = parse_provider_param(requested_value, default)
    if ort_module is None:
        return OnnxRuntimeResolution(
            requested=requested,
            available=[],
            resolved=[],
            enabled=False,
            reason='onnxruntime unavailable',
        )

    try:
        available = list(ort_module.get_available_providers())
    except Exception:
        available = []

    resolved: list[str] = []
    invalid: list[str] = []
    for provider in requested:
        if provider in available and provider not in resolved:
            resolved.append(provider)
        elif provider not in available:
            invalid.append(provider)

    if resolved:
        if invalid:
            reason = f'unavailable providers dropped: {", ".join(invalid)}'
        else:
            reason = 'requested providers available'
        return OnnxRuntimeResolution(
            requested=requested,
            available=available,
            resolved=resolved,
            enabled=True,
            reason=reason,
        )

    if not available:
        reason = 'onnxruntime reported no providers'
    elif invalid:
        reason = f'no requested providers available; dropped: {", ".join(invalid)}'
    else:
        reason = 'no requested providers available'
    return OnnxRuntimeResolution(
        requested=requested,
        available=available,
        resolved=[],
        enabled=False,
        reason=reason,
    )


def resolve_yolo_device(requested_value, torch_module) -> YoloRuntimeResolution:
    requested = str(requested_value or 'auto').strip().lower()
    prefix = ''
    valid = requested in ('auto', 'cpu') or re.fullmatch(r'cuda:\d+', requested) is not None
    if not valid:
        requested = 'auto'
        prefix = 'invalid device request; '

    cuda_available = False
    cuda_device_count = 0
    if torch_module is not None:
        try:
            cuda_available = bool(torch_module.cuda.is_available())
            if cuda_available:
                cuda_device_count = int(torch_module.cuda.device_count())
        except Exception:
            cuda_available = False
            cuda_device_count = 0

    resolved = 'cpu'
    if requested == 'cpu':
        resolved = 'cpu'
        reason = prefix + 'explicit CPU request'
    elif requested == 'auto':
        if cuda_available and cuda_device_count > 0:
            resolved = 'cuda:0'
            reason = prefix + 'auto selected first CUDA device'
        else:
            resolved = 'cpu'
            reason = prefix + 'CUDA unavailable; falling back to CPU'
    else:
        idx = int(requested.split(':', 1)[1])
        if cuda_available and idx < cuda_device_count:
            resolved = requested
            reason = prefix + f'CUDA device {idx} available'
        else:
            resolved = 'cpu'
            reason = prefix + f'requested CUDA device {idx} unavailable; falling back to CPU'

    return YoloRuntimeResolution(
        requested=str(requested_value or 'auto'),
        resolved=resolved,
        reason=reason,
        cuda_available=cuda_available,
        cuda_device_count=cuda_device_count,
    )
