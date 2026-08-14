from typing import Dict, Iterable, Optional

import numpy as np


CANONICAL_REID_REASONS = {
    'invalid_bbox',
    'crop_too_small',
    'extractor_returned_none',
    'rgb_unavailable',
    'rgb_stale',
}

REID_OVERLAY_REASON_ALIASES = {
    'invalid_bbox': 'bad_bbox',
    'crop_too_small': 'crop_small',
    'extractor_returned_none': 'extract_none',
    'rgb_unavailable': 'rgb_none',
    'rgb_stale': 'rgb_stale',
}


def canonical_reid_reason(reason: Optional[str]) -> str:
    text = str(reason or '').strip()
    return text if text in CANONICAL_REID_REASONS else 'extractor_returned_none'


def overlay_reid_reason(reason: Optional[str]) -> str:
    canonical = canonical_reid_reason(reason)
    return REID_OVERLAY_REASON_ALIASES.get(canonical, canonical)


def format_reid_preview(embedding, preview_n: int) -> str:
    if embedding is None:
        return '[]'
    arr = np.asarray(embedding, dtype=np.float32).reshape(-1)
    if arr.size == 0:
        return '[]'
    count = max(1, min(int(preview_n), int(arr.size)))
    return np.array2string(
        arr[:count],
        precision=3,
        separator=', ',
        suppress_small=False,
    )


def format_reid_log_ok(
    src: str,
    target_id: int,
    fields: Optional[Iterable[tuple[str, str]]] = None,
) -> str:
    parts = [f'[ReID] ok src={src} id={int(target_id)}']
    if fields is not None:
        for key, value in fields:
            if value is None or value == '':
                continue
            parts.append(f'{key}={value}')
    return ' '.join(parts)


def format_reid_log_fail(
    src: str,
    target_id: int,
    reason: Optional[str],
    fields: Optional[Iterable[tuple[str, str]]] = None,
) -> str:
    parts = [f'[ReID] fail src={src} id={int(target_id)} reason={canonical_reid_reason(reason)}']
    if fields is not None:
        for key, value in fields:
            if value is None or value == '':
                continue
            parts.append(f'{key}={value}')
    return ' '.join(parts)


def should_emit_reid_log(
    last_emit_sec: Dict[str, float],
    last_status: Dict[str, str],
    key: str,
    status: str,
    now_sec: float,
    period_sec: float,
) -> bool:
    prev_status = last_status.get(key)
    last_emit = last_emit_sec.get(key)
    if prev_status != status:
        last_status[key] = status
        last_emit_sec[key] = now_sec
        return True
    if last_emit is None:
        last_status[key] = status
        last_emit_sec[key] = now_sec
        return True
    if period_sec <= 0.0 or (now_sec - last_emit) >= period_sec:
        last_status[key] = status
        last_emit_sec[key] = now_sec
        return True
    return False
