"""Fail-closed conversion from experiment data to Agent-visible health."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from math import isfinite
from typing import Any

from pydantic import ValidationError

from resilient_nav_agent.schemas import HealthObservation, HealthState


GROUND_TRUTH_KEYS = frozenset({
    'faultstatus',
    'fault_status',
    'fault_injection_status',
    'scenario_id',
    'scenario_seed',
    'event_id',
    'faulted_topic',
    'model',
    'start_time',
    'end_time',
    'severity',
    'affected_fields',
    'parameters_yaml',
    'ground_truth',
    'truth_label',
    'expected_fault',
    'expected_state',
    'fault_model_truth',
    'benchmark_answer',
})

GROUND_TRUTH_TEXT_MARKERS = frozenset({
    'faultstatus',
    'scenario_id',
    'scenario_seed',
    'parameters_yaml',
    'ground_truth',
    'ground truth',
    'benchmark_answer',
    'benchmark answer',
    'truth_metadata',
    'fault model truth',
})

_ALLOWED_TOP_LEVEL = frozenset({
    'header',
    'sensor',
    'component',
    'source_topic',
    'state',
    'health_score',
    'confidence',
    'detector_confidence',
    'detected_fault',
    'detected_fault_hint',
    'reasons',
    'metric_names',
    'metric_values',
    'window',
    'window_start',
    'window_end',
    'window_start_sec',
    'window_end_sec',
    'sample_count',
    'observed_at_sec',
})
_STATE_BY_NUMBER = {
    0: HealthState.UNKNOWN,
    1: HealthState.HEALTHY,
    2: HealthState.DEGRADED,
    3: HealthState.FAULT,
}
_SAFE_MESSAGES = {
    'dangerous_content': 'Agent input contains prohibited content.',
    'encoded_payload': 'Agent input contains an encoded or oversized payload.',
    'ground_truth_field': 'Agent input contains a prohibited ground-truth field.',
    'invalid_health': 'Health input cannot be converted to the canonical contract.',
    'invalid_metrics': 'Health metrics are invalid or inconsistent.',
    'invalid_structure': 'Agent input has an invalid structure.',
    'unknown_field': 'Agent input contains an undeclared field.',
    'unsupported_component': 'Health input component is unsupported.',
}


class SanitizationError(ValueError):
    """Stable domain error that never echoes rejected payload contents."""

    def __init__(self, code: str):
        self.code = code if code in _SAFE_MESSAGES else 'invalid_structure'
        super().__init__(_SAFE_MESSAGES[self.code])


def _normalized_key(value: str) -> str:
    return value.strip().casefold().replace('-', '_').replace(' ', '_')


def _scan_for_prohibited_content(
    value: Any,
    *,
    path: tuple[str, ...] = (),
    depth: int = 0,
) -> None:
    if depth > 8:
        raise SanitizationError('invalid_structure')
    if isinstance(value, Mapping):
        if len(value) > 256:
            raise SanitizationError('invalid_structure')
        for key, item in value.items():
            if not isinstance(key, str):
                raise SanitizationError('invalid_structure')
            if _normalized_key(key) in GROUND_TRUTH_KEYS:
                raise SanitizationError('ground_truth_field')
            _scan_for_prohibited_content(
                item,
                path=path + (key,),
                depth=depth + 1,
            )
        return
    if isinstance(value, Sequence) and not isinstance(
        value,
        (str, bytes, bytearray),
    ):
        if len(value) > 512:
            raise SanitizationError('invalid_structure')
        for item in value:
            _scan_for_prohibited_content(item, path=path, depth=depth + 1)
        return
    if isinstance(value, (bytes, bytearray)):
        raise SanitizationError('encoded_payload')
    if isinstance(value, float) and not isfinite(value):
        raise SanitizationError('invalid_health')
    if not isinstance(value, str):
        return
    lowered = value.casefold()
    if '/fault_injection/status' in lowered:
        raise SanitizationError('dangerous_content')
    if any(marker in lowered for marker in GROUND_TRUTH_TEXT_MARKERS):
        raise SanitizationError('dangerous_content')
    source_topic_only = path == ('source_topic',)
    if '/faulted/' in lowered and not source_topic_only:
        raise SanitizationError('dangerous_content')
    stripped = lowered.lstrip()
    if stripped.startswith('data:') or 'base64,' in stripped:
        raise SanitizationError('encoded_payload')
    if len(value) > 2048:
        raise SanitizationError('encoded_payload')


def _validate_fields(
    value: Mapping[str, Any],
    allowed: frozenset[str],
) -> None:
    if any(key not in allowed for key in value):
        raise SanitizationError('unknown_field')


def _time_to_seconds(value: Any) -> float:
    if isinstance(value, bool):
        raise SanitizationError('invalid_health')
    if isinstance(value, (int, float)):
        result = float(value)
        if not isfinite(result) or result < 0.0:
            raise SanitizationError('invalid_health')
        return result
    if not isinstance(value, Mapping):
        raise SanitizationError('invalid_health')
    _validate_fields(value, frozenset({'sec', 'nanosec'}))
    sec = value.get('sec')
    nanosec = value.get('nanosec')
    if (
        isinstance(sec, bool)
        or isinstance(nanosec, bool)
        or not isinstance(sec, int)
        or not isinstance(nanosec, int)
        or sec < 0
        or nanosec < 0
        or nanosec >= 1_000_000_000
    ):
        raise SanitizationError('invalid_health')
    return sec + nanosec / 1_000_000_000.0


def _component_name(raw: Mapping[str, Any]) -> str:
    candidates = []
    for key in ('component', 'sensor'):
        if key in raw:
            candidates.append(raw[key])
    if not candidates and 'source_topic' in raw:
        candidates.append(raw['source_topic'])
    normalized = [_normalize_component(value) for value in candidates]
    if not normalized or any(item != normalized[0] for item in normalized[1:]):
        raise SanitizationError('unsupported_component')
    return normalized[0]


def _normalize_component(value: Any) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SanitizationError('unsupported_component')
    lowered = value.strip().casefold()
    tokens = {token for token in lowered.replace('-', '/').split('/') if token}
    if 'imu' in tokens or lowered == 'imu':
        return 'imu'
    if {'wheel', 'odometry', 'odom'}.intersection(tokens):
        return 'wheel'
    if {'scan', 'lidar', 'laser'}.intersection(tokens):
        return 'scan'
    if {'camera', 'c920', 'image', 'image_raw'}.intersection(tokens):
        return 'camera'
    raise SanitizationError('unsupported_component')


def _health_state(value: Any) -> HealthState:
    if isinstance(value, bool):
        raise SanitizationError('invalid_health')
    if isinstance(value, int):
        try:
            return _STATE_BY_NUMBER[value]
        except KeyError:
            raise SanitizationError('invalid_health') from None
    if isinstance(value, str):
        try:
            return HealthState(value.strip().upper())
        except ValueError:
            raise SanitizationError('invalid_health') from None
    raise SanitizationError('invalid_health')


def _optional_score(raw: Mapping[str, Any], *keys: str) -> float | None:
    present = [key for key in keys if key in raw]
    if len(present) > 1:
        raise SanitizationError('invalid_health')
    if not present or raw[present[0]] is None:
        return None
    value = raw[present[0]]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SanitizationError('invalid_health')
    score = float(value)
    if not isfinite(score):
        raise SanitizationError('invalid_health')
    if score == -1.0:
        return None
    if score < 0.0 or score > 1.0:
        raise SanitizationError('invalid_health')
    return score


def _fault_hint(raw: Mapping[str, Any]) -> str | None:
    present = [
        key for key in ('detected_fault', 'detected_fault_hint') if key in raw
    ]
    if len(present) > 1:
        raise SanitizationError('invalid_health')
    if not present or raw[present[0]] is None:
        return None
    value = raw[present[0]]
    if not isinstance(value, str):
        raise SanitizationError('invalid_health')
    hint = value.strip()
    if not hint or hint.casefold() == 'none':
        return None
    if len(hint) > 128:
        raise SanitizationError('invalid_health')
    return hint


def _reasons(value: Any) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, Sequence) or isinstance(
        value,
        (str, bytes, bytearray),
    ):
        raise SanitizationError('invalid_health')
    if len(value) > 32:
        raise SanitizationError('invalid_health')
    cleaned = []
    for item in value:
        if not isinstance(item, str):
            raise SanitizationError('invalid_health')
        reason = item.strip()
        if not reason:
            continue
        if len(reason) > 256:
            raise SanitizationError('encoded_payload')
        cleaned.append(reason)
    return cleaned


def _metrics(raw: Mapping[str, Any]) -> dict[str, float]:
    names = raw.get('metric_names', [])
    values = raw.get('metric_values', [])
    if (
        not isinstance(names, Sequence)
        or isinstance(names, (str, bytes, bytearray))
        or not isinstance(values, Sequence)
        or isinstance(values, (str, bytes, bytearray))
        or len(names) != len(values)
        or len(names) > 64
    ):
        raise SanitizationError('invalid_metrics')
    result = {}
    for name, value in zip(names, values):
        if (
            not isinstance(name, str)
            or not name.strip()
            or len(name.strip()) > 64
            or name.strip() in result
            or isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            raise SanitizationError('invalid_metrics')
        number = float(value)
        if not isfinite(number):
            raise SanitizationError('invalid_metrics')
        result[name.strip()] = number
    return result


def _window(raw: Mapping[str, Any]) -> tuple[float, float]:
    explicit = {
        'window_start',
        'window_end',
        'window_start_sec',
        'window_end_sec',
    }.intersection(raw)
    if 'window' in raw:
        if explicit or not isinstance(raw['window'], Mapping):
            raise SanitizationError('invalid_health')
        window = raw['window']
        _validate_fields(
            window,
            frozenset({'start', 'end', 'start_sec', 'end_sec'}),
        )
        start_keys = [key for key in ('start', 'start_sec') if key in window]
        end_keys = [key for key in ('end', 'end_sec') if key in window]
        if len(start_keys) != 1 or len(end_keys) != 1:
            raise SanitizationError('invalid_health')
        return (
            _time_to_seconds(window[start_keys[0]]),
            _time_to_seconds(window[end_keys[0]]),
        )
    start_keys = [
        key for key in ('window_start', 'window_start_sec') if key in raw
    ]
    end_keys = [key for key in ('window_end', 'window_end_sec') if key in raw]
    if len(start_keys) != 1 or len(end_keys) != 1:
        raise SanitizationError('invalid_health')
    return (
        _time_to_seconds(raw[start_keys[0]]),
        _time_to_seconds(raw[end_keys[0]]),
    )


def _observed_at(raw: Mapping[str, Any], window_end_sec: float) -> float:
    if 'observed_at_sec' in raw:
        return _time_to_seconds(raw['observed_at_sec'])
    if 'header' not in raw:
        return window_end_sec
    header = raw['header']
    if not isinstance(header, Mapping):
        raise SanitizationError('invalid_health')
    _validate_fields(header, frozenset({'stamp', 'frame_id'}))
    if 'stamp' not in header:
        raise SanitizationError('invalid_health')
    return _time_to_seconds(header['stamp'])


class AgentInputSanitizer:
    """Build allowlisted canonical health objects from plain mappings."""

    def assert_safe_payload(self, payload: Any) -> None:
        """Apply the recursive truth/leakage scan to a canonical payload."""
        _scan_for_prohibited_content(payload)

    def sanitize_health(self, raw: Mapping[str, Any]) -> HealthObservation:
        """Return a HealthObservation or fail closed with a safe error."""
        if not isinstance(raw, Mapping):
            raise SanitizationError('invalid_structure')
        _scan_for_prohibited_content(raw)
        if any(not isinstance(key, str) for key in raw):
            raise SanitizationError('invalid_structure')
        _validate_fields(raw, _ALLOWED_TOP_LEVEL)
        window_start_sec, window_end_sec = _window(raw)
        try:
            return HealthObservation(
                component=_component_name(raw),
                state=_health_state(raw.get('state')),
                health_score=_optional_score(raw, 'health_score'),
                detector_confidence=_optional_score(
                    raw,
                    'confidence',
                    'detector_confidence',
                ),
                detected_fault_hint=_fault_hint(raw),
                reasons=_reasons(raw.get('reasons')),
                metrics=_metrics(raw),
                window_start_sec=window_start_sec,
                window_end_sec=window_end_sec,
                sample_count=raw.get('sample_count'),
                observed_at_sec=_observed_at(raw, window_end_sec),
            )
        except SanitizationError:
            raise
        except (TypeError, ValueError, ValidationError):
            raise SanitizationError('invalid_health') from None
