"""Pure validation helpers for the explicitly armed BRNE control gate."""

from __future__ import annotations

import math


def validated_diff_drive_command(
    values: tuple[float, float, float, float, float, float],
    *,
    maximum_linear_velocity: float,
    maximum_angular_velocity: float,
    zero_tolerance: float = 1e-9,
) -> tuple[float, float] | None:
    """Return linear-x/angular-z only when the entire Twist is safe and bounded."""
    if (
        maximum_linear_velocity <= 0.0
        or maximum_angular_velocity <= 0.0
        or zero_tolerance < 0.0
        or len(values) != 6
        or not all(math.isfinite(value) for value in values)
    ):
        return None
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = values
    if (
        linear_x < 0.0
        or linear_x > maximum_linear_velocity
        or abs(angular_z) > maximum_angular_velocity
        or any(
            abs(value) > zero_tolerance
            for value in (linear_y, linear_z, angular_x, angular_y)
        )
    ):
        return None
    return linear_x, angular_z


def endpoint_is_self(endpoint, *, node_name: str, node_namespace: str) -> bool:
    """Identify this gate's endpoint without relying on opaque DDS identifiers."""
    namespace = node_namespace.rstrip('/') or '/'
    endpoint_namespace = str(endpoint.node_namespace).rstrip('/') or '/'
    return str(endpoint.node_name) == node_name and endpoint_namespace == namespace
