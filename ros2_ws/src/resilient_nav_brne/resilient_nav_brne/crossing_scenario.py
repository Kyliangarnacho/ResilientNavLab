"""Pure state machine for the fixed pedestrian crossing demo."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class CrossingScenario:
    """Start after a real Path and stop at a measured world-coordinate target."""

    speed: float = 0.25
    target_world_y: float = -2.50
    world_y_direction: int = 1
    start_delay_sec: float = 0.5
    maximum_duration_sec: float = 10.0
    ready_at: float | None = None
    started_at: float | None = None
    complete: bool = False
    failed: bool = False

    def __post_init__(self):
        if (
            not all(math.isfinite(value) for value in (
                self.speed, self.target_world_y, self.start_delay_sec,
                self.maximum_duration_sec,
            ))
            or self.speed <= 0.0
            or self.world_y_direction not in (-1, 1)
            or self.start_delay_sec < 0.0
            or self.maximum_duration_sec <= 0.0
        ):
            raise ValueError('crossing scenario parameters are invalid')

    def command(self, *, now_sec: float, plan_ready: bool, world_y: float | None) -> float:
        """Return crossing-joint velocity, with zero for every unsafe state."""
        if (
            self.complete or self.failed or not plan_ready or world_y is None
            or not math.isfinite(now_sec) or not math.isfinite(world_y)
        ):
            return 0.0
        if self.ready_at is None:
            self.ready_at = now_sec
            return 0.0
        if now_sec - self.ready_at < self.start_delay_sec:
            return 0.0
        if self.started_at is None:
            self.started_at = now_sec
        if self.world_y_direction * (world_y - self.target_world_y) >= 0.0:
            self.complete = True
            return 0.0
        if now_sec - self.started_at > self.maximum_duration_sec:
            self.failed = True
            return 0.0
        return self.speed
