"""ROS-free lifecycle for one active dynamic-agent interaction."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class InteractionTransition:
    """One interaction-state transition for diagnostics."""

    entered_pedestrian_id: int | None = None
    released_pedestrian_id: int | None = None
    reason: str | None = None


class InteractionState:
    """Own one nearby dynamic track until sustained separation is observed."""

    def __init__(
        self,
        *,
        separation_margin: float = 0.20,
        history_size: int = 4,
        entry_distance: float = float('inf'),
    ):
        if not math.isfinite(separation_margin) or separation_margin <= 0.0:
            raise ValueError('separation_margin must be finite and positive')
        if history_size < 2:
            raise ValueError('history_size must be at least two')
        if entry_distance <= 0.0 or math.isnan(entry_distance):
            raise ValueError('entry_distance must be positive')
        self.separation_margin = separation_margin
        self.history_size = history_size
        self.entry_distance = entry_distance
        self.pedestrian_id: int | None = None
        self.min_distance_seen: float | None = None
        self.distance_history: deque[float] = deque(maxlen=history_size)
        # A released track is one completed interaction, not a new interaction
        # every 200 ms. The sensor tracker assigns a new ID after track loss.
        self._completed_ids: set[int] = set()

    @property
    def active(self) -> bool:
        return self.pedestrian_id is not None

    def reset(self) -> None:
        self.pedestrian_id = None
        self.min_distance_seen = None
        self.distance_history.clear()
        self._completed_ids.clear()

    def update(
        self,
        robot_xy: tuple[float, float],
        pedestrians: dict[int, tuple[float, float]],
    ) -> InteractionTransition:
        """Acquire a confirmed track or release after sustained separation."""
        values = (*robot_xy, *(value for xy in pedestrians.values() for value in xy))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('interaction positions must be finite')

        visible_ids = set(pedestrians)
        self._completed_ids.intersection_update(visible_ids)

        if self.pedestrian_id is not None:
            owner_id = self.pedestrian_id
            owner = pedestrians.get(owner_id)
            if owner is None:
                self._clear_active()
                return InteractionTransition(
                    released_pedestrian_id=owner_id,
                    reason='tracked pedestrian disappeared',
                )
            current_distance = math.dist(robot_xy, owner)
            self.min_distance_seen = min(self.min_distance_seen, current_distance)
            self.distance_history.append(current_distance)
            if (
                current_distance - self.min_distance_seen > self.separation_margin
                and _distance_window_is_separating(self.distance_history, self.history_size)
            ):
                self._completed_ids.add(owner_id)
                self._clear_active()
                return InteractionTransition(
                    released_pedestrian_id=owner_id,
                    reason='separation gain and distance trend clear',
                )
            return InteractionTransition()

        candidates = {
            pedestrian_id: position
            for pedestrian_id, position in pedestrians.items()
            if pedestrian_id not in self._completed_ids
            and math.dist(robot_xy, position) <= self.entry_distance
        }
        if not candidates:
            return InteractionTransition()
        pedestrian_id, position = min(
            candidates.items(), key=lambda item: math.dist(robot_xy, item[1])
        )
        current_distance = math.dist(robot_xy, position)
        self.pedestrian_id = pedestrian_id
        self.min_distance_seen = current_distance
        self.distance_history.clear()
        return InteractionTransition(
            entered_pedestrian_id=pedestrian_id,
            reason='confirmed dynamic pedestrian interaction',
        )

    def _clear_active(self) -> None:
        self.pedestrian_id = None
        self.min_distance_seen = None
        self.distance_history.clear()


def _distance_window_is_separating(history, history_size: int) -> bool:
    """Require both net distance increase and a positive windowed slope."""
    if len(history) < history_size or history[-1] <= history[0]:
        return False
    mean_index = (len(history) - 1) / 2.0
    mean_distance = sum(history) / len(history)
    numerator = sum(
        (index - mean_index) * (distance - mean_distance)
        for index, distance in enumerate(history)
    )
    denominator = sum(
        (index - mean_index) ** 2 for index in range(len(history))
    )
    return numerator / denominator > 0.0
