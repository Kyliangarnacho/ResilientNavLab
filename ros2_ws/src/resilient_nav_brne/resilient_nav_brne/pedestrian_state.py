"""Pure planar conversion for the fixed Gazebo child-frame pedestrian contract."""

from __future__ import annotations

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(x_value: float, y_value: float, yaw: float) -> tuple[float, float]:
    """Rotate one planar vector by ``yaw``."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * x_value - sine * y_value,
        sine * x_value + cosine * y_value,
    )


@dataclass(frozen=True)
class PlanarPedestrianState:
    """One finite pedestrian pose and velocity in the BRNE odom frame."""

    x: float
    y: float
    yaw: float
    velocity_x: float
    velocity_y: float


def state_in_odom(
    *,
    world_x: float,
    world_y: float,
    world_yaw: float,
    twist_x: float,
    twist_y: float,
    odom_origin_world_x: float,
    odom_origin_world_y: float,
    odom_origin_world_yaw: float,
) -> PlanarPedestrianState | None:
    """Transform the pinned Gazebo child-frame twist into the BRNE odom frame."""
    values = (
        world_x, world_y, world_yaw, twist_x, twist_y,
        odom_origin_world_x, odom_origin_world_y, odom_origin_world_yaw,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    world_velocity = rotate_xy(twist_x, twist_y, world_yaw)

    delta_x = world_x - odom_origin_world_x
    delta_y = world_y - odom_origin_world_y
    odom_x, odom_y = rotate_xy(
        delta_x, delta_y, -odom_origin_world_yaw
    )
    odom_velocity = rotate_xy(
        world_velocity[0], world_velocity[1], -odom_origin_world_yaw
    )
    state = PlanarPedestrianState(
        x=odom_x,
        y=odom_y,
        yaw=normalize_angle(world_yaw - odom_origin_world_yaw),
        velocity_x=odom_velocity[0],
        velocity_y=odom_velocity[1],
    )
    if not all(math.isfinite(value) for value in state.__dict__.values()):
        return None
    return state
