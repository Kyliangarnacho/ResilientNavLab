import math

from resilient_nav_fusion.lidar_projection_evaluator import (
    compose_pose,
    interpolate_pose,
    Pose2D,
    project_scan_local,
    transform_points,
)
from sensor_msgs.msg import LaserScan


def test_interpolate_pose_uses_shortest_yaw_path():
    samples = [
        Pose2D(100, 0.0, 0.0, math.radians(170.0)),
        Pose2D(200, 2.0, 4.0, math.radians(-170.0)),
    ]

    pose = interpolate_pose(samples, 150)

    assert pose is not None
    assert (pose.x, pose.y) == (1.0, 2.0)
    assert abs(abs(pose.yaw) - math.pi) < 1e-12


def test_gt_lidar_pose_composes_static_extrinsic():
    base = Pose2D(100, 1.0, 2.0, math.pi / 2.0)

    lidar = compose_pose(base, -0.1, 0.0, 0.0)

    assert abs(lidar.x - 1.0) < 1e-12
    assert abs(lidar.y - 1.9) < 1e-12
    assert lidar.yaw == math.pi / 2.0


def test_local_scan_projection_obeys_strict_maximum_range():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [1.0, 12.0 - 1.0e-4, 12.0, float('inf')]

    points = project_scan_local(scan)
    projected = transform_points(points, Pose2D(1, 1.0, 2.0, 0.0))

    assert points.shape == (2, 2)
    assert abs(points[0, 0] - 1.0) < 1e-12
    assert abs(points[1, 1] - (12.0 - 1.0e-4)) < 1e-6
    assert abs(projected[0, 0] - 2.0) < 1e-12
    assert abs(projected[0, 1] - 2.0) < 1e-12
