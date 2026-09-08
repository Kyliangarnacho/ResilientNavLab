"""Static contracts for the single-pedestrian head-on Scene 3 overlay."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE_ROOT / 'launch' / 'brne_scene3_head_on_demo.launch.py'
DRIVER = (
    PACKAGE_ROOT
    / 'resilient_nav_brne'
    / 'brne_crossing_pedestrian_driver.py'
)
MODEL = PACKAGE_ROOT / 'models' / 'brne_crossing_pedestrian.sdf'


def test_scene3_reuses_scene2_profile_with_one_head_on_sensor_agent():
    source = LAUNCH.read_text(encoding='utf-8')
    for expected in (
        "'x': -0.5",
        "'y': -3.5",
        "'Y': 3.141592653589793",
        "'goal_x': 2.8",
        "'global_obstacle_layer_enabled': 'false'",
        "'config' / 'brne_v1_runtime.yaml'",
        "parameters=[str(runtime_profile), {'use_sim_time': True}]",
        "'motion_axis': 'x'",
        "'target_world_y': -3.5",
        "'world_y_direction': -1",
        "'speed': 0.20",
        "'maximum_duration_sec': 18.0",
        "executable='brne_lidar_dynamic_agent_node'",
        "executable='brne_control_gate'",
    ):
        assert expected in source
    assert source.count("executable='create'") == 1
    assert "executable='brne_pedestrian_odometry_adapter'" not in source
    assert 'commitment' not in source
    assert "DeclareLaunchArgument('crossing_" not in source


def test_scene3_axis_is_supported_by_the_existing_driver_and_joint_limit():
    driver = DRIVER.read_text(encoding='utf-8')
    model = MODEL.read_text(encoding='utf-8')

    assert "self.declare_parameter('motion_axis', 'y')" in driver
    assert "self.motion_axis not in ('x', 'y')" in driver
    assert "if self.motion_axis == 'x'" in driver
    assert '<upper>3.0</upper>' in model
    assert '<xyz expressed_in="__model__">1 0 0</xyz>' in model
