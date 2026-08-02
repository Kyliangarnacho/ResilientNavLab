from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'phase5_fault_injection.launch.py'
RVIZ_FILE = PACKAGE_DIR / 'rviz' / 'phase5_fault_injection.rviz'
FAULTED_EKF_FILE = PACKAGE_DIR / 'config' / 'faulted_ekf.yaml'
SCENARIO_DIR = PACKAGE_DIR / 'config' / 'scenarios'
SETUP_FILE = PACKAGE_DIR / 'setup.py'
RECORD_TOOL = (
    PACKAGE_DIR
    / 'resilient_nav_fault_injection'
    / 'phase5_record_bag.py'
)
PROBE_TOOL = (
    PACKAGE_DIR
    / 'resilient_nav_fault_injection'
    / 'fault_probe.py'
)


def test_unified_phase5_launch_declares_required_arguments():
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    for argument in ['scenario_file', 'use_rviz', 'record_bag', 'bag_output']:
        assert f"'{argument}'" in launch_source

    assert "'phase4_ekf_demo.launch.py'" in launch_source
    assert "'use_sensor_rviz': 'false'" in launch_source
    assert "name='faulted_ekf_filter_node'" in launch_source
    assert "('odometry/filtered', '/odometry/faulted')" in launch_source


def test_unified_phase5_launch_keeps_faulted_ekf_off_main_tf():
    with FAULTED_EKF_FILE.open('r', encoding='utf-8') as config_stream:
        faulted = yaml.safe_load(config_stream)

    parameters = faulted['faulted_ekf_filter_node']['ros__parameters']
    assert parameters['publish_tf'] is False
    assert parameters['odom0'] == '/faulted/wheel/odometry'
    assert parameters['imu0'] == '/faulted/imu/data'


def test_phase5_launch_creates_passthrough_faulted_topics():
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'output_topic': '/faulted/imu/data'" in launch_source
    assert "'output_topic': '/faulted/wheel/odometry'" in launch_source
    assert "'output_topic': '/faulted/scan'" in launch_source
    assert "'enabled': False" in launch_source


def test_phase5_scenarios_cover_required_experiments():
    expected = {
        'imu_bias_ekf_comparison.yaml': ('imu_bias_injector', 'bias'),
        'wheel_freeze_ekf_comparison.yaml': (
            'wheel_fault_injector',
            'freeze',
        ),
        'lidar_sector_blindness_demo.yaml': (
            'scan_fault_injector',
            'sector_blindness',
        ),
    }

    for file_name, (node_name, model) in expected.items():
        with (SCENARIO_DIR / file_name).open('r', encoding='utf-8') as stream:
            scenario = yaml.safe_load(stream)
        parameters = scenario[node_name]['ros__parameters']
        assert parameters['model'] == model
        assert parameters['start_time_sec'] == 5.0
        assert parameters['end_time_sec'] == 15.0
        assert parameters['enabled'] is True


def test_phase5_rviz_displays_required_topics_with_compatible_qos():
    rviz_source = RVIZ_FILE.read_text(encoding='utf-8')

    for topic in [
        '/robot_description',
        '/scan',
        '/faulted/scan',
        '/odometry/filtered',
        '/odometry/faulted',
    ]:
        assert topic in rviz_source

    assert 'Class: rviz_default_plugins/RobotModel' in rviz_source
    assert 'Class: rviz_default_plugins/TF' in rviz_source
    assert 'Reliability Policy: Best Effort' in rviz_source
    assert 'Fixed Frame: odom' in rviz_source


def test_bag_record_topic_set_and_unique_output_policy():
    record_source = RECORD_TOOL.read_text(encoding='utf-8')

    for topic in [
        '/clock',
        '/imu/data',
        '/faulted/imu/data',
        '/wheel/odometry',
        '/faulted/wheel/odometry',
        '/scan',
        '/faulted/scan',
        '/fault_injection/status',
        '/odometry/filtered',
        '/odometry/faulted',
        '/tf',
        '/tf_static',
    ]:
        assert topic in record_source

    assert 'while candidate.exists()' in record_source
    assert 'scenario_id_from_file' in record_source


def test_probe_and_bag_tools_are_installed():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert 'fault_probe =' in setup_source
    assert 'phase5_record_bag =' in setup_source
    assert 'phase5_replay_bag =' in setup_source
    assert "os.path.join('share', package_name, 'rviz')" in setup_source
    assert 'json.dumps' in PROBE_TOOL.read_text(encoding='utf-8')
