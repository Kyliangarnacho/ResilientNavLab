from pathlib import Path

import yaml


WORKSPACE_DIR = Path(__file__).resolve().parents[3]
FAULT_INJECTION_DIR = (
    WORKSPACE_DIR / 'src' / 'resilient_nav_fault_injection'
)
LOCALIZATION_DIR = WORKSPACE_DIR / 'src' / 'resilient_nav_localization'

HEALTHY_EKF_FILE = LOCALIZATION_DIR / 'config' / 'ekf.yaml'
FAULTED_EKF_FILE = FAULT_INJECTION_DIR / 'config' / 'faulted_ekf.yaml'
PHASE5_LAUNCH_FILE = (
    FAULT_INJECTION_DIR / 'launch' / 'phase5_imu_bias_ekf_demo.launch.py'
)
SETUP_FILE = FAULT_INJECTION_DIR / 'setup.py'


def load_healthy_ekf_parameters():
    with HEALTHY_EKF_FILE.open('r', encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)

    return config['ekf_filter_node']['ros__parameters']


def load_faulted_ekf_parameters():
    with FAULTED_EKF_FILE.open('r', encoding='utf-8') as config_stream:
        config = yaml.safe_load(config_stream)

    return config['faulted_ekf_filter_node']['ros__parameters']


def test_faulted_ekf_uses_faulted_input_topics():
    faulted = load_faulted_ekf_parameters()

    assert faulted['odom0'] == '/faulted/wheel/odometry'
    assert faulted['imu0'] == '/faulted/imu/data'


def test_faulted_ekf_publish_tf_is_disabled():
    faulted = load_faulted_ekf_parameters()

    assert faulted['publish_tf'] is False


def test_faulted_ekf_frames_and_fusion_match_stage4_health_config():
    healthy = load_healthy_ekf_parameters()
    faulted = load_faulted_ekf_parameters()
    shared_keys = [
        'two_d_mode',
        'frequency',
        'sensor_timeout',
        'publish_acceleration',
        'print_diagnostics',
        'world_frame',
        'odom_frame',
        'base_link_frame',
        'odom0_config',
        'odom0_queue_size',
        'odom0_nodelay',
        'odom0_differential',
        'odom0_relative',
        'imu0_config',
        'imu0_queue_size',
        'imu0_nodelay',
        'imu0_differential',
        'imu0_relative',
        'imu0_remove_gravitational_acceleration',
    ]

    for key in shared_keys:
        assert faulted[key] == healthy[key]


def test_phase5_launch_includes_stage4_complete_entry():
    launch_source = PHASE5_LAUNCH_FILE.read_text(encoding='utf-8')

    assert "get_package_share_directory('resilient_nav_localization')" in (
        launch_source
    )
    assert "'phase4_ekf_demo.launch.py'" in launch_source
    assert "'use_sensor_rviz': use_rviz" in launch_source


def test_phase5_launch_starts_fault_injection_and_faulted_ekf():
    launch_source = PHASE5_LAUNCH_FILE.read_text(encoding='utf-8')

    assert "executable='imu_bias_injector'" in launch_source
    assert "'imu_bias_demo.yaml'" in launch_source
    assert "executable='wheel_fault_injector'" in launch_source
    assert "'enabled': False" in launch_source
    assert "'input_topic': '/wheel/odometry'" in launch_source
    assert "'output_topic': '/faulted/wheel/odometry'" in launch_source
    assert "name='faulted_ekf_filter_node'" in launch_source
    assert "('odometry/filtered', '/odometry/faulted')" in launch_source


def test_phase5_launch_does_not_start_second_odom_tf_broadcaster():
    launch_source = PHASE5_LAUNCH_FILE.read_text(encoding='utf-8')

    assert 'odom_tf_broadcaster' not in launch_source
    assert 'start_odom_tf_broadcaster' not in launch_source


def test_setup_installs_faulted_ekf_config():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "os.path.join('share', package_name, 'config')" in setup_source
    assert "glob(os.path.join('config', '*.yaml'))" in setup_source
