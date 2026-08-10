"""Resource tests for the standalone camera health monitor."""

from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_DIR / 'config' / 'camera_health.yaml'
LAUNCH_FILE = PACKAGE_DIR / 'launch' / 'camera_health_monitor.launch.py'
SETUP_FILE = PACKAGE_DIR / 'setup.py'


def _parameters():
    document = yaml.safe_load(CONFIG_FILE.read_text(encoding='utf-8'))
    return document['camera_health_monitor']['ros__parameters']


def test_camera_config_uses_conservative_five_session_baseline_margin():
    parameters = _parameters()

    assert parameters['image_topic'] == '/camera/c920/image_raw'
    assert parameters['health_topic'] == '/health/camera'
    assert parameters['publish_rate_hz'] == 5.0
    assert parameters['stale_timeout_sec'] > 2.0 * 0.3821871270001793
    assert parameters['freeze_duration_sec'] == 2.0
    assert parameters['fault_confirmation_sec'] > 0.0
    assert parameters['recovery_confirmation_sec'] > 0.0


def test_exposure_v1_and_observational_candidate_config_are_present():
    parameters = _parameters()
    config_text = CONFIG_FILE.read_text(encoding='utf-8')

    for name in [
        'dark_mean_gray_candidate',
        'dark_p95_candidate',
        'dark_ratio_candidate',
        'bright_mean_gray_candidate',
        'bright_p05_candidate',
        'bright_p95_candidate',
        'bright_ratio_candidate',
        'blur_reference_max_age_sec',
        'blur_reference_laplacian_variance_min',
        'blur_reference_edge_density_min',
        'blur_laplacian_variance_candidate',
        'blur_edge_density_candidate',
        'blur_gray_std_min',
        'blur_entropy_min',
        'low_information_reference_max_age_sec',
        'low_information_reference_edge_density_min',
        'low_information_reference_entropy_min',
        'low_information_entropy_candidate',
        'low_information_edge_density_candidate',
        'low_information_dark_ratio_min',
        'low_information_gray_std_min',
    ]:
        assert name in parameters
    assert parameters['dark_mean_gray_candidate'] == 6.0
    assert parameters['dark_p95_candidate'] == 8.0
    assert parameters['dark_ratio_candidate'] == 0.90
    assert parameters['bright_mean_gray_candidate'] == 170.0
    assert parameters['bright_p05_candidate'] == 150.0
    assert parameters['bright_p95_candidate'] == 180.0
    assert parameters['blur_reference_max_age_sec'] == 3.0
    assert parameters['blur_reference_laplacian_variance_min'] == 100.0
    assert parameters['blur_reference_edge_density_min'] == 0.01
    assert parameters['blur_laplacian_variance_candidate'] == 10.0
    assert parameters['blur_edge_density_candidate'] == 0.001
    assert parameters['blur_gray_std_min'] == 20.0
    assert parameters['blur_entropy_min'] == 5.5
    assert parameters['low_information_reference_max_age_sec'] == 3.0
    assert parameters['low_information_reference_edge_density_min'] == 0.01
    assert parameters['low_information_reference_entropy_min'] == 5.2
    assert parameters['low_information_edge_density_candidate'] == 0.0005
    assert parameters['low_information_entropy_candidate'] == 5.8
    assert parameters['low_information_dark_ratio_min'] == 0.20
    assert parameters['low_information_gray_std_min'] == 20.0
    assert 'not final freeze' in config_text
    assert 'all three conditions must hold together' in config_text
    assert 'bright_ratio is intentionally excluded' in config_text
    assert 'naturally blank or low-texture scenes' in config_text
    assert 'not final frozen thresholds' in config_text


def test_camera_launch_starts_only_the_camera_monitor_with_config():
    launch_source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "executable='camera_health_monitor'" in launch_source
    assert "'camera_health.yaml'" in launch_source
    assert "default_value='/camera/c920/image_raw'" in launch_source
    assert "default_value='/health/camera'" in launch_source
    assert 'health_evaluator' not in launch_source
    assert 'FaultStatus' not in launch_source


def test_setup_registers_camera_monitor_and_installs_resources():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'camera_health_monitor = '" in setup_source
    assert 'camera_health_monitor:main' in setup_source
    assert "glob(os.path.join('launch', '*.launch.py'))" in setup_source
    assert "glob(os.path.join('config', '*.yaml'))" in setup_source
