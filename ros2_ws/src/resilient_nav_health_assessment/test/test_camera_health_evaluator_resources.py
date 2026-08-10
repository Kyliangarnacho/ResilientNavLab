"""Resource checks for optional camera health evaluation."""

from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_FILE = PACKAGE_DIR / 'config' / 'health_evaluator.yaml'
SOURCE_FILE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'health_evaluator.py'
)


def test_camera_subscription_is_disabled_by_default_for_phase6():
    with CONFIG_FILE.open('r', encoding='utf-8') as stream:
        parameters = yaml.safe_load(stream)['health_evaluator'][
            'ros__parameters'
        ]

    assert parameters['imu_health_topic'] == '/health/imu'
    assert parameters['wheel_health_topic'] == '/health/wheel'
    assert parameters['scan_health_topic'] == '/health/scan'
    assert parameters['camera_health_topic'] == ''


def test_evaluator_conditionally_subscribes_to_camera_health():
    source = SOURCE_FILE.read_text(encoding='utf-8')

    assert "'camera_health_topic'" in source
    assert "sensors.append('camera')" in source
    assert 'if camera_health_topic:' in source
