"""Installation tests for camera freeze source and health watch tools."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SETUP_FILE = PACKAGE_DIR / 'setup.py'
FREEZE_SOURCE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'camera_freeze_source.py'
)
WATCH_SOURCE = (
    PACKAGE_DIR
    / 'resilient_nav_health_assessment'
    / 'camera_health_watch.py'
)


def test_setup_registers_both_freeze_observation_tools():
    setup_source = SETUP_FILE.read_text(encoding='utf-8')

    assert "'camera_freeze_source = '" in setup_source
    assert 'camera_freeze_source:main' in setup_source
    assert "'camera_health_watch = '" in setup_source
    assert 'camera_health_watch:main' in setup_source


def test_freeze_source_defaults_keep_raw_and_test_topics_separate():
    source = FREEZE_SOURCE.read_text(encoding='utf-8')

    assert "DEFAULT_SOURCE_TOPIC = '/camera/c920/image_raw'" in source
    assert "DEFAULT_OUTPUT_TOPIC = '/test/camera/image_frozen'" in source
    assert 'output_topic must differ from source_topic' in source


def test_health_watch_is_display_only():
    source = WATCH_SOURCE.read_text(encoding='utf-8')

    assert 'create_subscription' in source
    assert 'create_publisher' not in source
    assert 'FaultStatus' not in source
    assert 'health_evaluator' not in source
